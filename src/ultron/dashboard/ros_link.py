"""ROS-free half of the dashboard <-> ROS 2 bridge (scripts/dashboard_bridge.py).

Kept free of rclpy so it runs and is tested without ROS: the HTTP client, the event-stream
reader, the OccupancyGrid -> PNG conversion, the fence and obstacle grids, and the explorer's process.
"""
import io
import json
import math
import os
import shlex
import shutil
import signal
import struct
import subprocess
import time
import urllib.error
import urllib.request

from PIL import Image

# nav2's action result codes (action_msgs/GoalStatus) -> dashboard goal status
ACTION_STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'FAILED'}


COARSE = (0.4, 0.4)   # goal tolerance (metres, radians): a view needs the object in frame, not an exact pose
FINE = (0.1, 0.1)     # for an operator who asks again where the robot already is: they want it precise


def goal_tolerance(goal, robot, coarse=COARSE, fine=FINE):
    """(mode, xy, yaw) for a goal. Coarse to fine: an operator goal set within the coarse tolerance of
    where the robot stands asks for more precision than coarse gives, so it is reached finely.
    Goals from afar, and the pipeline's own, are coarse. No robot pose known: coarse."""
    if goal.get('by', 'operator') != 'operator' or robot is None:
        return 'coarse', *coarse
    if math.hypot(float(goal['x']) - robot[0], float(goal['y']) - robot[1]) <= coarse[0]:
        return 'fine', *fine
    return 'coarse', *coarse


def grid_png(width, height, data):
    """OccupancyGrid data (row 0 = lowest y; -1 unknown, 0..100 occupied %) -> greyscale PNG with
    row 0 = highest y, coloured like RViz's map view: free white, occupied black, unknown grey."""
    if len(data) != width * height:
        raise ValueError(f'grid has {len(data)} cells, expected {width}x{height}')
    lut = bytes(205 if v < 0 or v > 100 else 254 - round(v * 254 / 100) for v in range(-128, 128))
    pixels = bytes(lut[(v + 128) & 0xFF] for v in data)
    image = Image.frombytes('L', (width, height), pixels).transpose(Image.Transpose.FLIP_TOP_BOTTOM)
    out = io.BytesIO()
    image.save(out, 'PNG', optimize=True)
    return out.getvalue()


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def iter_sse(lines):
    """Parse a Server-Sent Events byte stream into (event, data_dict) pairs."""
    event, data = None, []
    for raw in lines:
        line = raw.decode('utf-8').rstrip('\r\n')
        if not line:
            if event and data:
                yield event, json.loads('\n'.join(data))
            event, data = None, []
        elif line.startswith('event:'):
            event = line[6:].strip()
        elif line.startswith('data:'):
            data.append(line[5:].strip())


class Dashboard:
    """Minimal client for the dashboard endpoints the bridge uses."""

    def __init__(self, url='http://127.0.0.1:8000', timeout=5.0):
        self.url, self.timeout = url.rstrip('/'), timeout

    def _send(self, method, path, body=None, content_type='application/json'):
        if body is not None and content_type == 'application/json':
            body = json.dumps(body).encode()
        request = urllib.request.Request(self.url + path, data=body, method=method,
                                         headers={'Content-Type': content_type} if body is not None else {})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            text = response.read()
        return json.loads(text) if text else None

    def live(self):
        return self._send('GET', '/api/live')

    def post_map(self, png, resolution, origin_x, origin_y):
        self._send('POST', f'/api/map?resolution={resolution}&origin_x={origin_x}&origin_y={origin_y}', png, 'image/png')

    def post_pose(self, mission_id, time_s, x, y, yaw, state=''):
        self._send('POST', f'/api/missions/{mission_id}/pose', {'time': time_s, 'x': x, 'y': y, 'yaw': yaw, 'state': state})

    def goal_status(self, seq, status, detail=''):
        """Report progress on goal `seq`. Returns False when a newer goal already replaced it."""
        try:
            self._send('POST', '/api/goal/status', {'seq': seq, 'status': status, 'detail': detail})
            return True
        except urllib.error.HTTPError as error:
            if error.code == 409:
                return False
            raise

    def exploration_status(self, seq, state, detail='', by=''):
        """Confirm request `seq`. Returns False when a newer request already replaced it."""
        try:
            self._send('POST', '/api/exploration/status', {'seq': seq, 'state': state, 'detail': detail, 'by': by})
            return True
        except urllib.error.HTTPError as error:
            if error.code == 409:
                return False
            raise

    def obstacles(self):
        return self._send('GET', '/api/obstacles')

    def events(self):
        """Blocking iterator over the live stream. Ends when the connection drops."""
        with urllib.request.urlopen(self.url + '/api/stream', timeout=60) as response:
            yield from iter_sse(response)


# Fence and obstacles (docs/EXPLORATION_CONTRACT.md) -------------------------------------------

def fence_mask(width, height, resolution, origin_x, origin_y, polygon):
    """Grid cells (row 0 = lowest y) whose centre lies inside `polygon` ([[x, y], ...], map frame):
    a bytearray, 1 inside, 0 outside. One scanline per row (even-odd rule), so a big map stays cheap."""
    mask = bytearray(width * height)
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        xs = sorted(x0 + (y - y0) * (x1 - x0) / (y1 - y0) for (x0, y0), (x1, y1) in edges if (y0 <= y) != (y1 <= y))
        for a, b in zip(xs[::2], xs[1::2]):
            # cells whose centre x = origin_x + (i + 0.5) * resolution lies in [a, b)
            first = max(0, math.ceil((a - origin_x) / resolution - 0.5))
            last = min(width, math.ceil((b - origin_x) / resolution - 0.5))
            if first < last:
                mask[row * width + first:row * width + last] = b'\x01' * (last - first)
    return mask


def fenced_grid(data, mask):
    """The SLAM map with every cell outside the fence occupied (100): the explorer sees no frontier
    there, so it never chases the free space lidar maps beyond the windows."""
    return [v if inside else 100 for v, inside in zip(data, mask)]


def keepout_grid(mask):
    """The fence as an occupancy mask for Nav2's KeepoutFilter: 100 outside, 0 inside."""
    return [0 if inside else 100 for inside in mask]


OBSTACLE_HEIGHTS = (0.05, 0.3)  # a low point for a ladder on the floor, one an obstacle layer cannot miss


def obstacle_cloud(cells, heights=OBSTACLE_HEIGHTS):
    """Cell centres [[x, y], ...] -> (point count, PointCloud2 data): float32 x, y, z little-endian,
    12 bytes per point, one point per cell and height."""
    points = [(float(x), float(y), float(z)) for x, y in cells for z in heights]
    return len(points), struct.pack(f'<{3 * len(points)}f', *(v for p in points for v in p))


# Exploration (m-explore-ros2's explore_lite) ----------------------------------------------------

# the dashboard's request states -> what the bridge does about them
EXPLORE_REQUESTS = {'STARTING': 'start', 'PAUSING': 'pause', 'RESUMING': 'resume', 'FINISHING': 'finish'}
EXPLORE_IDLE = {'IDLE', 'MAPPED', 'FAILED'}
# explore_lite_msgs/ExploreStatus.status values
STATUS_RUNNING = {'exploration_started', 'exploration_in_progress'}
STATUS_PAUSED = 'exploration_paused'
STATUS_COMPLETE = 'exploration_complete'


def explore_request(event, last_seq):
    """What the bridge does about an `exploration` event: 'start', 'pause', 'resume', 'finish' or None.
    Each request has a new seq; the bridge's own confirmations and lease renewals keep it, so they
    are ignored. Compared with !=, not >: a dashboard restart starts seq again at 0."""
    if event['seq'] == last_seq:
        return None
    return EXPLORE_REQUESTS.get(event['state'])


def explore_on_connect(event, last_seq, running):
    """On (re)connecting to the dashboard, what makes the page and the robot agree. Nothing from before the
    bridge started is replayed (last_seq None): a pending Start then fails and the operator presses it again.

    Returns 'stop' (stop the explorer quietly: the dashboard restarted and shows nothing running), 'fail'
    (report FAILED: the page shows exploration but no explorer runs), a request to carry out (asked while the
    stream was down), or None."""
    state, seq = event['state'], event['seq']
    if state in EXPLORE_IDLE:
        return 'stop' if running else None
    if last_seq is not None and seq != last_seq and state in EXPLORE_REQUESTS:
        return EXPLORE_REQUESTS[state]
    if not running:
        return 'finish' if state == 'FINISHING' else 'fail'
    return None


class Explorer:
    """Runs the explore node as a child process and carries out the dashboard's exploration requests.

    ROS-free so it is tested without ROS; the bridge hands in:
      publish_resume(flag)   publish std_msgs/Bool on explore/resume
      listeners()            subscriptions matched on explore/resume (0: nobody would hear it)
      status()               every explore/status text so far, oldest first, or None without explore_lite_msgs
                             (a log, not the latest: explore_lite says "complete" and "returning" in one go)
      report(seq, state, detail, by)   tell the dashboard
      interrupted()          a newer request is waiting: stop waiting for this one
      nav_ready()            the Nav2 action server is up (explore_lite waits for it, with no timeout, before planning)
    Each request runs to its confirmation on the caller's thread (the bridge's worker), never on a ROS callback.
    Rule: when the explorer cannot be trusted to obey, it is stopped, so FAILED never leaves it driving.
    """

    def __init__(self, command, publish_resume, listeners, report, status=None, ready_wait_s=30.0,
                 confirm_wait_s=3.0, stop_wait_s=3.0, poll_s=0.1, settle_s=1.0, interrupted=lambda: False,
                 nav_ready=lambda: True):
        self.command, self.publish_resume, self.listeners, self.report = command, publish_resume, listeners, report
        self.status, self.interrupted, self.nav_ready = status, interrupted, nav_ready
        self.ready_wait_s, self.confirm_wait_s, self.stop_wait_s, self.poll_s = ready_wait_s, confirm_wait_s, stop_wait_s, poll_s
        self.settle_s = settle_s  # alive at least this long before it counts as started (a stale listener may answer)
        self.proc = None
        self.paused = False
        self.seen = 0          # statuses watch() has looked at

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def handle(self, request, seq, by=''):
        try:
            getattr(self, request)(seq, by)
        except Exception as error:  # never take the bridge down; say what happened and stop the explorer
            self.stop()
            self.report(seq, 'FAILED', f'{request} failed: {error}'[:200], '')

    def fail(self, seq, _by=''):
        self.stop()
        self.report(seq, 'FAILED', 'explorer not running (bridge restarted); press Start again', '')

    def start(self, seq, _by=''):
        self.stop()  # an explorer left idle after "no frontiers left" must not run twice
        argv = shlex.split(self.command)
        if not argv or shutil.which(argv[0]) is None:
            self.report(seq, 'FAILED', f'could not start the explorer: {argv[0] if argv else "no command"} not found', '')
            return
        if shutil.which('setpriv'):  # the explorer dies with the bridge, even when the bridge is killed
            argv = ['setpriv', '--pdeathsig', 'TERM', '--', *argv]
        try:
            self.proc = subprocess.Popen(argv, start_new_session=True, stdin=subprocess.DEVNULL)
        except (OSError, ValueError) as error:
            self.proc = None
            self.report(seq, 'FAILED', f'could not start the explorer: {error}'[:200], '')
            return
        self.paused = False
        before = self.seen = self._status_count()
        # Ready when it says it started (explore_lite_msgs; "complete" right away counts too: nothing left to
        # explore), or when it listens on explore/resume and Nav2 is up: it subscribes only after its first
        # costmap arrived, then waits for Nav2, then plans. The second also covers a status topic we cannot hear.
        settled = time.monotonic() + self.settle_s
        ready = self._wait(lambda: time.monotonic() >= settled and (
            bool(self._status_since(before) & (STATUS_RUNNING | {STATUS_COMPLETE}))
            or self.listeners() > 0 and self.nav_ready()), self.ready_wait_s)
        if not self.running():
            self._report_exit(seq, 'during start-up')
        elif not ready:
            self.stop()
            self.report(seq, 'FAILED', f'explorer sent no sign of life in {self.ready_wait_s:.0f} s '
                                       '(no map on its costmap_topic, no TF, or no Nav2?)', '')
        else:
            self.report(seq, 'EXPLORING', '', '')

    def pause(self, seq, _by=''):
        self._send(seq, False, STATUS_PAUSED, 'PAUSED')

    def resume(self, seq, _by=''):
        self._send(seq, True, 'exploration_in_progress', 'EXPLORING')

    def finish(self, seq, by=''):
        if self.running():
            # explore_lite cancels every Nav2 goal when told to stop; once paused it has nothing left to cancel
            if not self.paused and self.listeners() > 0:
                self.publish_resume(False)
            self.stop()
        self.report(seq, 'MAPPED', '', by or 'operator')

    def _send(self, seq, flag, status, confirm):
        if not self.running():
            if self.proc:
                self._report_exit(seq, '')
            else:
                self.report(seq, 'FAILED', 'explorer not running', '')
            return
        if not self._wait(lambda: self.listeners() > 0, self.confirm_wait_s):
            self.stop()
            self.report(seq, 'FAILED', 'explorer not listening on explore/resume; stopped it', '')
            return
        before = self._status_count()
        self.publish_resume(flag)
        # explore_lite says "paused" or "in progress"; without the msgs package there is nothing to wait for.
        # Either way a listener heard us: confirm unless the explorer died meanwhile.
        if status and self.status:
            self._wait(lambda: status in self._status_since(before), self.confirm_wait_s)
        if not self.running():
            self._report_exit(seq, '')
            return
        self.paused = not flag
        self.report(seq, confirm, '', '')

    def watch(self, seq, state):
        """Between requests: report an explorer that died, or that ran out of frontiers by itself."""
        if self.proc is not None and self.proc.poll() is not None:
            if state not in EXPLORE_IDLE:
                self._report_exit(seq, '')
            self.proc = None
            return
        new, self.seen = self._status_since(self.seen), self._status_count()
        if STATUS_COMPLETE in new:
            if state in ('EXPLORING', 'RESUMING'):
                # the node stays up (it may drive back to its start with return_to_init); the next Start replaces it
                self.report(seq, 'MAPPED', 'no frontiers left', 'explorer')

    def stop(self):
        """SIGINT to the whole process group (ros2 launch and the node), then SIGTERM, then SIGKILL."""
        proc, self.proc, self.paused = self.proc, None, False
        if proc is None or proc.poll() is not None:
            return
        for sig, wait_s in ((signal.SIGINT, self.stop_wait_s), (signal.SIGTERM, 1.0), (signal.SIGKILL, 1.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(wait_s)
                break
            except subprocess.TimeoutExpired:
                continue

    def _report_exit(self, seq, when):
        code = self.proc.poll() if self.proc else None
        self.proc = None
        self.report(seq, 'FAILED', f'explorer exited with code {code} {when}'.strip(), '')

    def _status_count(self):
        return len(self.status()) if self.status else 0

    def _status_since(self, count):
        return set(self.status()[count:]) if self.status else set()

    def _wait(self, ready, timeout_s):
        end = time.monotonic() + timeout_s
        while True:
            if ready():
                return True
            if not self.running() or time.monotonic() >= end or self.interrupted():
                return False
            time.sleep(self.poll_s)
