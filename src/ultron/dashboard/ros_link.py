"""ROS-free half of the dashboard <-> ROS 2 bridge (scripts/dashboard_bridge.py).

Kept free of rclpy so it runs and is tested without ROS: the HTTP client, the event-stream
reader, and the OccupancyGrid -> PNG conversion.
"""
import io
import json
import math
import urllib.error
import urllib.request

from PIL import Image

# nav2's action result codes (action_msgs/GoalStatus) -> dashboard goal status
ACTION_STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'FAILED'}


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

    def events(self):
        """Blocking iterator over the live stream. Ends when the connection drops."""
        with urllib.request.urlopen(self.url + '/api/stream', timeout=60) as response:
            yield from iter_sse(response)
