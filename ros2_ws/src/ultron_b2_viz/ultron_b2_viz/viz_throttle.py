"""viz_throttle: bandwidth-bounded copies of stack topics for visualization (Agent F).

Inputs are relative stack topics; outputs live under ``viz/`` and are the only data the Foxglove
whitelist and the domain-42 bridge carry. Rates are wall-clock limits (bandwidth is a wall-clock
concern). PROPOSED defaults, see config/viz_throttle.yaml.

* cloud streams (``points``, ``map_cloud``): voxel (first point per voxel) + max-points cap,
  output fields x, y, z, intensity (float32)
* ``image``: rgb8/bgr8/mono8 -> JPEG ``sensor_msgs/CompressedImage`` (image_transport naming
  ``.../image/compressed``, format ``"<enc>; jpeg compressed bgr8"``), optional downscale
* ``relays``: serialized pass-through with a rate cap (maps, costmaps, plan)
* ``viz/throttle_status``: JSON with measured in/out bytes per second per stream
"""
import io
import json
import time

import numpy as np

FLOAT32 = 7
POINTFIELD_SIZE = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}


def cloud_to_array(msg):
    """(N, 4) float32 x, y, z, intensity (intensity 0 if absent); drops non-finite points."""
    fields = {f.name: f for f in msg.fields}
    n = msg.width * msg.height
    if n == 0 or not all(k in fields for k in 'xyz'):
        return np.zeros((0, 4), np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * msg.point_step).reshape(n, msg.point_step)
    out = np.zeros((n, 4), np.float32)
    endian = '>' if msg.is_bigendian else '<'
    for i, name in enumerate(('x', 'y', 'z', 'intensity')):
        f = fields.get(name)
        if f is None:
            continue
        if f.datatype != FLOAT32:
            if name == 'intensity':
                continue
            raise ValueError(f'field {name} must be FLOAT32')
        out[:, i] = raw[:, f.offset:f.offset + 4].copy().view(endian + 'f4')[:, 0]
    return out[np.isfinite(out[:, :3]).all(axis=1)]


def voxel_downsample(points, voxel_m, max_points):
    """Keep the first point in each voxel, then cap the count with an even stride."""
    if len(points) == 0:
        return points
    if voxel_m > 0:
        keys = np.floor(points[:, :3] / voxel_m).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        points = points[np.sort(idx)]
    if max_points and len(points) > max_points:
        points = points[np.linspace(0, len(points) - 1, max_points).astype(np.int64)]
    return points


def jpeg_encode(data, width, height, encoding, quality, max_width):
    from PIL import Image
    channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, 'rgba8': 4, 'bgra8': 4}.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding {encoding}')
    arr = np.frombuffer(data, np.uint8, count=width * height * channels).reshape(height, width, channels)
    if encoding in ('bgr8', 'bgra8'):
        arr = arr[..., 2::-1]
    elif encoding == 'rgba8':
        arr = arr[..., :3]
    img = Image.fromarray(arr[..., 0] if channels == 1 else np.ascontiguousarray(arr))
    if max_width and width > max_width:
        img = img.resize((max_width, round(height * max_width / width)), Image.BILINEAR)
    buf = io.BytesIO()
    img.save(buf, format='JPEG', quality=int(quality))
    return buf.getvalue()


class _Meter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.t0 = time.monotonic()
        self.in_bytes = self.out_bytes = self.in_msgs = self.out_msgs = 0

    def snapshot(self):
        dt = max(time.monotonic() - self.t0, 1e-6)
        out = {'in_Bps': round(self.in_bytes / dt), 'out_Bps': round(self.out_bytes / dt),
               'in_hz': round(self.in_msgs / dt, 2), 'out_hz': round(self.out_msgs / dt, 2)}
        self.reset()
        return out


def main(args=None):
    import array

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image, PointCloud2, PointField
    from std_msgs.msg import String
    from rosidl_runtime_py.utilities import get_message

    rclpy.init(args=args)
    node = Node('viz_throttle')
    p = node.declare_parameter
    reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # Large sensor samples (~0.7 MB) are fragmented; best-effort readers lose them over loopback with
    # the default 212 KB socket buffer, so read reliable unless the source is best-effort only.
    sensor_in = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    if p('sensor_input_reliability', 'reliable').value == 'best_effort':
        sensor_in = qos_profile_sensor_data
    meters, last_out, pubs = {}, {}, {}

    def due(key, hz):
        now = time.monotonic()
        if hz <= 0 or now - last_out.get(key, -1e9) >= 1.0 / hz - 1e-3:
            last_out[key] = now
            return True
        return False

    fields = [PointField(name=n, offset=4 * i, datatype=FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]

    def cloud_stream(key, default_in, default_out, hz, voxel, max_pts):
        src = p(f'{key}.input', default_in).value
        if not src:
            return
        out = p(f'{key}.output', default_out).value
        rate = p(f'{key}.max_hz', hz).value
        vox = p(f'{key}.voxel_m', voxel).value
        cap = p(f'{key}.max_points', max_pts).value
        meters[key] = _Meter()
        pubs[key] = node.create_publisher(PointCloud2, out, reliable)

        def cb(msg, key=key):
            m = meters[key]
            m.in_msgs += 1
            m.in_bytes += len(msg.data)
            if not due(key, rate):
                return
            pts = voxel_downsample(cloud_to_array(msg), vox, cap)
            res = PointCloud2()
            res.header = msg.header
            res.height, res.width = 1, len(pts)
            res.fields, res.is_bigendian, res.is_dense = fields, False, True
            res.point_step, res.row_step = 16, 16 * len(pts)
            data = array.array('B')
            data.frombytes(np.ascontiguousarray(pts, dtype='<f4').tobytes())
            res.data = data
            pubs[key].publish(res)
            m.out_msgs += 1
            m.out_bytes += len(data)
        node.create_subscription(PointCloud2, src, cb, sensor_in)

    cloud_stream('points', 'sensors/lidar/points', 'viz/points', 2.0, 0.10, 30000)
    cloud_stream('map_cloud', '', 'viz/map_cloud', 1.0, 0.20, 200000)

    img_in = p('image.input', 'sensors/cam_head/color/image_raw').value
    if img_in:
        img_out = p('image.output', 'viz/cam_head/color/image/compressed').value
        img_hz = p('image.max_hz', 5.0).value
        quality = p('image.jpeg_quality', 70).value
        max_w = p('image.max_width', 640).value
        meters['image'] = _Meter()
        pubs['image'] = node.create_publisher(CompressedImage, img_out, reliable)

        def on_image(msg):
            m = meters['image']
            m.in_msgs += 1
            m.in_bytes += len(msg.data)
            if not due('image', img_hz):
                return
            try:
                jpg = jpeg_encode(msg.data, msg.width, msg.height, msg.encoding, quality, max_w)
            except ValueError as exc:
                node.get_logger().warn(str(exc), throttle_duration_sec=10.0)
                return
            res = CompressedImage()
            res.header = msg.header
            res.format = f'{msg.encoding}; jpeg compressed bgr8'
            data = array.array('B')
            data.frombytes(jpg)
            res.data = data
            pubs['image'].publish(res)
            m.out_msgs += 1
            m.out_bytes += len(jpg)
        node.create_subscription(Image, img_in, on_image, sensor_in)

    # Serialized relays: name, type, input, output, max_hz, latched
    relays = p('relays', [
        'map|nav_msgs/msg/OccupancyGrid|map|viz/map|1.0|latched',
        'global_costmap|nav_msgs/msg/OccupancyGrid|global_costmap/costmap|viz/global_costmap|1.0|volatile',
        'local_costmap|nav_msgs/msg/OccupancyGrid|local_costmap/costmap|viz/local_costmap|2.0|volatile',
        'plan|nav_msgs/msg/Path|plan|viz/plan|2.0|volatile',
    ]).value
    for spec in relays:
        key, typ, src, out, hz, durability = spec.split('|')
        msg_type = get_message(typ)
        is_latched = durability == 'latched'
        meters[key] = _Meter()
        pubs[key] = node.create_publisher(msg_type, out, latched if is_latched else reliable)
        pending = {}

        def on_raw(data, key=key, hz=float(hz), pending=pending):
            m = meters[key]
            m.in_msgs += 1
            m.in_bytes += len(data)
            if due(key, hz):
                pubs[key].publish(data)
                m.out_msgs += 1
                m.out_bytes += len(data)
                pending.pop('data', None)
            else:
                pending['data'] = data   # latest-wins; flushed by the timer so the final state is shown

        def flush(key=key, hz=float(hz), pending=pending):
            if 'data' in pending and due(key, hz):
                data = pending.pop('data')
                pubs[key].publish(data)
                meters[key].out_msgs += 1
                meters[key].out_bytes += len(data)
        node.create_subscription(msg_type, src, on_raw, latched if is_latched else reliable, raw=True)
        node.create_timer(1.0 / float(hz), flush)

    status_pub = node.create_publisher(String, p('status_topic', 'viz/throttle_status').value, latched)
    period = p('status_period_s', 5.0).value

    def status():
        msg = String()
        msg.data = json.dumps({'schema': 'ultron.viz_throttle_status/v1',
                               'streams': {k: m.snapshot() for k, m in meters.items()}})
        status_pub.publish(msg)
    node.create_timer(period, status)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
