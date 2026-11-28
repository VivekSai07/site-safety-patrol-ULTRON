"""Count PointCloud2 messages from the Isaac smoke test (run inside scripts/sim-slam.sh exec). Evaluation helper."""
import time, json, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
rclpy.init(); n = Node('ultron_isaac_listener'); got = []
def cb(m):
    pts = list(pc2.read_points(m, field_names=('x', 'y', 'z'), skip_nans=True))
    r = [ (p[0]**2+p[1]**2)**0.5 for p in pts ]
    got.append({'t': time.time(), 'frame': m.header.frame_id, 'points': len(pts), 'fields': [f.name for f in m.fields],
                'range_min': round(min(r), 3) if r else None, 'range_max': round(max(r), 3) if r else None})
n.create_subscription(PointCloud2, '/ultron/sim/isaac/points', cb, qos_profile_sensor_data)
end = time.time() + 900
while time.time() < end and len(got) < 50:
    rclpy.spin_once(n, timeout_sec=0.5)
print(json.dumps({'messages': len(got), 'first': got[:1], 'last': got[-1:]}, indent=1))
