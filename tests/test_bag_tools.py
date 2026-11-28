"""Bag tools in ros2_ws/tools: inspection of a synthetic rosbag2 and zip salvage (complete and truncated).

Needs the optional `rosbags` and NumPy packages (`pip install .[bags]`); skipped otherwise.
"""
import importlib.util
import shutil
import sqlite3
import tempfile
import unittest
import zipfile
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1] / "ros2_ws" / "tools"

try:
    import numpy as np
    import rosbags  # noqa: F401
    HAVE_DEPS = True
except ImportError:
    HAVE_DEPS = False


def _load(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


N_SCANS, N_IMU, RINGS, COLUMNS = 5, 50, 4, 30


def write_synthetic_bag(path, bag_inspect):
    """Tiny sqlite3 bag: tf_static, tf (orientation-only), imu 100 Hz, 10 Hz XYZIRT cloud, camera_info."""
    from rosbags.rosbag2 import Writer
    store = bag_inspect.typestore()
    T = store.types
    Time, Header = T["builtin_interfaces/msg/Time"], T["std_msgs/msg/Header"]
    Vec, Quat = T["geometry_msgs/msg/Vector3"], T["geometry_msgs/msg/Quaternion"]
    TfStamped, Transform = T["geometry_msgs/msg/TransformStamped"], T["geometry_msgs/msg/Transform"]
    TFMessage, Imu, PointField = T["tf2_msgs/msg/TFMessage"], T["sensor_msgs/msg/Imu"], T["sensor_msgs/msg/PointField"]
    PointCloud2, CameraInfo = T["sensor_msgs/msg/PointCloud2"], T["sensor_msgs/msg/CameraInfo"]
    RoiT = T["sensor_msgs/msg/RegionOfInterest"]
    t0 = 1_700_000_000

    def header(t, frame):
        return Header(stamp=Time(sec=t0 + int(t), nanosec=int(round((t % 1) * 1e9))), frame_id=frame)

    def tf(t, parent, child, xyz, q):
        return TfStamped(header=header(t, parent), child_frame_id=child,
                         transform=Transform(translation=Vec(x=xyz[0], y=xyz[1], z=xyz[2]),
                                             rotation=Quat(x=q[0], y=q[1], z=q[2], w=q[3])))

    dtype = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "timestamp"],
                      "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
                      "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 26})
    fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in
              [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4), ("timestamp", 18, 8)]]
    with Writer(path, version=8) as w:
        c_static = w.add_connection("/tf_static", TFMessage.__msgtype__, typestore=store)
        c_tf = w.add_connection("/tf", TFMessage.__msgtype__, typestore=store)
        c_imu = w.add_connection("/imu/data", Imu.__msgtype__, typestore=store)
        c_cloud = w.add_connection("/scan_cloud", PointCloud2.__msgtype__, typestore=store)
        c_info = w.add_connection("/cam/camera_info", CameraInfo.__msgtype__, typestore=store)
        static = TFMessage(transforms=[tf(0, "head_link", "head_color_frame", (0.0, -0.059, 0.0), (0, 0, 0, 1)),
                                       tf(0, "head_color_frame", "head_color_optical_frame", (0, 0, 0),
                                          (-0.5, 0.5, -0.5, 0.5))])
        w.write(c_static, (t0) * 10**9, store.serialize_cdr(static, TFMessage.__msgtype__))
        for i in range(N_IMU):
            t = i * 0.01
            yaw = 0.5 * t
            q = (0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2)))
            imu = Imu(header=header(t, "imu_link"), orientation=Quat(x=q[0], y=q[1], z=q[2], w=q[3]),
                      orientation_covariance=np.zeros(9), angular_velocity=Vec(x=0.0, y=0.0, z=0.5),
                      angular_velocity_covariance=np.zeros(9), linear_acceleration=Vec(x=0.0, y=0.0, z=9.81),
                      linear_acceleration_covariance=np.zeros(9))
            recv = int((t0 + t + 0.002) * 1e9)
            w.write(c_imu, recv, store.serialize_cdr(imu, Imu.__msgtype__))
            w.write(c_tf, recv, store.serialize_cdr(TFMessage(transforms=[tf(t, "world", "imu_link", (0, 0, 0), q)]),
                                                    TFMessage.__msgtype__))
        for s in range(N_SCANS):
            t = s * 0.1
            pts = np.zeros(RINGS * COLUMNS, dtype=dtype)
            az = np.repeat(np.linspace(0, 2 * np.pi, COLUMNS, endpoint=False)[None, :], RINGS, 0).ravel()
            pts["x"], pts["y"] = 5 * np.cos(az), 5 * np.sin(az)
            pts["z"] = np.repeat(np.linspace(-0.5, 0.5, RINGS), COLUMNS)
            pts["intensity"] = 10
            pts["ring"] = np.repeat(np.arange(RINGS), COLUMNS)
            pts["timestamp"] = t0 + t + np.tile(np.linspace(0, 0.099, COLUMNS), RINGS)
            cloud = PointCloud2(header=header(t, "lidar_link"), height=len(pts), width=1, fields=fields,
                                is_bigendian=False, point_step=26, row_step=26,
                                data=np.frombuffer(pts.tobytes(), np.uint8), is_dense=True)
            w.write(c_cloud, int((t0 + t + 0.1) * 1e9), store.serialize_cdr(cloud, PointCloud2.__msgtype__))
            info = CameraInfo(header=header(t, "head_color_optical_frame"), height=720, width=1280,
                              distortion_model="plumb_bob", d=np.zeros(5),
                              k=np.array([640.0, 0, 640, 0, 640, 360, 0, 0, 1]), r=np.eye(3).ravel(),
                              p=np.array([640.0, 0, 640, 0, 0, 640, 360, 0, 0, 0, 1, 0]), binning_x=0, binning_y=0,
                              roi=RoiT(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
            w.write(c_info, int((t0 + t + 0.01) * 1e9), store.serialize_cdr(info, CameraInfo.__msgtype__))


@unittest.skipUnless(HAVE_DEPS, "rosbags and numpy are optional (pip install .[bags])")
class BagToolsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bag_inspect = _load("bag_inspect")
        cls.recover = _load("recover_bag_zip")
        cls.tmp = Path(tempfile.mkdtemp(prefix="ultron_bagtools_", dir=Path(__file__).resolve().parent))
        cls.bag = cls.tmp / "synthetic_bag"
        write_synthetic_bag(cls.bag, cls.bag_inspect)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def test_inspect_reports_topics_tf_and_cloud_layout(self):
        rep = self.bag_inspect.inspect(self.bag, cloud_every=1)
        topics = rep["topics"]
        self.assertEqual(topics["/imu/data"]["count"], N_IMU)
        self.assertAlmostEqual(topics["/imu/data"]["header_rate_hz_mean"], 100.0, places=3)
        self.assertEqual(topics["/scan_cloud"]["frame_ids"], {"lidar_link": N_SCANS})
        self.assertAlmostEqual(topics["/scan_cloud"]["recv_minus_header_s"]["median"], 0.1, places=3)
        self.assertEqual({(s["parent"], s["child"]) for s in rep["tf"]["static"]},
                         {("head_link", "head_color_frame"), ("head_color_frame", "head_color_optical_frame")})
        self.assertAlmostEqual(rep["tf"]["static"][0]["rpy_deg"][0], -90.0, places=6)  # optical frame convention
        dyn = rep["tf"]["dynamic"][0]
        self.assertEqual((dyn["parent"], dyn["child"], dyn["translation_constant"]), ("world", "imu_link", True))
        self.assertAlmostEqual(dyn["yaw_range_deg"], float(np.degrees(0.5 * 0.49)), places=2)
        self.assertEqual(sorted(rep["tf"]["roots"]), ["head_link", "world"])
        cloud = rep["point_clouds"]["/scan_cloud"]
        self.assertEqual([f["name"] for f in cloud["fields"]], ["x", "y", "z", "intensity", "ring", "timestamp"])
        self.assertFalse(cloud["organized"])  # height=N, width=1 is a column, not an organized grid
        self.assertEqual(cloud["ring_values"]["count"], RINGS)
        self.assertEqual(cloud["time_field"], "timestamp")
        self.assertAlmostEqual(cloud["time_span_per_scan_s"]["median"], 0.099, places=4)
        self.assertAlmostEqual(cloud["range_m"]["min"], float(np.hypot(5, 0.5)), delta=0.6)
        self.assertEqual(rep["camera_info"]["/cam/camera_info"]["first"]["k"][0], 640.0)
        self.assertIn("Bag inspection", self.bag_inspect.markdown(rep))

    def _zip(self, name, method=zipfile.ZIP_DEFLATED):
        path = self.tmp / name
        with zipfile.ZipFile(path, "w", method) as z:
            for f in sorted(self.bag.iterdir()):
                z.write(f, f"{self.bag.name}/{f.name}")
        return path

    def test_recover_complete_zip(self):
        rep = self.recover.recover(self._zip("ok.zip"), self.tmp / "out_ok")
        self.assertEqual(rep["mode"], "central_directory")
        db = rep["databases"][0]
        self.assertEqual(db["integrity"], "ok")
        self.assertEqual(db["survival_total_fraction"], 1.0)

    def test_recover_truncated_zip_salvages_partial_db(self):
        full = self._zip("full_stored.zip", zipfile.ZIP_STORED)
        data = full.read_bytes()
        cut = self.tmp / "truncated.zip"
        cut.write_bytes(data[: int(len(data) * 0.6)])
        meta = self.bag / "metadata.yaml"
        rep = self.recover.recover(cut, self.tmp / "out_cut", metadata=meta)
        self.assertEqual(rep["mode"], "local_headers_streaming")
        self.assertEqual(full.read_bytes(), data)  # input untouched
        db = rep["databases"][0]
        self.assertNotEqual(db["integrity"], "ok")
        self.assertIn("recovered_path", db)
        con = sqlite3.connect(db["recovered_path"])
        self.assertGreaterEqual(con.execute("SELECT COUNT(*) FROM topics").fetchone()[0], 1)
        con.close()
        for topic, s in db["survival"].items():
            self.assertLessEqual(s["recovered"], s["expected"], topic)
        self.assertGreater(sum(s["recovered"] for s in db["survival"].values()), 0)
        self.assertLess(db["survival_total_fraction"], 1.0)


if __name__ == "__main__":
    unittest.main()
