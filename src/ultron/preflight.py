"""Which sensor topics are live, and is that enough to record? Standard library only.

`scripts/b2_check.py` measures message rates with rclpy and hands them to `evaluate`; the
verdict and the table live here so they are tested without ROS. A group passes when its
topics (all of them, or any one for `any` groups) arrive at least at `min_hz`.
"""
from .recording import COLOR, COLOR_INFO, DEPTH

GROUPS = {
    "camera": {"topics": [COLOR, DEPTH, COLOR_INFO], "min_hz": 10.0, "any": False,
               "fix": "RealSense driver (camera_namespace camera_depth, camera_name head)"},
    "lidar": {"topics": ["/scan_cloud", "/rslidar_points"], "min_hz": 5.0, "any": True,
              "fix": "RoboSense driver (rslidar_sdk); hall bag published /scan_cloud in lidar_link"},
    "imu": {"topics": ["/imu/data"], "min_hz": 20.0, "any": True,
            "fix": "Xsens driver (xsens_mti_ros2_driver); it also publishes /tf world->imu_link"},
    "tf": {"topics": ["/tf"], "min_hz": 5.0, "any": True, "fix": "comes from the IMU driver or localisation"},
    "unitree": {"topics": ["/sportmodestate", "/lf/sportmodestate", "/lowstate"], "min_hz": 1.0, "any": True,
                "fix": "Unitree ROS 2 environment (CycloneDDS on the 192.168.123.x interface, unitree_go messages)"},
    "pose": {"topics": ["/ultron/real/odom", "/odom"], "min_hz": 1.0, "any": True, "fix": "localisation"},
}
REQUIRED = ("camera", "lidar", "imu")
CAMERA_ONLY = ("camera",)


def all_topics():
    return list(dict.fromkeys(t for g in GROUPS.values() for t in g["topics"]))


def evaluate(rates, required=REQUIRED, missing_types=()):
    """(ok, rows). `rates` maps topic -> Hz for topics that exist (absent = not published)."""
    rows, ok = [], True
    for name, group in GROUPS.items():
        statuses = []
        for topic in group["topics"]:
            if topic in missing_types:
                statuses.append((topic, None, "NO TYPE"))
            elif topic not in rates:
                statuses.append((topic, None, "MISSING"))
            else:
                hz = rates[topic]
                statuses.append((topic, hz, "OK" if hz >= group["min_hz"] else "LOW"))
        good = [s for s in statuses if s[2] == "OK"]
        passed = bool(good) if group["any"] else len(good) == len(statuses)
        needed = name in required
        ok &= passed or not needed
        rows.append({"group": name, "required": needed, "passed": passed, "topics": statuses,
                     "fix": group["fix"]})
    return ok, rows


def table(rows):
    lines = []
    for row in rows:
        mark = "OK  " if row["passed"] else ("FAIL" if row["required"] else "--  ")
        tag = "required" if row["required"] else "optional"
        lines.append(f"[{mark}] {row['group']:<8} ({tag})")
        for topic, hz, status in row["topics"]:
            rate = f"{hz:6.1f} Hz" if hz is not None else "       --"
            lines.append(f"         {status:<8} {rate}  {topic}")
        if not row["passed"]:
            lines.append(f"         needs: {row['fix']}")
    return "\n".join(lines)
