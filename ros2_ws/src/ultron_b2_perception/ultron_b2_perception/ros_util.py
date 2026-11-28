"""ROS message <-> numpy helpers shared by the perception nodes (rclpy messages only, no cv_bridge)."""
import numpy as np

from .geometry import transform

_DTYPES = {'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3), 'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4),
           'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1), 'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1),
           '32FC1': (np.float32, 1), '32SC1': (np.int32, 1), '16SC1': (np.int16, 1)}


def image_to_numpy(msg):
    if msg.encoding not in _DTYPES:
        raise ValueError(f'unsupported image encoding {msg.encoding}')
    dtype, channels = _DTYPES[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    row = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    itemsize = dtype.itemsize * channels
    arr = row[:, :msg.width * itemsize].copy().view(dtype)
    arr = arr.reshape(msg.height, msg.width, channels) if channels > 1 else arr.reshape(msg.height, msg.width)
    if msg.encoding in ('bgr8', 'bgra8'):
        arr = arr[..., [2, 1, 0] + ([3] if channels == 4 else [])]
    return arr.astype(dtype.newbyteorder('='), copy=False)


def numpy_to_image(arr, encoding, stamp, frame_id):
    from sensor_msgs.msg import Image  # noqa: PLC0415
    msg = Image()
    msg.header.stamp, msg.header.frame_id = stamp, frame_id
    msg.height, msg.width = arr.shape[:2]
    msg.encoding, msg.is_bigendian = encoding, 0
    arr = np.ascontiguousarray(arr)
    msg.step = int(arr.strides[0])
    msg.data = arr.tobytes()
    return msg


def tf_to_matrix(tf_stamped):
    t, q = tf_stamped.transform.translation, tf_stamped.transform.rotation
    return transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def pose_to_matrix(pose):
    p, q = pose.position, pose.orientation
    return transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def ladder_type_from_label(label):
    if label.startswith('ORACLE_SIM:'):
        return label.split(':', 1)[1]
    return 'UNKNOWN'  # learned labels give no validated type; type stays a proposal for review


def run_node(factory, argv=None, executor_threads=0):
    """Init rclpy, spin the node until SIGINT/SIGTERM, shut down cleanly."""
    import rclpy  # noqa: PLC0415
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor  # noqa: PLC0415
    rclpy.init(args=argv)
    node = factory()
    try:
        if executor_threads:
            executor = MultiThreadedExecutor(num_threads=executor_threads)
            executor.add_node(node)
            executor.spin()
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
