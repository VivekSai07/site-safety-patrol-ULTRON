"""Common rclpy entry point: init, construct, spin, clean shutdown on SIGINT/SIGTERM."""
import signal


def run(node_name, factory, argv=None, multithreaded=False):
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.node import Node

    rclpy.init(args=argv)
    node = Node(node_name)
    impl = None
    executor = MultiThreadedExecutor(num_threads=4) if multithreaded else SingleThreadedExecutor()
    try:
        impl = factory(node)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if impl is not None and hasattr(impl, 'close'):
            impl.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
