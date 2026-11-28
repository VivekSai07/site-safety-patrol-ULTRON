"""Placeholder node for a sub-launch whose owner agent has not delivered it yet.

Publishes nothing on the topic contract (no fake sensors, no commands); it only logs which
agent owns the component so `ros2 node list` shows the namespace layout.
"""
import signal
import sys


def main(argv=None):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=argv)
    node = Node('placeholder')
    component = node.declare_parameter('component', 'unknown').value
    owner = node.declare_parameter('owner', 'unknown').value
    node.get_logger().info(f'STUB {component}: not implemented yet (owner Agent {owner}); publishes nothing')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:                       # ExternalShutdownException on SIGINT/SIGTERM
        pass
    finally:
        # Ignore repeated SIGINT/SIGTERM while tearing down so shutdown stays clean.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
