"""active_source_monitor: publishes ``control/active_source`` (hold | teleop | autonomy | none).

Uses the twist_mux priority table (same YAML, ``topics.*`` / ``locks.*``) and input freshness.
Input timeouts are shortened by ``timeout_margin_s`` so the reported source leaves ``teleop``
slightly BEFORE twist_mux does (falling back to the stricter ``autonomy`` early, never late).
A lock with timeout 0 holds its last value; a lock with timeout > 0 that goes silent counts as locked.
Publishes on every change and at ``rate_hz``.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


def select_source(now, inputs, locks):
    """inputs: {name: (priority, timeout, last_stamp)}; locks: {name: (priority, timeout, value, last_stamp)}.

    Returns 'hold' if any lock with priority >= the highest input priority is active, else the
    name of the highest-priority fresh input, else 'none'.
    """
    top_input = max((p for p, _, _ in inputs.values()), default=0)
    for prio, timeout, value, stamp in locks.values():
        expired = timeout > 0 and (stamp is None or now - stamp > timeout)
        if (value or expired) and prio >= top_input:
            return 'hold'
    lock_prio = max((p for p, t, v, s in locks.values() if v or (t > 0 and (s is None or now - s > t))), default=-1)
    best = None
    for name, (prio, timeout, stamp) in inputs.items():
        if stamp is None or now - stamp > timeout or prio <= lock_prio:
            continue
        if best is None or prio > inputs[best][0]:
            best = name
    return best or 'none'


class ActiveSourceMonitor(Node):
    def __init__(self):
        super().__init__('active_source_monitor', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        margin = self._param('timeout_margin_s', 0.05)
        rate = self._param('rate_hz', 20.0)
        self.inputs, self.locks = {}, {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        for name in self._names('topics'):
            topic = self._param(f'topics.{name}.topic', '')
            timeout = max(0.0, float(self._param(f'topics.{name}.timeout', 0.5)) - margin)
            prio = int(self._param(f'topics.{name}.priority', 0))
            self.inputs[name] = [prio, timeout, None]
            self.create_subscription(Twist, topic, lambda _m, n=name: self._on_input(n), qos)
        for name in self._names('locks'):
            topic = self._param(f'locks.{name}.topic', '')
            timeout = float(self._param(f'locks.{name}.timeout', 0.0))
            prio = int(self._param(f'locks.{name}.priority', 255))
            self.locks[name] = [prio, timeout, False, None]
            self.create_subscription(Bool, topic, lambda m, n=name: self._on_lock(n, m), qos)
        if not self.inputs:
            self.get_logger().error('no twist_mux topics configured; reporting none')
        self.pub = self.create_publisher(String, 'control/active_source', qos)
        self.current = None
        self.create_timer(1.0 / rate, self._publish)

    def _param(self, name, default):
        if self.has_parameter(name):
            v = self.get_parameter(name).value
            return default if v is None else v
        return default

    def _names(self, prefix):
        return sorted({k.split('.')[1] for k in self._parameters if k.startswith(prefix + '.')})

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_input(self, name):
        self.inputs[name][2] = self._now()
        self._publish(on_event=True)

    def _on_lock(self, name, msg):
        self.locks[name][2], self.locks[name][3] = bool(msg.data), self._now()
        self._publish(on_event=True)

    def _publish(self, on_event=False):
        now = self._now()
        src = select_source(now, {k: tuple(v) for k, v in self.inputs.items()},
                            {k: tuple(v) for k, v in self.locks.items()})
        if on_event and src == self.current:
            return
        if src != self.current:
            self.get_logger().info(f'active source: {self.current} -> {src}')
            self.current = src
        self.pub.publish(String(data=src))


def main(args=None):
    rclpy.init(args=args)
    node = ActiveSourceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if rclpy.ok() or str(exc) != 'Unable to convert call argument to Python object (compile in debug mode for details)':
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
