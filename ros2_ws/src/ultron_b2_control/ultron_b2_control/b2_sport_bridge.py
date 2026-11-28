"""b2_sport_bridge: gated ``cmd_vel`` -> Unitree sport API (``unitree_api/msg/Request`` on /api/sport/request).

DISABLED BY DEFAULT and only ever tested against ``mock_sport_server``. Rules:
  * subscribes ONLY ``cmd_vel`` (the motion_gate output);
  * ``hardware_enable`` false (default) => no request publisher is even created, Arm fails;
  * preflight (sport_logic.preflight) refuses wrong domain / not localhost (mock) or anything but
    domain 0 + /ultron/live + valid hardware token (robot) => exit code 2, nothing published;
  * ``arm`` service (ultron_b2_interfaces/Arm) required; armed state latched off on any fault
    (non-finite cmd_vel, non-zero response code);
  * Move (api 1008) at ``move_rate_hz`` from the latest fresh command, clamped (velocity + accel);
  * watchdog: no fresh cmd_vel for ``watchdog_s`` => exactly one StopMove (api 1003), then silence;
    zero command or disarm => one StopMove;
  * reads /api/sport/response and /sportmodestate for health; publishes ``diagnostics``.
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist

from ultron_b2_interfaces.srv import Arm

from .sport_logic import API_MOVE, API_STOPMOVE, BridgeCore, Limits, move_parameter, preflight

LIMIT_KEYS = ('max_vx', 'min_vx', 'max_vy', 'max_wz', 'max_ax', 'max_ay', 'max_awz')


def _hw_token():
    try:
        from ultron_b2_bringup.preflight import check_hw_token
        from ultron_b2_bringup.modes import DEFAULT_HW_TOKEN, HW_TOKEN_ENV
    except ImportError as exc:
        return False, f'cannot check hardware token ({exc})'
    return check_hw_token(os.environ.get(HW_TOKEN_ENV, DEFAULT_HW_TOKEN))


class SportBridge(Node):
    def __init__(self):
        super().__init__('b2_sport_bridge')
        defaults = {'hardware_enable': False, 'target': 'mock', 'expected_domain_id': -1, 'move_rate_hz': 20.0,
                    'watchdog_s': 0.3, 'response_timeout_s': 1.0, 'request_topic': '/api/sport/request',
                    'response_topic': '/api/sport/response', 'state_topic': '/sportmodestate',
                    **{k: getattr(Limits(), k) for k in LIMIT_KEYS}}
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in defaults}
        self.p = p
        self.hardware_enable = bool(p['hardware_enable'])
        ns = self.get_namespace()
        token = _hw_token() if p['target'] == 'robot' else None
        self.preflight_ok, self.preflight_reason = preflight(
            target=p['target'], hardware_enable=self.hardware_enable, env=os.environ, namespace=ns,
            expected_domain_id=p['expected_domain_id'], hw_token_ok=token)
        self.core = BridgeCore(Limits(**{k: float(p[k]) for k in LIMIT_KEYS}), float(p['watchdog_s']),
                               float(p['move_rate_hz']))
        self.req_pub = None
        self.req_id = int(time.time() * 1e3) % (1 << 40)
        self.counts = {'move': 0, 'stopmove': 0, 'responses': 0, 'response_errors': 0}
        self.last_response = None
        self.last_state = None
        self.diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_timer(1.0, self._diagnostics)
        if not self.preflight_ok:
            self.get_logger().error(f'REFUSED: {self.preflight_reason}')
            return
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, qos)
        self.create_service(Arm, 'control/arm', self._on_arm)
        if not self.hardware_enable:
            self.get_logger().warn('hardware_enable=false: no request publisher; bridge is inert')
            return
        from unitree_api.msg import Request, Response
        self.Request = Request
        self.req_pub = self.create_publisher(Request, p['request_topic'], 10)
        self.create_subscription(Response, p['response_topic'], self._on_response, 10)
        try:
            from unitree_go.msg import SportModeState
            self.create_subscription(SportModeState, p['state_topic'], self._on_state, 10)
        except ImportError:
            self.get_logger().warn('unitree_go not available: no sportmodestate health input')
        self.create_timer(1.0 / float(p['move_rate_hz']), self._tick)
        self.create_timer(0.01, self._watchdog)
        self.get_logger().info(f'preflight PASS ({self.preflight_reason}); DISARMED until control/arm')

    def _now(self):
        return time.monotonic()     # wall-monotonic: the watchdog must not depend on a sim clock

    # ---------------------------------------------------------------- inputs
    def _on_cmd(self, msg):
        was_fault = self.core.fault
        self.core.on_cmd(msg.linear.x, msg.linear.y, msg.angular.z, self._now())
        if self.core.fault and not was_fault:
            self.get_logger().error(f'fault latched: {self.core.fault}; disarmed')
            self._watchdog()

    def _on_arm(self, req, res):
        res.schema_version = Arm.Request.SCHEMA_VERSION
        if req.arm:
            if not self.hardware_enable or self.req_pub is None:
                res.success, res.armed, res.message = False, False, 'refused: hardware_enable=false'
            elif not req.operator_id.strip():
                res.success, res.armed, res.message = False, False, 'refused: operator_id required'
            else:
                ok, why = self.core.arm(self._now())
                res.success, res.armed, res.message = ok, self.core.armed, why
        else:
            self.core.disarm(f'DISARM by {req.operator_id or "?"}: {req.reason}')
            self._watchdog()
            res.success, res.armed, res.message = True, False, 'disarmed'
        self.get_logger().warn(f'arm request arm={req.arm} operator={req.operator_id!r}: {res.message}')
        return res

    def _on_response(self, msg):
        self.counts['responses'] += 1
        self.last_response = (self._now(), int(msg.header.identity.api_id), int(msg.header.status.code))
        if msg.header.status.code != 0:
            self.counts['response_errors'] += 1
            self.core.latch_fault(f'RESPONSE_ERROR api {msg.header.identity.api_id} code {msg.header.status.code}')
            self.get_logger().error(f'fault latched: {self.core.fault}; disarmed')
            self._watchdog()

    def _on_state(self, _msg):
        self.last_state = self._now()

    # ---------------------------------------------------------------- output
    def _send(self, api_id, parameter=''):
        if self.req_pub is None:
            return
        r = self.Request()
        self.req_id += 1
        r.header.identity.id = self.req_id
        r.header.identity.api_id = api_id
        r.parameter = parameter
        self.req_pub.publish(r)
        self.counts['move' if api_id == API_MOVE else 'stopmove'] += 1

    def _apply(self, action):
        if action is None:
            return
        if action[0] == 'MOVE':
            self._send(API_MOVE, move_parameter(*action[1]))
        else:
            self._send(API_STOPMOVE)
            self.get_logger().info(f'StopMove: {action[1]}')

    def _tick(self):
        self._apply(self.core.tick(self._now()))

    def _watchdog(self):
        self._apply(self.core.check_watchdog(self._now()))

    def _diagnostics(self):
        now = self._now()
        s = DiagnosticStatus()
        s.name, s.hardware_id = 'b2_sport_bridge', str(self.p['target'])
        if not self.preflight_ok or self.core.fault:
            s.level = DiagnosticStatus.ERROR
        elif not self.core.armed:
            s.level = DiagnosticStatus.WARN
        else:
            s.level = DiagnosticStatus.OK
        s.message = (f'REFUSED: {self.preflight_reason}' if not self.preflight_ok else
                     f'fault {self.core.fault}' if self.core.fault else 'armed' if self.core.armed else 'disarmed')
        resp_age = now - self.last_response[0] if self.last_response else -1.0
        values = {'hardware_enable': self.hardware_enable, 'armed': self.core.armed, 'fault': self.core.fault,
                  'preflight': self.preflight_reason, **self.counts,
                  'last_response_age_s': round(resp_age, 3),
                  'response_missing': self.core.armed and not self.core.stopped and
                  (resp_age < 0 or resp_age > float(self.p['response_timeout_s'])),
                  'sportmodestate_age_s': round(now - self.last_state, 3) if self.last_state else -1.0}
        s.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        a = DiagnosticArray()
        a.header.stamp = self.get_clock().now().to_msg()
        a.status = [s]
        self.diag.publish(a)


def main(args=None):
    argv = sys.argv if args is None else args
    # Before any DDS participant: never join domain 0 unless explicitly targeting the robot.
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0') and not any('target:=robot' in a for a in argv):
        print('[b2_sport_bridge] REFUSED: ROS_DOMAIN_ID unset/0 without target:=robot', file=sys.stderr)
        sys.exit(2)
    rclpy.init(args=args)
    node = SportBridge()
    code = 0 if node.preflight_ok else 2
    try:
        if code == 0:
            rclpy.spin(node)
        else:
            node._diagnostics()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if code == 0 and node.req_pub is not None and not node.core.stopped:
            try:
                node._send(API_STOPMOVE)             # best effort on shutdown
            except Exception:                        # noqa: BLE001 - context may already be shut down
                pass
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
