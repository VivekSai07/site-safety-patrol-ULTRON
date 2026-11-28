"""Map-scoped dashboard action backed by the simulation's checked Nav2 adapter."""

import argparse
import math
from pathlib import Path

from .slam_logger import yaw_of
from .navigation_contract import make_goal

DASHBOARD_ACTION = '/ultron/sim/nav/dashboard_goal'


class DashboardGoalServer:
    def __init__(self, node, nav):
        from rclpy.action import ActionServer, CancelResponse
        from ultron_nav_interfaces.action import NavigateDashboard

        self.nav = nav
        self.action = NavigateDashboard
        self.pending = None
        self.server = ActionServer(node, NavigateDashboard, DASHBOARD_ACTION, self.execute,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT)
        node.create_timer(0.05, self.tick)

    def finish(self, handle, status, reason):
        if status == 'REACHED':
            handle.succeed()
        elif status == 'CANCELLED' and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        return self.action.Result(status=status, reason=reason)

    async def execute(self, handle):
        from rclpy.task import Future

        if handle.is_cancel_requested:
            return self.finish(handle, 'CANCELLED', 'Cancelled before dispatch')
        if self.pending is not None:
            return self.finish(handle, 'REJECTED', 'GOAL_ALREADY_ACTIVE')
        request = handle.request
        pose = request.pose.pose
        q = pose.orientation
        if not math.isclose(math.hypot(q.x, q.y, q.z, q.w), 1.0, abs_tol=1e-3) or q.x or q.y:
            return self.finish(handle, 'REJECTED', 'INVALID_PLANAR_ORIENTATION')
        try:
            goal = make_goal(pose.position.x, pose.position.y, yaw_of(q),
                             mission_id=request.mission_id, map_id=request.map_id, map_version=request.map_version,
                             params=self.nav.config.navigation, reason='Dashboard goal', now=self.nav.now(),
                             frame_id=request.pose.header.frame_id)
        except ValueError as error:
            return self.finish(handle, 'REJECTED', str(error))
        result = self.nav.submit(goal)
        if result is not None:
            return self.finish(handle, result.status, result.reason)

        completion = Future()
        self.pending = (handle, completion)
        handle.publish_feedback(self.action.Feedback(status='ACCEPTED'))
        result = await completion
        return self.finish(handle, result.status, result.reason)

    def tick(self):
        if self.pending is None:
            return
        handle, completion = self.pending
        if handle.is_cancel_requested:
            self.nav.cancel('DASHBOARD_CANCEL')
        self.nav.tick()
        result = self.nav.tracker.result
        if result is not None:
            # Dashboard arrival is the Nav2 action outcome. Capture readiness is a separate condition.
            self.pending = None
            completion.set_result(result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from .config import load_sim_slam_config
    from .nav2_adapter import Nav2Adapter
    from .sim_bridge import check_isolation

    config = load_sim_slam_config()
    check_isolation(config)
    rclpy.init()
    node = Node('ultron_dashboard_goal', parameter_overrides=[Parameter('use_sim_time', value=True)])
    try:
        server = DashboardGoalServer(node, Nav2Adapter(node, config, args.map))
        try:
            rclpy.spin(node)
        finally:
            server.server.destroy()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
