"""Execute goals using the verified DimOS NavigationInterfaceSpec RPC contract."""
import math
import time


class DimosNavigator:
    def __init__(self,navigation,*,frame_id='map',pose_factory=None,clock=time.monotonic,sleep=time.sleep):
        if pose_factory is None:
            from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
            pose_factory=PoseStamped
        self.navigation=navigation
        self.pose_factory=pose_factory
        self.frame_id=frame_id
        self.clock=clock
        self.sleep=sleep

    def navigate(self,position,*,yaw=0.,timeout=45.,operator_enabled=lambda:True):
        if not all(math.isfinite(v) for v in (*position,yaw,timeout)) or timeout<=0:
            raise ValueError('Finite goal and positive timeout required')
        goal=self.pose_factory(frame_id=self.frame_id,position=[*position[:2],0.],
                               orientation=[0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
        if not operator_enabled():
            self.navigation.cancel_goal('ULTRON operator inhibition')
            return {'status':'CANCELLED','reason':'Operator disabled motion'}
        if not self.navigation.set_goal(goal):
            return {'status':'REJECTED','reason':'DimOS rejected navigation goal'}
        started=self.clock()
        try:
            while True:
                if not operator_enabled():
                    self.navigation.cancel_goal('ULTRON operator inhibition')
                    return {'status':'CANCELLED','reason':'Operator disabled motion'}
                if self.navigation.is_goal_reached():
                    return {'status':'REACHED'}
                if self.navigation.is_goal_cancelled():
                    return {'status':'CANCELLED','reason':self.navigation.get_goal_cancel_reason()}
                if self.clock()-started>=timeout:
                    self.navigation.cancel_goal('ULTRON timeout')
                    return {'status':'TIMEOUT'}
                self.sleep(.05)
        except BaseException:
            self.navigation.cancel_goal('ULTRON interrupted')
            raise

    def execute(self,decision,**kwargs):
        if decision['action'] not in ('INSPECT','RETURN_BASE','RETURN_BASE_WITH_GAPS'):
            return {'status':'NO_MOTION','reason':decision['action']}
        return self.navigate(decision['goal'],yaw=decision.get('yaw',0.),**kwargs)
