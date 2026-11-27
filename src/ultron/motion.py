"""Shared B2 SportClient command semantics and bounded kinematic simulation profiles."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Command:
    vx: float
    vy: float
    vyaw: float
    dt: float
    operation: str = 'Move'

    def validate(self):
        if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
                   for v in (self.vx,self.vy,self.vyaw,self.dt)):
            raise ValueError('Motion values must be finite numbers')
        if not 0<self.dt<=.2 or math.hypot(self.vx,self.vy)>.500001 or abs(self.vyaw)>.800001:
            raise ValueError('Command exceeds prototype velocity/time limits')
        if self.operation not in ('Move','StopMove'):
            raise ValueError('Unsupported command')
        if self.operation=='StopMove' and any((self.vx,self.vy,self.vyaw)):
            raise ValueError('StopMove must have zero velocity')


def _profile(distance, speed, acceleration, dt):
    """Rest-to-rest trapezoidal distance profile sampled at fixed intervals."""
    ramp=min(speed/acceleration,math.sqrt(distance/acceleration))
    peak=acceleration*ramp
    cruise=max(0.,(distance-acceleration*ramp*ramp)/peak) if peak else 0.
    total=2*ramp+cruise
    previous=0.
    for step in range(1,math.ceil(total/dt)+1):
        t=min(step*dt,total)
        if t<=ramp:
            position=.5*acceleration*t*t
        elif t<=ramp+cruise:
            position=.5*acceleration*ramp*ramp+peak*(t-ramp)
        else:
            position=distance-.5*acceleration*(total-t)**2
        yield position,min(speed,max(0.,(position-previous)/dt))
        previous=position


def motion_samples(start, path, final_yaw, dt=.1):
    if not 0<dt<=.2:
        raise ValueError('dt must be in (0,.2]')
    if not all(math.isfinite(v) for v in (*start,final_yaw)):
        raise ValueError('Invalid pose')
    points=[tuple(start[:2])]
    for p in path:
        p=tuple(p[:2])
        if not all(math.isfinite(v) for v in p):
            raise ValueError('Invalid path')
        if math.dist(points[-1],p)<1e-8:
            continue
        if len(points)>=2:
            a,b=points[-2],points[-1]
            cross=(b[0]-a[0])*(p[1]-b[1])-(b[1]-a[1])*(p[0]-b[0])
            dot=(b[0]-a[0])*(p[0]-b[0])+(b[1]-a[1])*(p[1]-b[1])
            if abs(cross)<1e-8 and dot>0:
                points[-1]=p
                continue
        points.append(p)
    x,y,yaw=map(float,start)
    for goal in points[1:]+[None]:
        target=final_yaw if goal is None else math.atan2(goal[1]-y,goal[0]-x)
        delta=math.atan2(math.sin(target-yaw),math.cos(target-yaw))
        original=yaw
        sign=1 if delta>=0 else -1
        for angle,speed in _profile(abs(delta),.8,1.4,dt):
            yaw=original+sign*angle
            yield (x,y,yaw),Command(0.,0.,sign*speed,dt)
        if goal is not None:
            origin=(x,y);distance=math.dist(origin,goal)
            for travel,speed in _profile(distance,.5,.6,dt):
                x=origin[0]+math.cos(yaw)*travel
                y=origin[1]+math.sin(yaw)*travel
                yield (x,y,yaw),Command(speed,0.,0.,dt)
    yield (x,y,yaw),Command(0.,0.,0.,dt,'StopMove')


class B2SdkTransport:
    """Adapter for an injected *B2* SportClient. No network initialization on import.

    Caller owns fresh state, command cadence, independent watchdog and RC priority.
    This adapter alone is not a hardware safety supervisor.
    """
    def __init__(self, client):
        self.client=client

    def send(self, command, *, state_age_s, operator_enabled):
        try:
            command.validate()
            if (operator_enabled is not True or isinstance(state_age_s,bool)
                    or not math.isfinite(state_age_s) or not 0<=state_age_s<=.25):
                raise RuntimeError('Motion inhibited: operator disabled or state stale')
            result=self.client.StopMove() if command.operation=='StopMove' else self.client.Move(command.vx,command.vy,command.vyaw)
            if result!=0:
                raise RuntimeError(f'B2 SDK returned error {result}')
        except Exception:
            self.client.StopMove()
            raise


def connect_b2(*, network_interface, domain_id, operator_authorized=False):
    """Explicit connection hook for verified event integration; never called by demo."""
    if operator_authorized is not True:
        raise RuntimeError('A verified operator must authorize the real B2 connection')
    if not network_interface or not isinstance(domain_id,int) or isinstance(domain_id,bool) or domain_id<0:
        raise ValueError('Explicit verified NIC and DDS domain are required')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.b2.sport.sport_client import SportClient
    ChannelFactoryInitialize(domain_id,network_interface)
    client=SportClient()
    client.SetTimeout(.5)
    client.Init()
    return B2SdkTransport(client)
