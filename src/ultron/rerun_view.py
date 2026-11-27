"""Portable recording -> Rerun 0.32 SDK visualization. Optional dependency."""
import json
from pathlib import Path
import numpy as np
from PIL import Image


def _frames(path,follow):
    import time
    with path.open() as handle:
        while True:
            offset=handle.tell()
            line=handle.readline()
            if line.endswith('\n'):
                yield json.loads(line)
            elif follow:
                handle.seek(offset)
                time.sleep(.1)
                yield None
            else:
                break


def _events(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines(keepends=True) if line.endswith('\n')]


def export_rerun(run, spawn=False,follow=False):
    try:
        import rerun as rr
        import rerun.blueprint as rrb
    except ImportError as exc:
        raise RuntimeError('Rerun is not installed. Install rerun-sdk==0.32.0, then run `python -m ultron rerun RUN`. Portable telemetry is preserved.') from exc
    run=Path(run)
    rr.init('ULTRON · B2 Site Safety Patrol',spawn=spawn)
    rr.save(str(run/'ultron.rrd'))
    rr.log('world',rr.ViewCoordinates.RIGHT_HAND_Z_UP,static=True)
    rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(
        rrb.Spatial3DView(origin='world',name='B2 patrol and spatial memory'),
        rrb.Vertical(rrb.Spatial2DView(origin='world/camera/image',name='Simulated RGB'),
                     rrb.Spatial2DView(origin='world/camera/depth',name='Depth (metres)'),
                     rrb.TextLogView(origin='events',name='Mission and inspections')))))
    trajectory=[]
    pins={}
    events=_events(run/'events.jsonl')
    event_idx=0
    def log_event(event):
        rr.log('events',rr.TextLog(json.dumps(event)))
        if event['type']=='LADDER_CONFIRMED':
            observation=event['observation']
            pins[event['asset_id']]=observation['position']
            rr.log('world/assets/'+event['asset_id'],rr.Points3D([observation['position']],
                labels=[event['asset_id']],radii=.10,colors=[250,190,65]))
        if event['type']=='INSPECTION_COMMITTED' and event['asset_id'] in pins:
            color={'FLAGGED':[245,100,100],'MANUAL_REVIEW':[250,190,65],'AUTOMATED_PASS':[75,210,170]}[event['status']]
            rr.log('world/assets/'+event['asset_id'],rr.Points3D([pins[event['asset_id']]],labels=[event['asset_id']+' '+event['status']],radii=.12,colors=color))

    for frame in _frames(run/'telemetry.jsonl',follow):
        if follow:
            events=_events(run/'events.jsonl')
        if frame is None:
            while event_idx<len(events):
                event=events[event_idx]
                rr.set_time('sim_time',duration=event['time'])
                log_event(event)
                event_idx+=1
            rr.flush()
            continue
        rr.set_time('sim_time',duration=frame['time'])
        if 'scene_geoms' in frame:
            rr.log('world/scene',rr.Clear(recursive=True))
            trajectory=[]
            for geom in frame['scene_geoms']:
                log_geom(rr,'world/scene',geom)
        for geom in frame['robot_geoms']:
            log_geom(rr,'world/b2',geom)
        x,y,yaw=frame['pose']
        trajectory.append([x,y,.08])
        rr.log('world/trajectory',rr.LineStrips3D([trajectory],colors=[80,190,250]))
        # MuJoCo camera axes: right/up/back. Convert to Rerun optical right/down/forward.
        rotation=np.array(frame['camera_rotation']) @ np.diag([1,-1,-1])
        rr.log('world/camera',rr.Transform3D(translation=frame['camera_position'],mat3x3=rotation))
        for stream in ('image','depth'):
            rr.log('world/camera/'+stream,rr.Pinhole(image_from_camera=frame['intrinsics'],resolution=[640,360]))
        rr.log('world/camera/image',rr.Image(np.array(Image.open(run/frame['rgb']))))
        rr.log('world/camera/depth',rr.DepthImage(np.load(run/frame['depth']),meter=1.0))
        while event_idx<len(events) and events[event_idx]['time']<=frame['time']:
            event=events[event_idx]
            log_event(event)
            event_idx+=1
    while event_idx<len(events):
        event=events[event_idx]
        rr.set_time('sim_time',duration=event['time'])
        rr.log('events',rr.TextLog(json.dumps(event)))
        event_idx+=1
    rr.flush()
    print(run/'ultron.rrd')


def log_geom(rr,root,geom):
    size=np.array(geom['size'])
    # MuJoCo primitive enum: plane 0, sphere 2, capsule 3, cylinder 5, box 6.
    if geom['kind']==0:
        return
    if geom['kind']==2:
        size=np.repeat(size[0],3)
    elif geom['kind'] in (3,5):
        size=np.array([size[0],size[0],size[1]+(size[0] if geom['kind']==3 else 0)])
    rr.log(root+'/'+geom['name'],rr.Boxes3D(centers=[geom['position']],half_sizes=[size],
        quaternions=[geom['quaternion_xyzw']],colors=[geom['color']]))
