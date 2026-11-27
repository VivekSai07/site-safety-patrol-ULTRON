"""24-view open A-frame capture with a virtual inspection camera, no robot I/O."""
from datetime import date
import hashlib
import json
import math
from pathlib import Path


def capture360(output):
    import numpy as np
    import mujoco
    from .mujoco_sim import Simulation
    from .scene import scenarios
    from .planning import Grid
    from .viewpoints import candidate_views
    from .assessment import assess_coverage
    from .preview import make_preview
    output=Path(output);output.mkdir(parents=True,exist_ok=False);(output/'frames').mkdir()
    ladder=dict(scenarios()[2],position=[6.,4.5,0.])
    sim=Simulation([ladder],output,clear_clutter=True)
    manifest={'mission_id':'SIM-360-01','asset_id':'LAD-360-001','map_version':'open-aframe-360-v1',
        'inspection_date':date.today().isoformat(),'ladder_type':ladder['type'],'position':ladder['position'],
        'verified_not_applicable':[],'synthetic':True,'views':[],
        'camera_control':'SIMULATED_VIRTUAL_PAN_TILT_NOT_B2_ACTUATION',
        'identity_source':'MUJOCO_SEGMENTATION_ORACLE','motion':'KINEMATIC_NOT_PHYSICS_WALKING',
        'azimuth_basis':'Assumed ladder frame: front=-worldY, right=+worldX; not estimated from images'}
    grid=Grid((.2,.2,11.8,8.6),[(5.6,4.3,6.4,5.65)],radius=.65)
    frames=[]

    def move(goal,yaw):
        path=grid.path(sim.pose[:2],goal)
        if not path:return False
        for point in path:
            sim.t+=.25;sim.motion_speed=.3;sim.place([*point,yaw])
        sim.motion_speed=0.
        return True

    try:
        candidates=candidate_views(ladder['position'],ladder['type'],grid,full_orbit=True,
            start=sim.pose[:2],radius=2.2,open_space_confirmed=True,rear_evidence_missing=True)
        body=mujoco.mj_name2id(sim.model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
        ladder_body=mujoco.mj_name2id(sim.model,mujoco.mjtObj.mjOBJ_BODY,ladder['key'])
        geoms=np.where(sim.model.geom_bodyid==ladder_body)[0]
        for candidate in candidates:
            if not move(candidate['position'],candidate['yaw']):continue
            for band,height,limits in [('lower',.45,(0.,.9)),('middle',1.35,(.9,1.8)),('upper',2.35,(1.8,3.1))]:
                origin=sim.data.cam_xpos[sim.camera_id].copy()
                forward=np.asarray([6.,4.5,height])-origin;forward/=np.linalg.norm(forward)
                right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
                up=np.cross(right,forward)
                world_rotation=np.column_stack([right,up,-forward])
                body_rotation=sim.data.xmat[body].reshape(3,3)
                local_rotation=body_rotation.T@world_rotation
                quat=np.empty(4);mujoco.mju_mat2Quat(quat,local_rotation.ravel())
                sim.model.cam_quat[sim.camera_id]=quat
                sim.t+=.8;mujoco.mj_forward(sim.model,sim.data)
                frame,rgb,depth,seg=sim.capture()
                frame.update(mission_id=manifest['mission_id'],state='INSPECT_360_'+band.upper())
                frames.append(frame)
                v,u=np.nonzero(np.isin(seg[:,:,0],geoms))
                z=depth[v,u];K=np.asarray(frame['intrinsics']);R=np.asarray(frame['camera_rotation'])
                camera=np.column_stack([(u-K[0,2])*z/K[0,0],-(v-K[1,2])*z/K[1,1],-z])
                world=camera@R.T+np.asarray(frame['camera_position'])
                visible_count=int(np.count_nonzero((world[:,2]>=limits[0])&(world[:,2]<limits[1])))
                grey=rgb.astype(float).mean(axis=2)
                lap=grey[1:-1,:-2]+grey[1:-1,2:]+grey[:-2,1:-1]+grey[2:,1:-1]-4*grey[1:-1,1:-1]
                sharpness=float(lap.var())
                angle=(candidate['angle_deg']+90)%360
                view={'image_id':f'view-a{angle:03}-{band}','path':frame['rgb'],'azimuth_deg':angle,'band':band,
                    'pose_valid':True,'same_asset_confirmed':len(v)>=100,'region_visible':visible_count>=20,
                    'quality_accepted':sharpness>=10,'sha256':hashlib.sha256((output/frame['rgb']).read_bytes()).hexdigest(),
                    'pose':frame['pose'],'camera_position':frame['camera_position'],'camera_rotation':frame['camera_rotation'],
                    'intrinsics':frame['intrinsics'],'stamp_sim_s':frame['time'],
                    'visible_band_pixels_oracle':visible_count,'laplacian_variance':sharpness,
                    'quality_threshold_status':'PROPOSED_SIMULATION_HEURISTIC_NOT_REAL_CALIBRATION'}
                manifest['views'].append(view)
                print(f"{view['image_id']}: {visible_count} band pixels; sharpness {sharpness:.1f}",flush=True)
        manifest['returned_to_base']=move((1.5,1.5),math.pi/2)
    finally:sim.close()
    with (output/'telemetry.jsonl').open('w') as f:
        for frame in frames:f.write(json.dumps(frame)+'\n')
    manifest['coverage']=assess_coverage(manifest)
    (output/'keyframes.json').write_text(json.dumps(manifest,indent=2))
    make_preview(output)
    (output/'README.txt').write_text('Synthetic 24-view sampling exercise with virtual camera pan/tilt, kinematic robot motion and segmentation identity/visibility oracle. Not B2 camera actuation or proof of physical full-surface inspection. See keyframes.json for gaps and provenance.\n')
    return manifest


def main():
    import argparse,os
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();os.environ.setdefault('MUJOCO_GL','egl')
    result=capture360(args.out)
    print(json.dumps({'output':str(args.out),'coverage':result['coverage'],'returned_to_base':result['returned_to_base']},indent=2))


if __name__=='__main__':main()
