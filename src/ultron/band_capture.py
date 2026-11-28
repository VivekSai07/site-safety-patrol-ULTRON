"""Virtual pan/tilt and segmentation visibility audit. Simulation fixtures ONLY."""
import hashlib
import math
import numpy as np
import mujoco

BANDS={'lower':(.45,0.,.9),'middle':(1.35,.9,1.8),'upper':(2.35,1.8,3.1)}


def capture_band(sim, target, key, band):
    height, low, high=BANDS[band]
    body=mujoco.mj_name2id(sim.model,mujoco.mjtObj.mjOBJ_BODY,'base_link')
    ladder_body=mujoco.mj_name2id(sim.model,mujoco.mjtObj.mjOBJ_BODY,key)
    if ladder_body<0:raise ValueError('Synthetic target body missing')
    original=sim.model.cam_quat[sim.camera_id].copy()
    origin=sim.data.cam_xpos[sim.camera_id].copy()
    forward=np.asarray([target[0],target[1],target[2]+height])-origin
    forward/=np.linalg.norm(forward)
    right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
    up=np.cross(right,forward)
    rotation=sim.data.xmat[body].reshape(3,3).T@np.column_stack([right,up,-forward])
    mujoco.mju_mat2Quat(sim.model.cam_quat[sim.camera_id],rotation.ravel())
    try:
        mujoco.mj_forward(sim.model,sim.data)
        frame,rgb,depth,seg=sim.capture()
    finally:
        sim.model.cam_quat[sim.camera_id]=original
        mujoco.mj_forward(sim.model,sim.data)
    ids=np.where(sim.model.geom_bodyid==ladder_body)[0]
    v,u=np.nonzero(np.isin(seg[:,:,0],ids))
    z=depth[v,u];K=np.asarray(frame['intrinsics'])
    camera=np.column_stack([(u-K[0,2])*z/K[0,0],-(v-K[1,2])*z/K[1,1],-z])
    world=camera@np.asarray(frame['camera_rotation']).T+np.asarray(frame['camera_position'])
    visible=int(np.count_nonzero((world[:,2]>=target[2]+low)&(world[:,2]<target[2]+high)))
    grey=rgb.astype(float).mean(axis=2)
    lap=grey[1:-1,:-2]+grey[1:-1,2:]+grey[:-2,1:-1]+grey[2:,1:-1]-4*grey[1:-1,1:-1]
    sharpness=float(lap.var())
    angle=(math.degrees(math.atan2(sim.pose[1]-target[1],sim.pose[0]-target[0]))+90)%360
    frame.update(azimuth_deg=angle,band=band,pose_valid=True,same_asset_confirmed=len(v)>=100,
                 region_visible=visible>=20,quality_accepted=sharpness>=10,
                 visible_band_pixels_oracle=visible,laplacian_variance=sharpness,
                 camera_control='SIMULATED_VIRTUAL_PAN_TILT_NOT_B2_ACTUATION')
    return frame,rgb,depth,seg
