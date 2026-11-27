"""Rendered B2 workflow spike. Base motion is kinematic, detections are oracle masks."""
import json
import math
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image
from .scene import make_scene


class Simulation:
    def __init__(self, ladders, output, viewer=False, clear_clutter=False, group_prefix="ultron_mock_site_v1"):
        self.output=Path(output)
        self.ladders=ladders
        self.xml=make_scene(ladders,clear_clutter)
        self.group_prefix=group_prefix
        (self.output/'scene.xml').write_text(self.xml)
        self.model=mujoco.MjModel.from_xml_string(self.xml)
        self.data=mujoco.MjData(self.model)
        if viewer:
            # Avoid large MSAA/shadow buffers on WSL/software GL.
            self.model.vis.quality.offsamples=0
            self.model.vis.quality.shadowsize=512
        self.renderer=mujoco.Renderer(self.model,360,640)
        self.camera_id=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_CAMERA,'inspection')
        self.t=0.
        self.pose=[1.5,1.5,math.pi/2]
        self.motion_speed=0.
        self.frames=0
        self.scene_logged=False
        self.viewer=None
        if viewer:
            import os
            from .live_view import LiveView
            self.viewer=LiveView(self.model,self.data,fps=float(os.environ.get('ULTRON_VIEWER_FPS','5')))
        self.place(self.pose)

    def place(self, pose):
        self.pose=list(pose)
        x,y,yaw=pose
        self.data.qpos[:7]=[x,y,.65,math.cos(yaw/2),0,0,math.sin(yaw/2)]
        for j in range(1,self.model.njnt):
            name=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_JOINT,j)
            # Cosmetic diagonal-leg cycle only; no contact/torque locomotion claim.
            phase=self.t*2*math.pi*1.3+(math.pi if name.startswith(('FR_','RL_')) else 0.)
            swing=min(self.motion_speed/.5,1.)*.18*math.sin(phase)
            self.data.qpos[self.model.jnt_qposadr[j]] = .65+swing if 'thigh' in name else -1.3-.7*swing if 'calf' in name else 0.
        self.data.time=self.t
        mujoco.mj_forward(self.model,self.data)
        if self.viewer:
            import time
            self.viewer.sync()
            time.sleep(.05)

    def geometry(self, robot):
        result=[]
        for gid in range(self.model.ngeom):
            body=int(self.model.geom_bodyid[gid])
            name=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_BODY,body) or 'world'
            is_robot=name=='base_link' or name.startswith(('FL_','FR_','RL_','RR_'))
            if is_robot != robot:
                continue
            q=np.empty(4)
            mujoco.mju_mat2Quat(q,self.data.geom_xmat[gid])
            result.append({'entity':name if body else (mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,gid) or 'world'),'id':gid,'name':mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,gid) or f'geom_{gid}',
                'position':self.data.geom_xpos[gid].tolist(),'quaternion_xyzw':[float(q[1]),float(q[2]),float(q[3]),float(q[0])],
                'size':self.model.geom_size[gid].tolist(),'kind':int(self.model.geom_type[gid]),
                'color':self.model.geom_rgba[gid].tolist()})
        return result

    def capture(self, save=True):
        self.renderer.update_scene(self.data,camera='inspection')
        rgb=self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth=self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        self.renderer.enable_segmentation_rendering()
        segmentation=self.renderer.render().copy()
        self.renderer.disable_segmentation_rendering()
        f=180/math.tan(math.radians(float(self.model.cam_fovy[self.camera_id]))/2)
        rotation=self.data.cam_xmat[self.camera_id].reshape(3,3).copy()
        position=self.data.cam_xpos[self.camera_id].copy()
        frame={'index':self.frames,'time':self.t,'pose':self.pose.copy(),
               'camera_position':position.tolist(),'camera_rotation':rotation.tolist(),
               'intrinsics':[[f,0,320],[0,f,180],[0,0,1]],'synthetic':True}
        frame['robot_geoms']=self.geometry(True)
        if not self.scene_logged:
            frame['scene_geoms']=self.geometry(False)
            self.scene_logged=True
        if save:
            stem=f'frames/{self.frames:05}'
            Image.fromarray(rgb).save(self.output/(stem+'.jpg'),quality=87)
            np.save(self.output/(stem+'-depth.npy'),depth.astype(np.float32))
            frame.update(rgb=stem+'.jpg',depth=stem+'-depth.npy',overview=stem+'-overview.jpg')
            Image.fromarray(self.overview()).save(self.output/frame['overview'],quality=80)
        if save:
            objects=[]
            for ladder in self.ladders:
                body=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,ladder['key'])
                geom_ids=np.where(self.model.geom_bodyid==body)[0]
                v,u=np.nonzero(np.isin(segmentation[:,:,0],geom_ids))
                if len(u)>=20:
                    objects.append({'box_xyxy':[int(u.min()),int(v.min()),int(u.max())+1,int(v.max())+1],
                        'asset_group':self.group_prefix+':'+ladder['key'],'ladder_type':ladder['type'],
                        'defect_components':[ladder['defect']] if ladder.get('defect') else [],
                        'visible_pixels':int(len(u))})
            with (self.output/'annotations.jsonl').open('a') as annotations:
                annotations.write(json.dumps({'rgb':frame['rgb'],'frame_index':self.frames,'group_prefix':self.group_prefix,'objects':objects,'synthetic':True})+'\n')
        self.frames+=1
        return frame,rgb,depth,segmentation

    def visible_parts(self,key,segmentation):
        groups={'rear_feet':[f'{key}_rear_foot_{s}' for s in (-1,1)],
                'spreader':[f'{key}_spreader']}
        visible={}
        for part,names in groups.items():
            ids=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,name) for name in names]
            if all(gid>=0 and np.count_nonzero(segmentation[:,:,0]==gid)>=3 for gid in ids):
                visible[part]='PASS'
        return visible

    def oracle_detections(self, frame, depth, segmentation):
        detections=[]
        ids=segmentation[:,:,0]
        K=np.array(frame['intrinsics'])
        R=np.array(frame['camera_rotation'])
        origin=np.array(frame['camera_position'])
        for ladder in self.ladders:
            geom_ids=[]
            for side in (-1,1):
                geom_ids.append(mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,f"{ladder['key']}_rail_{side}"))
            if any(np.count_nonzero(ids==gid)<20 for gid in geom_ids):
                continue
            mask=np.isin(ids,geom_ids)
            v,u=np.nonzero(mask)
            if len(u)<35:
                continue
            # Segmentation oracle selects actual rail pixels; depth is rendered, not fixture distance.
            z=depth[v,u]
            camera=np.column_stack(((u-320)*z/K[0,0],-(v-180)*z/K[1,1],-z))
            world=camera @ R.T + origin
            centroid=np.median(world,axis=0)
            detections.append({'oracle_key':ladder['key'],'position':centroid.tolist(),
                'type':ladder['type'],'pixels':int(len(u)),
                'source':'MUJOCO_SEGMENTATION_ORACLE','synthetic':True})
        return detections

    def overview(self):
        self.renderer.update_scene(self.data,camera='overview')
        return self.renderer.render().copy()

    def close(self):
        if self.viewer:
            self.viewer.close()
        self.renderer.close()
