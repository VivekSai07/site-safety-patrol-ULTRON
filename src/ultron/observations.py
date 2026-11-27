"""Depth projection and persistent model detection tracks without fixture IDs."""
import math
import numpy as np


def project_box(box,depth,frame):
    height,width=depth.shape
    if len(box)!=4 or not all(math.isfinite(v) for v in box):
        raise ValueError('Invalid model box')
    x0,y0,x1,y1=box
    x0,y0=max(0,int(x0)),max(0,int(y0))
    x1,y1=min(width,int(math.ceil(x1))),min(height,int(math.ceil(y1)))
    if x1<=x0 or y1<=y0:
        raise ValueError('Empty box')
    stripe=max(1,(x1-x0)//6)
    mask=np.zeros(depth.shape,dtype=bool)
    mask[y0:y1,x0:x0+stripe]=True
    mask[y0:y1,x1-stripe:x1]=True
    mask &= np.isfinite(depth)&(depth>.2)&(depth<15.)
    v,u=np.nonzero(mask)
    if len(u)<10:
        raise ValueError('Insufficient valid depth in rail-strip heuristic')
    values=depth[v,u]
    # Prefer foreground depth rather than the wall visible through rung gaps.
    selected=values<=np.percentile(values,35)
    v,u,z=v[selected],u[selected],values[selected]
    K=np.asarray(frame['intrinsics']);R=np.asarray(frame['camera_rotation'])
    camera=np.column_stack(((u-K[0,2])*z/K[0,0],-(v-K[1,2])*z/K[1,1],-z))
    return np.median(camera@R.T+np.asarray(frame['camera_position']),axis=0).tolist()


class DetectionTracks:
    def __init__(self,radius=.65):
        self.tracks=[];self.radius=radius

    def update(self,detections):
        results=[];assigned=set()
        for detection in detections:
            neighbors=[t for t in self.tracks if t['track_id'] not in assigned and
                       math.dist(t['position'][:2],detection['position'][:2])<=self.radius]
            if len(neighbors)==1:
                track=neighbors[0]
                track['confirmation_frames']+=1
            elif neighbors:
                # Ambiguous association remains unconfirmed; never merge identities blindly.
                continue
            else:
                track=dict(detection,track_id=f'MODEL-{len(self.tracks)+1:03}',confirmation_frames=1)
                self.tracks.append(track)
            assigned.add(track['track_id'])
            results.append(dict(track,box_xyxy=detection.get('box_xyxy')))
        return results
