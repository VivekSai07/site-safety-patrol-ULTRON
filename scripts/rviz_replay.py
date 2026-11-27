#!/usr/bin/env python3
"""ROS 2 Humble RViz replay from ULTRON portable recordings. No motion publishers."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


def primitive(geom, stamp, namespace):
    marker=Marker()
    marker.header.frame_id='map'
    marker.header.stamp=stamp
    marker.ns=namespace
    marker.id=geom['id']
    marker.type=Marker.CUBE
    marker.action=Marker.ADD
    marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=geom['position']
    q=geom['quaternion_xyzw']
    marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w=q
    size=geom['size']
    if geom['kind']==2:
        marker.type=Marker.SPHERE
        marker.scale.x=marker.scale.y=marker.scale.z=2*size[0]
    elif geom['kind'] in (3,5):
        marker.type=Marker.CYLINDER
        marker.scale.x=marker.scale.y=2*size[0]
        marker.scale.z=2*(size[1]+(size[0] if geom['kind']==3 else 0))
    else:
        marker.scale.x,marker.scale.y,marker.scale.z=[2*max(.005,v) for v in size]
    marker.color.r,marker.color.g,marker.color.b,marker.color.a=geom['color']
    return marker


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--rate',type=float,default=8,help='Replay frames per wall-clock second')
    parser.add_argument('--once',action='store_true')
    args=parser.parse_args()
    if args.rate<=0:
        parser.error('--rate must be positive')
    frames=[json.loads(line) for line in (args.run/'telemetry.jsonl').read_text().splitlines()]
    protocol=json.loads((args.run/'protocol.json').read_text())
    rclpy.init()
    node=Node('ultron_sim_replay')
    transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    scene_pub=node.create_publisher(MarkerArray,'/ultron/sim/scene',transient)
    robot_pub=node.create_publisher(MarkerArray,'/ultron/sim/b2',10)
    asset_pub=node.create_publisher(MarkerArray,'/ultron/sim/assets',transient)
    path_pub=node.create_publisher(NavPath,'/ultron/sim/path',10)
    image_pub=node.create_publisher(Image,'/ultron/sim/rgb',10)
    tf=TransformBroadcaster(node)
    from PIL import Image as PILImage
    try:
        while rclpy.ok():
            path=NavPath();path.header.frame_id='map'
            for frame in frames:
                stamp=node.get_clock().now().to_msg()
                if 'scene_geoms' in frame:
                    clear=Marker();clear.action=Marker.DELETEALL
                    scene_pub.publish(MarkerArray(markers=[clear]+[
                        primitive(g,stamp,'scene') for g in frame['scene_geoms'] if g['kind']!=0]))
                    path.poses=[]
                robot_pub.publish(MarkerArray(markers=[primitive(g,stamp,'b2') for g in frame['robot_geoms']]))
                x,y,yaw=frame['pose']
                transform=TransformStamped()
                transform.header.frame_id='map';transform.header.stamp=stamp
                transform.child_frame_id='ultron_sim_base'
                transform.transform.translation.x=x;transform.transform.translation.y=y;transform.transform.translation.z=.65
                transform.transform.rotation.z=math.sin(yaw/2);transform.transform.rotation.w=math.cos(yaw/2)
                tf.sendTransform(transform)
                pose=PoseStamped();pose.header=transform.header
                pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1.
                path.poses.append(pose);path.header.stamp=stamp;path_pub.publish(path)
                image=PILImage.open(args.run/frame['rgb']).convert('RGB')
                message=Image();message.header.stamp=stamp;message.header.frame_id='ultron_sim_camera'
                message.width=image.width;message.height=image.height;message.encoding='rgb8';message.step=image.width*3
                message.data=image.tobytes();image_pub.publish(message)
                # Mission snapshots: do not display future patrol verdicts during earlier replay.
                mission=next(m for m in protocol['missions'] if m['mission_id']==frame['mission_id'])
                markers=[]
                for idx,record in enumerate(mission['records']):
                    if record['views'] and record['views'][-1]['time']>frame['time']:
                        continue
                    marker=Marker();marker.header.frame_id='map';marker.header.stamp=stamp
                    marker.ns='assets';marker.id=idx;marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD
                    marker.pose.position.x,marker.pose.position.y,_=record['position'];marker.pose.position.z=3.0
                    marker.pose.orientation.w=1.;marker.scale.z=.22
                    marker.color.r,marker.color.g,marker.color.b,marker.color.a={'FLAGGED':(1.,.3,.3,1.),'MANUAL_REVIEW':(1.,.7,.2,1.),'AUTOMATED_PASS':(.2,.9,.6,1.)}[record['status']]
                    marker.text=record['asset_id']+' '+record['status'];markers.append(marker)
                clear=Marker();clear.action=Marker.DELETEALL
                asset_pub.publish(MarkerArray(markers=[clear]+markers))
                rclpy.spin_once(node,timeout_sec=0)
                time.sleep(1/args.rate)
            if args.once:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
