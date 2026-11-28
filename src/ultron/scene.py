"""MuJoCo construction scene; official B2 kinematics with primitive visuals."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ASSETS = Path(__file__).resolve().parent / "assets" / "b2"


def scenarios():
    return [
        dict(key="ladder_a", position=[3.,6.,0.], type="LEANING_SINGLE", due="2027-12-31", defect=None),
        dict(key="ladder_b", position=[6.,6.,0.], type="LEANING_SINGLE", due="2026-10-01", defect="rungs"),
        dict(key="ladder_c", position=[9.,6.,0.], type="A_FRAME_STEPLADDER", due="2027-12-31", defect=None),
    ]


def safety_assets():
    return [
        dict(key="ext_01", kind="extinguisher", position=[10.8, 8.4, 0.]),
        dict(key="fa_01", kind="firstaid", position=[1.2, 8.4, 0.]),
        dict(key="es_01", kind="escape_sign", position=[0.4, 1.6, 0.]),
        dict(key="person_ok", kind="person", position=[10.5, 5.2, 0.], helmet=True, vest=True),
        dict(key="person_nohelm", kind="person", position=[7.8, 4.8, 0.], helmet=False, vest=False),
    ]


def box(parent, name, pos, size, color):
    return ET.SubElement(parent,"geom",name=name,type="box",pos=" ".join(map(str,pos)),
                         size=" ".join(map(str,size)),rgba=color)


def make_scene(ladders,clear_clutter=False,safety_assets=None):
    root=ET.parse(ASSETS / "upstream.xml").getroot()
    root.set("model","ULTRON_B2_inspection_sim")
    root.find("compiler").attrib.pop("meshdir",None)
    root.remove(root.find("asset"))
    # Preserve manufacturer's kinematic tree and inertials; omit all mesh visuals.
    for parent in root.iter():
        for child in list(parent):
            if child.tag=="geom" and child.get("type")=="mesh":
                parent.remove(child)
    ET.SubElement(root.find("default/default"),"geom",type="box")
    for geom in root.findall(".//geom"):
        if geom.get("class")=="collision":
            geom.set("rgba","0.28 0.58 0.68 1")
            geom.set("group","1")
    ET.SubElement(root,"option",timestep="0.01",gravity="0 0 -9.81")
    visual=ET.SubElement(root,"visual")
    ET.SubElement(visual,"global",offwidth="960",offheight="540")
    ET.SubElement(visual,"headlight",ambient=".4 .4 .4",diffuse=".7 .7 .7")
    ET.SubElement(visual,"map",znear="0.01",zfar="30")
    assets=ET.SubElement(root,"asset")
    ET.SubElement(assets,"texture",name="sky",type="skybox",builtin="gradient",rgb1=".08 .12 .18",rgb2=".25 .32 .4",width="256",height="1536")
    ET.SubElement(assets,"texture",name="floor_tex",type="2d",builtin="checker",rgb1=".16 .19 .23",rgb2=".20 .23 .27",width="256",height="256")
    ET.SubElement(assets,"material",name="floor_mat",texture="floor_tex",texrepeat="12 9",reflectance=".05")
    world=root.find("worldbody")
    ET.SubElement(world,"light",pos="3 1 7",dir="0 0 -1",diffuse=".8 .8 .8")
    ET.SubElement(world,"light",pos="10 6 6",dir="0 0 -1",diffuse=".6 .6 .6")
    ET.SubElement(world,"geom",name="floor",type="plane",size="14 11 .1",material="floor_mat")
    for name,pos,size in [("wall_back",[6,8.8,1.5],[6,.1,1.5]),("wall_left",[0,4.5,1.5],[.1,4.5,1.5])]:
        box(world,name,pos,size,".36 .40 .46 1")
    if not clear_clutter:
        box(world,"crate",[4.6,3.0,.5],[.4,.55,.5],".65 .36 .13 1")
        box(world,"pallet",[10.9,3.4,.16],[.5,.8,.16],".55 .39 .23 1")
    for ladder in ladders:
        x,y,z=ladder['position']; key=ladder['key']
        body=ET.SubElement(world,"body",name=key,pos=f"{x} {y} {z}")
        # Front rails; each geom name is used by the explicit segmentation oracle.
        for side in (-1,1):
            box(body,f"{key}_rail_{side}",[side*.30,0,1.35],[.035,.055,1.35],".78 .82 .86 1")
            if ladder.get('defect')!='bottom_feet' or side!=1:
                box(body,f"{key}_foot_{side}",[side*.30,-.015,.055],[.065,.085,.055],".04 .045 .05 1")
        for n in range(8):
            if ladder.get('defect')=='rungs' and n==3: continue
            box(body,f"{key}_rung_{n}",[0,-.01,.28+n*.30],[.29,.04,.035],".7 .75 .79 1")
        # Sticker colors are scene presentation only; fixture metadata is the reader oracle.
        label_color=".85 .2 .13 1" if ladder['due']<'2026-11-27' else ".12 .65 .45 1"
        box(body,f"{key}_label",[-.3,-.06,1.15],[.06,.008,.14],label_color)
        if ladder['type']=="A_FRAME_STEPLADDER":
            for side in (-1,1):
                ET.SubElement(body,"geom",name=f"{key}_rear_rail_{side}",type="capsule",
                    fromto=f"{side*.3} .0 2.65 {side*.3} 1.05 .07",size=".035",rgba=".7 .75 .8 1")
                box(body,f"{key}_rear_foot_{side}",[side*.3,1.05,.055],[.065,.08,.055],".04 .045 .05 1")
            box(body,f"{key}_spreader",[.30,.46,1.25],[.025,.46,.025],".9 .62 .12 1")
            box(body,f"{key}_hinge",[0,0,2.65],[.34,.08,.06],".9 .62 .12 1")
        else:
            box(world,key+'_support_wall',[x,y+.28,1.5],[.65,.08,1.5],".34 .39 .45 1")
    for asset in safety_assets or []:
        x, y, z = asset['position']; key = asset['key']; kind = asset['kind']
        body = ET.SubElement(world, "body", name=key, pos=f"{x} {y} {z}")
        if kind == 'extinguisher':
            box(body, f"{key}_post", [0, .14, .6], [.05, .02, .6], ".45 .47 .5 1")
            ET.SubElement(body, "geom", name=f"{key}_tank", type="cylinder",
                          pos="0 0 .95", size=".085 .28", rgba=".82 .08 .06 1")
            box(body, f"{key}_tag", [0, -.09, 1.08], [.05, .004, .06], ".95 .95 .92 1")
        elif kind == 'firstaid':
            box(body, f"{key}_post", [0, .08, .75], [.04, .02, .75], ".45 .47 .5 1")
            box(body, f"{key}_box", [0, 0, 1.5], [.16, .05, .16], ".93 .95 .94 1")
            box(body, f"{key}_cross_v", [0, -.055, 1.5], [.03, .004, .11], ".08 .62 .25 1")
            box(body, f"{key}_cross_h", [0, -.055, 1.5], [.11, .004, .03], ".08 .62 .25 1")
        elif kind == 'escape_sign':
            box(body, f"{key}_post", [0, 0, 1.1], [.03, .03, 1.1], ".45 .47 .5 1")
            box(body, f"{key}_plate", [0, 0, 2.3], [.22, .012, .11], ".06 .55 .25 1")
        elif kind == 'person':
            # Vest must contrast with the arms to read as a vest: blue workwear arms,
            # hi-vis orange torso only when vest=True. Round primitives keep the figure
            # human-readable from any approach azimuth (box torsos look like slabs side-on).
            shirt = ".25 .35 .6 1"
            torso = ".95 .5 .05 1" if asset.get('vest') else shirt
            for side, tag in ((-1, 'leg_l'), (1, 'leg_r')):
                ET.SubElement(body, "geom", name=f"{key}_{tag}", type="capsule",
                              fromto=f"{side * .09} 0 .84 {side * .09} 0 .05", size=".06", rgba=".2 .22 .3 1")
            ET.SubElement(body, "geom", name=f"{key}_torso", type="cylinder",
                          pos="0 0 1.14", size=".155 .3", rgba=torso)
            for side, tag in ((-1, 'arm_l'), (1, 'arm_r')):
                ET.SubElement(body, "geom", name=f"{key}_{tag}", type="capsule",
                              fromto=f"{side * .21} 0 1.38 {side * .3} 0 .92", size=".045", rgba=shirt)
            ET.SubElement(body, "geom", name=f"{key}_head", type="sphere",
                          pos="0 0 1.56", size=".1", rgba=".87 .72 .6 1")
            if asset.get('helmet'):
                ET.SubElement(body, "geom", name=f"{key}_helmet", type="sphere",
                              pos="0 0 1.63", size=".105", rgba=".95 .8 .05 1")
    for ladder in ladders:
        body=world.find("body[@name='"+ladder['key']+"']")
        width=ladder.get('width_factor',1.);height=ladder.get('height_factor',1.)
        for geom in body.findall('geom'):
            if 'pos' in geom.attrib:
                pos=list(map(float,geom.get('pos').split()));pos[0]*=width;pos[2]*=height
                geom.set('pos',' '.join(map(str,pos)))
            if 'size' in geom.attrib and geom.get('type')=='box':
                size=list(map(float,geom.get('size').split()));size[0]*=width;size[2]*=height
                geom.set('size',' '.join(map(str,size)))
            if 'fromto' in geom.attrib:
                points=list(map(float,geom.get('fromto').split()))
                for i in (0,3): points[i]*=width
                for i in (2,5): points[i]*=height
                geom.set('fromto',' '.join(map(str,points)))
    base=world.find("body[@name='base_link']")
    # Camera looks along robot +X, slightly upwards. This extrinsic is a demo assumption.
    ET.SubElement(base,"camera",name="inspection",pos=".45 0 .13",xyaxes="0 -1 0 -.22 0 .9755",fovy="75")
    ET.SubElement(world,"camera",name="overview",pos="11 -4 8",xyaxes=".848 .53 0 -.342 .548 .762",fovy="55")
    return ET.tostring(root,encoding="unicode")
