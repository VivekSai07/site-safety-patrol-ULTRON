"""Patrol -> discover -> approach -> inspect -> bounded retry -> resume."""
from collections import Counter
from datetime import date
import hashlib
import json
import math
import uuid
from pathlib import Path
from PIL import Image
from .inspection import REQUIRED, fuse, verdict
from .memory import Registry
from .mujoco_sim import Simulation
from .planning import Grid
from .scene import scenarios


def run_demo(output, database=None, patrols=2, variant='normal', recovery=True, viewer=False, motion='smooth'):
    output=Path(output)
    output.mkdir(parents=True,exist_ok=False)
    (output/'frames').mkdir()
    run_id=uuid.uuid4().hex
    db=Registry(database or output/'memory.sqlite')
    all_frames=[]
    mission_results=[]
    events=[]
    clock=0.
    summary={}
    try:
        for patrol in range(patrols):
            ladders=scenarios()
            if patrol>0:
                ladders[0]['defect']='bottom_feet'
                if variant=='missing':
                    ladders=ladders[:-1]
            sim=Simulation(ladders,output,viewer)
            sim.frames=len(all_frames)
            sim.t=clock
            obstacles=[(4.2,2.45,5.0,3.55),(10.4,2.6,11.4,4.2)]
            obstacles += [(a['position'][0]-.4,5.8,a['position'][0]+.4,
                           7.15 if a['type']=='A_FRAME_STEPLADDER' else 6.4) for a in ladders]
            if variant=='blocked':
                obstacles.append((7.7,4.15,10.3,5.15))
            if variant=='blocked-closeup':
                obstacles.append((1.4,4.4,4.6,4.5))
            grid=Grid((.2,.2,11.8,8.6),obstacles,radius=.65)
            mid=db.start_mission('ultron_mock_site_v1')
            db.remember_objects(mid,'ultron_mock_site_v1',sim.geometry(False))
            inspected=set()
            seen=Counter()
            records=[]
            skipped=0
            reached=0
            route=[(1.5,1.5),(3.,3.2),(6.,3.2),(9.,3.2),(8.,2.),(1.5,1.5)]
            interventions=1 if variant=='intervention' and patrol==0 else 0
            mode='DEGRADED_TELEOP_ASSISTED' if interventions else 'AUTONOMOUS_SIMULATION'

            def emit(kind, **payload):
                event={'time':sim.t,'mission_id':mid,'type':kind,'synthetic':True,**payload}
                events.append(event)
                with (output/'events.jsonl').open('a') as handle:
                    handle.write(json.dumps(event)+'\n')
                print(f"[{mid} {sim.t:6.1f}s] {kind}: {payload.get('asset_id',payload.get('status',''))}",flush=True)

            def capture(state):
                frame,rgb,depth,seg=sim.capture()
                frame.update(mission_id=mid,state=state)
                all_frames.append(frame)
                with (output/'telemetry.jsonl').open('a') as handle:
                    handle.write(json.dumps(frame)+'\n')
                return frame,rgb,depth,seg

            def move(goal, state, yaw=math.pi/2):
                path=grid.path(sim.pose[:2],goal)
                if not path:
                    emit('POSE_REJECTED',goal=list(goal),status='BLOCKED_OR_OUTSIDE_ZONE')
                    return False
                if motion=='legacy':
                    for idx,p in enumerate(path):
                        sim.t+=.25
                        sim.place([*p,yaw])
                        if idx%3==0 or idx==len(path)-1:
                            capture(state)
                else:
                    from .motion import motion_samples
                    from dataclasses import asdict
                    with (output/'motion_commands.jsonl').open('a') as commands:
                        for idx,(pose,command) in enumerate(motion_samples(sim.pose,path,yaw)):
                            command.validate()
                            if not grid.free(pose):
                                emit('MOTION_HALTED',status='FOOTPRINT_INVALID')
                                return False
                            sim.t+=command.dt
                            sim.motion_speed=abs(command.vx)
                            sim.place(pose)
                            commands.write(json.dumps(dict(asdict(command),time=sim.t,mission_id=mid,
                                backend='KINEMATIC_SDK_COMMAND_SIMULATION'))+'\n')
                            if idx%12==0 or command.operation=='StopMove':
                                capture(state)
                return True

            emit('MISSION_START',mission_mode=mode,operator_interventions=interventions)
            if interventions:
                emit('OPERATOR_INTERVENTION',reason='INJECTED_TEST',motion_halted=True)
            try:
                for waypoint in route:
                    emit('PATROL_WAYPOINT',goal=list(waypoint))
                    if not move(waypoint,'PATROL'):
                        skipped+=1
                        continue
                    reached+=1
                    confirmed={}
                    for burst in range(3):
                        sim.t+=.2
                        frame,rgb,depth,seg=capture('DISCOVER')
                        for detection in sim.oracle_detections(frame,depth,seg):
                            key=detection['oracle_key']
                            seen[key]+=1
                            if seen[key]>=3 and key not in inspected:
                                confirmed[key]=detection
                    from .supervisor import inspection_queue
                    for key,detection,selection in inspection_queue(confirmed,lambda:sim.pose[:2],grid):
                        if key in inspected:
                            continue
                        inspected.add(key)
                        fixture=next(a for a in ladders if a['key']==key)
                        asset,reid=db.associate(detection['position'],detection['type'],'ultron_mock_site_v1')
                        emit('LADDER_CONFIRMED',asset_id=asset,association=reid,observation=detection)
                        emit('SUPERVISOR_TARGET_SELECTED',asset_id=asset,decision=selection)
                        x,y,_=detection['position']
                        views=[]
                        checks={}
                        label='UNKNOWN'
                        retry_count=0
                        stop='TIER1_STATIC_BASELINE'
                        positions=[('OVERVIEW',(x,y-2.8)),('CLOSEUP',(x,y-1.6))]
                        if selection['action']!='INSPECT':
                            positions=[]
                            stop='NEXT_POSE_UNSAFE_BLOCKED'
                            emit('INSPECTION_UNREACHABLE',asset_id=asset,reason=selection['reason'])
                        for name,goal in positions:
                            if not move(goal,'APPROACH_'+name):
                                stop='NEXT_POSE_UNSAFE_BLOCKED'
                                break
                            sim.t+=.8
                            frame,rgb,depth,seg=capture('INSPECT_'+name)
                            views.append(frame)
                            # Findings are scripted sensor-oracle outputs, not visual inference.
                            new={c:'PASS' for c in REQUIRED[fixture['type']]}
                            if name=='OVERVIEW':
                                for component in ('bottom_feet','front_feet'):
                                    if component in new:
                                        new[component]='UNKNOWN'
                            if fixture['type']=='A_FRAME_STEPLADDER':
                                new.update(rear_feet='UNKNOWN',spreader='UNKNOWN')
                            if fixture.get('defect') and (name=='CLOSEUP' or fixture['defect']=='rungs'):
                                new[fixture['defect']]='FAIL'
                            checks=fuse(checks,new)
                            label='OVERDUE' if date.fromisoformat(fixture['due'])<date(2026,11,27) else 'VALID'
                            result=verdict(checks,REQUIRED[fixture['type']],label)
                            emit('EVIDENCE_CAPTURED',asset_id=asset,view=name,checks=checks.copy(),label=label)
                            if result['status']=='FLAGGED':
                                stop='HARD_FAIL_EARLY_EXIT'
                                break
                        result=verdict(checks,REQUIRED[fixture['type']],label)
                        if result['status']=='MANUAL_REVIEW' and recovery and views:
                            retry_count=1
                            emit('SECOND_LOOK_REQUESTED',asset_id=asset,missing=result['missing'])
                            side_goal=(x-1.4,y-1.65)
                            if move(side_goal,'SECOND_LOOK',yaw=math.atan2(y-side_goal[1],x-side_goal[0])):
                                sim.t+=.8
                                frame,rgb,depth,seg=capture('INSPECT_SIDE')
                                views.append(frame)
                                # Bounded scripted recovery for the rear-foot/spreader evidence gap.
                                recovered={c:'PASS' for c in REQUIRED[fixture['type']]}
                                if fixture.get('defect'):
                                    recovered[fixture['defect']]='FAIL'
                                checks=fuse(checks,recovered)
                                result=verdict(checks,REQUIRED[fixture['type']],label)
                                stop='ALL_REQUIRED_COMPLETE'
                            else:
                                stop='NEXT_POSE_UNSAFE_BLOCKED'
                        if reid=='REID_AMBIGUOUS' and result['status']=='AUTOMATED_PASS':
                            result.update(status='MANUAL_REVIEW',reason='Ambiguous spatial association')
                        payload={**result,'asset_id':asset,'mission_id':mid,'position':detection['position'],
                            'ladder_type':fixture['type'],'checks':checks,'label_status':label,
                            'required_components':REQUIRED[fixture['type']],
                            'label_reading':{'raw_text':None,'due_date':fixture['due'] if views else None,'status':label,'source':'SCENARIO_FIXTURE'},
                            'views':views,'retry_count':retry_count,'stop_reason':stop,
                            'source':'SIMULATION_ORACLE','oracle_key':key,'synthetic':True,
                            'run_id':run_id,'evidence_root':str(output.resolve())}
                        db.append(mid,asset,payload)
                        records.append(payload)
                        emit('INSPECTION_COMMITTED',asset_id=asset,status=payload['status'],retry_count=retry_count,reason=payload['reason'],
                             evidence=[view['rgb'] for view in views],checks=checks)
                        emit('ROUTE_RESUME',asset_id=asset)
                missing=db.missing(mid)
                from .supervisor import decide
                decision=decide([a for a in db.assets() if a['map_version']=='ultron_mock_site_v1'],
                    {r['asset_id'] for r in records},sim.pose[:2],grid,survey_remaining=False,
                    coverage_verified=False,missing_expected=missing)
                emit('SUPERVISOR_DECISION',**decision)
                returned_to_base=False
                if decision['action'] in ('RETURN_BASE','RETURN_BASE_WITH_GAPS'):
                    returned_to_base=move(decision['goal'],'RETURN_BASE')
                    emit('BASE_REACHED' if returned_to_base else 'BASE_UNREACHABLE',
                         status='KNOWN_ASSETS_HANDLED_COVERAGE_UNVERIFIED' if returned_to_base else 'AWAIT_OPERATOR')
                mission_status='COVERAGE_INCOMPLETE' if missing or skipped or len(inspected)<len(ladders) else 'MISSION_COMPLETE'
                summary={'mission_id':mid,'status':mission_status,'mission_mode':mode,
                    'planned_waypoints':len(route),'reached_waypoints':reached,'skipped_waypoints':skipped,
                    'zone_bounds':list(grid.bounds),'keepout_rectangles':grid.obstacles,
                    'robot_inflation_radius_m':grid.radius,'grid_resolution_m':grid.resolution,
                    'waypoint_coverage_pct':100*reached/len(route),'unscanned_area_pct':None,
                    'coverage_note':'Waypoint completion is NOT measured area coverage',
                    'expected_ground_truth':len(ladders),'found':len(inspected),
                    'ladder_recall_simulation':len(inspected)/len(ladders),'missing_expected':missing,
                    'operator_interventions':interventions,'duration_sim_s':round(sim.t-clock,2),
                    'records':records,'synthetic':True,'supervisor_decision':decision,'returned_to_base':returned_to_base,
                    'recovery_attempts':sum(r['retry_count'] for r in records),
                    'recovery_resolved':sum(r['retry_count']==1 and r['status']!='MANUAL_REVIEW' for r in records),
                    'hard_fail_cases_oracle':sum(bool(a.get('defect')) or a['due']<'2026-11-27' for a in ladders),
                    'false_scoped_passes_oracle':sum(r['status']=='AUTOMATED_PASS' and
                        any(a['key']==r['oracle_key'] and (a.get('defect') or a['due']<'2026-11-27') for a in ladders)
                        for r in records)}
                db.finish(mid,summary)
                mission_results.append(summary)
                emit('MISSION_FINISHED',status=mission_status,missing_expected=missing)
                Image.fromarray(sim.overview()).save(output/f'{mid}-overview.png')
                clock=sim.t+1.
            finally:
                sim.close()
        from .archive import export_history
        assets=db.query()
        export_history(assets,output)
        for mission in mission_results:
            for record in mission['records']:
                record['evidence_root']='.'
        exported={'team':'ULTRON','run_id':run_id,'variant':variant,'recovery_enabled':recovery,'motion_backend':motion,'synthetic':True,'missions':mission_results,
                  'assets':assets,'world_objects':db.objects(),'limitations':['Kinematic base; no validated locomotion',
                  'Segmentation and inspection oracles; no learned perception',
                  'No hardware connection; no formal inspection certification']}
        (output/'protocol.json').write_text(json.dumps(exported,indent=2))
        write_report(exported,output)
        from .preview import make_preview
        make_preview(output)
        manifest={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in output.rglob('*') if p.is_file() and p.suffix in ('.jpg','.png','.npy','.xml','.jsonl','.gif','.json')}
        (output/'manifest.json').write_text(json.dumps(manifest,indent=2))
        return exported
    finally:
        db.close()


def write_report(data,output):
    from html import escape
    cards=[]
    for mission in data['missions']:
        cards.append(f"<h2>{escape(mission['mission_id'])} · {escape(mission['status'])}</h2>"
                     f"<p>{mission['found']}/{mission['expected_ground_truth']} simulated ladders · "
                     f"{mission['operator_interventions']} interventions · missing expected: {escape(str(mission['missing_expected']))}</p>")
        for record in mission['records']:
            images=''.join(f'<a href="{v["rgb"]}"><img src="{v["rgb"]}" alt="Synthetic camera view"></a>' for v in record['views'])
            cards.append(f'<article class="{record["status"]}"><h3>{escape(record["asset_id"])} · {escape(record["status"])}</h3>'
                f'<p>{escape(record["reason"])}</p><p>{escape(record["ladder_type"])} · Retry: {record["retry_count"]} · {escape(record["stop_reason"])}</p>'
                f'{images}<details><summary>Checks</summary><pre>{escape(json.dumps(record["checks"],indent=2))}</pre></details></article>')
    html='''<!doctype html><html lang="en"><meta charset="utf-8"><title>ULTRON Patrol Evidence</title>
<style>body{font:16px system-ui;background:#0d1421;color:#e6edf5;margin:40px auto;max-width:1200px;padding:24px}h1{font-size:42px;letter-spacing:.12em}article{background:#172439;padding:22px;margin:18px 0;border-left:5px solid #d4a34b;border-radius:8px}.FLAGGED{border-color:#f17474}.AUTOMATED_PASS{border-color:#5ec8ab}img{width:31%;margin:1%;border-radius:6px}a{color:#79b8ff}summary{cursor:pointer}.notice{padding:20px;background:#27354c}</style>
<h1>ULTRON</h1><p>Site Safety Patrol · B2 simulation · persistent evidence memory</p>
<p class="notice">SIMULATED RGB/depth. Kinematic robot motion. Oracle detections and findings. Automated visual screening only; tactile checks and qualified human sign-off remain pending.</p>
<p><a href="preview.gif">Animated simulation preview</a> · <a href="protocol.json">Inspection JSON</a> · <a href="manifest.json">Evidence hashes</a></p>
'''
    (output/'report.html').write_text(html+''.join(cards)+'</html>')
