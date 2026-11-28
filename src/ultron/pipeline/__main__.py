"""Command line for the site patrol pipeline.

    python -m ultron.pipeline patrol  --source relay:5600 --detector yolo --dashboard URL   everything, one process
    python -m ultron.pipeline calibrate --source bag:PATH,rate=1          choose detector prompts for this site
    python -m ultron.pipeline survey  --source ros2:/image,pose=/odom,cloud=/points --calibration cal.json
    python -m ultron.pipeline inspect --navigator dashboard --camera ros2:/image,pose=/odom
    python -m ultron.pipeline status

Both stages work on the same --data folder; stop and restart either at any time. With a map id
(--map-id, or --map with a saved map) the object map lives with the site and map instead, under
--sites, so the next patrol on the same map re-observes the objects found before.
"""
import argparse
import json
from pathlib import Path
import sys
import uuid

from . import calibrate
from .detect import ReplayDetector, VlmDetector, YoloDetector, load_prompts
from .inspection import Inspection
from .keyframes import KeyframeStore
from .navigate import ContractNavigator, DashboardNavigator, Exploration, Operator, Passive, free_space, http_json
from .objects import ObjectMap
from .gate import DashboardMap, Fence, Gate
from .online import OnlinePatrol, stdin_commands
from .survey import Survey
from .verify import Verifier
from ..protocol.__main__ import build_vlm, connect_dashboard
from ..protocol.lidar import Calibration
from ..protocol.sources import open_source
from ..protocol.store import Store
from ..protocol.vlm import VlmError


def build_detector(args, vlm):
    kind, _, option = args.detector.partition(":")
    if kind == "vlm":
        return VlmDetector(vlm)
    if kind == "yolo":
        return YoloDetector(option or "yoloe-11l-seg.pt", prompts=load_prompts(args.data), imgsz=args.imgsz,
                            guard=not args.no_glass_guard)
    if kind == "replay":
        with open(option) as handle:
            return ReplayDetector(json.load(handle))
    raise ValueError(f"Unknown detector {args.detector!r}")


def load_calibration(args):
    """--calibration FILE, or `bag:PATH` / `sim:PATH` to take the colour camera from a recording."""
    if not args.calibration:
        return None
    if args.calibration.startswith("bag:"):
        from ..protocol import bag
        return bag.calibration(args.calibration[4:])
    if args.calibration.startswith("sim:"):
        from ..protocol import sim
        return sim.calibration(args.calibration[4:])
    return Calibration.load(args.calibration)


def map_id(args):
    """The id of the map the robot's poses are in: --map-id, or that of the saved map in --map."""
    if args.map_id:
        return args.map_id
    if args.map:
        meta = json.loads((Path(args.map) / "map_meta.json").read_text())
        return f"{meta['map_id']}-v{meta['map_version']}"
    return None


def object_folder(args):
    """Where the object map and its keyframes live: one folder per site and map, so every patrol
    on that map adds to the same objects; without a map id, the run's own --data folder.

    Only poses in one map's frame can share objects. Lidar odometry starts a new frame on every
    run, so give a map id only when the robot localises in a saved map (or the runs are known to
    start from the same place)."""
    known = map_id(args)
    return Path(args.sites) / args.site / known if known else Path(args.data)


def build_survey(args, store, vlm):
    folder = object_folder(args)
    survey = Survey(build_detector(args, vlm), ObjectMap(folder), KeyframeStore(folder), store,
                    load_calibration(args), args.interval, args.min_sharpness,
                    verifier=Verifier(vlm) if vlm and not args.no_verify else None, map_id=map_id(args))
    survey.fence = load_fence(args)
    if survey.fence:
        survey.inside = survey.fence.contains  # objects beyond the fence are reflections or behind glass
        store.emit("fence", polygon=survey.fence.polygon)
    return survey


def build_gate(args, survey, navigator, saved_map=None):
    """Where goals may go: inside the fence, known free and reachable in the dashboard's map (or
    free in the saved map), clear of known objects. Also filters the focus guidance without a navigator."""
    moving = navigator is not None and getattr(navigator, "moves", False)
    return Gate(survey.fence, DashboardMap(args.dashboard) if args.dashboard and not saved_map else None,
                survey.objects, clearance=getattr(args, "clearance", 0.45), also=saved_map,
                require_map=moving and not getattr(args, "no_map_check", False))


def print_events(store, quiet=()):
    """Every event as a JSON line on stdout; kinds in `quiet` (the per-frame `look` while a person
    follows the focus events in the terminal) only go to events.jsonl."""
    store.subscribe(lambda event: event["kind"] in quiet or print(json.dumps(event, ensure_ascii=False), flush=True))


def command_survey(args):
    store, mission = Store(args.data), args.mission or "PATROL-" + uuid.uuid4().hex[:6]
    survey = build_survey(args, store, None if args.no_verify and args.detector != "vlm" else build_vlm(args))
    connect_dashboard(store, args, mission)  # the mission stays open for the inspection stage
    print_events(store)
    try:
        survey.run(open_source(args.source, args.pace))
    except KeyboardInterrupt:
        pass
    print(f"{len(survey.objects.all())} objects in {args.data}/objects.json", flush=True)


def build_navigator(args, mission):
    """(navigator, is_free) for --navigator. Only "contract" knows the map, so only it filters goals."""
    if args.navigator == "operator":
        return Operator(), None
    if args.navigator == "dashboard":
        if not args.dashboard:
            sys.exit("--navigator dashboard needs --dashboard URL")
        return DashboardNavigator(http_json(args.dashboard)), None
    if args.navigator == "contract":
        # Through ultron.autonomy to Nav2. Needs ROS 2 with Nav2 and localisation running on the
        # saved map in --map. Written against the contract; not yet run with Nav2.
        if not args.map:
            sys.exit("--navigator contract needs --map DIR (a map saved by ultron.autonomy)")
        import rclpy
        from ..autonomy.config import load_sim_slam_config
        from ..autonomy.nav2_adapter import Nav2Adapter
        rclpy.init()
        node = rclpy.create_node("ultron_pipeline_navigation")
        config = load_sim_slam_config()
        backend = Nav2Adapter(node, config, args.map)
        navigator = ContractNavigator(backend, config.navigation, mission, backend.map_id, backend.map_version,
                                      spin=lambda: rclpy.spin_once(node, timeout_sec=0.02))
        return navigator, free_space(backend.footprint, config.navigation.get("target_clearance_m", args.clearance))
    return Passive(), None


def command_inspect(args):
    store, mission, vlm = Store(args.data), args.mission or "PATROL-" + uuid.uuid4().hex[:6], build_vlm(args)
    survey = build_survey(args, store, vlm)
    navigator, is_free = build_navigator(args, mission)
    is_free = build_gate(args, survey, navigator, is_free)
    frames = iter(open_source(args.camera)) if args.camera else None
    publisher = connect_dashboard(store, args, mission)
    print_events(store)
    Inspection(survey.objects, survey.keyframes, vlm, store, survey, navigator,
               camera=(lambda: next(frames)) if frames else None, is_free=is_free, mission=mission).run()
    if publisher:
        publisher.end()
    command_status(args)


def command_patrol(args):
    """One process for a whole patrol: survey the stream, check what was found, inspect it, report.

    The survey runs until the stream ends, --survey-seconds have passed or Ctrl-C is pressed
    once; the inspection then uses the same stream for the views it asks the navigator for.
    """
    store, mission, vlm = Store(args.data), args.mission or "PATROL-" + uuid.uuid4().hex[:6], build_vlm(args)
    survey = build_survey(args, store, vlm)
    publisher = connect_dashboard(store, args, mission)
    print_events(store)
    frames, first = iter(open_source(args.source, args.pace)), None
    try:
        for frame in frames:
            first = frame.stamp if first is None else first
            survey.feed(frame)
            if args.survey_seconds and frame.stamp - first >= args.survey_seconds:
                break
    except KeyboardInterrupt:
        print("Survey stopped by Ctrl-C: inspecting what was found. Ctrl-C again aborts.", flush=True)
    store.emit("stage", name="inspection", objects=len(survey.objects.all()))
    navigator, is_free = build_navigator(args, mission)
    is_free = build_gate(args, survey, navigator, is_free)
    Inspection(survey.objects, survey.keyframes, vlm, store, survey, navigator,
               camera=(lambda: next(frames)) if navigator.moves else None, is_free=is_free, mission=mission).run()
    if publisher:
        publisher.end()
    command_status(args)


def load_fence(args):
    """The room's fence: --fence FILE, else the dashboard's (`/api/fence`), else none (warned)."""
    if args.fence:
        return Fence.load(args.fence)
    if args.dashboard:
        status, answer = http_json(args.dashboard, timeout=3)("GET", "/api/fence")
        if status == 200 and answer and answer.get("polygon"):
            return Fence.from_json(answer)
    print("WARNING: no fence: goals and objects are not kept inside the room (lidar sees through windows)",
          flush=True)
    return None


def realtime(frames):
    """Hold a recording back to the pace it was recorded at, so the vision model gets the time it
    would have on the robot (a replay otherwise runs as fast as frames can be processed)."""
    import time
    start = None
    for frame in frames:
        if start is None:
            start = (time.monotonic(), frame.stamp)
        ahead = (frame.stamp - start[1]) - (time.monotonic() - start[0])
        if ahead > 0:
            time.sleep(ahead)
        yield frame


def build_candidates(args):
    """--candidates lidar: upright blobs in the lidar cloud the camera should look at, and low long blobs
    (ladders lying on the floor) kept clear as obstacles. Needs frames that carry `cloud` in the map
    frame: sim:PATH,lidar=1, bag:PATH,lidar=1 and relay (when the robot side forwards clouds)."""
    if args.candidates == "lidar":
        from .discover import LidarCandidates
        return LidarCandidates(floor=args.floor)
    return None


def command_online(args):
    """One process, objects read as the robot goes: survey every frame; each object found is
    checked by the vision model, collected until the robot is done with it, then read on a worker
    thread while the survey goes on to the next one. Leftovers are inspected when the stream ends."""
    store, mission, vlm = Store(args.data), args.mission or "PATROL-" + uuid.uuid4().hex[:6], build_vlm(args)
    survey = build_survey(args, store, vlm)
    publisher = connect_dashboard(store, args, mission)
    print_events(store, quiet={"look"} if args.navigator == "operator" else ())
    # operator: a person drives by teleop, follows the `focus` events and types n when done with one
    navigator, saved_map = (None, None) if args.navigator == "operator" else build_navigator(args, mission)
    dashboard = http_json(args.dashboard, timeout=2) if args.dashboard else None
    gate = build_gate(args, survey, navigator, saved_map)
    exploration = Exploration(dashboard) if dashboard and args.navigator == "dashboard" else None
    obstacles = (lambda cells: dashboard("PUT", "/api/obstacles", {"frame_id": "map", "cell": 0.1, "cells": cells})) \
        if dashboard else None
    inspection = Inspection(survey.objects, survey.keyframes, vlm, store, survey, Passive(), is_free=gate,
                            mission=mission)
    online = OnlinePatrol(survey, inspection, store, navigator=navigator, is_free=gate,
                          leave_seconds=args.leave_seconds, leave_distance=args.leave_distance,
                          budget_seconds=args.budget_seconds, max_reads=args.max_reads,
                          stall_seconds=args.stall_seconds, exploration=exploration, pause_cap=args.pause_cap,
                          obstacles=obstacles, commands=stdin_commands(), candidates=build_candidates(args))
    frames = open_source(args.source, args.pace)
    online.run(realtime(frames) if args.realtime else frames)
    if publisher:
        publisher.end()
    command_status(args)


def command_fence(args):
    """Set the room's fence: corners typed (--polygon "x,y x,y ...") or taken from where the robot
    stands (--from-robot: drive it to each corner, press Enter; empty line + d to finish). Saved to
    --out and, with --dashboard, to the dashboard (PUT /api/fence), where the bridge and every
    pipeline run pick it up. Stand inside the glass: the gate keeps goals a clearance inside it."""
    points = []
    if args.polygon:
        points = [[float(v) for v in pair.split(",")] for pair in args.polygon.split()]
    elif args.from_robot:
        if not args.dashboard:
            sys.exit("--from-robot needs --dashboard URL (the robot's pose comes from there)")
        call = http_json(args.dashboard, timeout=3)
        while True:
            reply = input(f"{len(points)} corners. Drive to the next corner and press Enter (d + Enter: done): ")
            if reply.strip().lower() == "d":
                break
            status, live = call("GET", "/api/live")
            pose = (live or {}).get("pose") if status == 200 else None
            if not pose:
                print("No robot pose on the dashboard (is a mission running and the bridge connected?)")
                continue
            points.append([round(pose["x"], 2), round(pose["y"], 2)])
            print(f"corner {len(points)}: {points[-1]}")
    fence = Fence(points)  # raises with fewer than 3 points
    data = {"frame_id": "map", "polygon": [list(p) for p in fence.polygon]}
    if args.out:
        Path(args.out).write_text(json.dumps(data, indent=1))
        print(f"saved {args.out}")
    if args.dashboard:
        status, answer = http_json(args.dashboard, timeout=5)("PUT", "/api/fence", data)
        print(f"dashboard: HTTP {status} {answer if status not in (200, 201, 204) else 'fence set'}")


def command_calibrate(args):
    """Pick prompt and threshold per kind from sample frames, judged by the vision model."""
    # No extra classes here: their boxes would count for every ladder prompt on trial.
    detector = YoloDetector(args.detector.partition(":")[2] or "yoloe-11l-seg.pt", track=False, also=(),
                            imgsz=args.imgsz)
    frames = list(open_source(args.source))
    report = calibrate.calibrate(frames, detector, Verifier(build_vlm(args)), per_prompt=args.per_prompt)
    print(f"{len(frames)} frames; saved {calibrate.save(report, args.data)}")


def command_export(args):
    """Bake this workspace's prompts into the detector and export it (TensorRT FP16 by default)."""
    from .detect import export
    weights = args.detector.partition(":")[2] or "yoloe-11l-seg.pt"
    print(f"saved {export(weights, load_prompts(args.data), args.format, not args.fp32)}; "
          f"use it as --detector yolo:<that file>")


def command_status(args):
    for site_object in ObjectMap(object_folder(args)).all():
        result = site_object.result
        print(f"{site_object.id:8} {site_object.kind:18} {site_object.status:11} "
              f"position={site_object.position} sightings={site_object.sightings} "
              f"{result.get('status', '')} open={result.get('open', '')} missing_views={len(result.get('missing_views', []))} "
              f"gone_if_missed_from={gone_check(site_object)}")


def gone_check(site_object):
    """Sides (map bearing from the object, 45 degree steps: 0 = east, 2 = north) from which a later
    patrol that misses this object counts it as gone; "none" says why not. Patrol 1 must have seen it
    within 1-4.5 m for there to be any."""
    from .survey import reliable_sides
    sides = reliable_sides(site_object)
    if sides:
        return ",".join(f"{side}({rate:.0%})" for side, rate in sorted(sides.items()))
    return f"none({site_object.detected}/{site_object.views}_clear_views)"


def main(argv=None):
    parser = argparse.ArgumentParser(prog="ultron-pipeline", description=__doc__.splitlines()[0])
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--data", default="runs/pipeline", help="workspace folder shared by all stages")
    common.add_argument("--dashboard", help="URL of the team dashboard")
    common.add_argument("--site", default="demo-site")
    common.add_argument("--sites", default="runs/sites", help="object maps kept per site and map (with a map id)")
    common.add_argument("--map-id", help="id of the map the poses are in; runs with the same id share their objects")
    common.add_argument("--map", help="saved map folder (ultron.autonomy): its id keys the object map; "
                                      "--navigator contract navigates in it")
    common.add_argument("--mission", help="mission id; use the same one for survey and inspect")
    common.add_argument("--detector", default="vlm", help="vlm | yolo[:WEIGHTS] | replay:FILE")
    common.add_argument("--calibration", help="camera/lidar calibration JSON, or bag:PATH")
    common.add_argument("--interval", type=float, default=1.0, help="seconds between processed frames")
    common.add_argument("--min-sharpness", type=float, default=0.0)
    common.add_argument("--model")
    common.add_argument("--triage-model")
    common.add_argument("--base-url")
    common.add_argument("--language", default="German")
    common.add_argument("--replay", help="recorded model answers instead of a model")
    common.add_argument("--no-cache", action="store_true")
    common.add_argument("--fence", help="room polygon JSON {frame_id, polygon: [[x, y], ...]} in the map frame "
                                        "(default: the dashboard's): no goal or object outside it")
    common.add_argument("--imgsz", type=int, help="detector input size (default $ULTRON_DETECTOR_IMGSZ or 640); "
                                                  "1280 finds a lying ladder but split objects on the hall bag")
    common.add_argument("--no-glass-guard", action="store_true",
                        help="keep detections with no depth or seen over a sill (lift.doubt drops them)")
    common.add_argument("--no-verify", action="store_true", help="survey without the vision model's second opinion")
    commands = parser.add_subparsers(dest="command", required=True)

    survey = commands.add_parser("survey", parents=[common], help="stage 1: build the object map")
    survey.add_argument("--source", required=True, help="relay[:PORT] | bag:PATH | sim:PATH[,rate=..][,depth=noisy][,error=DEG/DX/DY] | folder:PATH | cv:TARGET | ros2:/topic[,pose=..][,cloud=..] | unitree")
    survey.add_argument("--pace", type=float, default=0.0)
    survey.set_defaults(action=command_survey)

    moving = argparse.ArgumentParser(add_help=False)
    moving.add_argument("--navigator", choices=["passive", "operator", "dashboard", "contract"], default="passive",
                        help="how missing views are reached: not at all, a person drives, goals through "
                             "the dashboard bridge, or the ultron.autonomy navigation contract")
    moving.add_argument("--clearance", type=float, default=0.45, help="free radius a goal needs, metres")
    moving.add_argument("--no-map-check", action="store_true", help="send goals even before the dashboard has a map")

    inspect = commands.add_parser("inspect", parents=[common, moving], help="stage 2: fill each object's form")
    inspect.add_argument("--camera", help="live frame source for views taken during inspection")
    inspect.set_defaults(action=command_inspect)

    patrol = commands.add_parser("patrol", parents=[common, moving], help="survey, check and inspect in one process")
    patrol.add_argument("--source", required=True, help="relay[:PORT] | bag:PATH | ros2:... | folder:PATH | cv:TARGET")
    patrol.add_argument("--survey-seconds", type=float, default=0.0, help="end the survey after this long (0: when the stream ends or on Ctrl-C)")
    patrol.add_argument("--pace", type=float, default=0.0)
    patrol.set_defaults(action=command_patrol)

    live = commands.add_parser("online", parents=[common, moving],
                               help="survey and read each object while the robot walks on to the next")
    live.add_argument("--source", required=True, help="relay[:PORT] | bag:PATH | sim:PATH | ros2:... | folder:PATH")
    live.add_argument("--pace", type=float, default=0.0)
    live.add_argument("--leave-seconds", type=float, default=6.0, help="an object unseen this long ...")
    live.add_argument("--leave-distance", type=float, default=3.5, help="... and this far away (m) is read")
    live.add_argument("--budget-seconds", type=float, default=120.0, help="read an object after this long whatever its views")
    live.add_argument("--max-reads", type=int, default=1,
                      help="reads per object: 2 reads it again when the robot comes back with views it lacked "
                           "(the dashboard keeps the first ladder verdict of a mission)")
    live.add_argument("--stall-seconds", type=float, default=20.0, help="an object whose views stop growing this long is read")
    live.add_argument("--pause-cap", type=float, default=90.0, help="longest pause of the explorer for one batch of objects (s)")
    live.add_argument("--realtime", action="store_true", help="replay a recording at the pace it was recorded")
    live.add_argument("--candidates", choices=["none", "lidar"], default="none",
                      help="lidar: also upright and lying blobs in the lidar cloud (sim/bag with lidar=1, relay)")
    live.add_argument("--floor", type=float, help="floor height in the map frame for lidar candidates (default: estimated)")
    live.set_defaults(action=command_online)

    tune = commands.add_parser("calibrate", parents=[common], help="choose detector prompts for this site")
    tune.add_argument("--source", required=True, help="sample frames, for example bag:PATH,rate=1")
    tune.add_argument("--per-prompt", type=int, default=6, help="boxes per candidate shown to the vision model")
    tune.set_defaults(action=command_calibrate)

    commands.add_parser("status", parents=[common], help="list the object map").set_defaults(action=command_status)

    room = commands.add_parser("fence", help="set the room polygon no goal or object may lie outside")
    room.add_argument("--polygon", help='corners in the map frame, "x,y x,y x,y ..."')
    room.add_argument("--from-robot", action="store_true", help="take each corner from the robot's pose on the dashboard")
    room.add_argument("--dashboard", help="URL of the dashboard: the fence is stored there")
    room.add_argument("--out", help="also save it as JSON (for --fence FILE)")
    room.set_defaults(action=command_fence)

    bake = commands.add_parser("export", parents=[common],
                               help="bake the prompts into the detector for TensorRT (build it on the robot)")
    bake.add_argument("--format", choices=["engine", "onnx"], default="engine")
    bake.add_argument("--fp32", action="store_true", help="full precision instead of FP16")
    bake.set_defaults(action=command_export)

    args = parser.parse_args(argv)
    try:
        args.action(args)
    except (VlmError, ValueError, OSError) as error:
        sys.exit(f"Error: {error}")


if __name__ == "__main__":
    main()
