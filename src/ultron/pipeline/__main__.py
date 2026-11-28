"""Command line for the site patrol pipeline.

    python -m ultron.pipeline patrol  --source relay:5600 --detector yolo --dashboard URL   everything, one process
    python -m ultron.pipeline calibrate --source bag:PATH,rate=1          choose detector prompts for this site
    python -m ultron.pipeline survey  --source ros2:/image,pose=/odom,cloud=/points --calibration cal.json
    python -m ultron.pipeline inspect --navigator dashboard --camera ros2:/image,pose=/odom
    python -m ultron.pipeline status

Both stages work on the same --data folder; stop and restart either at any time.
"""
import argparse
import json
import sys
import uuid

from . import calibrate
from .detect import ReplayDetector, VlmDetector, YoloDetector, load_prompts
from .inspection import Inspection
from .keyframes import KeyframeStore
from .navigate import ContractNavigator, DashboardNavigator, Operator, Passive, free_space, http_json
from .objects import ObjectMap
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
        return YoloDetector(option or "yoloe-11l-seg.pt", prompts=load_prompts(args.data))
    if kind == "replay":
        with open(option) as handle:
            return ReplayDetector(json.load(handle))
    raise ValueError(f"Unknown detector {args.detector!r}")


def load_calibration(args):
    """--calibration FILE, or `bag:PATH` to take the colour camera from a recording."""
    if not args.calibration:
        return None
    if args.calibration.startswith("bag:"):
        from ..protocol import bag
        return bag.calibration(args.calibration[4:])
    return Calibration.load(args.calibration)


def build_survey(args, store, vlm):
    return Survey(build_detector(args, vlm), ObjectMap(args.data), KeyframeStore(args.data), store,
                  load_calibration(args), args.interval, args.min_sharpness,
                  verifier=Verifier(vlm) if vlm and not args.no_verify else None)


def print_events(store):
    store.subscribe(lambda event: print(json.dumps(event, ensure_ascii=False), flush=True))


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
    Inspection(survey.objects, survey.keyframes, vlm, store, survey, navigator,
               camera=(lambda: next(frames)) if navigator.moves else None, is_free=is_free, mission=mission).run()
    if publisher:
        publisher.end()
    command_status(args)


def command_calibrate(args):
    """Pick prompt and threshold per kind from sample frames, judged by the vision model."""
    detector = YoloDetector(args.detector.partition(":")[2] or "yoloe-11l-seg.pt", track=False)
    frames = list(open_source(args.source))
    report = calibrate.calibrate(frames, detector, Verifier(build_vlm(args)), per_prompt=args.per_prompt)
    print(f"{len(frames)} frames; saved {calibrate.save(report, args.data)}")


def command_status(args):
    for site_object in ObjectMap(args.data).all():
        result = site_object.result
        print(f"{site_object.id:8} {site_object.kind:18} {site_object.status:11} "
              f"position={site_object.position} sightings={site_object.sightings} "
              f"{result.get('status', '')} open={result.get('open', '')} missing_views={len(result.get('missing_views', []))}")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="ultron-pipeline", description=__doc__.splitlines()[0])
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--data", default="runs/pipeline", help="workspace folder shared by all stages")
    common.add_argument("--dashboard", help="URL of the team dashboard")
    common.add_argument("--site", default="demo-site")
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
    common.add_argument("--no-verify", action="store_true", help="survey without the vision model's second opinion")
    commands = parser.add_subparsers(dest="command", required=True)

    survey = commands.add_parser("survey", parents=[common], help="stage 1: build the object map")
    survey.add_argument("--source", required=True, help="relay[:PORT] | bag:PATH | folder:PATH | cv:TARGET | ros2:/topic[,pose=..][,cloud=..] | unitree")
    survey.add_argument("--pace", type=float, default=0.0)
    survey.set_defaults(action=command_survey)

    moving = argparse.ArgumentParser(add_help=False)
    moving.add_argument("--navigator", choices=["passive", "operator", "dashboard", "contract"], default="passive",
                        help="how missing views are reached: not at all, a person drives, goals through "
                             "the dashboard bridge, or the ultron.autonomy navigation contract")
    moving.add_argument("--map", help="saved map folder for --navigator contract")
    moving.add_argument("--clearance", type=float, default=0.45, help="free radius a goal needs, metres")

    inspect = commands.add_parser("inspect", parents=[common, moving], help="stage 2: fill each object's form")
    inspect.add_argument("--camera", help="live frame source for views taken during inspection")
    inspect.set_defaults(action=command_inspect)

    patrol = commands.add_parser("patrol", parents=[common, moving], help="survey, check and inspect in one process")
    patrol.add_argument("--source", required=True, help="relay[:PORT] | bag:PATH | ros2:... | folder:PATH | cv:TARGET")
    patrol.add_argument("--survey-seconds", type=float, default=0.0, help="end the survey after this long (0: when the stream ends or on Ctrl-C)")
    patrol.add_argument("--pace", type=float, default=0.0)
    patrol.set_defaults(action=command_patrol)

    tune = commands.add_parser("calibrate", parents=[common], help="choose detector prompts for this site")
    tune.add_argument("--source", required=True, help="sample frames, for example bag:PATH,rate=1")
    tune.add_argument("--per-prompt", type=int, default=6, help="boxes per candidate shown to the vision model")
    tune.set_defaults(action=command_calibrate)

    commands.add_parser("status", parents=[common], help="list the object map").set_defaults(action=command_status)

    args = parser.parse_args(argv)
    try:
        args.action(args)
    except (VlmError, ValueError, OSError) as error:
        sys.exit(f"Error: {error}")


if __name__ == "__main__":
    main()
