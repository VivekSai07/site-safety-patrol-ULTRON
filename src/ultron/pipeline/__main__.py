"""Command line for the site patrol pipeline.

    python -m ultron.pipeline survey  --source ros2:/image,pose=/odom,cloud=/points --calibration cal.json
    python -m ultron.pipeline inspect --navigator dashboard --camera ros2:/image,pose=/odom
    python -m ultron.pipeline status

Both stages work on the same --data folder; stop and restart either at any time.
"""
import argparse
import json
import sys
import uuid

from .detect import ReplayDetector, VlmDetector, YoloDetector
from .inspection import Inspection
from .keyframes import KeyframeStore
from .navigate import DashboardNavigator, Operator, Passive, http_json
from .objects import ObjectMap
from .survey import Survey
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
        return YoloDetector(option) if option else YoloDetector()
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
    calibration = load_calibration(args)
    return Survey(build_detector(args, vlm), ObjectMap(args.data), KeyframeStore(args.data), store,
                  calibration, args.interval, args.min_sharpness)


def print_events(store):
    store.subscribe(lambda event: print(json.dumps(event, ensure_ascii=False), flush=True))


def command_survey(args):
    store, mission = Store(args.data), args.mission or "PATROL-" + uuid.uuid4().hex[:6]
    survey = build_survey(args, store, build_vlm(args) if args.detector == "vlm" else None)
    connect_dashboard(store, args, mission)  # the mission stays open for the inspection stage
    print_events(store)
    try:
        survey.run(open_source(args.source, args.pace))
    except KeyboardInterrupt:
        pass
    print(f"{len(survey.objects.all())} objects in {args.data}/objects.json", flush=True)


def command_inspect(args):
    store, mission, vlm = Store(args.data), args.mission or "PATROL-" + uuid.uuid4().hex[:6], build_vlm(args)
    survey = build_survey(args, store, vlm)
    navigator = {"passive": Passive, "operator": Operator}.get(args.navigator, lambda: None)()
    if args.navigator == "dashboard":
        if not args.dashboard:
            sys.exit("--navigator dashboard needs --dashboard URL")
        navigator = DashboardNavigator(http_json(args.dashboard))
    frames = iter(open_source(args.camera)) if args.camera else None
    publisher = connect_dashboard(store, args, mission)
    print_events(store)
    Inspection(survey.objects, survey.keyframes, vlm, store, survey, navigator,
               camera=(lambda: next(frames)) if frames else None, mission=mission).run()
    if publisher:
        publisher.end()
    command_status(args)


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
    commands = parser.add_subparsers(dest="command", required=True)

    survey = commands.add_parser("survey", parents=[common], help="stage 1: build the object map")
    survey.add_argument("--source", required=True, help="bag:PATH | folder:PATH | cv:TARGET | ros2:/topic[,pose=..][,cloud=..] | unitree")
    survey.add_argument("--pace", type=float, default=0.0)
    survey.set_defaults(action=command_survey)

    inspect = commands.add_parser("inspect", parents=[common], help="stage 2: fill each object's form")
    inspect.add_argument("--navigator", choices=["passive", "operator", "dashboard"], default="passive")
    inspect.add_argument("--camera", help="live frame source for views taken during inspection")
    inspect.set_defaults(action=command_inspect)

    commands.add_parser("status", parents=[common], help="list the object map").set_defaults(action=command_status)

    args = parser.parse_args(argv)
    try:
        args.action(args)
    except (VlmError, ValueError, OSError) as error:
        sys.exit(f"Error: {error}")


if __name__ == "__main__":
    main()
