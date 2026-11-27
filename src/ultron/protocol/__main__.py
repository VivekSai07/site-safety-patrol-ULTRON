"""Command line for the nesseler ladder checklist reader.

    python -m ultron.protocol run --source cv:0 --dashboard http://127.0.0.1:8000   patrol a stream
    python -m ultron.protocol inspect a.jpg b.jpg                                    one ladder from chosen photos
    python -m ultron.protocol check                                                  is the vision model reachable?

Results are written to --data and, with --dashboard, posted to the team dashboard (`python -m ultron.dashboard`).
"""
import argparse
import json
import sys
import uuid

from .assess import assess
from .frames import Frame
from .lidar import Calibration
from .patrol import Patrol
from .sources import open_source
from .store import Store
from .vlm import OpenAIVlm, ReplayVlm, VlmError


def build_vlm(args):
    if args.replay:
        return ReplayVlm(args.replay)
    vlm = OpenAIVlm(model=args.model, triage_model=args.triage_model, base_url=args.base_url,
                    cache_dir=None if args.no_cache else f"{args.data}/vlm-cache", language=args.language)
    if not vlm.api_key and "api.openai.com" in vlm.base_url:
        sys.exit("Set OPENAI_API_KEY (or use --replay FILE / --base-url for a local model server).")
    return vlm


def connect_dashboard(store, args, mission):
    """Publisher for --dashboard, or None. Imported late: the reader also runs without the dashboard extra."""
    if not args.dashboard:
        return None
    from .publish import Publisher, http_post
    publisher = Publisher(store, http_post(args.dashboard), mission, args.site)
    if not publisher.start():
        sys.exit(f"Dashboard at {args.dashboard} did not accept mission {mission}. Is it running?")
    store.subscribe(publisher.on_event)
    return publisher


def command_run(args):
    store = Store(args.data)
    patrol = Patrol(build_vlm(args), store, interval=args.interval, min_sharpness=args.min_sharpness,
                    gap=args.gap, calibration=Calibration.load(args.calibration) if args.calibration else None,
                    mission=args.mission)
    publisher = connect_dashboard(store, args, patrol.mission)
    store.subscribe(lambda event: print(json.dumps(event, ensure_ascii=False), flush=True))
    try:
        patrol.run(open_source(args.source, args.pace))
    except KeyboardInterrupt:
        patrol.finish()
        patrol.wait()
    if publisher:
        publisher.end()


def command_inspect(args):
    store, vlm = Store(args.data), build_vlm(args)
    mission = args.mission or "MANUAL-" + uuid.uuid4().hex[:6]
    publisher = connect_dashboard(store, args, mission)
    frames = [Frame.from_file(path) for path in args.images]
    sheet = store.record(assess(vlm.inspect(frames), len(frames)), frames, mission=mission,
                         model=getattr(vlm, "model", None))
    if publisher:
        publisher.end()
    last = sheet["inspections"][-1]
    print(json.dumps({"ladder": sheet["id"], "result": last["result_label"], "flags": last["flags"],
                      "coverage": last["coverage"], "needs_view": last["needs_view"],
                      "sheet": f"{args.data}/ladders/{sheet['id']}/sheet.json"}, ensure_ascii=False, indent=1))


def command_check(args):
    vlm = build_vlm(args)
    from PIL import Image
    look = vlm.triage(Frame(Image.new("RGB", (64, 64), "grey"), name="check"))
    print(f"OK: {vlm.triage_model} answered {look}. Full readings use {vlm.model}.")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="ultron-protocol", description=__doc__.splitlines()[0])
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--data", default="runs/protocol", help="folder holding the ladder sheets")
    common.add_argument("--dashboard", help="URL of the team dashboard to post results to")
    common.add_argument("--site", default="demo-site", help="site id shown on the dashboard")
    model = argparse.ArgumentParser(add_help=False)
    model.add_argument("--model", help="model for the full reading (default $ULTRON_VLM_MODEL or gpt-5)")
    model.add_argument("--triage-model", help="model for the quick look (default $ULTRON_TRIAGE_MODEL or gpt-5-mini)")
    model.add_argument("--base-url", help="OpenAI-compatible endpoint (default $OPENAI_BASE_URL)")
    model.add_argument("--language", default="German", help="language of remarks")
    model.add_argument("--replay", help="answer from a recorded JSON file instead of a model")
    model.add_argument("--no-cache", action="store_true", help="do not reuse cached model answers")
    model.add_argument("--mission", help="name of this patrol")
    commands = parser.add_subparsers(dest="command", required=True)

    run = commands.add_parser("run", parents=[common, model], help="patrol a frame source")
    run.add_argument("--source", required=True, help="folder:PATH | cv:TARGET | ros2:/topic[,pose=..][,cloud=..] | unitree")
    run.add_argument("--interval", type=float, default=2.0, help="seconds between quick looks")
    run.add_argument("--gap", type=int, default=2, help="quick looks without a ladder that end a sighting")
    run.add_argument("--min-sharpness", type=float, default=0.0, help="skip frames blurrier than this")
    run.add_argument("--pace", type=float, default=0.0, help="folder source: seconds between images")
    run.add_argument("--calibration", help="camera/lidar calibration JSON for ladder positions")
    run.set_defaults(action=command_run)

    inspect = commands.add_parser("inspect", parents=[common, model], help="one ladder from chosen photos")
    inspect.add_argument("images", nargs="+")
    inspect.set_defaults(action=command_inspect)

    commands.add_parser("check", parents=[common, model], help="test the model connection").set_defaults(action=command_check)

    args = parser.parse_args(argv)
    try:
        args.action(args)
    except (VlmError, ValueError, OSError) as error:
        sys.exit(f"Error: {error}")


if __name__ == "__main__":
    main()
