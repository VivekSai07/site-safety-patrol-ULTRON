#!/usr/bin/env python3
"""Capture synthetic images from bounded, explicitly positioned virtual cameras."""

import argparse

from ultron.detection_world.world import capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", help="New output directory; default is a unique runs/ directory")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--position", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--target", type=float, nargs=3, metavar=("X", "Y", "Z"))
    args = parser.parse_args()
    if (args.position is None) != (args.target is None):
        parser.error("--position and --target must be supplied together")
    options = {"width": args.width, "height": args.height}
    if args.position is not None:
        options["views"] = [(args.position, args.target)]
    print(capture(args.out, **options))


if __name__ == "__main__":
    main()
