#!/usr/bin/env python3
"""Open the stationary simulation-only detection hall in the MuJoCo viewer."""

import argparse
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=None,
                        help="Optional positive time limit in seconds for a bounded viewer check")
    args = parser.parse_args()
    if args.duration is not None:
        import math
        if not math.isfinite(args.duration) or args.duration <= 0:
            parser.error("--duration must be a positive finite number")

    import mujoco
    import mujoco.viewer
    from ultron.detection_world.world import _initial_data, build_xml

    model = mujoco.MjModel.from_xml_string(build_xml())
    data = _initial_data(model)
    previous_threads = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data)
    # MuJoCo 3.3 uses a daemon GUI thread; wait for its cleanup before GLFW's
    # process-exit handler runs, avoiding a shutdown race on WSLg.
    viewer_thread = next((t for t in threading.enumerate()
                          if t not in previous_threads and
                          getattr(t, "_target", None) is getattr(mujoco.viewer, "_launch_internal", None)), None)
    try:
        viewer.cam.lookat[:] = [6, 5, 0.8]
        viewer.cam.distance = 18
        viewer.cam.elevation = -35
        print("SIMULATION ONLY: stationary B2 and hall; viewer camera is not the capture camera.",
              flush=True)
        end = time.monotonic() + args.duration if args.duration is not None else None
        while viewer.is_running() and (end is None or time.monotonic() < end):
            viewer.sync()
            time.sleep(0.03)
    finally:
        viewer.close()
        if viewer_thread is not None:
            viewer_thread.join()


if __name__ == "__main__":
    main()
