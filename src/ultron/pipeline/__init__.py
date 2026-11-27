"""Site patrol pipeline: survey the site into an object map, then inspect each object.

    survey      frames + poses -> detections -> 3D objects + keyframes, saved as it goes
    inspection  per object: collect the views its form needs, read the form, report

See docs/PIPELINE.md for the module map and who owns what.
"""
