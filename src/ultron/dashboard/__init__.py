"""Live ladder-inspection dashboard: ingest API, SQLite store and live event stream.

The dashboard only stores and displays records. Verdicts are decided upstream
(simulation or ROS side); records that contradict the verdict rules are rejected,
never corrected.
"""
