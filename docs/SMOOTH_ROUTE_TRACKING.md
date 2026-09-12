# Smooth Route Tracking

The route pipeline has two separate stages:

1. `route_recorder_node.py` and `rtk_waypoint_recorder_node.py` collect quality-checked
   GNSS samples and route metadata.
2. `route_smoothing.py` fits a bounded natural cubic spline while preserving mission
   and speed-change anchors.

At runtime `RouteFollower` projects the current vehicle pose onto a forward metric
window, keeps progress monotonic and chooses a continuous lookahead target. Pure
Pursuit turns that target into steering while the follower applies steering-rate,
curvature and tracking-corridor checks. Invalid or stale pose data causes a stopped
result.

`routes/example_route.csv` is synthetic and exists only to document the CSV schema. A
real route must be recorded, reviewed and calibrated locally before any vehicle test.
