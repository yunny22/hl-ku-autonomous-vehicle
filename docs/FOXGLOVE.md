# Monitoring topics

The ROS 2 nodes expose the following monitoring surfaces for a local Foxglove or
equivalent ROS 2 viewer:

- `/localization/gnss_pose`, `/gnss/fix`, `/gnss/status`
- `/planning/reference_path`, `/planning/tracking_target`, `/planning/route_s`
- `/planning/tracking_status`, `/planning/path_command`
- `/perception/state`, `/mission/status`
- `/vehicle/actuator_command_safe`, `/vehicle/feedback`

Topic names describe the public interfaces. Device-specific bridge settings and
recorded telemetry are intentionally omitted.
