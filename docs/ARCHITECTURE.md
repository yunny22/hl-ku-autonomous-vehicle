# System Architecture

The public source follows this high-level path:

```text
UM982 dual-antenna GNSS
        ↓
NMEA validation → local ENU pose / heading / speed
        ↓
recorded route → cubic-spline preparation
        ↓
RouteFollower + Pure Pursuit
        ↓
mission and safety supervision
        ↓
ROS 2 actuator command → NUCLEO-H743ZI2 → vehicle hardware
```

Camera and LiDAR nodes publish perception state for the mission and safety layers.
Their device topics and calibration belong in deployment-local configuration. The
public tree describes the interfaces without including recorded tracks or sensor logs.

## Monitoring

The nodes publish pose, route progress, reference path, tracking target, mission state,
perception state and vehicle feedback topics. These can be inspected with a ROS 2 topic
viewer such as Foxglove after connecting a local bridge. No dashboard recording or
private network profile is included here.
