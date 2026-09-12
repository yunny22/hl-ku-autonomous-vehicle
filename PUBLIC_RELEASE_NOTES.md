# Public Release Notes

## Final source selection

- Source candidate: local clone `HL_KU_repo` (machine path intentionally omitted)
- Branch: `codex/smooth-route-tracking`
- Commit: `2f88330`
- Remote: `yunny22/HL_KU`
- Selection reason: local clone was clean and the selected branch matched its remote;
  it contains the latest verified route smoothing and continuous tracking work.
- The repository's `main` history and the older embedded copy under the shared
  Gazebo workspace were not selected as the release source.

## Included

- ROS 2 `hl_ku_core` source, tests and package metadata
- `hl_ku_interfaces` messages
- NUCLEO-H743ZI2 protocol/control firmware modules and host test
- Safe configuration and route templates
- Public launch example and architecture/monitoring notes

## Excluded

- Real route CSVs, recorded GNSS logs, bags, datasets and media
- Raw calibration records, ADC values, datum coordinates and device serial paths
- NTRIP credentials, private network addresses and local machine settings
- Model weights and generated perception artifacts
- Build/install/log output, Python caches and the original `.git` directory
- Keyboard teleoperation and UDP bridge experiments that are not required for the
  public route-following story
- The older duplicate source under the shared `xycar_kookmin_gazebo_track` workspace

## Provenance classification

| Area | Classification | Basis |
| --- | --- | --- |
| Route smoothing and continuous tracking | PERSONAL evidence | Selected commit `2f88330` is authored by `yunny22` and adds the route work. |
| Vehicle platform, GNSS, perception, mission and firmware integration | CO-DEVELOPED / TEAM | Source is from the team repository and history contains multiple contributors. |
| ROS 2, NumPy, SciPy, pyserial and other runtime packages | THIRD-PARTY | Declared dependencies; not redistributed here. |
| Real routes, calibration and deployment settings | TEAM / LOCAL DATA | Excluded from the public release; deployment-specific files remain outside this tree. |

## Sanitization

Hardware nodes default to disabled and accept local device paths only through a
deployment parameter file. Real route and calibration files were replaced with
synthetic examples or omitted. Firmware notes use deployment-supplied network values
instead of recorded addresses and ports. The public tree contains no source history,
model weights, bags, private credentials or absolute user paths.

## Validation performed

- ROS 2 package build and Python test suite are run in the public release tree.
- NUCLEO control module is compiled with `gcc -Wall -Wextra -Werror` and its host test
  is executed.
- Launch files, YAML and package metadata are checked for syntax.
- A repository scan checks for machine paths, private addresses, credentials, large
  generated artifacts and excluded route/log files.

## Remaining review items

- Package metadata is `UNLICENSED`; this portfolio repository grants no separate
  open-source reuse license. Team scope and attribution boundaries are
  documented and publication consent is confirmed.
- Supply deployment-local GNSS, NUCLEO, calibration and route files outside this tree.
- YOLO obstacle perception and mission integration remain ongoing and require a later
  reviewed release.
- Full hardware validation must be repeated on the intended vehicle after local
  configuration and safety checks.
