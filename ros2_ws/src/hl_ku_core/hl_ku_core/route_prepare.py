"""Prepare a sparse measured route for deterministic vehicle tracking."""

from __future__ import annotations

import argparse
import csv
import math
import warnings
from pathlib import Path
from typing import Sequence

from .geometry import antenna_to_base
from .route import Route, Waypoint


def densify_route(route: Route, maximum_spacing_m: float) -> Route:
    """Linearly subdivide route segments without changing the measured polyline."""
    if not math.isfinite(maximum_spacing_m) or maximum_spacing_m <= 0.0:
        raise ValueError("maximum_spacing_m must be finite and positive")

    source = route.waypoints
    dense = [
        Waypoint(
            index=0,
            s_m=0.0,
            x_m=source[0].x_m,
            y_m=source[0].y_m,
            target_speed_mps=source[0].target_speed_mps,
            mission=source[0].mission,
            direction=source[0].direction,
        )
    ]
    accumulated = 0.0
    for start, end in zip(source, source[1:]):
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        length = math.hypot(dx, dy)
        subdivisions = max(1, math.ceil(length / maximum_spacing_m))
        step_length = length / subdivisions
        for step in range(1, subdivisions + 1):
            ratio = step / subdivisions
            accumulated += step_length
            at_endpoint = step == subdivisions
            dense.append(
                Waypoint(
                    index=len(dense),
                    s_m=accumulated,
                    x_m=start.x_m + dx * ratio,
                    y_m=start.y_m + dy * ratio,
                    target_speed_mps=(
                        end.target_speed_mps
                        if at_endpoint
                        else start.target_speed_mps
                    ),
                    mission=end.mission if at_endpoint else start.mission,
                    direction=end.direction if at_endpoint else start.direction,
                )
            )
    return Route(dense)


def antenna_path_to_base_link(
    route: Route,
    base_to_antenna_x_m: float,
    base_to_antenna_y_m: float,
    vehicle_yaws_rad: Sequence[float] | None = None,
) -> Route:
    """Remove the lever arm using measured body yaw, or an explicit approximation."""
    if not all(
        math.isfinite(value)
        for value in (base_to_antenna_x_m, base_to_antenna_y_m)
    ):
        raise ValueError("antenna lever arm must be finite")

    source = route.waypoints
    if vehicle_yaws_rad is not None and (
        len(vehicle_yaws_rad) != len(source)
        or not all(math.isfinite(yaw) for yaw in vehicle_yaws_rad)
    ):
        raise ValueError("one finite vehicle yaw is required for each antenna point")
    shifted: list[Waypoint] = []
    accumulated = 0.0
    previous_xy: tuple[float, float] | None = None
    for index, waypoint in enumerate(source):
        if index == 0:
            start, end = source[0], source[1]
        elif index == len(source) - 1:
            start, end = source[-2], source[-1]
        else:
            start, end = source[index - 1], source[index + 1]
        yaw = math.atan2(end.y_m - start.y_m, end.x_m - start.x_m)
        if vehicle_yaws_rad is not None:
            yaw = vehicle_yaws_rad[index]
        elif waypoint.direction < 0:
            yaw += math.pi
        x_m, y_m = antenna_to_base(
            waypoint.x_m,
            waypoint.y_m,
            yaw,
            base_to_antenna_x_m,
            base_to_antenna_y_m,
        )
        if previous_xy is not None:
            accumulated += math.hypot(x_m - previous_xy[0], y_m - previous_xy[1])
        shifted.append(
            Waypoint(
                index=index,
                s_m=accumulated,
                x_m=x_m,
                y_m=y_m,
                target_speed_mps=waypoint.target_speed_mps,
                mission=waypoint.mission,
                direction=waypoint.direction,
            )
        )
        previous_xy = (x_m, y_m)
    return Route(shifted)


def save_route(route: Route, output_file: str | Path) -> None:
    path = Path(output_file).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            ("s_m", "x_m", "y_m", "target_speed_mps", "mission", "direction",
             "yaw_rad", "curvature_inv_m")
        )
        for waypoint in route.waypoints:
            writer.writerow(
                (
                    f"{waypoint.s_m:.6f}",
                    f"{waypoint.x_m:.6f}",
                    f"{waypoint.y_m:.6f}",
                    f"{waypoint.target_speed_mps:.3f}",
                    waypoint.mission,
                    waypoint.direction,
                    "" if waypoint.yaw_rad is None else f"{waypoint.yaw_rad:.9f}",
                    "" if waypoint.curvature_inv_m is None else f"{waypoint.curvature_inv_m:.9f}",
                )
            )
    Route.load_csv(temporary)  # reject invalid geometry after text rounding
    temporary.replace(path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a smooth, deviation- and curvature-checked HL_KU route"
    )
    parser.add_argument("input_file")
    parser.add_argument("output_file")
    parser.add_argument("--maximum-spacing-m", type=float, default=0.10)
    parser.add_argument("--maximum-deviation-m", type=float, default=0.20)
    parser.add_argument("--minimum-turning-radius-m", type=float, default=1.12)
    parser.add_argument("--maximum-smoothing-length-m", type=float, default=3.0)
    parser.add_argument("--method", choices=("smooth", "linear"), default="smooth")
    parser.add_argument("--input-reference", choices=("base_link", "antenna", "antenna-tangent"),
                        default="base_link", help="antenna requires a vehicle_yaw_rad CSV column")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--base-to-antenna-x-m", type=float, default=0.0)
    parser.add_argument("--base-to-antenna-y-m", type=float, default=0.0)
    return parser


def main(argv: Sequence[str] | None = None) -> None:
    arguments = _parser().parse_args(argv)
    input_path = Path(arguments.input_file).expanduser().resolve()
    output_path = Path(arguments.output_file).expanduser().resolve()
    if input_path == output_path:
        raise ValueError("keep the source route: input and output must be different files")
    if output_path.exists() and not arguments.overwrite:
        raise FileExistsError("output already exists; choose another file or pass --overwrite")
    route = Route.load_csv(input_path)
    lever_x, lever_y = arguments.base_to_antenna_x_m, arguments.base_to_antenna_y_m
    if not all(math.isfinite(v) for v in (lever_x, lever_y)):
        raise ValueError("antenna lever arm must be finite")
    if arguments.input_reference == "base_link":
        if lever_x != 0.0 or lever_y != 0.0:
            raise ValueError("base_link input must not receive antenna correction again; specify --input-reference")
    else:
        yaws = None
        if arguments.input_reference == "antenna":
            with input_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            if not all(row.get("vehicle_yaw_rad", "").strip() for row in rows):
                raise ValueError("antenna input needs measured ENU vehicle_yaw_rad, not a path tangent")
            yaws = [float(row["vehicle_yaw_rad"]) for row in rows]
        else:
            warnings.warn("antenna-tangent is approximate on curves; prefer measured vehicle yaw or base_link waypoints")
        # Correct the original measurements BEFORE resampling or smoothing.
        route = antenna_path_to_base_link(route, lever_x, lever_y, yaws)
    if arguments.method == "linear":
        prepared = densify_route(route, arguments.maximum_spacing_m)
    else:
        from .route_smoothing import smooth_route
        prepared, report = smooth_route(route, arguments.maximum_spacing_m,
            arguments.maximum_deviation_m, arguments.minimum_turning_radius_m,
            arguments.maximum_smoothing_length_m)
        print(f"checked deviation={report.maximum_deviation_m:.4f} m, "
              f"max_curvature={report.maximum_curvature_inv_m:.4f} 1/m, "
              f"validation_step={report.validation_step_m:.4f} m")
    save_route(prepared, arguments.output_file)
    print(
        f"prepared {len(prepared.waypoints)} waypoints, "
        f"length={prepared.waypoints[-1].s_m:.3f} m, "
        f"output={Path(arguments.output_file).expanduser()}"
    )


if __name__ == "__main__":
    main()
