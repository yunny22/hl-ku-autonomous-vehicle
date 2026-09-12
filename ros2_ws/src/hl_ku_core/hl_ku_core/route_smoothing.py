"""Offline, bounded cubic-spline fairing of measured route geometry.

NumPy/SciPy are imported only here; the online tracker has no SciPy dependency.
Deviation and analytic curvature are validated on a fine parameter grid before
any output is accepted. An infeasible route raises, rather than relaxing limits.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.sparse import diags
from scipy.sparse.linalg import spsolve

from .route import Route, Waypoint


@dataclass(frozen=True)
class SmoothingReport:
    maximum_deviation_m: float
    maximum_curvature_inv_m: float
    smoothing_length_m: float
    validation_step_m: float


def smooth_route(
    route: Route,
    maximum_spacing_m: float = 0.10,
    maximum_deviation_m: float = 0.20,
    minimum_turning_radius_m: float = 1.12,
    maximum_smoothing_length_m: float = 3.0,
) -> tuple[Route, SmoothingReport]:
    """Fair a single-direction route near its source; keep metadata anchors fixed.

    The regularizer approximates integral |d2p/du2|^2. A natural cubic spline
then supplies continuous tangent/curvature. u denotes source arc distance;
the output is resampled by the resulting curve's arc distance, not by u.
"""
    parameters = (maximum_spacing_m, maximum_deviation_m,
                  minimum_turning_radius_m, maximum_smoothing_length_m)
    if not all(math.isfinite(v) and v > 0.0 for v in parameters):
        raise ValueError("smoothing spacing, deviation, radius and length must be positive")
    if len({p.direction for p in route.waypoints}) != 1:
        raise ValueError("split forward/reverse legs before smoothing; direction cusps are hard stops")

    points = route.waypoints
    source_u = np.asarray(route.stations)
    source_xy = np.asarray([(p.x_m, p.y_m) for p in points])
    # Translation improves conditioning for maps far from the ENU origin.
    origin = source_xy[0].copy()
    source_xy = source_xy - origin
    anchor_indices = {0, len(points) - 1}
    for i in range(1, len(points) - 1):
        a, b = points[i - 1], points[i]
        if (a.mission, a.target_speed_mps, a.direction) != (b.mission, b.target_speed_mps, b.direction):
            anchor_indices.add(i)
    anchors = source_u[sorted(anchor_indices)]
    grid = np.linspace(0.0, route.length_m, max(6, math.ceil(route.length_m / 0.20) + 1))
    # Do not leave nearly coincident grid and mandatory event anchors.
    grid = np.asarray([u for u in grid if np.min(np.abs(anchors - u)) > 1.0e-4])
    grid = np.unique(np.concatenate((grid, anchors)))
    original = np.column_stack([np.interp(grid, source_u, source_xy[:, j]) for j in (0, 1)])
    h = np.diff(grid)
    weights = np.concatenate(([h[0] / 2], (h[:-1] + h[1:]) / 2, [h[-1] / 2]))
    hp, hn = h[:-1], h[1:]
    factors = np.sqrt((hp + hn) / 2)
    from scipy.sparse import coo_matrix
    rows = np.repeat(np.arange(len(grid) - 2), 3)
    cols = np.column_stack((np.arange(len(grid) - 2), np.arange(1, len(grid) - 1), np.arange(2, len(grid)))).ravel()
    values = np.column_stack((2 / ((hp + hn) * hp),
                              -2 / (hp + hn) * (1 / hp + 1 / hn),
                              2 / ((hp + hn) * hn))) * factors[:, None]
    second = coo_matrix((values.ravel(), (rows, cols)), shape=(len(grid) - 2, len(grid))).tocsc()
    regularizer = second.T @ second
    fixed = np.searchsorted(grid, anchors)
    free = np.setdiff1d(np.arange(len(grid)), fixed)
    validation_step = min(0.025, maximum_spacing_m / 4, maximum_deviation_m / 5)
    check_u = np.unique(np.concatenate((np.linspace(0, route.length_m,
        math.ceil(route.length_m / validation_step) + 1), source_u, grid)))
    reference = np.column_stack([np.interp(check_u, source_u, source_xy[:, j]) for j in (0, 1)])
    best_stats = (math.inf, math.inf)
    lengths = np.geomspace(0.03, maximum_smoothing_length_m, 48)
    for smoothing_length in lengths:
        matrix = diags(weights) + smoothing_length ** 4 * regularizer
        fitted = original.copy()
        rhs = weights[:, None] * original
        if len(free):
            reduced = matrix.tocsc()[free, :]
            fitted[free] = spsolve(reduced[:, free], rhs[free] - reduced[:, fixed] @ original[fixed])
        spline = CubicSpline(grid, fitted, bc_type="natural")
        xy, derivative, second_derivative = spline(check_u), spline(check_u, 1), spline(check_u, 2)
        norm = np.linalg.norm(derivative, axis=1)
        if not np.all(np.isfinite(xy)) or np.min(norm) < 0.05:
            continue  # no cusps, stationary tangent, or non-finite geometry
        curvature = (derivative[:, 0] * second_derivative[:, 1] - derivative[:, 1] * second_derivative[:, 0]) / norm ** 3
        deviation = float(np.max(np.linalg.norm(xy - reference, axis=1)))
        maximum_curvature = float(np.max(np.abs(curvature)))
        if deviation < best_stats[0]:
            best_stats = (deviation, maximum_curvature)
        if deviation > maximum_deviation_m or maximum_curvature > 1 / minimum_turning_radius_m:
            continue
        arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))))
        # Include exact mission/speed anchors as well as equally spaced curve samples.
        output_arc = np.linspace(0, arc[-1], math.ceil(arc[-1] / maximum_spacing_m) + 1)
        output_u = np.unique(np.concatenate((np.interp(output_arc, arc, check_u), anchors)))
        # Avoid rounding a near-anchor extra sample onto the anchor in the CSV.
        output_u = np.asarray([u for u in output_u if u in anchors or np.min(np.abs(anchors - u)) > 1.0e-4])
        result_xy = spline(output_u) + origin
        d1, d2 = spline(output_u, 1), spline(output_u, 2)
        output_curvature = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.linalg.norm(d1, axis=1) ** 3
        result: list[Waypoint] = []
        accumulated = 0.0
        for i, (u, xy_i, tangent, kappa) in enumerate(zip(output_u, result_xy, d1, output_curvature)):
            if i:
                accumulated += float(np.linalg.norm(xy_i - result_xy[i - 1]))
            original_index = min(len(points) - 1, max(0, bisect.bisect_right(source_u, u + 1.0e-9) - 1))
            metadata = points[original_index]
            result.append(Waypoint(i, accumulated, float(xy_i[0]), float(xy_i[1]),
                metadata.target_speed_mps, metadata.mission, metadata.direction,
                math.atan2(tangent[1], tangent[0]), float(kappa)))
        return Route(result), SmoothingReport(deviation, maximum_curvature, float(smoothing_length), validation_step)
    raise ValueError(
        "no smooth route satisfied the deviation/turning-radius limits; "
        "check waypoint order, endpoint headings and corridor width, or remeasure the corner "
        f"(smallest observed deviation={best_stats[0]:.3f} m, "
        f"curvature there={best_stats[1]:.3f} 1/m)"
    )
