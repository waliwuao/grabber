import math

from spear_locator.core import ScanPoint
from spear_locator.temporal_recognition import (
    StableComponent,
    TemporalGrid,
    arrangement_alignment_error,
    arrangement_line_equation,
    arrangement_spacing_metrics,
    compensate_polar_coordinates,
    compensate_target_centers,
    estimate_arrangement_axis,
    select_target_components,
    split_targets_have_plausible_parent,
)


def point(index, x_m, y_m):
    return ScanPoint(index, 0.0, 0.0, x_m, y_m)


def test_temporal_grid_rejects_one_frame_noise():
    grid = TemporalGrid(resolution_m=0.005)
    for frame in range(20):
        points = [
            point(0, -0.05, -0.20),
            point(1, -0.045, -0.20),
            point(2, 0.05, -0.20),
            point(3, 0.055, -0.20),
        ]
        if frame == 0:
            points.append(point(4, 0.30, -0.30))
        grid.add_frame(points)

    components = grid.stable_components(
        minimum_occupancy=0.5,
        minimum_cells=2,
        connection_radius_cells=1,
    )
    targets = select_target_components(components, expected_count=2)
    assert len(targets) == 2
    assert targets[0].x_m < targets[1].x_m


def test_expected_count_fails_when_targets_are_merged():
    grid = TemporalGrid(resolution_m=0.005)
    for _ in range(20):
        grid.add_frame([
            point(0, -0.005, -0.10),
            point(1, 0.000, -0.10),
            point(2, 0.005, -0.10),
        ])
    components = grid.stable_components(
        minimum_occupancy=0.5,
        minimum_cells=2,
        connection_radius_cells=1,
    )
    assert select_target_components(components, expected_count=2) == []


def test_auto_axis_orders_a_rotated_row():
    grid = TemporalGrid(resolution_m=0.005)
    for _ in range(20):
        grid.add_frame([
            point(0, -0.10, -0.10),
            point(1, -0.095, -0.095),
            point(2, 0.10, 0.10),
            point(3, 0.105, 0.105),
        ])
    components = grid.stable_components(
        minimum_occupancy=0.5,
        minimum_cells=2,
        connection_radius_cells=1,
    )
    targets = select_target_components(
        components,
        expected_count=2,
        alignment_axis='auto',
    )
    axis_x, axis_y = estimate_arrangement_axis(targets)
    assert len(targets) == 2
    assert axis_x > 0.0
    assert axis_y > 0.0


def test_near_split_rejects_pair_cut_from_long_structure():
    standard_grid = TemporalGrid(resolution_m=0.005)
    split_grid = TemporalGrid(resolution_m=0.003)
    for _ in range(20):
        standard_grid.add_frame([
            point(index, -0.18 + index * 0.01, -0.30)
            for index in range(37)
        ])
        split_grid.add_frame([
            point(0, -0.03, -0.28),
            point(1, -0.027, -0.28),
            point(2, 0.006, -0.28),
            point(3, 0.009, -0.28),
        ])

    standard = standard_grid.stable_components(
        minimum_occupancy=0.5,
        minimum_cells=2,
        connection_radius_cells=2,
    )
    split = split_grid.stable_components(
        minimum_occupancy=0.5,
        minimum_cells=2,
        connection_radius_cells=1,
    )
    targets = select_target_components(split, expected_count=2)
    assert len(targets) == 2
    assert not split_targets_have_plausible_parent(
        standard, targets, expected_count=2, maximum_component_span_m=0.12
    )


def test_near_split_rejects_widely_separated_pair():
    grid = TemporalGrid(resolution_m=0.003)
    for _ in range(20):
        grid.add_frame([
            point(0, -0.15, -0.28),
            point(1, -0.147, -0.28),
            point(2, 0.06, -0.28),
            point(3, 0.063, -0.28),
        ])
    targets = select_target_components(
        grid.stable_components(
            minimum_occupancy=0.5,
            minimum_cells=2,
            connection_radius_cells=1,
        ),
        expected_count=2,
    )
    assert len(targets) == 2
    assert not split_targets_have_plausible_parent(
        [], targets, expected_count=2, maximum_component_span_m=0.12
    )


def test_near_split_accepts_one_merged_pair_among_six_targets():
    standard = [
        StableComponent(-0.20, -0.25, 5, 0.02, 0.02, 1.0),
        StableComponent(-0.12, -0.25, 5, 0.02, 0.02, 1.0),
        StableComponent(-0.04, -0.25, 5, 0.02, 0.02, 1.0),
        StableComponent(0.06, -0.25, 5, 0.02, 0.02, 1.0),
        StableComponent(0.15, -0.25, 10, 0.10, 0.02, 1.0),
    ]
    split = [
        StableComponent(-0.20, -0.25, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.12, -0.25, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.04, -0.25, 5, 0.01, 0.01, 1.0),
        StableComponent(0.06, -0.25, 5, 0.01, 0.01, 1.0),
        StableComponent(0.12, -0.25, 5, 0.01, 0.01, 1.0),
        StableComponent(0.18, -0.25, 5, 0.01, 0.01, 1.0),
    ]
    assert split_targets_have_plausible_parent(
        standard, split, expected_count=6, maximum_component_span_m=0.12
    )


def test_near_split_accepts_two_merged_pairs_among_six_targets():
    standard = [
        StableComponent(-0.16, -0.07, 5, 0.02, 0.02, 1.0),
        StableComponent(-0.07, -0.07, 10, 0.08, 0.03, 1.0),
        StableComponent(0.05, -0.08, 10, 0.08, 0.03, 1.0),
        StableComponent(0.15, -0.09, 5, 0.02, 0.02, 1.0),
    ]
    split = [
        StableComponent(-0.16, -0.07, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.10, -0.07, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.04, -0.07, 5, 0.01, 0.01, 1.0),
        StableComponent(0.02, -0.08, 5, 0.01, 0.01, 1.0),
        StableComponent(0.08, -0.08, 5, 0.01, 0.01, 1.0),
        StableComponent(0.15, -0.09, 5, 0.01, 0.01, 1.0),
    ]
    assert split_targets_have_plausible_parent(
        standard, split, expected_count=6, maximum_component_span_m=0.12
    )


def test_arrangement_line_equation_contains_target_centers():
    grid = TemporalGrid(resolution_m=0.005)
    for _ in range(20):
        grid.add_frame([
            point(0, -0.10, -0.20),
            point(1, -0.095, -0.20),
            point(2, 0.10, -0.20),
            point(3, 0.105, -0.20),
        ])
    targets = select_target_components(
        grid.stable_components(
            minimum_occupancy=0.5,
            minimum_cells=2,
            connection_radius_cells=1,
        ),
        expected_count=2,
    )
    a, b, c = arrangement_line_equation(targets)
    assert abs(a * targets[0].x_m + b * targets[0].y_m + c) < 1e-9
    assert abs(a * targets[1].x_m + b * targets[1].y_m + c) < 1e-9


def test_six_target_selection_accepts_approximately_equal_spacing():
    grid = TemporalGrid(resolution_m=0.005)
    xs = [-0.125, -0.075, -0.020, 0.030, 0.080, 0.135]
    for _ in range(20):
        grid.add_frame([
            sample
            for index, x_m in enumerate(xs)
            for sample in (
                point(index * 2, x_m, -0.25),
                point(index * 2 + 1, x_m + 0.005, -0.25),
            )
        ])
    targets = select_target_components(
        grid.stable_components(
            minimum_occupancy=0.5,
            minimum_cells=2,
            connection_radius_cells=1,
        ),
        expected_count=6,
        maximum_spacing_deviation_ratio=0.20,
    )
    mean_spacing, maximum_deviation = arrangement_spacing_metrics(targets)
    assert len(targets) == 6
    assert 0.04 < mean_spacing < 0.06
    assert maximum_deviation < 0.20


def test_six_target_selection_rejects_unequal_spacing():
    grid = TemporalGrid(resolution_m=0.005)
    xs = [-0.15, -0.10, -0.05, 0.00, 0.05, 0.16]
    for _ in range(20):
        grid.add_frame([
            sample
            for index, x_m in enumerate(xs)
            for sample in (
                point(index * 2, x_m, -0.25),
                point(index * 2 + 1, x_m + 0.005, -0.25),
            )
        ])
    targets = select_target_components(
        grid.stable_components(
            minimum_occupancy=0.5,
            minimum_cells=2,
            connection_radius_cells=1,
        ),
        expected_count=6,
        maximum_spacing_deviation_ratio=0.35,
    )
    assert targets == []


def test_six_targets_allow_independent_y_when_hard_spacing_is_disabled():
    components = [
        StableComponent(-0.20, -0.24, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.13, -0.27, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.06, -0.23, 5, 0.01, 0.01, 1.0),
        StableComponent(0.04, -0.28, 5, 0.01, 0.01, 1.0),
        StableComponent(0.12, -0.25, 5, 0.01, 0.01, 1.0),
        StableComponent(0.21, -0.22, 5, 0.01, 0.01, 1.0),
    ]
    targets = select_target_components(
        components,
        expected_count=6,
        maximum_spacing_deviation_ratio=-1.0,
    )
    assert len(targets) == 6
    assert len({target.y_m for target in targets}) == 6


def test_alignment_error_penalizes_one_distant_environment_component():
    target_row = [
        StableComponent(-0.15, -0.06, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.09, -0.07, 5, 0.01, 0.01, 1.0),
        StableComponent(-0.03, -0.05, 5, 0.01, 0.01, 1.0),
        StableComponent(0.03, -0.08, 5, 0.01, 0.01, 1.0),
        StableComponent(0.09, -0.06, 5, 0.01, 0.01, 1.0),
        StableComponent(0.15, -0.07, 5, 0.01, 0.01, 1.0),
    ]
    contaminated = target_row[:-1] + [
        StableComponent(0.00, -0.34, 5, 0.01, 0.01, 1.0)
    ]
    assert arrangement_alignment_error(target_row) < 0.02
    assert (
        arrangement_alignment_error(contaminated)
        > arrangement_alignment_error(target_row) + 0.03
    )


def test_center_compensation_uses_target_row_frame():
    components = [
        StableComponent(-0.20, -0.30, 5, 0.01, 0.01, 1.0),
        StableComponent(0.00, -0.30, 5, 0.01, 0.01, 1.0),
        StableComponent(0.20, -0.30, 5, 0.01, 0.01, 1.0),
    ]
    corrected = compensate_target_centers(
        components,
        [0.01, 0.02, 0.03],
        [0.04, 0.05, 0.06],
    )
    assert [round(item.x_m, 3) for item in corrected] == [
        -0.19, 0.02, 0.23
    ]
    assert [round(item.y_m, 3) for item in corrected] == [
        -0.26, -0.25, -0.24
    ]


def test_polar_compensation_uses_documented_feature_order():
    component = StableComponent(
        0.0, -0.2, 5, 0.01, 0.01, 1.0
    )
    corrected = compensate_polar_coordinates(
        [component],
        [0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
    )[0]
    assert abs(corrected.range_m - 0.21) < 1e-12
    assert abs(
        math.atan2(corrected.y_m, corrected.x_m)
        - (-math.pi / 2.0 + 0.1)
    ) < 1e-12
