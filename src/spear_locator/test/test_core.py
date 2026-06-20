import math

from spear_locator.core import (
    ProcessorConfig,
    angle_in_window,
    calibrate_range,
    process_scan,
)


def make_scan(targets):
    angle_min = -math.pi / 2.0
    increment = math.radians(0.25)
    count = 721
    ranges = [math.inf] * count
    for x_m, y_m in targets:
        bearing = math.atan2(y_m, x_m)
        center = round((bearing - angle_min) / increment)
        target_range = math.hypot(x_m, y_m)
        for offset in range(-2, 3):
            ranges[center + offset] = target_range
    return ranges, angle_min, increment


def test_angle_window_supports_wrapping():
    assert angle_in_window(math.radians(179), 170, -170)
    assert angle_in_window(math.radians(-179), 170, -170)
    assert not angle_in_window(0.0, 170, -170)


def test_direction_dependent_range_calibration():
    config = ProcessorConfig(
        range_calibration_scale=0.96370434,
        range_calibration_offset_m=-0.00795114,
        range_calibration_cos_m=-0.00072278,
        range_calibration_sin_m=-0.00192741,
        range_calibration_cos2_m=-0.00036139,
    )
    assert abs(calibrate_range(0.134, 0.0, config) - 0.12010) < 1e-5
    assert (
        abs(calibrate_range(0.234, -math.pi / 2.0, config) - 0.21984)
        < 1e-5
    )


def test_process_scan_detects_and_orders_targets():
    targets = [(1.2, -0.2), (1.2, 0.0), (1.2, 0.2)]
    ranges, angle_min, increment = make_scan(targets)
    config = ProcessorConfig(
        range_max_m=2.0,
        angle_min_deg=-90.0,
        angle_max_deg=90.0,
        x_min_m=0.1,
        x_max_m=2.0,
        y_min_m=-0.5,
        y_max_m=0.5,
        max_cluster_width_m=0.10,
    )

    points, detections = process_scan(
        ranges,
        angle_min,
        increment,
        0.03,
        25.0,
        config,
        sort_axis='y',
    )

    assert len(points) == 15
    assert len(detections) == 3
    assert [item.target_id for item in detections] == [0, 1, 2]
    assert detections[0].y_m < detections[1].y_m < detections[2].y_m
    assert all(abs(item.x_m - 1.2) < 0.01 for item in detections)


def test_roi_rejects_targets_outside_work_area():
    ranges, angle_min, increment = make_scan([(1.0, 0.0), (2.5, 0.5)])
    config = ProcessorConfig(
        range_max_m=1.5,
        angle_min_deg=-90.0,
        angle_max_deg=90.0,
        x_min_m=0.1,
        x_max_m=1.5,
        y_min_m=-1.5,
        y_max_m=1.5,
    )
    _, detections = process_scan(
        ranges, angle_min, increment, 0.03, 25.0, config
    )
    assert len(detections) == 1


def test_negative_y_layout_is_ordered_along_x():
    targets = [(-0.1, -0.3), (0.1, -0.3)]
    angle_min = -math.pi
    increment = math.radians(0.25)
    ranges = [math.inf] * 1441
    for x_m, y_m in targets:
        bearing = math.atan2(y_m, x_m)
        center = round((bearing - angle_min) / increment)
        target_range = math.hypot(x_m, y_m)
        for offset in range(-2, 3):
            ranges[center + offset] = target_range

    config = ProcessorConfig()
    _, detections = process_scan(
        ranges, angle_min, increment, 0.03, 25.0, config
    )

    assert len(detections) == 2
    assert detections[0].x_m < detections[1].x_m
    assert all(item.y_m < 0.0 for item in detections)
