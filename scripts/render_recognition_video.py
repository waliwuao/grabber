#!/usr/bin/env python3
"""Render a LaserScan rosbag as an MP4 with recognized regions highlighted."""

import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src' / 'spear_locator'))

from spear_locator.core import ProcessorConfig, scan_to_points  # noqa: E402
from spear_locator.temporal_recognition import (  # noqa: E402
    TemporalGrid,
    select_target_components,
    split_targets_have_plausible_parent,
)


def read_frames(bag: Path):
    """Read filtered XY points and timestamps from the rosbag."""
    config = ProcessorConfig()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    frames = []
    first_timestamp = None
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != '/scan':
            continue
        scan = deserialize_message(data, LaserScan)
        points = scan_to_points(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            config,
            scan.intensities,
        )
        if first_timestamp is None:
            first_timestamp = timestamp
        frames.append((
            (timestamp - first_timestamp) / 1e9,
            np.asarray([(point.x_m, point.y_m) for point in points]),
        ))
    return frames


def recognize(frames, expected_count):
    """Run the same standard/near-split recognition used by the ROS node."""
    profiles = [
        ('standard', TemporalGrid(0.005), 0.15, 2),
        ('near_split', TemporalGrid(0.003), 0.60, 2),
    ]
    for _, points in frames:
        scan_points = [
            type('Point', (), {'x_m': xy[0], 'y_m': xy[1]})
            for xy in points
        ]
        for _, grid, _, _ in profiles:
            grid.add_frame(scan_points)

    standard_components = []
    for profile, grid, occupancy, radius in profiles:
        components = grid.stable_components(
            minimum_occupancy=occupancy,
            connection_radius_cells=radius,
        )
        if profile == 'standard':
            standard_components = components
        targets = select_target_components(
            components,
            expected_count=expected_count,
            maximum_component_span_m=0.12,
        )
        if (
            profile == 'near_split'
            and targets
            and not split_targets_have_plausible_parent(
                standard_components, targets, expected_count, 0.12
            )
        ):
            continue
        if len(targets) == expected_count:
            return profile, targets
    return 'none', []


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--expected-count', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()

    frames = read_frames(args.bag)
    profile, targets = recognize(frames, args.expected_count)
    if not targets:
        raise SystemExit('No targets recognized')

    width, height = 1280, 720
    plot_left, plot_top = 80, 85
    plot_right, plot_bottom = 1230, 660
    x_min, x_max = -0.105, 0.055
    y_min, y_max = -0.330, -0.240

    def pixel(x_m, y_m):
        u = plot_left + (x_m - x_min) / (x_max - x_min) * (
            plot_right - plot_left
        )
        v = plot_bottom - (y_m - y_min) / (y_max - y_min) * (
            plot_bottom - plot_top
        )
        return int(round(u)), int(round(v))

    all_points = np.concatenate(
        [points for _, points in frames if len(points)], axis=0
    )
    background = np.full((height, width, 3), 248, dtype=np.uint8)
    for x_m, y_m in all_points[::3]:
        if x_min <= x_m <= x_max and y_min <= y_m <= y_max:
            cv2.circle(background, pixel(x_m, y_m), 1, (205, 205, 205), -1)

    cv2.rectangle(
        background,
        (plot_left, plot_top),
        (plot_right, plot_bottom),
        (80, 80, 80),
        2,
    )
    for x_m in np.arange(-0.10, 0.051, 0.02):
        u, _ = pixel(x_m, y_min)
        cv2.line(
            background,
            (u, plot_top),
            (u, plot_bottom),
            (225, 225, 225),
            1,
        )
        cv2.putText(
            background,
            f'{x_m:.2f}',
            (u - 22, plot_bottom + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (50, 50, 50),
            1,
            cv2.LINE_AA,
        )
    for y_m in np.arange(-0.32, -0.239, 0.01):
        _, v = pixel(x_min, y_m)
        cv2.line(
            background,
            (plot_left, v),
            (plot_right, v),
            (225, 225, 225),
            1,
        )
        cv2.putText(
            background,
            f'{y_m:.2f}',
            (10, v + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (50, 50, 50),
            1,
            cv2.LINE_AA,
        )

    output_fps = 10.0
    writer = cv2.VideoWriter(
        str(args.output),
        cv2.VideoWriter_fourcc(*'mp4v'),
        output_fps,
        (width, height),
    )
    if not writer.isOpened():
        raise SystemExit('OpenCV could not open the MP4 writer')

    colors = [(40, 40, 255), (255, 210, 0)]
    for frame_number, (time_s, points) in enumerate(frames):
        image = background.copy()
        cv2.putText(
            image,
            '50 mm ROSBAG - LIVE SCAN WITH FINAL RECOGNITION',
            (105, 45),
            cv2.FONT_HERSHEY_DUPLEX,
            1.05,
            (15, 15, 15),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f't={time_s:05.1f}s   frame={frame_number + 1}/{len(frames)}'
            f'   profile={profile}',
            (760, 78),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )

        for x_m, y_m in points:
            if x_min <= x_m <= x_max and y_min <= y_m <= y_max:
                cv2.circle(image, pixel(x_m, y_m), 3, (20, 20, 20), -1)

        overlay = image.copy()
        for index, target in enumerate(targets):
            color = colors[index % len(colors)]
            half_x = max(0.5 * target.span_x_m + 0.006, 0.0125)
            half_y = max(0.5 * target.span_y_m + 0.006, 0.0125)
            center = pixel(target.x_m, target.y_m)
            edge_x = pixel(target.x_m + half_x, target.y_m)[0]
            edge_y = pixel(target.x_m, target.y_m + half_y)[1]
            axes = (abs(edge_x - center[0]), abs(edge_y - center[1]))
            cv2.ellipse(overlay, center, axes, 0, 0, 360, color, -1)
        image = cv2.addWeighted(overlay, 0.16, image, 0.84, 0)

        for index, target in enumerate(targets):
            color = colors[index % len(colors)]
            half_x = max(0.5 * target.span_x_m + 0.006, 0.0125)
            half_y = max(0.5 * target.span_y_m + 0.006, 0.0125)
            for x_m, y_m in points:
                normalized = (
                    ((x_m - target.x_m) / half_x) ** 2
                    + ((y_m - target.y_m) / half_y) ** 2
                )
                if normalized <= 1.0:
                    cv2.circle(image, pixel(x_m, y_m), 5, color, -1)
            center = pixel(target.x_m, target.y_m)
            edge_x = pixel(target.x_m + half_x, target.y_m)[0]
            edge_y = pixel(target.x_m, target.y_m + half_y)[1]
            axes = (abs(edge_x - center[0]), abs(edge_y - center[1]))
            cv2.ellipse(image, center, axes, 0, 0, 360, color, 5)
            cv2.drawMarker(
                image,
                center,
                (0, 255, 255),
                cv2.MARKER_TILTED_CROSS,
                30,
                5,
            )
            label_y = 125 + index * 70
            cv2.rectangle(
                image,
                (850, label_y - 35),
                (1210, label_y + 22),
                color,
                -1,
            )
            cv2.putText(
                image,
                f'ID{index}: ({target.x_m:.3f}, {target.y_m:.3f}) m',
                (870, label_y),
                cv2.FONT_HERSHEY_DUPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.arrowedLine(
                image,
                (850, label_y),
                center,
                (0, 255, 255),
                4,
                tipLength=0.05,
            )

        writer.write(image)
    writer.release()
    print(args.output)


if __name__ == '__main__':
    main()
