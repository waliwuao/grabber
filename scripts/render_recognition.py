#!/usr/bin/env python3
"""Render an offline recognition result over accumulated LaserScan points."""

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--expected-count', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--highlight-output',
        type=Path,
        help='Optional high-contrast close-up of recognized rosbag points',
    )
    args = parser.parse_args()

    config = ProcessorConfig()
    profiles = [
        ('standard', TemporalGrid(0.005), 0.15, 2),
        ('near_split', TemporalGrid(0.003), 0.60, 2),
    ]
    xs = []
    ys = []
    frame_index = 0
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    while reader.has_next():
        topic, data, _ = reader.read_next()
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
        for _, grid, _, _ in profiles:
            grid.add_frame(points)
        if frame_index % 5 == 0:
            xs.extend(point.x_m for point in points)
            ys.extend(point.y_m for point in points)
        frame_index += 1

    standard_components = []
    selected_profile = 'none'
    targets = []
    for profile, grid, occupancy, connection_radius in profiles:
        components = grid.stable_components(
            minimum_occupancy=occupancy,
            connection_radius_cells=connection_radius,
        )
        if profile == 'standard':
            standard_components = components
        candidates = select_target_components(
            components,
            expected_count=args.expected_count,
            maximum_component_span_m=0.12,
        )
        if (
            profile == 'near_split'
            and candidates
            and not split_targets_have_plausible_parent(
                standard_components, candidates, args.expected_count, 0.12
            )
        ):
            continue
        if len(candidates) == args.expected_count:
            selected_profile = profile
            targets = candidates
            break

    figure, axes = plt.subplots(1, 2, figsize=(14, 7), dpi=160)
    full_axis, zoom_axis = axes
    for axis in axes:
        axis.scatter(
            xs, ys, s=3, c='#8a949e', alpha=0.14, label='ROI scan points'
        )
    colors = ['#ff3030', '#ff8c00', '#00a6ff', '#8f4cff', '#00a85a', '#d400a5']
    for index, target in enumerate(targets):
        color = colors[index % len(colors)]
        for axis in axes:
            ellipse = Ellipse(
                (target.x_m, target.y_m),
                max(target.span_x_m + 0.012, 0.025),
                max(target.span_y_m + 0.012, 0.025),
                fill=False,
                edgecolor=color,
                linewidth=2.5,
            )
            axis.add_patch(ellipse)
            axis.scatter(
                [target.x_m],
                [target.y_m],
                s=70,
                c=color,
                marker='x',
                linewidths=2.5,
            )
            text_offset = (-58, 18) if index == 0 else (12, -42)
            axis.annotate(
                f'ID{index}\n({target.x_m:.3f}, {target.y_m:.3f}) m',
                (target.x_m, target.y_m),
                xytext=text_offset,
                textcoords='offset points',
                color=color,
                fontsize=9,
                weight='bold',
                bbox=dict(facecolor='white', alpha=0.82, edgecolor=color),
            )

    for axis in axes:
        axis.scatter(
            [0.0], [0.0], s=110, marker='*', c='#0066cc', label='LiDAR'
        )
        axis.axhline(0.0, color='#57a773', linewidth=1, alpha=0.65)
        axis.axvline(0.0, color='#d9534f', linewidth=1, alpha=0.65)
        axis.set_aspect('equal', adjustable='box')
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')
        axis.grid(True, alpha=0.22)

    full_axis.set_xlim(-0.43, 0.43)
    full_axis.set_ylim(-0.43, 0.02)
    full_axis.set_title('Full ROI')
    full_axis.legend(loc='lower right')
    zoom_axis.set_xlim(-0.09, 0.045)
    zoom_axis.set_ylim(-0.325, -0.245)
    zoom_axis.set_title('Recognized area close-up')
    figure.suptitle(
        f'50 mm recording: {len(targets)} targets, profile={selected_profile}'
    )
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output)
    print(args.output)

    if args.highlight_output:
        highlight_figure, highlight_axis = plt.subplots(
            figsize=(12, 7), dpi=180
        )
        highlight_axis.scatter(
            xs,
            ys,
            s=7,
            c='#151515',
            alpha=0.18,
            label='Raw rosbag scan points',
        )
        highlight_colors = ['#ff1744', '#00d9ff']
        for index, target in enumerate(targets):
            color = highlight_colors[index % len(highlight_colors)]
            half_x = max(0.5 * target.span_x_m + 0.006, 0.0125)
            half_y = max(0.5 * target.span_y_m + 0.006, 0.0125)
            selected_x = []
            selected_y = []
            for x_m, y_m in zip(xs, ys):
                normalized = (
                    ((x_m - target.x_m) / half_x) ** 2
                    + ((y_m - target.y_m) / half_y) ** 2
                )
                if normalized <= 1.0:
                    selected_x.append(x_m)
                    selected_y.append(y_m)
            highlight_axis.scatter(
                selected_x,
                selected_y,
                s=18,
                c=color,
                alpha=0.75,
                edgecolors='none',
                zorder=4,
            )
            ellipse = Ellipse(
                (target.x_m, target.y_m),
                2.0 * half_x,
                2.0 * half_y,
                facecolor=color,
                alpha=0.10,
                edgecolor=color,
                linewidth=5,
                zorder=5,
            )
            highlight_axis.add_patch(ellipse)
            highlight_axis.scatter(
                [target.x_m],
                [target.y_m],
                s=280,
                c='#ffff00',
                marker='x',
                linewidths=5,
                zorder=7,
            )
            label_x = target.x_m + (-0.018 if index == 0 else 0.020)
            label_y = -0.253 if index == 0 else -0.313
            highlight_axis.annotate(
                (
                    f'ID{index}  RECOGNIZED HERE\n'
                    f'center=({target.x_m:.3f}, {target.y_m:.3f}) m'
                ),
                xy=(target.x_m, target.y_m),
                xytext=(label_x, label_y),
                fontsize=16,
                weight='bold',
                color='#ffffff',
                ha='center',
                va='center',
                bbox=dict(
                    boxstyle='round,pad=0.5',
                    facecolor=color,
                    edgecolor='#ffff00',
                    linewidth=3,
                    alpha=0.96,
                ),
                arrowprops=dict(
                    arrowstyle='-|>',
                    color='#ffff00',
                    linewidth=4,
                    mutation_scale=24,
                ),
                zorder=8,
            )

        highlight_axis.set_xlim(-0.105, 0.055)
        highlight_axis.set_ylim(-0.33, -0.24)
        highlight_axis.set_aspect('equal', adjustable='box')
        highlight_axis.set_facecolor('#f7f7f7')
        highlight_axis.set_xlabel('X (m)', fontsize=13)
        highlight_axis.set_ylabel('Y (m)', fontsize=13)
        highlight_axis.set_title(
            '50 mm ROSBAG — FINAL RECOGNIZED REGIONS',
            fontsize=18,
            weight='bold',
        )
        highlight_axis.grid(True, color='#999999', alpha=0.25)
        highlight_figure.tight_layout()
        args.highlight_output.parent.mkdir(parents=True, exist_ok=True)
        highlight_figure.savefig(args.highlight_output)
        print(args.highlight_output)


if __name__ == '__main__':
    main()
