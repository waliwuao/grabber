"""Offline recognition of stable target regions in a ROS 2 bag."""

import argparse
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

from .core import ProcessorConfig, scan_to_points
from .temporal_recognition import (
    TemporalGrid,
    arrangement_alignment_error,
    arrangement_line_equation,
    arrangement_spacing_metrics,
    compensate_polar_coordinates,
    compensate_target_centers,
    select_target_components,
    split_targets_have_plausible_parent,
)

CENTER_OFFSET_ALONG_M = [
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
]
CENTER_OFFSET_NORMAL_M = [
    0.01696429, 0.01449106, 0.02197713,
    0.01447321, 0.01695537, 0.01692860,
]
POLAR_RANGE_COEFFICIENTS_M = [
    0.00883056907398, 0.00494075146535,
    -0.00461366266983, -0.0000114361849171,
    0.03486982091, -0.0024647133478,
]
POLAR_ANGLE_COEFFICIENTS_RAD = [
    0.0898957868697, 0.0668936983864,
    0.037292691619, -0.115131471909,
    -0.108171945595, -0.0783418855768,
]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path, help='Bag directory containing metadata.yaml')
    parser.add_argument('--expected-count', type=int, required=True)
    parser.add_argument('--occupancy', type=float, default=0.15)
    parser.add_argument('--grid-mm', type=float, default=5.0)
    parser.add_argument('--max-span-mm', type=float, default=120.0)
    parser.add_argument(
        '--max-spacing-deviation',
        type=float,
        default=-1.0,
        help='negative disables hard spacing rejection',
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not (args.bag / 'metadata.yaml').exists():
        raise SystemExit(f'Invalid bag directory: {args.bag}')

    config = ProcessorConfig(
        range_calibration_scale=0.96370434,
        range_calibration_offset_m=-0.00795114,
        range_calibration_cos_m=-0.00072278,
        range_calibration_sin_m=-0.00192741,
        range_calibration_cos2_m=-0.00036139,
    )
    grids = [
        (
            'standard',
            TemporalGrid(resolution_m=args.grid_mm / 1000.0),
            args.occupancy,
            2,
        ),
        (
            'near_split',
            TemporalGrid(resolution_m=0.003),
            0.60,
            2,
        ),
        (
            'ultra_near_split',
            TemporalGrid(resolution_m=0.002),
            0.50,
            1,
        ),
    ]
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
        for _, grid, _, _ in grids:
            grid.add_frame(points)

    selected_profile = 'none'
    components = []
    targets = []
    standard_components = []
    profile_solutions = []
    profile_penalties = {
        'standard': 0.0,
        'near_split': 0.0005,
        'ultra_near_split': 0.0010,
    }
    for profile, grid, occupancy, connection_radius in grids:
        profile_components = grid.stable_components(
            minimum_occupancy=occupancy,
            connection_radius_cells=connection_radius,
        )
        if profile == 'standard':
            standard_components = profile_components
        profile_targets = select_target_components(
            profile_components,
            expected_count=args.expected_count,
            maximum_component_span_m=args.max_span_mm / 1000.0,
            maximum_spacing_deviation_ratio=args.max_spacing_deviation,
            alignment_axis='auto',
        )
        if (
            profile in ('near_split', 'ultra_near_split')
            and profile_targets
            and not split_targets_have_plausible_parent(
                standard_components,
                profile_targets,
                args.expected_count,
                args.max_span_mm / 1000.0,
            )
        ):
            continue
        if len(profile_targets) == args.expected_count:
            score = (
                arrangement_alignment_error(profile_targets)
                + profile_penalties[profile]
            )
            profile_solutions.append(
                (score, profile, profile_components, profile_targets)
            )
    if profile_solutions:
        _, selected_profile, components, targets = min(profile_solutions)
    raw_targets = targets
    if len(raw_targets) == len(CENTER_OFFSET_ALONG_M):
        targets = compensate_target_centers(
            raw_targets,
            CENTER_OFFSET_ALONG_M,
            CENTER_OFFSET_NORMAL_M,
        )
    if targets:
        targets = compensate_polar_coordinates(
            targets,
            POLAR_RANGE_COEFFICIENTS_M,
            POLAR_ANGLE_COEFFICIENTS_RAD,
        )
    line = arrangement_line_equation(targets) if len(targets) >= 2 else None
    spacing = (
        arrangement_spacing_metrics(targets) if len(targets) >= 2 else None
    )
    payload = {
        'bag': str(args.bag),
        'frames': grids[0][1].frame_count,
        'profile': selected_profile,
        'stable_component_count': len(components),
        'expected_count': args.expected_count,
        'recognized_count': len(targets),
        'arrangement_line_2d': (
            {
                'form': 'a*x + b*y + c = 0',
                'a': round(line[0], 6),
                'b': round(line[1], 6),
                'c': round(line[2], 6),
            }
            if line is not None else None
        ),
        'mean_adjacent_spacing_m': (
            round(spacing[0], 6) if spacing is not None else None
        ),
        'maximum_spacing_deviation_ratio': (
            round(spacing[1], 4) if spacing is not None else None
        ),
        'targets': [
            {
                'id': index,
                'x_m': round(target.x_m, 4),
                'y_m': round(target.y_m, 4),
                'range_m': round(target.range_m, 4),
                'raw_component_x_m': round(raw_target.x_m, 4),
                'raw_component_y_m': round(raw_target.y_m, 4),
                'span_x_m': round(target.span_x_m, 4),
                'span_y_m': round(target.span_y_m, 4),
                'peak_occupancy': round(target.peak_occupancy, 3),
            }
            for index, (raw_target, target) in enumerate(
                zip(raw_targets, targets)
            )
        ],
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))
