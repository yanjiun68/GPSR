#!/usr/bin/env python3

import argparse
import math
import os
import sys

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def resolve_default_locations_file() -> str:
    pkg_name = 'my_robot_nav'

    try:
        pkg_share = get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        return ''

    install_file = os.path.join(pkg_share, 'config', 'semantic_locations.yaml')
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))
    )
    source_file = os.path.join(
        workspace_root,
        'src',
        pkg_name,
        'config',
        'semantic_locations.yaml',
    )

    if os.path.isfile(source_file):
        return source_file

    return install_file


def parse_args():
    parser = argparse.ArgumentParser(
        description='Add or update a semantic navigation location offline.'
    )
    parser.add_argument('name', help='Semantic location name, for example kitchen')
    parser.add_argument('x', type=float, help='Map-frame X coordinate in meters')
    parser.add_argument('y', type=float, help='Map-frame Y coordinate in meters')
    parser.add_argument(
        '--yaw',
        type=float,
        default=None,
        help='Yaw in radians',
    )
    parser.add_argument(
        '--yaw-deg',
        type=float,
        default=None,
        help='Yaw in degrees',
    )
    parser.add_argument(
        '--description',
        default='',
        help='Optional human-readable description',
    )
    parser.add_argument(
        '--frame-id',
        default='map',
        help='Frame id for the location, default: map',
    )
    parser.add_argument(
        '--file',
        default=resolve_default_locations_file(),
        help='Semantic locations YAML file to edit',
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Replace an existing location with the same name',
    )
    return parser.parse_args()


def load_locations(file_path: str):
    if os.path.exists(file_path):
        with open(file_path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError('Semantic locations YAML root must be a mapping.')
    else:
        data = {}

    if 'locations' not in data:
        data['locations'] = {}

    if not isinstance(data['locations'], dict):
        raise ValueError('"locations" must be a mapping in the YAML file.')

    return data


def main():
    args = parse_args()

    if not args.file:
        print('Could not resolve a semantic locations file path.', file=sys.stderr)
        return 1

    if args.yaw is not None and args.yaw_deg is not None:
        print('Use either --yaw or --yaw-deg, not both.', file=sys.stderr)
        return 1

    yaw = args.yaw if args.yaw is not None else math.radians(args.yaw_deg or 0.0)

    data = load_locations(args.file)
    locations = data['locations']

    if args.name in locations and not args.overwrite:
        print(
            f'Location "{args.name}" already exists. Use --overwrite to replace it.',
            file=sys.stderr,
        )
        return 1

    location_entry = {
        'x': round(args.x, 4),
        'y': round(args.y, 4),
        'yaw': round(yaw, 4),
        'yaw_deg': round(math.degrees(yaw), 1),
        'frame_id': args.frame_id,
    }

    if args.description:
        location_entry['description'] = args.description

    locations[args.name] = location_entry

    parent_dir = os.path.dirname(args.file)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    with open(args.file, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(
            data,
            stream,
            sort_keys=False,
            allow_unicode=False,
        )

    print(
        f'Saved semantic location "{args.name}" to {args.file}: '
        f'x={location_entry["x"]:.4f}, y={location_entry["y"]:.4f}, '
        f'yaw={location_entry["yaw"]:.4f} rad ({location_entry["yaw_deg"]:.1f} deg)'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
