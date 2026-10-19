#!/usr/bin/env python3
"""Rebuild the 2025 vision world around the externally supplied Gazebo models.

Object geometry is owned by RoboCup_MY2025_GPSR_URDF_Gazebo_Objects.zip. This
script only places those models and refreshes the derived catalog; it never
generates or modifies model assets.
"""

import argparse
import csv
import json
from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / 'worlds' / 'models'
WORLD = ROOT / 'worlds' / 'robocup_home_2025_vision.sdf'
CATALOG = ROOT / 'config' / 'robocup_malaysia_2025_objects.json'
SOURCE_ARCHIVE = 'RoboCup_MY2025_GPSR_URDF_Gazebo_Objects.zip'

# Category, object slug, display name, and reference image filename.
ITEMS = [
    ('drinks', 'cola', 'Cola', 'cola.jpg'),
    ('drinks', 'coffee', 'Coffee', 'coffee.jpg'),
    ('drinks', 'water', 'Water', 'water.jpg'),
    ('drinks', 'milk', 'Milk', 'milk.jpg'),
    ('food', 'cereal', 'Cereal', 'cereal.jpg'),
    ('food', 'tom_yam_cup_noodle', 'Tom-Yam Cup Noodle', 'cup_noodle-1.jpg'),
    ('food', 'yen_ta_four_cup_noodle', 'Yen-Ta-Four Cup Noodle', 'cup_noodle-2.jpg'),
    ('snacks', 'biscuit_roll', 'Biscuit Roll', 'biscuit-roll.jpg'),
    ('snacks', 'paprika', 'Paprika', 'paprika.jpg'),
    ('snacks', 'potae', 'Potae', 'potae.jpg'),
    ('snacks', 'cornae', 'Cornae', 'cornae.jpg'),
    ('snacks', 'pringles', 'Pringles', 'pringles.jpg'),
    ('snacks', 'lay_stax', 'Lay Stax', 'lay-stax.jpg'),
    ('snacks', 'pocky', 'Pocky', 'pocky.jpg'),
    ('tableware', 'spoon', 'Spoon', 'spoon.jpg'),
    ('tableware', 'plate', 'Plate', 'plate.jpg'),
    ('tableware', 'cup', 'Cup', 'cup.jpg'),
    ('tableware', 'mug', 'Mug', 'mug.jpg'),
    ('tableware', 'fork', 'Fork', 'fork.jpg'),
    ('tableware', 'bowl', 'Bowl', 'bowl.jpg'),
]

# All supplied models use an origin at the bottom centre.
PLACEMENTS = {
    'cola': (2.05, -.39, .75, 0), 'coffee': (-1.83, -1.48, .473, 0),
    'water': (2.55, -.39, .75, 0), 'milk': (2.30, -.39, .75, 0),
    'cereal': (.65, 2.53, 1.305, 0),
    'tom_yam_cup_noodle': (.89, 2.53, 1.305, 0),
    'yen_ta_four_cup_noodle': (1.13, 2.53, 1.305, 0),
    'biscuit_roll': (1.46, 2.45, 1.065, 0),
    'paprika': (1.65, 2.45, 1.065, 0), 'potae': (1.84, 2.45, 1.065, 0),
    'cornae': (2.025, 2.45, 1.065, 0), 'pringles': (1.48, 2.69, 1.065, 0),
    'lay_stax': (1.73, 2.69, 1.065, 0), 'pocky': (1.97, 2.69, 1.065, 0),
    'plate': (2.10, .63, .745, 0), 'bowl': (2.39, .63, .745, 0),
    'cup': (2.69, .63, .745, 0), 'mug': (2.13, .91, .745, 0),
    'spoon': (2.40, .91, .745, 0), 'fork': (2.69, .91, .745, 0),
}


def model_properties(model_name):
    """Read mass and proxy dimensions directly from one supplied model SDF."""
    root = ET.parse(MODELS / model_name / 'model.sdf').getroot()
    mass = float(root.findtext('.//inertial/mass'))
    geometry = root.find('.//collision/geometry')
    box = geometry.find('box/size')
    if box is not None:
        dimensions = [float(value) for value in box.text.split()]
    else:
        cylinder = geometry.find('cylinder')
        radius = float(cylinder.findtext('radius'))
        dimensions = [2.0 * radius, 2.0 * radius, float(cylinder.findtext('length'))]
    return mass, dimensions


def make_world():
    root = ET.parse(ROOT / 'worlds' / 'robocup_home_training.sdf')
    world = root.find('world')
    world.set('name', 'robocup_home_2025_vision')

    light = ET.SubElement(world, 'light', {'type': 'point', 'name': 'indoor_diffuse_fill'})
    for tag, value in [('pose', '0 0 3 0 0 0'), ('diffuse', '.85 .86 .88 1')]:
        ET.SubElement(light, tag).text = value
    attenuation = ET.SubElement(light, 'attenuation')
    for tag, value in [('range', '18'), ('constant', '0.3'),
                       ('linear', '0.04'), ('quadratic', '0.007')]:
        ET.SubElement(attenuation, tag).text = value

    for _category, slug, _title, _photo in ITEMS:
        model_name = f'robocup25_{slug}'
        if not (MODELS / model_name / 'model.sdf').is_file():
            raise FileNotFoundError(f'Missing supplied Gazebo model: {model_name}')
        x, y, z, yaw = PLACEMENTS[slug]
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = f'model://{model_name}'
        ET.SubElement(include, 'name').text = f'object_2025_{slug}'
        ET.SubElement(include, 'pose').text = f'{x:.5f} {y:.5f} {z:.5f} 0 0 {yaw:.5f}'

    from furniture_details import upgrade_furniture
    detailed = upgrade_furniture(world)
    print(f'Upgraded {len(detailed)} furniture models with detailed primitives')
    root.write(WORLD, encoding='utf-8', xml_declaration=True)


def make_catalog():
    records = []
    for category, slug, title, photo in ITEMS:
        model_name = f'robocup25_{slug}'
        mass, dimensions = model_properties(model_name)
        records.append({
            'id': slug, 'official_name': title, 'category': category,
            'model': model_name, 'image_source': photo,
            'object_dimensions_m': dimensions,
            'dimensions_source': 'supplied Gazebo collision proxy',
            'world_pose_xyz_yaw': list(PLACEMENTS[slug]),
            'appearance': 'SUPPLIED_STL_VISUALS', 'mass_kg': mass,
        })
    CATALOG.write_text(json.dumps({
        'source_archive': SOURCE_ARCHIVE,
        'note': '20 supplied RoboCup Malaysia Open 2025 GPSR Gazebo models',
        'items': records,
    }, indent=2) + '\n', encoding='utf-8')
    with (ROOT / 'config' / '2025_item_poses.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['name', 'category', 'x', 'y', 'z_bottom', 'yaw', 'mesh_model'])
        for record in records:
            writer.writerow([record['id'], record['category'],
                             *record['world_pose_xyz_yaw'], record['model']])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world-only', action='store_true',
                        help='Compatibility option; supplied model files are always left untouched')
    parser.parse_args()
    make_world()
    make_catalog()
    print(f'Updated world and catalog for {len(ITEMS)} supplied object models: {WORLD}')


if __name__ == '__main__':
    main()
