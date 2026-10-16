#!/usr/bin/env python3
"""Generate a self-contained SDF from the traced official Incheon 2026 plan.

No online meshes are required. Furniture geometry and metric scale are estimates.
Run with --config and --output; see world/ROBOCUP2026.md for provenance.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def numbers(values):
    return ' '.join(f'{v:.8g}' for v in values)


WOOD = '0.40 0.25 0.12 1'
WHITE = '0.88 0.89 0.87 1'
DARK = '0.09 0.11 0.13 1'
METAL = '0.50 0.56 0.59 1'


def model(world, name, x, y, yaw=0):
    item = element(world, 'model', name=name)
    element(item, 'static', 'true')
    element(item, 'pose', numbers([x, y, 0, 0, 0, yaw]))
    return element(item, 'link', name='body')


def shape(link, name, size, xyz, color=WOOD, kind='box', collision=True, rpy=(0, 0, 0)):
    for tag in (('collision', 'visual') if collision else ('visual',)):
        part = element(link, tag, name=f'{name}_{tag}')
        element(part, 'pose', numbers([*xyz, *rpy]))
        geometry = element(element(part, 'geometry'), kind)
        if kind == 'box':
            element(geometry, 'size', numbers(size))
        elif kind == 'cylinder':
            element(geometry, 'radius', size[0])
            element(geometry, 'length', size[1])
        elif kind == 'sphere':
            element(geometry, 'radius', size[0])
        if tag == 'visual':
            material = element(part, 'material')
            element(material, 'ambient', color)
            element(material, 'diffuse', color)


def inertia(link, mass, size):
    inertial = element(link, 'inertial')
    element(inertial, 'mass', mass)
    tensor = element(inertial, 'inertia')
    x, y, z = size
    for name, value in [('ixx', mass * (y*y + z*z) / 12),
                        ('iyy', mass * (x*x + z*z) / 12),
                        ('izz', mass * (x*x + y*y) / 12),
                        ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        element(tensor, name, value)


def dynamic_model(world, name, xyz, mass, size):
    item = element(world, 'model', name=name)
    element(item, 'pose', numbers([*xyz, 0, 0, 0]))
    element(item, 'static', 'false')
    link = element(item, 'link', name='body')
    inertia(link, mass, size)
    return item, link


def ring(link, name, radius, depth, thickness, centre, color, axis='y', segments=32):
    """Segmented hollow ring: no invisible solid-cylinder collision in the opening."""
    for i in range(segments):
        angle = i * 2 * math.pi / segments
        if axis == 'y':
            pos = [centre[0] + radius*math.cos(angle), centre[1], centre[2] + radius*math.sin(angle)]
            size = [2*math.pi*radius/segments*1.05, depth, thickness]
            rpy = (0, -angle-math.pi/2, 0)
        else:
            pos = [centre[0] + radius*math.cos(angle), centre[1] + radius*math.sin(angle), centre[2]]
            size = [thickness, 2*math.pi*radius/segments*1.05, depth]
            rpy = (0, 0, angle)
        shape(link, f'{name}_{i}', size, pos, color, rpy=rpy)


def washing_machine(world, spec, xy, scale):
    x, y = xy(*spec['pixel'])
    w, d = [v * scale for v in spec['size_pixels']]
    h = spec['height']
    item, body = dynamic_model(world, spec['name'], [x, y, 0], 40, [w, d, h])
    # Cabinet shell; the front faces south (-Y), toward free space in the laundry.
    for side in (-1, 1):
        shape(body, f'side_{side}', [0.025, d, h], [side*(w/2-0.0125), 0, h/2], WHITE)
    shape(body, 'top', [w, d, 0.025], [0, 0, h-0.0125], WHITE)
    shape(body, 'bottom', [w, d, 0.08], [0, 0, 0.04], WHITE)
    shape(body, 'rear', [w, 0.025, h], [0, d/2-0.0125, h/2], WHITE)
    shape(body, 'upper_fascia', [w, 0.035, 0.14], [0, -d/2, h-0.07], WHITE)
    shape(body, 'lower_fascia', [w, 0.035, 0.14], [0, -d/2, 0.07], WHITE)
    ring(body, 'front_rim', 0.267, 0.045, 0.035, [0, -d/2, 0.43], METAL)
    shape(body, 'detergent_drawer', [0.20, 0.015, 0.08], [-0.17, -d/2-0.025, h-0.07], METAL)
    shape(body, 'display', [0.15, 0.015, 0.075], [0.07, -d/2-0.025, h-0.07], DARK)
    shape(body, 'dial', [0.032, 0.025], [0.23, -d/2-0.04, h-0.07], METAL,
          kind='cylinder', rpy=(math.pi/2, 0, 0))
    # Accessible side-panel buttons; contact sensors also accept a robot's press.
    shape(body, 'side_panel', [0.018, 0.36, 0.14], [w/2+0.009, -0.07, h-0.11], DARK)
    for name, py, color in [('power', -0.20, '0.2 0.8 0.3 1'),
                            ('start', -0.07, '0.2 0.5 0.95 1'),
                            ('door', 0.06, '0.95 0.6 0.1 1')]:
        shape(body, f'button_{name}', [0.025, 0.045, 0.045], [w/2+0.03, py, h-0.11], color)
        sensor = element(body, 'sensor', name=f'{name}_contact', type='contact')
        element(sensor, 'always_on', 'true')
        element(sensor, 'update_rate', '20')
        element(sensor, 'topic', f'/washing_machine/buttons/{name}')
        element(element(sensor, 'contact'), 'collision', f'button_{name}_collision')
    fixed = element(item, 'joint', name='cabinet_anchor', type='fixed')
    element(fixed, 'parent', 'world'); element(fixed, 'child', 'body')

    drum = element(item, 'link', name='drum')
    element(drum, 'pose', '0 -0.01 0.43 0 0 0')
    inertia(drum, 4, [0.49, 0.46, 0.49])
    ring(drum, 'drum_wall', 0.245, 0.46, 0.012, [0, 0, 0], METAL)
    shape(drum, 'drum_back', [0.247, 0.012], [0, 0.236, 0], METAL,
          kind='cylinder', rpy=(math.pi/2, 0, 0))
    drum_joint = element(item, 'joint', name='drum_joint', type='revolute')
    element(drum_joint, 'parent', 'body'); element(drum_joint, 'child', 'drum')
    axis = element(drum_joint, 'axis'); element(axis, 'xyz', '0 1 0')
    element(element(axis, 'dynamics'), 'damping', '0.1')

    door = element(item, 'link', name='door')
    element(door, 'pose', numbers([-0.275, -d/2-0.04, 0.43, 0, 0, 0]))
    inertia(door, 1.5, [0.55, 0.05, 0.55])
    ring(door, 'door_rim', 0.252, 0.045, 0.035, [0.275, 0, 0], DARK)
    shape(door, 'window', [0.234, 0.012], [0.275, 0, 0], '0.45 0.7 0.85 0.35',
          kind='cylinder', rpy=(math.pi/2, 0, 0))
    # Three bars form a graspable handle rather than a solid decorative block.
    for z in (-0.055, 0.055):
        shape(door, f'handle_mount_{z}', [0.025, 0.07, 0.02], [0.51, -0.045, z], METAL)
    shape(door, 'handle_grip', [0.025, 0.025, 0.13], [0.51, -0.08, 0], METAL)
    hinge = element(item, 'joint', name='door_hinge', type='revolute')
    element(hinge, 'parent', 'body'); element(hinge, 'child', 'door')
    axis = element(hinge, 'axis'); element(axis, 'xyz', '0 0 -1')
    limits = element(axis, 'limit')
    element(limits, 'lower', '0'); element(limits, 'upper', '1.9')
    element(limits, 'effort', '15'); element(limits, 'velocity', '0.6')
    element(element(axis, 'dynamics'), 'damping', '0.6')
    controller = element(item, 'plugin', filename='ignition-gazebo-joint-position-controller-system',
                         name='ignition::gazebo::systems::JointPositionController')
    element(controller, 'joint_name', 'door_hinge')
    element(controller, 'topic', '/washing_machine/door_target')
    element(controller, 'p_gain', '8'); element(controller, 'd_gain', '2')
    element(controller, 'cmd_max', '3'); element(controller, 'cmd_min', '-3')
    spin = element(item, 'plugin', filename='ignition-gazebo-joint-controller-system',
                   name='ignition::gazebo::systems::JointController')
    element(spin, 'joint_name', 'drum_joint')
    element(spin, 'topic', '/washing_machine/drum_speed')
    states = element(item, 'plugin', filename='ignition-gazebo-joint-state-publisher-system',
                     name='ignition::gazebo::systems::JointStatePublisher')
    element(states, 'joint_name', 'door_hinge'); element(states, 'joint_name', 'drum_joint')
    element(states, 'topic', '/washing_machine/joint_states')


def task_objects(world, config, xy, scale):
    locations = {f['name']: xy(*f['pixel']) for f in config['furniture']}
    wx, wy = locations['washing_machine']
    locations['laundry_basket'] = (wx+0.9, wy-0.8)
    _, basket = dynamic_model(world, 'laundry_basket', [wx+0.9, wy-0.8, 0], 1.2, [0.65, 0.5, 0.35])
    shape(basket, 'bottom', [0.65, 0.5, 0.035], [0, 0, 0.04], '0.65 0.78 0.87 1')
    for sx in (-1, 1):
        shape(basket, f'end_{sx}', [0.02, 0.5, 0.28], [sx*0.315, 0, 0.19], '0.65 0.78 0.87 1')
        shape(basket, f'handle_{sx}', [0.025, 0.23, 0.025], [sx*0.315, 0, 0.40], METAL)
        for sy in (-1, 1):
            shape(basket, f'handle_mount_{sx}_{sy}', [0.025, 0.025, 0.08], [sx*0.315, sy*0.10, 0.36], METAL)
    for sy in (-1, 1):
        shape(basket, f'side_{sy}', [0.65, 0.02, 0.28], [0, sy*0.24, 0.19], '0.65 0.78 0.87 1')
    for spec in config['task_objects']:
        x, y = locations[spec['location']]
        dx, dy, z = spec['offset']
        kind = spec['kind']
        defaults = {'shirt':[0.16,0.16,0.035], 'towel':[0.2,0.15,0.035],
                    'cube':[0.057]*3, 'fruit':[0.08]*3, 'pepper':[0.09,0.09,0.11],
                    'bottle':[0.065,0.065,0.23], 'can':[0.065,0.065,0.12],
                    'cup':[0.08,0.08,0.10], 'bowl':[0.13,0.13,0.07],
                    'plate':[0.17,0.17,0.018], 'spoon':[0.035,0.15,0.008],
                    'fork':[0.025,0.15,0.008], 'knife':[0.025,0.17,0.008]}
        size = spec.get('size', defaults.get(kind, [0.1,0.1,0.1]))
        _, link = dynamic_model(world, spec['name'], [x+dx, y+dy, z], spec.get('mass', 0.15), size)
        color = spec.get('color', WHITE)
        if kind in ('shirt', 'towel'):
            shape(link, 'folded_fabric', size, [0,0,0], color)
            if kind == 'shirt':
                for side in (-1,1):
                    shape(link, f'sleeve_{side}', [0.045,0.065,0.018], [side*0.094,0.04,0], color)
            # Thin tab can be pinched by a parallel gripper; not fixed to the drum.
            shape(link, 'grasp_tab', [0.05,0.04,0.018], [0,-size[1]/2-0.015,0.01], color)
        elif kind in ('fruit','pepper'):
            shape(link, 'fruit', [size[0]/2], [0,0,0], color, kind='sphere')
            shape(link, 'stem', [0.008,0.025], [0,0,size[0]/2], '0.12 0.32 0.08 1', kind='cylinder')
        elif kind in ('can','bottle'):
            shape(link, 'container', [size[0]/2,size[2]*0.8], [0,0,-size[2]*0.1], color, kind='cylinder')
            shape(link, 'cap', [size[0]*(0.25 if kind=='bottle' else 0.48),size[2]*0.2],
                  [0,0,size[2]*0.4], METAL, kind='cylinder')
            shape(link, 'label', [size[0]*0.8,0.002,size[2]*0.35], [0,-size[0]/2-0.001,0], WHITE, collision=False)
        elif kind in ('cup','bowl','plate'):
            radius, height = size[0]/2, size[2]
            shape(link, 'base', [radius,0.01], [0,0,-height/2+0.005], color, kind='cylinder')
            ring(link, 'rim', radius-0.003, height, 0.006, [0,0,0], color, axis='z', segments=16)
            if kind=='cup':
                for zz in (-0.03,0.03):
                    shape(link, f'handle_{zz}', [0.04,0.013,0.013], [radius+0.017,0,zz], color)
                shape(link, 'handle_grip', [0.013,0.013,0.07], [radius+0.037,0,0], color)
        elif kind in ('spoon','knife','fork'):
            shape(link, 'handle', [0.012,size[1]*0.7,0.007], [0,-size[1]*0.15,0], METAL)
            if kind=='fork':
                for i in range(3):
                    shape(link, f'tine_{i}', [0.004,0.045,0.004], [(i-1)*0.008,size[1]*0.35,0], METAL)
            else:
                shape(link, 'head', [size[0],size[1]*0.35,0.006], [0,size[1]*0.33,0], METAL)
        elif kind=='cube':
            shape(link, 'cube', size, [0,0,0], DARK)
            for i in range(3):
                for j in range(3):
                    shape(link, f'tile_{i}_{j}', [0.016,0.016,0.001], [(i-1)*0.019,(j-1)*0.019,0.029],
                          ['0.9 0.1 0.1 1','0.05 0.6 0.2 1','0.95 0.85 0.1 1'][(i+j)%3], collision=False)
        else:
            shape(link, 'package', size, [0,0,0], color)


def furniture(world, spec, xy, scale):
    if spec['name'] == 'washing_machine':
        washing_machine(world, spec, xy, scale)
        return
    x, y = xy(*spec['pixel'])
    w, d = [v * scale for v in spec['size_pixels']]
    h, kind = spec['height'], spec['type']
    link = model(world, spec['name'], x, y, spec.get('yaw', 0))
    if kind in ('table', 'chair', 'round_table'):
        top_z = h if kind != 'chair' else 0.45
        if kind == 'round_table':
            shape(link, 'top', [w / 2, 0.06], [0, 0, top_z - 0.03], kind='cylinder')
            shape(link, 'pedestal', [0.10, top_z - 0.06], [0, 0, (top_z - 0.06) / 2], kind='cylinder')
            shape(link, 'foot', [0.32, 0.06], [0, 0, 0.03], kind='cylinder')
        else:
            shape(link, 'top', [w, d, 0.06], [0, 0, top_z - 0.03], DARK if kind == 'chair' else WOOD)
            for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
                shape(link, f'leg_{i}', [0.06, 0.06, top_z - 0.06],
                      [sx * (w / 2 - 0.05), sy * (d / 2 - 0.05), (top_z - 0.06) / 2], DARK)
        if kind == 'chair':
            shape(link, 'back', [w, 0.06, h - 0.45], [0, -d / 2 + 0.03, (h + 0.45) / 2], DARK)
    elif kind == 'sofa':
        shape(link, 'base', [w, d, 0.32], [0, 0, 0.16], METAL)
        shape(link, 'back', [w, 0.18, h - 0.28], [0, -d / 2 + 0.09, (h + 0.28) / 2], WHITE)
        for side in (-1, 1):
            shape(link, f'arm_{side}', [0.17, d, 0.55], [side * (w / 2 - 0.085), 0, 0.275], WHITE)
        for i in range(3):
            shape(link, f'cushion_{i}', [(w - 0.38) / 3, d - 0.22, 0.14],
                  [(i - 1) * (w - 0.34) / 3, 0.08, 0.39], WHITE)
    elif kind == 'bed':
        shape(link, 'frame', [w, d, h - 0.18], [0, 0, (h - 0.18) / 2])
        shape(link, 'mattress', [w - 0.05, d - 0.04, 0.18], [0, 0, h - 0.09], WHITE)
        shape(link, 'headboard', [0.08, d, h + 0.2], [-w / 2 + 0.04, 0, (h + 0.2) / 2])
        shape(link, 'pillow', [0.38, d * 0.7, 0.09], [-w / 2 + 0.35, 0, h + 0.045], WHITE)
    elif kind in ('bin', 'plant', 'coat_rack'):
        radius = min(w, d) / 2
        if kind == 'bin':
            shape(link, 'bin', [radius, h], [0, 0, h / 2], METAL, kind='cylinder')
            shape(link, 'lid', [radius * 0.84, 0.015], [0, 0, h + 0.0075], DARK, kind='cylinder')
        elif kind == 'plant':
            shape(link, 'pot', [radius, h * 0.4], [0, 0, h * 0.2], WOOD, kind='cylinder')
            shape(link, 'leaves', [radius * 1.1], [0, 0, h * 0.65], '0.12 0.36 0.12 1', kind='sphere')
        else:
            shape(link, 'foot', [radius, 0.06], [0, 0, 0.03], WOOD, kind='cylinder')
            shape(link, 'pole', [0.035, h], [0, 0, h / 2], WOOD, kind='cylinder')
            shape(link, 'hooks', [w, 0.05, 0.05], [0, 0, h - 0.12])
    elif kind == 'cabinet':
        # Open storage facing +X, toward the kitchen, with an accessible drinks shelf.
        shape(link, 'back', [0.025,d,h], [-w/2+0.0125,0,h/2], WHITE)
        for side in (-1,1):
            shape(link, f'side_{side}', [w,0.025,h], [0,side*(d/2-0.0125),h/2], WHITE)
        for i, z in enumerate((0.025,0.7,h-0.02)):
            shape(link, f'board_{i}', [w,d,0.035], [0,0,z], WHITE)
    elif kind == 'shelf':
        for side in (-1, 1):
            shape(link, f'side_{side}', [0.04, d, h], [side * (w / 2 - 0.02), 0, h / 2], WHITE)
        shape(link, 'back', [w, 0.035, h], [0, -d / 2 + 0.0175, h / 2], WHITE)
        for i in range(4):
            shape(link, f'board_{i}', [w, d, 0.035], [0, 0, 0.025 + i * (h - 0.05) / 3], WHITE)
    else:
        shape(link, 'body', [w, d, h], [0, 0, h / 2], WHITE if kind in ('washer', 'fridge', 'cabinet') else WOOD)
        if kind in ('counter', 'sink', 'cooker'):
            shape(link, 'worktop', [w + 0.015, d + 0.015, 0.04], [0, 0, h + 0.02], METAL)
        if kind == 'sink':
            shape(link, 'basin_visual', [w * 0.65, d * 0.65, 0.01], [0, 0, h + 0.045], DARK, collision=False)
            shape(link, 'tap', [0.025, 0.025, 0.2], [0, -d * 0.36, h + 0.14], METAL)
        if kind == 'cooker':
            for i, dx in enumerate((-w * 0.24, w * 0.24)):
                shape(link, f'burner_{i}', [w * 0.17, 0.01], [dx, 0, h + 0.045], DARK, kind='cylinder', collision=False)
        if kind == 'tv':
            shape(link, 'screen', [w * 0.8, 0.07, 0.6], [0, 0, h + 0.3], DARK)
        if kind in ('fridge', 'washer', 'cabinet'):
            shape(link, 'handle', [min(0.035, w * 0.1), 0.04, h * 0.25],
                  [w * 0.32, d / 2 + 0.02, h * 0.63], DARK)


def generate(config):
    s = config['scale']
    scale = s['width_metres'] / s['width_pixels']
    def xy(px, py):
        return (px - s['west_pixel']) * scale, (s['south_pixel'] - py) * scale

    root = ET.Element('sdf', version='1.9')
    root.append(ET.Comment('Generated from official Incheon2026 diagrams; metric dimensions are assumptions. See ROBOCUP2026.md.'))
    world = element(root, 'world', name=config['world_name'])
    physics = element(world, 'physics', name='1ms', type='ignored')
    element(physics, 'max_step_size', '0.001')
    element(physics, 'real_time_factor', '1.0')
    for filename, classname in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                                ('scene-broadcaster', 'SceneBroadcaster'), ('contact', 'Contact'), ('sensors', 'Sensors')]:
        plugin = element(world, 'plugin', filename=f'ignition-gazebo-{filename}-system',
                         name=f'ignition::gazebo::systems::{classname}')
        if classname == 'Sensors':
            element(plugin, 'render_engine', 'ogre2')
    scene = element(world, 'scene')
    element(scene, 'ambient', '0.6 0.6 0.6 1')
    element(scene, 'background', '0.80 0.86 0.92 1')
    light = element(world, 'light', name='sun', type='directional')
    element(light, 'pose', '5 5 12 0 0 0')
    element(light, 'diffuse', '0.85 0.85 0.85 1')
    element(light, 'specular', '0.15 0.15 0.15 1')
    element(light, 'direction', '-0.4 0.2 -0.9')
    element(light, 'cast_shadows', 'true')
    attenuation = element(light, 'attenuation')
    element(attenuation, 'range', '100')
    element(attenuation, 'constant', '1')
    element(attenuation, 'linear', '0')
    element(attenuation, 'quadratic', '0')
    width = s['width_metres']
    shape(model(world, 'outside_floor', width / 2, width / 2), 'floor',
          [width + 8, width + 8, 0.10], [0, 0, -0.05], '0.35 0.39 0.42 1')
    # Visual overlays only: floor collision remains a single flat surface at z=0.
    for name, bounds, color in [
        ('living_room', (410, 236, 661, 525), '0.77 0.63 0.44 1'),
        ('laundry', (410, 21, 661, 236), '0.73 0.78 0.80 1'),
        ('bedroom', (661, 21, 914, 236), '0.75 0.67 0.58 1'),
        ('kitchen', (661, 236, 914, 525), '0.86 0.82 0.72 1'),
    ]:
        ax, ay = xy(bounds[0], bounds[1]); bx, by = xy(bounds[2], bounds[3])
        shape(model(world, name + '_floor', (ax + bx) / 2, (ay + by) / 2), 'finish',
              [abs(bx - ax), abs(by - ay), 0.001], [0, 0, -0.0004], color, collision=False)
    for name, px1, py1, px2, py2 in config['walls']:
        ax, ay = xy(px1, py1); bx, by = xy(px2, py2)
        length = math.hypot(bx - ax, by - ay)
        shape(model(world, 'wall_' + name, (ax + bx) / 2, (ay + by) / 2, math.atan2(by - ay, bx - ax)),
              'wall', [length, s['wall_thickness'], s['wall_height']],
              [0, 0, s['wall_height'] / 2], '0.89 0.88 0.83 1')
    # Living-room entrance, parked open inward against the north side of its gap.
    hinge_x, hinge_y = xy(410, 322)
    door_width = (374 - 322) * scale - 0.02
    door = model(world, 'entrance_door_open', hinge_x, hinge_y, math.pi / 2)
    shape(door, 'leaf', [0.04, door_width, s['wall_height']],
          [0, -door_width / 2, s['wall_height'] / 2], '0.25 0.45 0.59 1')
    for item in config['furniture']:
        furniture(world, item, xy, scale)
    task_objects(world, config, xy, scale)
    entry_x, entry_y = xy(*config['entrance']['pixel'])
    yaw = config['entrance']['inward_yaw']
    distance = config['entrance']['start_distance']
    # Frame consumed by robocup_arena.launch.py, so rescaling also updates spawn.
    frame = element(world, 'frame', name='arena_entrance', attached_to='world')
    element(frame, 'pose', numbers([entry_x, entry_y, 0, 0, 0, yaw]))
    marker = model(world, 'robot_start_marker', entry_x - distance * math.cos(yaw), entry_y - distance * math.sin(yaw), yaw)
    shape(marker, 'pad', [0.08, 0.08, 0.001], [0, 0, 0.001], '0.15 0.85 0.25 1', collision=False)
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    root = generate(config)
    ET.indent(root, space='  ')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(args.output, encoding='utf-8', xml_declaration=True)
    print(f'Generated {args.output}: {len(root.findall("world/model"))} models')


if __name__ == '__main__':
    main()
