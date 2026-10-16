#!/usr/bin/env python3
"""Generate the Home25 training arena from the supplied 6.8 x 6.3 m floor plan.

World coordinates: +X right, +Y towards the top of the drawing, +Z up.
All rectangle dimensions are metres.  The reference image shows an approximately
6.30 x 5.80 m CLEAR INTERIOR bounded by 0.25 m exterior walls; 0.90 m door gaps.
This is a best-fit dimensional reconstruction, not a CAD measurement.

Regenerate with:
  python3 scripts/generate.py
  python3 scripts/build_2025_vision_world.py --world-only
  python3 scripts/generate_vision_reference_map.py
"""
from pathlib import Path
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'worlds' / 'robocup_home_training.sdf'
MAP = ROOT / 'maps'
W, H = 6.80, 6.30
# Keep the original map extent for downstream Nav2 consumers (280x220 at 5 cm).
RES = 0.05
MAP_X0, MAP_Y0 = -7.0, -5.5
MAP_NX, MAP_NY = 280, 220
WALL = 0.25
boxes = []


def b(name, x, y, sx, sy, h=1.6, color=(.72, .73, .73), yaw=0.0):
    boxes.append(dict(name=name, x=x, y=y, sx=sx, sy=sy,
                      h=h, color=color, yaw=yaw))


def wall(name, x, y, sx, sy):
    b(name, x, y, sx, sy, 1.60, (.65, .62, .58))


# Exterior walls.  The west entrance and south exit are separate 0.90 m openings.
wall('north_wall', 0, 3.025, 6.80, WALL)
wall('east_wall', 3.275, 0, WALL, 6.30)
wall('west_wall_upper', -3.275, (3.15 - .06)/2, WALL, 3.15 + .06)
wall('west_wall_lower', -3.275, (-.96 - 3.15)/2, WALL, 3.15 - .96)
wall('south_wall_left', (-3.40 - 1.03)/2, -3.025, 3.40 - 1.03, WALL)
wall('south_wall_right', (-.13 + 3.40)/2, -3.025, 3.40 + .13, WALL)

# Four zones: NW bedroom, NE kitchen/dining, SW entrance/living, SE office.
# The opening in each horizontal separator is 0.90 m, per the drawing.
wall('north_vertical_partition', -.025, (2.90 + .19)/2, WALL, 2.90 - .19)
wall('bedroom_partition', (-2.25 + .43)/2, .065, 2.25 + .43, WALL)
wall('dining_partition', (1.34 + 3.15)/2, .065, 3.15 - 1.34, WALL)
wall('office_short_partition', 1.47, (-.06 - .67)/2, WALL, .67 - .06)
# Bottom central wall leaves a 0.90+ m central connecting passage above it.
wall('south_vertical_partition', -.025, (-.96 - 2.90)/2, WALL, 2.90 - .96)

# Bedroom (upper left). Bed foot is toward the room centre (+X).
b('bed', -1.39, 2.18, 2.05, 1.22, .52, (.39, .55, .77))

# Kitchen/dining (upper right). Furniture fronts face inward (toward -Y).
b('dining_cabinet', .89, 2.56, .76, .55, 1.30, (.52, .33, .20))
b('office_snack_cabinet', 1.73, 2.56, .76, .55, 1.06, (.37, .28, .19))
# Dishwasher visual front is +Y in local coordinates, rotate 180 degrees for -Y.
b('dishwasher', 2.64, 2.56, .72, .59, .87, (.67, .72, .76), yaw=math.pi)
b('dining_table', 2.40, .77, .90, .65, .745, (.68, .45, .27))
# The northern dining chair faces south, toward the table / room centre.
b('chair_d1', 2.40, 1.63, .45, .45, .8, (.23, .32, .42))

# Living / entrance room (lower left). Chairs face the coffee table.
b('coffee_table', -1.70, -1.48, .77, .54, .47, (.69, .48, .30))
b('chair_living_west', -2.67, -1.48, .45, .45, .8, (.25, .40, .50), yaw=math.pi/2)
b('chair_living_south_1', -1.96, -2.37, .45, .45, .8, (.25, .40, .50))
b('chair_living_south_2', -1.25, -2.37, .45, .45, .8, (.25, .40, .50))

# Office (lower right). Desk and seated chair face each other.
b('office_desk', 2.30, -.39, 1.0, .55, .75, (.49, .38, .25))
b('chair_office', 2.30, -1.23, .45, .45, .8, (.24, .34, .45))
# Cabinet fronts face north (+Y), toward the office room centre.
b('kitchen_cupboard', .74, -2.55, .68, .63, 1.40, (.58, .43, .27), yaw=math.pi)


def ranges(q):
    """AABB of a rectangular footprint after rotation (angles are multiples of 90°)."""
    ca, sa = abs(math.cos(q['yaw'])), abs(math.sin(q['yaw']))
    dx = q['sx'] * ca + q['sy'] * sa
    dy = q['sx'] * sa + q['sy'] * ca
    return q['x'] - dx/2, q['x'] + dx/2, q['y'] - dy/2, q['y'] + dy/2


def make_reference_map():
    # The raycast reference map is synthetic, NOT produced by robot SLAM.
    occ = np.zeros((MAP_NY, MAP_NX), dtype=np.bool_)
    for q in boxes:
        if q['h'] < .36:
            continue
        x0, x1, y0, y1 = ranges(q)
        i0 = max(0, int((x0 - MAP_X0)/RES))
        i1 = min(MAP_NX, int((x1 - MAP_X0)/RES)+1)
        j0 = max(0, int((y0 - MAP_Y0)/RES))
        j1 = min(MAP_NY, int((y1 - MAP_Y0)/RES)+1)
        occ[j0:j1, i0:i1] = True
    observed = np.full((MAP_NY, MAP_NX), 205, dtype=np.uint8)
    route = [(-2.88,-.62),(-2.12,-.66),(-1.25,-.74),(-1.85,-1.95),
             (-2.92,-2.40),(-2.85,.37),(-2.65,1.30),(-1.75,1.25),
             (-.75,1.40),(.75,.75),(.90,1.75),(1.80,1.75),
             (2.85,.35),(2.90,-.86),(1.02,-1.35),(1.75,-2.04),
             (.45,-.44),(-.60,-.48)]
    for ax, ay in route:
        for deg in range(360):
            co, si = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            for k in range(1, 241):
                d = .025*k
                ix = int((ax + d*co - MAP_X0)/RES)
                iy = int((ay + d*si - MAP_Y0)/RES)
                if not (0 <= ix < MAP_NX and 0 <= iy < MAP_NY):
                    break
                if occ[iy, ix]:
                    observed[iy, ix] = 0
                    break
                observed[iy, ix] = 254
    Image.fromarray(np.flipud(observed), 'L').save(MAP/'reference_lidar_map.pgm')
    (MAP/'reference_lidar_map.yaml').write_text(yaml.safe_dump({
        'image':'reference_lidar_map.pgm', 'mode':'trinary', 'resolution':RES,
        'origin':[MAP_X0,MAP_Y0,0.0], 'negate':0,
        'occupied_thresh':.65, 'free_thresh':.196}, sort_keys=False))


def make_preview():
    sc=155
    im=Image.new('RGB',(1230,1160),'#edf2f4')
    dr=ImageDraw.Draw(im)
    def uv(x,y):return round((x+W/2)*sc+80),round((H/2-y)*sc+70)
    for q in boxes:
        x0,x1,y0,y1=ranges(q)
        px0,py0=uv(x0,y1);px1,py1=uv(x1,y0)
        col=tuple(round(c*255) for c in q['color'])
        dr.rectangle((px0,py0,px1,py1), fill=col,outline='#36444c',width=2)
    try:ft=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
    except OSError:ft=ImageFont.load_default()
    labels=[('BEDROOM',(-2.50,.55)),('KITCHEN / DINING',(.91,.35)),
            ('LIVING ROOM',(-2.12,-.20)),('OFFICE',(.93,-1.66)),
            ('BED',(-1.39,2.18)),('SHELVES',(1.30,2.02)),
            ('DW',(2.64,2.56)),('TABLE',(2.40,.77)),
            ('COFFEE',(-1.70,-1.48)),('DESK',(2.30,-.39)),
            ('CABINET',(.74,-2.55))]
    for label,p in labels:dr.text(uv(*p),label,font=ft,fill='#102735',anchor='mm')
    dr.text((7,uv(-3.40,-.51)[1]-12),'ENTRY',font=ft,fill='#1269ab')
    dr.text(uv(-.60,-3.15),'EXIT',font=ft,fill='#1269ab',anchor='mt')
    cx,cy=uv(-2.06,-.72)
    dr.ellipse((cx-18,cy-18,cx+18,cy+18),fill='#d92132')
    dr.text((90,30),'HOME25 — REVISED REFERENCE LAYOUT (METRES)',font=ft,fill='#15232a')
    im.save(MAP/'arena_layout_preview.png')


def make_world():
    lines=['<?xml version="1.0"?>','<sdf version="1.9">',
      '<world name="robocup_home_training">',
      '<physics name="step" type="ignored"><max_step_size>0.005</max_step_size><real_time_factor>1.0</real_time_factor></physics>',
      '<gravity>0 0 -9.81</gravity>',
      '<plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>',
      '<plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>',
      '<plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>',
      '<plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>',
      '<frame name="arena_entrance" attached_to="world"><pose>-3.40 -0.51 0 0 0 0</pose></frame>',
      '<frame name="arena_exit" attached_to="world"><pose>-0.58 -3.15 0 0 0 1.57079632679</pose></frame>',
      '<frame name="instruction_point" attached_to="world"><pose>-2.06 -0.72 0 0 0 0</pose></frame>',
      '<light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 -0.5 -1</direction></light>',
      '<model name="floor"><static>true</static><link name="floor"><collision name="collision"><geometry><box><size>8.5 8.0 0.1</size></box></geometry></collision><visual name="visual"><geometry><box><size>8.5 8.0 0.1</size></box></geometry><material><diffuse>.88 .85 .78 1</diffuse></material></visual></link><pose>0 0 -0.05 0 0 0</pose></model>']
    for q in boxes:
        x,y,sx,sy,h=q['x'],q['y'],q['sx'],q['sy'],q['h']
        rgba=' '.join(str(round(v,3)) for v in q['color'])+' 1'
        lines.append(f'<model name="{q["name"]}"><static>true</static><pose>{x:.6f} {y:.6f} {h/2:.6f} 0 0 {q["yaw"]:.9f}</pose><link name="body"><collision name="collision"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision><visual name="visual"><geometry><box><size>{sx} {sy} {h}</size></box></geometry><material><diffuse>{rgba}</diffuse><ambient>{rgba}</ambient></material></visual></link></model>')
    # Non-colliding floor decal at the GPSR instruction point. It is a visual
    # landmark only, so no fake obstacle is introduced into the LiDAR/Nav2 map.
    lines.append('<model name="instruction_marker"><static>true</static>'
                 '<pose>-2.06 -0.72 0.0015 0 0 0</pose><link name="marker">'
                 '<visual name="red_circle"><geometry><cylinder>'
                 '<radius>0.20</radius><length>0.003</length></cylinder>'
                 '</geometry><material><ambient>0.85 0.04 0.08 1</ambient>'
                 '<diffuse>0.85 0.04 0.08 1</diffuse></material></visual>'
                 '</link></model>')
    lines.extend(['</world>','</sdf>'])
    WORLD.write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    MAP.mkdir(parents=True,exist_ok=True)
    make_reference_map();make_preview();make_world()
    print('Generated revised Home25 world, reference map and annotated layout:',WORLD)
