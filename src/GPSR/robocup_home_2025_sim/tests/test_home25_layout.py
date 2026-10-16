"""Regression guards for the supplied Home25 floor plan and furniture facing.

These source-level geometry tests do not require a ROS/Gazebo installation.
"""
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD = ET.parse(ROOT/'worlds'/'robocup_home_2025_vision.sdf').getroot().find('world')
MODELS = {m.get('name'):m for m in WORLD.findall('model')}


def pose(model):
    return tuple(float(v) for v in model.findtext('pose').split())


def bounds(model):
    x,y,z,_,_,yaw=pose(model)
    sx,sy,_=(float(v) for v in model.find('.//collision/geometry/box/size').text.split())
    dx=abs(math.cos(yaw))*sx+abs(math.sin(yaw))*sy
    dy=abs(math.sin(yaw))*sx+abs(math.cos(yaw))*sy
    return (x-dx/2,x+dx/2,y-dy/2,y+dy/2)


class Home25LayoutTest(unittest.TestCase):
    def test_entrance_exit_and_instruction_pose(self):
        actual={f.get('name'):tuple(float(x) for x in f.findtext('pose').split())
                for f in WORLD.findall('frame')}
        self.assertIn('arena_entrance',actual)
        self.assertIn('arena_exit',actual)
        self.assertIn('instruction_point',actual)
        self.assertEqual(actual['arena_entrance'][:2],(-3.4,-.51))
        self.assertEqual(actual['arena_exit'][:2],(-.58,-3.15))
        self.assertEqual(actual['instruction_point'][:2],(-2.06,-.72))
        self.assertAlmostEqual(actual['arena_entrance'][5],0.0)

    def test_four_zones_and_floor_plan_inventory(self):
        want={
          'bed','dining_cabinet','office_snack_cabinet','dishwasher',
          'dining_table','chair_d1','coffee_table','chair_living_west',
          'chair_living_south_1','chair_living_south_2',
          'office_desk','chair_office','kitchen_cupboard',
        }
        self.assertTrue(want.issubset(MODELS.keys()))
        # Ensure furniture is in the intended quadrant, not leftover old positions.
        def xy(name):return pose(MODELS[name])[:2]
        x,y=xy('bed');self.assertLess(x,0);self.assertGreater(y,0)
        for name in ('dining_table','dishwasher','office_snack_cabinet'):
            x,y=xy(name);self.assertGreater(x,0);self.assertGreater(y,0)
        x,y=xy('coffee_table');self.assertLess(x,0);self.assertLess(y,0)
        for name in ('office_desk','kitchen_cupboard'):
            x,y=xy(name);self.assertGreater(x,0);self.assertLess(y,0)

    def test_doorway_clear_widths(self):
        # West and south exterior gaps: exactly 0.90 m as dimensioned.
        up=bounds(MODELS['west_wall_upper']);down=bounds(MODELS['west_wall_lower'])
        self.assertAlmostEqual(up[2]-down[3],.90,places=5)
        left=bounds(MODELS['south_wall_left']);right=bounds(MODELS['south_wall_right'])
        self.assertAlmostEqual(right[0]-left[1],.90,places=5)
        # Bedroom and dining entries should also have a 0.90m passage.
        bedroom=bounds(MODELS['bedroom_partition']);north=bounds(MODELS['north_vertical_partition'])
        dining=bounds(MODELS['dining_partition'])
        self.assertAlmostEqual(bedroom[0]-(-3.15),.90,places=5)
        self.assertAlmostEqual(dining[0]-bedroom[1],.91,places=5)

    def test_furniture_has_no_wall_intersection(self):
        static_walls=[m for n,m in MODELS.items()
                      if 'wall' in n or 'partition' in n]
        furniture=[m for n,m in MODELS.items()
                   if n not in ('floor','instruction_marker') and m not in static_walls]
        for f in furniture:
            fx0,fx1,fy0,fy1=bounds(f)
            for w in static_walls:
                wx0,wx1,wy0,wy1=bounds(w)
                ix=min(fx1,wx1)-max(fx0,wx0)
                iy=min(fy1,wy1)-max(fy0,wy0)
                self.assertFalse(ix>1e-4 and iy>1e-4,(f.get('name'),w.get('name')))

    def test_inward_facing_furniture(self):
        # Express the front side in each object's LOCAL XY. Check that it
        # points toward a relevant free room target, seating or table.
        fronts={
          'bed':((1,0),(-.60,1.25)),  # foot faces open bedroom space
          'dining_cabinet':((0,-1),(.89,1.55)),
          'office_snack_cabinet':((0,-1),(1.73,1.55)),
          'dishwasher':((0,1),(2.64,1.55)),
          'chair_d1':((0,-1),(2.40,.77)),
          'chair_living_west':((0,-1),(-1.70,-1.48)),
          'chair_living_south_1':((0,1),(-1.70,-1.48)),
          'chair_living_south_2':((0,1),(-1.70,-1.48)),
          'chair_office':((0,1),(2.30,-.39)),
          'kitchen_cupboard':((0,-1),(.74,-1.50)),
        }
        for name,(direction,target) in fronts.items():
            x,y,_,_,_,yaw=pose(MODELS[name]);dx,dy=direction
            fx=dx*math.cos(yaw)-dy*math.sin(yaw)
            fy=dx*math.sin(yaw)+dy*math.cos(yaw)
            dot=fx*(target[0]-x)+fy*(target[1]-y)
            self.assertGreater(dot,.2,(name,dot))

    def test_object_support_poses(self):
        # All 20 supplied dynamic objects must rest on the intended surfaces.
        import json
        items=json.loads((ROOT/'config'/'robocup_malaysia_2025_objects.json').read_text())['items']
        self.assertEqual(len(items),20)
        groups={
          **{k:('coffee_table',.473) for k in ('cola','coffee','water','milk')},
          **{k:('dining_cabinet',1.305) for k in
             ('cereal','tom_yam_cup_noodle','yen_ta_four_cup_noodle')},
          **{k:('office_snack_cabinet',1.065) for k in
             ('biscuit_roll','paprika','potae','cornae','pringles','lay_stax','pocky')},
          **{k:('dining_table',.745) for k in
             ('plate','bowl','cup','mug','spoon','fork')},
        }
        for i in items:
            x,y,z,yaw=i['world_pose_xyz_yaw'];parent,h=groups[i['id']]
            x0,x1,y0,y1=bounds(MODELS[parent])
            self.assertTrue(x0<=x<=x1 and y0<=y<=y1,(i['id'],parent))
            self.assertAlmostEqual(z,h,places=4)

if __name__=='__main__':unittest.main()
