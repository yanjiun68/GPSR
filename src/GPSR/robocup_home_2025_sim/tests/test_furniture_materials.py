"""Regression guards for no-photo textures and detailed domestic furniture."""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
W=ET.parse(ROOT/'worlds'/'robocup_home_2025_vision.sdf').getroot().find('world')
MODELS={m.get('name'):m for m in W.findall('model')}

class DetailedFurnitureTest(unittest.TestCase):
    def test_major_furniture_has_distinct_visual_parts(self):
        expected={
          'bed':14, 'dishwasher':7, 'dining_table':8,
          'chair_d1':8, 'coffee_table':8, 'office_desk':8,
          'office_snack_cabinet':5, 'dining_cabinet':5,
          'chair_living_west':8, 'chair_living_south_1':8,
          'chair_living_south_2':8, 'chair_office':8,
          'kitchen_cupboard':5,
        }
        for key, minimum in expected.items():
            self.assertIn(key, MODELS)
            self.assertGreaterEqual(len(MODELS[key].findall('.//visual')),
                                    minimum, key)

    def test_requested_details_present(self):
        required={
          'bed':['pillow_cover','folded_duvet','mattress','headboard'],
          'dishwasher':['washer_horizontal_handle','washer_selector_knob','washer_front_door'],
          'dining_table':['table_top','table_leg'],
          'chair_d1':['chair_seat','chair_backrest','chair_leg'],
          'dining_cabinet':['shelf_side','shelf_back'],
          'office_snack_cabinet':['display_top','shelf_side'],
          'chair_living_west':['chair_backrest','chair_seat'],
          'chair_office':['chair_backrest','chair_seat'],
        }
        for model, parts in required.items():
            names=' '.join(v.get('name','') for v in MODELS[model].findall('.//visual'))
            for part in parts:self.assertIn(part,names,(model,part))
        self.assertEqual(sum('pillow_cover' in v.get('name','') for v in MODELS['bed'].findall('.//visual')),2)

    def test_furniture_collisions_kept(self):
        for name in ('bed','chair_d1','dining_table','dishwasher',
                     'office_desk','dining_cabinet','chair_office'):
            self.assertGreater(len(MODELS[name].findall('.//collision')),0)

    def test_image_projection_script_removed(self):
        self.assertFalse((ROOT/'scripts'/'apply_official_photos.py').exists())
    def test_supplied_object_models_have_meshes(self):
        for obj in ('cola','water','milk','cereal','paprika','pocky'):
            model=ROOT/'worlds'/'models'/('robocup25_'+obj)
            self.assertTrue((model/'model.config').is_file(),obj)
            self.assertTrue((model/'model.sdf').is_file(),obj)
            self.assertGreater(len(list((model/'meshes').glob('*.stl'))),0,obj)

if __name__=='__main__':unittest.main()
