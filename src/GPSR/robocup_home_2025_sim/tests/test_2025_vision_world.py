"""Asset/data checks independent of Gazebo/ROS runtime."""
import csv
import json
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]

class VisionWorldAssetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world=ET.parse(ROOT/'worlds'/'robocup_home_2025_vision.sdf').getroot().find('world')
        cls.cat=json.loads((ROOT/'config'/'robocup_malaysia_2025_objects.json').read_text())['items']

    def test_official_inventory(self):
        self.assertEqual(len(self.cat),20)
        self.assertEqual({c:sum(it['category']==c for it in self.cat) for c in {'drinks','food','snacks','tableware'}},
                         {'drinks':4,'food':3,'snacks':7,'tableware':6})
        self.assertEqual(len(set(x['id'] for x in self.cat)),20)
    def test_included_models_match_catalog(self):
        inc=self.world.findall('include')
        self.assertEqual(len(inc),20)
        self.assertEqual({x.findtext('name') for x in inc},{'object_2025_'+i['id'] for i in self.cat})
        self.assertEqual(len(self.world.findall("frame[@name='arena_entrance']")),1)
    def test_compatible_fortress_plugins(self):
        for p in self.world.findall('plugin'):
            self.assertTrue(p.get('filename','').startswith('ignition-gazebo-'))
    def test_supplied_stl_mesh_references(self):
        for it in self.cat:
            p=ROOT/'worlds'/'models'/it['model']
            md=ET.parse(p/'model.sdf')
            meshes=md.findall('.//visual/geometry/mesh/uri')
            self.assertGreater(len(meshes),0,it['id'])
            for mesh in meshes:
                prefix=f'model://{it["model"]}/'
                self.assertTrue(mesh.text.startswith(prefix),mesh.text)
                asset=p/mesh.text.removeprefix(prefix)
                self.assertEqual(asset.suffix,'.stl')
                self.assertTrue(asset.is_file(),asset)
                self.assertGreater(asset.stat().st_size,0,asset)
            self.assertIsNotNone(md.find('.//collision/geometry'))
    def test_reference_map(self):
        mp=ROOT/'maps'/'vision_geometric_reference.pgm'
        with Image.open(mp) as im:
            self.assertEqual(im.size,(280,220))
        self.assertIn('vision_geometric_reference.pgm',(ROOT/'maps'/'vision_geometric_reference.yaml').read_text())
    def test_item_poses_bounded_and_nonoverlapping(self):
        for it in self.cat:
            x,y,z,yaw=it['world_pose_xyz_yaw']; self.assertLess(abs(x),6);self.assertLess(abs(y),4.5);self.assertGreater(z,0)
            self.assertLess(max(it['object_dimensions_m']),.3)
        for i,a in enumerate(self.cat):
            ax,ay,az,_=a['world_pose_xyz_yaw'];aw,ad,ah=a['object_dimensions_m']
            for b in self.cat[i+1:]:
                bx,by,bz,_=b['world_pose_xyz_yaw'];bw,bd,bh=b['object_dimensions_m']
                if abs(az-bz)>.03:continue
                overlap=abs(ax-bx)<(aw+bw)/2 and abs(ay-by)<(ad+bd)/2
                self.assertFalse(overlap,f'Placed objects collide: {a["id"]}, {b["id"]}')

if __name__=='__main__':unittest.main()
