"""Unit tests for runtime object toggle command construction."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'scripts' / 'object_toggle_manager.py'
spec = importlib.util.spec_from_file_location('object_toggle_manager', MODULE_PATH)
toggle_module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = toggle_module
spec.loader.exec_module(toggle_module)


class ObjectToggleManagerTest(unittest.TestCase):
    def setUp(self):
        self.calls = []

        def runner(command, **_kwargs):
            self.calls.append(command)
            return subprocess.CompletedProcess(command, 0, stdout='data: true', stderr='')

        registry = toggle_module.load_registry(ROOT / 'config' / '2025_item_poses.csv',
                                               ROOT / 'worlds' / 'models')
        self.manager = toggle_module.GazeboObjectToggle(
            registry, 'robocup_home_2025_vision', runner=runner)

    def test_remove_friendly_name(self):
        ok, active, _ = self.manager.toggle('cola', False)
        self.assertTrue(ok)
        self.assertFalse(active)
        command = self.calls[-1]
        self.assertIn('/world/robocup_home_2025_vision/remove', command)
        self.assertIn('name: "object_2025_cola" type: MODEL', command)

    def test_respawn_uses_original_model_and_pose(self):
        self.manager.toggle('cola', False)
        ok, active, _ = self.manager.toggle('object_2025_cola', True)
        self.assertTrue(ok)
        self.assertTrue(active)
        request = self.calls[-1][-1]
        self.assertIn('sdf_filename:', request)
        self.assertIn('robocup25_cola/model.sdf', request)
        self.assertIn('x: -2.0 y: -1.48 z: 0.473', request)

    def test_unknown_object_does_not_call_gazebo(self):
        ok, active, message = self.manager.toggle('not_an_object', False)
        self.assertFalse(ok)
        self.assertFalse(active)
        self.assertIn('Unknown object', message)
        self.assertEqual(self.calls, [])

    def test_request_is_idempotent(self):
        ok, active, message = self.manager.toggle('cola', True)
        self.assertTrue(ok)
        self.assertTrue(active)
        self.assertIn('already active', message)
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main()
