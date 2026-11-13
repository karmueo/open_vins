"""Check YAML launch decisions against ROS parameter-file parsing."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
import yaml


class BagLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'slam_from_bag.launch.py'
        spec = importlib.util.spec_from_file_location('bag_launch', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def nodes(self, path):
        context = LaunchContext()
        context.launch_configurations['params_file'] = str(path)
        with patch.object(self.module, 'Node', side_effect=lambda **kwargs: kwargs):
            return self.module._make_node(context)

    def test_disabled_by_default_and_uses_original_symlink_name(self):
        with tempfile.TemporaryDirectory() as directory:
            original = Path(directory) / 'source.yaml'
            original.write_text('slam_from_bag:\n  ros__parameters:\n    in_bag: /tmp/job.mcap\n')
            alias = Path(directory) / 'trial.v2.yaml'
            alias.symlink_to(original)
            nodes = self.nodes(alias)
            self.assertEqual(len(nodes), 1)
            self.assertEqual(nodes[0]['parameters'][-1]['config_name'], 'trial.v2')
            self.assertEqual(nodes[0]['parameters'][0], str(original))

    def test_display_does_not_change_processing_speed(self):
        for realtime in (False, True):
            with self.subTest(realtime=realtime), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'config.yaml'
                path.write_text(yaml.safe_dump({'/slam_from_bag': {'ros__parameters': {
                    'rviz2': True, 'realtime': realtime}}}))
                nodes = self.nodes(path)
                self.assertEqual(len(nodes), 2)
                self.assertEqual(nodes[1]['executable'], 'rviz2')
                self.assertTrue(nodes[1]['parameters'][0]['use_sim_time'])
                self.assertNotIn('realtime', nodes[0]['parameters'][-1])

    def test_rejects_string_boolean(self):
        for key in ('rviz2', 'realtime'):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'config.yaml'
                path.write_text(yaml.safe_dump({'slam_from_bag': {'ros__parameters': {key: 'false'}}}))
                with self.assertRaisesRegex(ValueError, key):
                    self.nodes(path)

    def test_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            self.nodes('/nonexistent/openvins-config.yaml')

    def test_shipped_configs_use_new_interface(self):
        directory = Path(__file__).resolve().parents[1] / 'config'
        for path in directory.glob('slam_from_bag*.yaml'):
            with self.subTest(path=path):
                values = yaml.safe_load(path.read_text())['slam_from_bag']['ros__parameters']
                self.assertTrue(values['outdir'])
                self.assertIs(values['save_mcap'], False)
                self.assertIs(values['rviz2'], False)
                for key in ('outbag', 'trajectory_file', 'save_trajectory'):
                    self.assertNotIn(key, values)
                self.assertEqual(self.nodes(path)[0]['parameters'][-1]['config_name'], path.stem)

    def test_old_per_argument_launch_options_are_rejected(self):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'bag_slam.launch.py'
        spec = importlib.util.spec_from_file_location('argument_launch', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for key in ('outbag', 'trajectory_file', 'save_trajectory', 'rviz_enable'):
            context = LaunchContext()
            context.launch_configurations[key] = 'false'
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, key):
                module._make_vio_node(context)
