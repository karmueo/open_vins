"""Check raw launch decisions against ROS parameter-file parsing and directory resolution."""
import importlib.util
import os
from datetime import datetime
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import yaml


class RawLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'slam_from_raw.launch.py'
        spec = importlib.util.spec_from_file_location('raw_launch', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.config = self.directory / 'config.yaml'
        self.write_config({'save_mcap': False})
        self.in_raw = self.directory / 'test1'
        self.in_raw.mkdir()
        self.cameras = self.in_raw / 'calib.yaml'
        self.cameras.touch()

    def write_config(self, values, node='slam_from_raw'):
        self.config.write_text(yaml.safe_dump({node: {'ros__parameters': values}}))

    def nodes(self, path=None, **arguments):
        context = LaunchContext()
        context.launch_configurations.update({
            'config': str(path if path is not None else self.config),
            'in_raw': str(self.in_raw),
            'cameras': '',
            'outdir': str(self.directory / 'results'),
        })
        for name, value in arguments.items():
            if value is None:
                context.launch_configurations.pop(name, None)
            else:
                context.launch_configurations[name] = str(value)
        with patch.object(self.module, 'Node', side_effect=lambda **kwargs: kwargs):
            return self.module._make_node(context)

    def path_parameter(self, nodes, name):
        return nodes[0]['parameters'][-1][name].evaluate(LaunchContext())

    def test_accepts_shipped_slam_from_bag_yaml_and_extracts_parameters(self):
        shipped = Path(__file__).resolve().parents[1] / 'config' / 'slam_from_bag.yaml'
        nodes = self.nodes(shipped)
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0]['executable'], 'slam_from_raw')
        self.assertEqual(nodes[0]['name'], 'slam_from_raw')
        self.assertEqual(nodes[0]['parameters'][-1]['config_name'], 'slam_from_bag')
        self.assertIs(nodes[0]['parameters'][0]['save_mcap'], True)
        self.assertEqual(nodes[0]['parameters'][0]['num_pts'], 400)

    def test_prefers_slam_from_raw_key_when_present(self):
        self.config.write_text(yaml.safe_dump({
            'slam_from_raw': {'ros__parameters': {'save_mcap': False, 'rviz2': True}},
            'slam_from_bag': {'ros__parameters': {'save_mcap': True, 'rviz2': False}},
        }))
        nodes = self.nodes()
        self.assertEqual(len(nodes), 2)
        self.assertEqual(nodes[1]['executable'], 'rviz2')
        self.assertIs(nodes[0]['parameters'][0]['save_mcap'], False)

    def test_rejects_string_boolean_and_legacy_args(self):
        for key in ('rviz2', 'realtime'):
            with self.subTest(key=key):
                self.write_config({key: 'false'})
                with self.assertRaisesRegex(ValueError, key):
                    self.nodes()
        with self.assertRaisesRegex(ValueError, 'params_file.*config:='):
            self.nodes(params_file=self.config)

    def test_rejects_path_keys_in_either_node_namespace(self):
        for node in ('slam_from_raw', '/slam_from_raw', 'slam_from_bag', '/**'):
            for key in ('in_bag', 'in_raw', 'cameras', 'outdir'):
                with self.subTest(node=node, key=key):
                    self.write_config({key: ''}, node=node)
                    with self.assertRaisesRegex(ValueError, f'{key}.*launch argument'):
                        self.nodes()

    def test_launch_arguments_and_required_in_raw_directory(self):
        arguments = {action.name: action for action in self.module.generate_launch_description().entities
                     if isinstance(action, DeclareLaunchArgument)}
        self.assertEqual(set(arguments), {'config', 'in_raw', 'cameras', 'outdir'})
        self.assertIsNone(arguments['in_raw'].default_value)
        for value in (None, '', '   '):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'in_raw is required'):
                self.nodes(in_raw=value)
        regular_file = self.directory / 'file.txt'
        regular_file.touch()
        for value in (self.directory / 'missing_dir', regular_file):
            with self.subTest(value=value), self.assertRaisesRegex(FileNotFoundError, 'in_raw'):
                self.nodes(in_raw=value)

    def test_calibration_lookup_checks_in_raw_then_parent(self):
        self.assertEqual(self.path_parameter(self.nodes(cameras=''), 'cameras'), str(self.cameras))
        self.cameras.unlink()
        parent_calib = self.directory / 'calib.yaml'
        parent_calib.touch()
        self.assertEqual(self.path_parameter(self.nodes(cameras=''), 'cameras'), str(parent_calib))
        parent_calib.unlink()
        with self.assertRaisesRegex(FileNotFoundError, 'cameras:='):
            self.nodes(cameras='')

    def test_default_output_and_trailing_slash_handling(self):
        working = self.directory / 'working'
        working.mkdir()
        with patch('os.getcwd', return_value=str(working)), \
             patch.object(self.module, 'datetime') as clock:
            clock.now.return_value = datetime(2026, 11, 17, 16, 45, 0)
            nodes = self.nodes(in_raw=str(self.in_raw) + '/', outdir='')
            expected = working / 'output' / 'test1_20261117164500'
            self.assertEqual(self.path_parameter(nodes, 'outdir'), str(expected))
            self.assertTrue(expected.is_dir())
