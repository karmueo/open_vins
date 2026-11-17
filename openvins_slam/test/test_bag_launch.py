"""Check YAML launch decisions against ROS parameter-file parsing."""
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


class BagLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'slam_from_bag.launch.py'
        spec = importlib.util.spec_from_file_location('bag_launch', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.config = self.directory / 'config.yaml'
        self.write_config({'save_mcap': False})
        self.in_bag = self.directory / 'job.part.mcap'
        self.in_bag.touch()
        self.cameras = self.directory / 'calib.yaml'
        self.cameras.touch()

    def write_config(self, values, node='slam_from_bag'):
        self.config.write_text(yaml.safe_dump({node: {'ros__parameters': values}}))

    def nodes(self, path=None, **arguments):
        context = LaunchContext()
        context.launch_configurations.update({
            'config': str(path if path is not None else self.config),
            'in_bag': str(self.in_bag),
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

    def test_disabled_by_default_and_uses_original_symlink_name(self):
        alias = self.directory / 'trial.v2.yaml'
        alias.symlink_to(self.config)
        nodes = self.nodes(alias)
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0]['parameters'][-1]['config_name'], 'trial.v2')
        self.assertEqual(nodes[0]['parameters'][0], str(self.config))

    def test_display_does_not_change_processing_speed(self):
        for realtime in (False, True):
            with self.subTest(realtime=realtime):
                self.write_config({'rviz2': True, 'realtime': realtime}, node='/slam_from_bag')
                nodes = self.nodes()
                self.assertEqual(len(nodes), 2)
                self.assertEqual(nodes[1]['executable'], 'rviz2')
                self.assertTrue(nodes[1]['parameters'][0]['use_sim_time'])
                self.assertNotIn('realtime', nodes[0]['parameters'][-1])

    def test_rejects_string_boolean(self):
        for key in ('rviz2', 'realtime'):
            with self.subTest(key=key):
                self.write_config({key: 'false'})
                with self.assertRaisesRegex(ValueError, key):
                    self.nodes()

    def test_missing_file(self):
        with self.assertRaises(FileNotFoundError):
            self.nodes('/nonexistent/openvins-config.yaml')

    def test_shipped_configs_use_new_interface(self):
        directory = Path(__file__).resolve().parents[1] / 'config'
        for path in directory.glob('slam_from_bag*.yaml'):
            with self.subTest(path=path):
                values = yaml.safe_load(path.read_text())['slam_from_bag']['ros__parameters']
                self.assertIs(values['save_mcap'], True)
                self.assertIs(values['rviz2'], False)
                for key in ('in_bag', 'cameras', 'outdir', 'outbag', 'trajectory_file', 'save_trajectory'):
                    self.assertNotIn(key, values)
                self.assertEqual(self.nodes(path)[0]['parameters'][-1]['config_name'], path.stem)

    def test_launch_arguments_and_required_input(self):
        arguments = {action.name: action for action in self.module.generate_launch_description().entities
                     if isinstance(action, DeclareLaunchArgument)}
        self.assertEqual(set(arguments), {'config', 'in_bag', 'cameras', 'outdir'})
        self.assertIsNone(arguments['in_bag'].default_value)
        context = LaunchContext()
        arguments['config'].execute(context)
        self.assertTrue(context.launch_configurations['config'].endswith('/config/slam_from_bag.yaml'))
        for name in ('cameras', 'outdir'):
            arguments[name].execute(context)
            self.assertEqual(context.launch_configurations[name], '')

    def test_rejects_old_params_file_argument(self):
        with self.assertRaisesRegex(ValueError, 'params_file.*config:='):
            self.nodes(params_file=self.config)

    def test_rejects_paths_in_yaml_including_wildcard_parameters(self):
        for node in ('slam_from_bag', '/slam_from_bag', '/**'):
            for key in ('in_bag', 'cameras', 'outdir'):
                with self.subTest(node=node, key=key):
                    self.write_config({key: ''}, node=node)
                    with self.assertRaisesRegex(ValueError, f'{key}.*launch argument'):
                        self.nodes()

    def test_input_is_required_and_must_be_a_file(self):
        for value in (None, '', '   '):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'in_bag is required'):
                self.nodes(in_bag=value)
        for value in (self.directory / 'missing.mcap', self.directory):
            with self.subTest(value=value), self.assertRaisesRegex(FileNotFoundError, 'in_bag'):
                self.nodes(in_bag=value)

    def test_default_and_explicit_calibration(self):
        for value in (None, '', '   '):
            with self.subTest(value=value):
                self.assertEqual(self.path_parameter(self.nodes(cameras=value), 'cameras'), str(self.cameras))
        other = self.directory / 'other.yaml'
        other.touch()
        self.assertEqual(self.path_parameter(self.nodes(cameras=other), 'cameras'), str(other))
        self.cameras.unlink()
        self.assertEqual(self.path_parameter(self.nodes(cameras=other), 'cameras'), str(other))
        with self.assertRaisesRegex(FileNotFoundError, f'{self.cameras}.*cameras:='):
            self.nodes()
        for value in (self.directory / 'missing.yaml', self.directory):
            with self.subTest(value=value), self.assertRaisesRegex(FileNotFoundError, 'cameras'):
                self.nodes(cameras=value)

    def test_default_output_uses_launch_directory_and_local_timestamp(self):
        for location in ('first', 'second'):
            working = self.directory / location
            working.mkdir()
            for omitted in (None, ''):
                with self.subTest(location=location, omitted=omitted), \
                     patch('os.getcwd', return_value=str(working)), \
                     patch.object(self.module, 'datetime') as clock:
                    clock.now.return_value = datetime(2026, 11, 16, 14, 30, 25)
                    nodes = self.nodes(outdir=omitted)
                    expected = working / 'output' / 'job.part_20261116143025'
                    self.assertEqual(self.path_parameter(nodes, 'outdir'), str(expected))
                    self.assertTrue(expected.is_dir())
                    with self.assertRaises(FileExistsError):
                        self.nodes(outdir=omitted)
                    expected.rmdir()

    def test_explicit_output_is_used_directly(self):
        destination = self.directory / 'existing results'
        destination.mkdir()
        (destination / 'keep.txt').write_text('previous output')
        self.assertEqual(self.path_parameter(self.nodes(outdir=destination), 'outdir'), str(destination))
        self.assertEqual(list(destination.iterdir()), [destination / 'keep.txt'])
        new_destination = self.directory / 'new results'
        self.assertEqual(self.path_parameter(self.nodes(outdir=new_destination), 'outdir'), str(new_destination))

    def test_relative_paths_spaces_and_string_parameters(self):
        working = self.directory / 'working directory'
        working.mkdir()
        bag = working / 'input bag.mcap'
        bag.touch()
        calibration = working / 'camera calibration.yaml'
        calibration.touch()
        config = working / 'custom config.yaml'
        config.write_text(self.config.read_text())
        with patch('os.getcwd', return_value=str(working)):
            nodes = self.nodes('custom config.yaml', in_bag='input bag.mcap',
                               cameras='camera calibration.yaml', outdir='output results')
        self.assertEqual(nodes[0]['parameters'][0], str(config))
        for name, expected in (('in_bag', bag), ('cameras', calibration), ('outdir', working / 'output results')):
            self.assertEqual(self.path_parameter(nodes, name), str(expected))
            self.assertIsInstance(self.path_parameter(nodes, name), str)

    def test_user_home_expansion(self):
        def home_path(path):
            return '~/' + os.path.relpath(path, Path.home())

        nodes = self.nodes(home_path(self.config), in_bag=home_path(self.in_bag),
                           cameras=home_path(self.cameras), outdir=home_path(self.directory / 'results'))
        self.assertEqual(nodes[0]['parameters'][0], str(self.config))
        for name, expected in (('in_bag', self.in_bag), ('cameras', self.cameras),
                               ('outdir', self.directory / 'results')):
            self.assertEqual(Path(self.path_parameter(nodes, name)).resolve(), expected)

    def test_symlink_bag_uses_supplied_name_and_adjacent_calibration(self):
        alias_directory = self.directory / 'alias'
        alias_directory.mkdir()
        alias = alias_directory / 'renamed.mcap'
        alias.symlink_to(self.in_bag)
        calibration = alias_directory / 'calib.yaml'
        calibration.touch()
        nodes = self.nodes(in_bag=alias)
        self.assertEqual(self.path_parameter(nodes, 'in_bag'), str(alias))
        self.assertEqual(self.path_parameter(nodes, 'cameras'), str(calibration))

    def test_invalid_arguments_do_not_create_default_output(self):
        with patch('os.getcwd', return_value=str(self.directory)):
            for changes in ({'in_bag': ''}, {'in_bag': self.directory / 'missing.mcap'},
                            {'cameras': self.directory / 'missing.yaml'}, {'params_file': self.config}):
                with self.subTest(changes=changes), self.assertRaises((ValueError, FileNotFoundError)):
                    self.nodes(outdir='', **changes)
            self.write_config({'rviz2': 'false'})
            with self.assertRaises(ValueError):
                self.nodes(outdir='')
            self.write_config({'in_bag': str(self.in_bag)})
            with self.assertRaises(ValueError):
                self.nodes(outdir='')
        self.assertFalse((self.directory / 'output').exists())

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
