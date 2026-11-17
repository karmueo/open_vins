"""验证 MCAP 回放参数、RViz 视频映射和暂停恢复服务提示。"""
import importlib.util
from pathlib import Path
import tempfile
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
import yaml


class ReplayLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'launch' / 'replay_mcap.launch.py'
        spec = importlib.util.spec_from_file_location('replay_launch', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.in_bag = self.directory / 'openvins_test1_slam_from_bag.mcap'
        self.in_bag.touch()
        self.rviz_config = self.directory / 'custom.rviz'
        self.rviz_config.touch()

    def actions(self, **arguments):
        context = LaunchContext()
        context.launch_configurations.update({
            'in_bag': str(self.in_bag),
            'rate': '1.0',
            'loop': 'false',
            'start_paused': 'false',
            'delay_sec': '1.5',
            'stereo_topic': '/stereo/image_raw',
            'rviz_config': str(self.rviz_config),
        })
        for name, value in arguments.items():
            if value is None:
                context.launch_configurations.pop(name, None)
            else:
                context.launch_configurations[name] = str(value)
        return self.module._make_actions(context)

    def test_launch_arguments_and_default_actions(self):
        arguments = {action.name: action for action in self.module.generate_launch_description().entities
                     if isinstance(action, DeclareLaunchArgument)}
        self.assertEqual(set(arguments), {'in_bag', 'rate', 'loop', 'start_paused', 'delay_sec', 'stereo_topic', 'rviz_config'})
        self.assertIsNone(arguments['in_bag'].default_value)

        result = self.actions(rviz_config='', rate='2.0', loop='true', start_paused='true', delay_sec='1.5')
        self.assertEqual(len(result), 5)
        rviz, path_helper, timer, controls, handler = result
        self.assertIsInstance(rviz, Node)
        self.assertEqual(rviz.node_package, 'rviz2')
        self.assertEqual(rviz.node_executable, 'rviz2')
        self.assertTrue(rviz._Node__arguments[1].endswith('/launch/replay_mcap.rviz'))
        ctx = LaunchContext()
        params = {''.join(part.perform(ctx) for part in key): value for key, value in rviz._Node__parameters[0].items()}
        self.assertEqual(params, {'use_sim_time': True})
        self.assertIsInstance(path_helper, Node)
        self.assertEqual(path_helper.node_executable, 'replay_path_node.py')
        self.assertIsInstance(handler, RegisterEventHandler)
        self.assertIsInstance(controls, LogInfo)
        self.assertIsInstance(timer, TimerAction)
        bag_play = timer.actions[0]
        self.assertIsInstance(bag_play, ExecuteProcess)
        cmd = [''.join(sub.perform(ctx) for sub in part) for part in bag_play.cmd]
        self.assertEqual(cmd, [
            'ros2', 'bag', 'play', '--input', str(self.in_bag),
            'mcap', '--clock', '--rate', '2.0',
            '--disable-keyboard-controls', '--remap', '__node:=openvins_replay_player',
            '/stereo/image_raw:=/stereo/image_raw/compressed',
            '--loop', '--start-paused',
        ])

    def test_zero_delay_runs_play_directly(self):
        result = self.actions(delay_sec='0')
        self.assertIsInstance(result[2], ExecuteProcess)

    def test_custom_video_topics_match_default_rviz_subscription(self):
        """自定义输入及压缩后缀输入均能到达默认 RViz 视频话题。"""
        config = Path(__file__).resolve().parents[1] / 'launch' / 'replay_mcap.rviz'
        displays = yaml.safe_load(config.read_text())['Visualization Manager']['Displays']
        video_topic = next(display['Topic']['Value'] for display in displays
                           if display['Name'] == 'Stereo video')
        for topic in ('/camera/stereo', '/camera/stereo/compressed', video_topic):
            with self.subTest(topic=topic):
                player = self.actions(stereo_topic=topic, delay_sec='0')[2]
                context = LaunchContext()
                cmd = [''.join(sub.perform(context) for sub in part) for part in player.cmd]
                remaps = dict(rule.split(':=', 1) for rule in cmd[cmd.index('--remap') + 1:])
                self.assertEqual(remaps.get(topic, topic), video_topic)

    def test_paused_player_has_documented_service_controls(self):
        """暂停启动使用稳定的服务名称，并显示可执行的恢复和暂停命令。"""
        result = self.actions(start_paused='true', delay_sec='0')
        context = LaunchContext()
        cmd = [''.join(sub.perform(context) for sub in part) for part in result[2].cmd]
        self.assertIn('--start-paused', cmd)
        self.assertIn('--disable-keyboard-controls', cmd)
        self.assertIn('__node:=openvins_replay_player', cmd)
        message = ''.join(part.perform(context) for part in result[3].msg)
        for service, service_type in (('resume', 'Resume'), ('pause', 'Pause')):
            self.assertIn(
                f"ros2 service call /openvins_replay_player/{service} "
                f"rosbag2_interfaces/srv/{service_type} '{{}}'", message)

    def test_rejects_invalid_inputs(self):
        for bad in (None, '', '   '):
            with self.subTest(in_bag=bad), self.assertRaisesRegex(ValueError, 'in_bag is required'):
                self.actions(in_bag=bad)
        with self.assertRaisesRegex(FileNotFoundError, 'in_bag'):
            self.actions(in_bag=self.directory / 'missing.mcap')
        with self.assertRaisesRegex(FileNotFoundError, 'rviz_config'):
            self.actions(rviz_config=self.directory / 'missing.rviz')
        for rate in ('0', '-1', 'abc', 'nan'):
            with self.subTest(rate=rate), self.assertRaisesRegex(ValueError, 'rate'):
                self.actions(rate=rate)
        for delay in ('-0.1', 'abc', 'inf'):
            with self.subTest(delay=delay), self.assertRaisesRegex(ValueError, 'delay_sec'):
                self.actions(delay_sec=delay)
        for key in ('loop', 'start_paused'):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, key):
                self.actions(**{key: 'yes'})
