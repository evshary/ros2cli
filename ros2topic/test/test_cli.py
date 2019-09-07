# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import contextlib
import functools
import itertools
import math
import os
import re
import sys
import time

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

from launch_testings.asserts import EXIT_OK
from launch_testings.tools import expect_output

from rmw_implementation import get_available_rmw_implementations


@pytest.marker.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    path_to_fixtures = os.path.join(os.path.dirname(__file__), 'fixtures')
    additional_env = {
        'RMW_IMPLEMENTATION': rmw_implementation, 'PYTHONUNBUFFERED': '1'
    }

    path_to_talker_node_script = os.path.join(path_to_fixtures, 'talker_node.py')
    path_to_listener_node_script = os.path.join(path_to_fixtures, 'listener_node.py')

    hidden_talker_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_talker_node_script],
        remappings=[('chatter', '_hidden_chatter')],
        additional_env=additional_env,
    )
    talker_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_talker_node_script]
        additional_env=additional_env,
    )
    listener_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_listener_node_script],
        remappings=[('chatter', 'chit_chatter')],
        additional_env=additional_env,
    )

    path_to_repeater_node_script = os.path.join(path_to_fixtures, 'repeater_node.py')

    array_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_repeater_node_script, 'test_msgs/msg/Arrays']
        node_name='array_repeater',
        remappings=[('~/output', '/arrays')],
        additional_env=additional_env,
    )
    defaults_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_repeater_node_script, 'test_msgs/msg/Defaults'],
        node_name='defaults_repeater',
        remappings=[('~/output', '/defaults')],
        additional_env=additional_env,
    )
    bounded_sequences_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[
            path_to_repeater_node_script, 'test_msgs/msg/BoundedSequences'
        ],
        node_name='bounded_sequences_repeater',
        remappings=[('~/output', '/bounded_sequences')],
        additional_env=additional_env,
    )
    unbounded_sequences_repeater_node_action = Node(
        node_executable=sys.executable,
        arguments=[
            path_to_repeater_node_script, 'test_msgs/msg/UnboundedSequences'
        ],
        node_name='unbounded_sequences_repeater',
        remappings=[('~/output', '/unbounded_sequences')],
        additional_env=additional_env,
    )

    path_to_controller_node_script = os.path.join(path_to_fixtures, 'controller_node.py')

    cmd_vel_controller_node_action = Node(
        node_executable=sys.executable,
        arguments=[path_to_controller_node_script],
        additional_env=additional_env,
    )

    start_time = time.time()
    return LaunchDescription([
        # Always restart daemon to isolate tests.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        # Add talker/listener pair.
                        talker_node_action,
                        listener_node_action,
                        # Add hidden talker.
                        hidden_talker_node_action,
                        # Add topic repeaters.
                        array_repeater_node_action,
                        defaults_repeater_node_action,
                        bounded_sequences_repeater_node_action,
                        unbounded_sequences_repeater_node_action,
                        # Add stamped data publisher.
                        cmd_vel_controller_node_action,
                        OpaqueFunction(function=lambda context: ready_fn())
                    ]
                )
            ]
        ),
    ]), locals()


class TestROS2TopicCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation,
        listener_node_action
    ):
        @contextlib.manager
        def launch_topic_command(self, arguments):
            topic_command_action = ExecuteProcess(
                cmd=['ros2', 'topic', *arguments], name='ros2topic-cli', output='screen',
                additional_env={'RMW_IMPLEMENTATION': rmw_implementation, 'PYTHONUNBUFFERED': '1'}
            )
            with launch_testing.tools.launch_process(
                launch_service, proc_info, proc_output, topic_command_action,
                output_filter=launch_testing.tools.basic_output_filter(
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as topic_command:
                yield topic_command
        cls.launch_topic_command = launch_topic_command

        cls.listener_node = launch_testing.tools.wrap_process(
            launch_service, proc_info, proc_output, listener_node_action,
            output_filter=launch_testing.tools.basic_output_filter(
                filtered_rmw_implementation=rmw_implementation
            )
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_topics(self):
        with self.launch_topic_command(arguments=['list']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                '/arrays',
                '/basic',
                '/bounded_sequences',
                '/chatter',
                '/chit_chatter',
                '/parameter_events',
                '/rosout',
                '/cmd_vel',
                '/unbounded_sequences',
            ],
            output=topic_command.output
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_all_topics(self):
        with self.launch_topic_command(
            arguments=['list', '--include-hidden-topics']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                '/_hidden_chatter',
                '/arrays',
                '/basic',
                '/bounded_sequences',
                '/chatter',
                '/chit_chatter',
                '/parameter_events',
                '/rosout',
                '/stamped_point',
                '/unbounded_sequences',
            ],
            output=topic_command.output
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_with_types(self):
        with self.launch_topic_command(arguments=['list', '-t']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                '/arrays [test_msgs/msg/Arrays]',
                '/defaults [test_msgs/msg/Defaults]',
                '/bounded_sequences [test_msgs/msg/BoundedSequences]',
                '/chatter [std_msgs/msg/String]',
                '/chit_chatter [std_msgs/msg/String]',
                '/parameter_events [rcl_interfaces/msg/ParameterEvent]',
                '/rosout [rcl_interfaces/msg/Log]'
                '/cmd_vel [geometry_msgs/msg/TwistStamped]',
                '/unbounded_sequences [test_msgs/msg/UnboundedSequences]',
            ],
            output=topic_command.output
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_count(self):
        with self.launch_topic_command(arguments=['list', '-c']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = topic_command.output.splitlines()
        assert len(output_lines) == 1
        assert int(output_lines[0]) == 9

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_info(self):
        with self.launch_topic_command(arguments=['info', '/chatter']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                'Topic: /chatter',
                'Publisher count: 1',
                'Subscriber count: 0'
            ],
            output=topic_command.output
        )

    def test_info_on_unknown_topic(self):
        with self.launch_topic_command(arguments=['info', '/unknown_topic']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                'Topic: /not_a_chatter',
                'Publisher count: 0',
                'Subscriber count: 0'
            ],
            output=topic_command.output
        )

    def test_topic_type(self):
        with self.launch_topic_command(arguments=['type', '/chatter']) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=['std_msgs/msg/String'], output=topic_command.output
        )

    def test_hidden_topic_type(self):
        with self.launch_topic_command(
            arguments=['type', '/_hidden_chatter']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=10)
        assert topic_command.exit_code == 1
        assert topic_command.output == ''

    @launch_testing.markers.retry_on_failure(times=5)
    def test_find_topic_type(self):
        with self.launch_topic_command(
            arguments=['find', 'rcl_interfaces/msg/Log']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=2)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert expect_output(expected_output=['/rosout'], output=topic_command.output)

    def test_find_not_a_topic_typename(self):
        with self.launch_topic_command(
            arguments=['find', 'rcl_interfaces/msg/NotAMessageTypeName']
        ) as topic_command:
            assert topic_command.wait_for_shutdown(timeout=2)
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK
        assert not find_command.output

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '/chatter']
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    re.compile(r"data: 'Hello World: \d+'"),
                    '---'
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_str_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-str', '/chatter']
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    re.compile(r"data: '<string length: <\d+>>'"),
                    '---'
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_csv_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '--csv', '/defaults']
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    "True,b'2',100,1.125,1.125,-50,200,-1000,2000,-30000,60000,-40000000,50000000"
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_arr_topic_echo_on_array_message(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-arr', '/arrays'],
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    "byte_values: '<array type: octet[3]>'",
                    "float64_values: '<array type: double[3]>'",
                    "int8_values: '<array type: int8[3]>'",
                    "uint8_values: '<array type: uint8[3]>'",
                    "int16_values: '<array type: int16[3]>'",
                    "uint16_values: '<array type: uint16[3]>'",
                    "int32_values: '<array type: int32[3]>'",
                    "uint32_values: '<array type: uint32[3]>'",
                    "int64_values: '<array type: int64[3]>'",
                    "uint64_values: '<array type: uint64[3]>'",
                    "string_values: '<array type: unknown[3]>'",
                    "basic_types_values: '<array type: test_msgs/msg/BasicTypes[3]>'",
                    "constants_values: '<array type: test_msgs/msg/Constants[3]>'",
                    "defaults_values: '<array type: test_msgs/msg/Defaults[3]>'",
                    "bool_values_default: '<array type: boolean[3]>'",
                    "byte_values_default: '<array type: octet[3]>'",
                    "char_values_default: '<array type: uint8[3]>'",
                    "float32_values_default: '<array type: float[3]>'",
                    "float64_values_default: '<array type: double[3]>'",
                    "int8_values_default: '<array type: int8[3]>'",
                    "uint8_values_default: '<array type: uint8[3]>'",
                    "int16_values_default: '<array type: int16[3]>'",
                    "uint16_values_default: '<array type: uint16[3]>'",
                    "int32_values_default: '<array type: int32[3]>'",
                    "uint32_values_default: '<array type: uint32[3]>'",
                    "int64_values_default: '<array type: int64[3]>'",
                    "uint64_values_default: '<array type: uint64[3]>'",
                    "string_values_default: '<array type: unknown[3]>'",
                    'alignment_check: 0',
                    '---'
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_arr_topic_echo_on_seq_message(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-arr', '/unbounded_sequences'],
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    "bool_values: '<sequence type: boolean, length: 0>'",
                    "byte_values: '<sequence type: octet, length: 0>'",
                    "char_values: '<sequence type: uint8, length: 0>'",
                    "float32_values: '<sequence type: float, length: 0>'",
                    "float64_values: '<sequence type: double, length: 0>'",
                    "int8_values: '<sequence type: int8, length: 0>'",
                    "uint8_values: '<sequence type: uint8, length: 0>'",
                    "int16_values: '<sequence type: int16, length: 0>'",
                    "uint16_values: '<sequence type: uint16, length: 0>'",
                    "int32_values: '<sequence type: int32, length: 0>'",
                    "uint32_values: '<sequence type: uint32, length: 0>'",
                    "int64_values: '<sequence type: int64, length: 0>'",
                    "uint64_values: '<sequence type: uint64, length: 0>'",
                    "string_values: '<sequence type: string, length: 0>'",
                    "basic_types_values: '<sequence type: test_msgs/msg/BasicTypes, length: 0>'",
                    "constants_values: '<sequence type: test_msgs/msg/Constants, length: 0>'",
                    "defaults_values: '<sequence type: test_msgs/msg/Defaults, length: 0>'",
                    "bool_values_default: '<sequence type: boolean, length: 3>'",
                    "byte_values_default: '<sequence type: octet, length: 3>'",
                    "char_values_default: '<sequence type: uint8, length: 3>'",
                    "float32_values_default: '<sequence type: float, length: 3>'",
                    "float64_values_default: '<sequence type: double, length: 3>'",
                    "int8_values_default: '<sequence type: int8, length: 3>'",
                    "uint8_values_default: '<sequence type: uint8, length: 3>'",
                    "int16_values_default: '<sequence type: int16, length: 3>'",
                    "uint16_values_default: '<sequence type: uint16, length: 3>'",
                    "int32_values_default: '<sequence type: int32, length: 3>'",
                    "uint32_values_default: '<sequence type: uint32, length: 3>'",
                    "int64_values_default: '<sequence type: int64, length: 3>'",
                    "uint64_values_default: '<sequence type: uint64, length: 3>'",
                    "string_values_default: '<sequence type: string, length: 3>'",
                    'alignment_check: 0',
                    '---'
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_no_arr_topic_echo_on_bounded_seq_message(self):
        with self.launch_topic_command(
            arguments=['echo', '--no-arr', '/bounded_sequences'],
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    "int8_values: '<sequence type: int8[3], length: 0>'",
                    "uint8_values: '<sequence type: uint8[3], length: 0>'",
                    "int16_values: '<sequence type: int16[3], length: 0>'",
                    "uint16_values: '<sequence type: uint16[3], length: 0>'",
                    "int32_values: '<sequence type: int32[3], length: 0>'",
                    "uint32_values: '<sequence type: uint32[3], length: 0>'",
                    "int64_values: '<sequence type: int64[3], length: 0>'",
                    "uint64_values: '<sequence type: uint64[3], length: 0>'",
                    "string_values: '<sequence type: string[3], length: 0>'",
                    "basic_types_values: '<sequence type: test_msgs/msg/BasicTypes[3], length: 0>'",
                    "constants_values: '<sequence type: test_msgs/msg/Constants[3], length: 0>'",
                    "defaults_values: '<sequence type: test_msgs/msg/Defaults[3], length: 0>'",
                    "bool_values_default: '<sequence type: boolean[3], length: 3>'",
                    "byte_values_default: '<sequence type: octet[3], length: 3>'",
                    "char_values_default: '<sequence type: uint8[3], length: 3>'",
                    "float32_values_default: '<sequence type: float[3], length: 3>'",
                    "float64_values_default: '<sequence type: double[3], length: 3>'",
                    "int8_values_default: '<sequence type: int8[3], length: 3>'",
                    "uint8_values_default: '<sequence type: uint8[3], length: 3>'",
                    "int16_values_default: '<sequence type: int16[3], length: 3>'",
                    "uint16_values_default: '<sequence type: uint16[3], length: 3>'",
                    "int32_values_default: '<sequence type: int32[3], length: 3>'",
                    "uint32_values_default: '<sequence type: uint32[3], length: 3>'",
                    "int64_values_default: '<sequence type: int64[3], length: 3>'",
                    "uint64_values_default: '<sequence type: uint64[3], length: 3>'",
                    "string_values_default: '<sequence type: string[3], length: 3>'",
                    'alignment_check: 0',
                    '---'
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_truncate_length_topic_echo(self):
        with self.launch_topic_command(
            arguments=['echo', '--truncate-length', '5', '/chatter'],
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    re.compile(r'data: Hello...'),
                    '---'
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    def test_topic_pub(self):
        with self.launch_topic_command(
            arguments=['pub', '/chit_chatter', 'std_msgs/msg/String', '{data: foo}'],
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    'publisher: beginning loop',
                    "publishing #1: std_msgs.msg.String(data='foo')",
                    ''
                ]),
                timeout=10
            )
            self.listener_node.wait_for_output(
                functools.partial(expect_output, [
                    '[INFO] [listener]: I heard: [foo]'
                ] * 3),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    def test_topic_pub_once(self):
        with self.launch_topic_command(
            arguments=[
                'pub', '--once',
                '/chit_chatter',
                'std_msgs/msg/String',
                '{data: bar}'
            ]
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    'publisher: beginning loop',
                    "publishing #1: std_msgs.msg.String(data='bar')",
                    ''
                ]),
                timeout=5
            )
            assert topic_command.wait_for_shutdown(timeout=5)
            self.listener_node.wait_for_output(
                functools.partial(expect_output, [
                    '[INFO] [listener]: I heard: [bar]'
                ]),
                timeout=10
            )
        assert topic_command.exit_code == launch_testing.asserts.EXIT_OK

    def test_topic_pub_print_every_two(self):
        with self.launch_topic_command(
            arguments=[
                'pub',
                '-p', '2',
                '/chit_chatter',
                'std_msgs/msg/String',
                '{data: fizz}'
            ]
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    'publisher: beginning loop',
                    "publishing #2: std_msgs.msg.String(data='fizz')",
                    ''
                    "publishing #4: std_msgs.msg.String(data='fizz')",
                    ''
                ]),
                timeout=5
            )
            self.listener_node.wait_for_output(
                functools.partial(expect_output, [
                    '[INFO] [listener]: I heard: [fizz]'
                ]),
                timeout=5
            )
        assert topic_command.wait_for_shutdown(timeout=5)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_delay(self, start_time):
        average_delay_line_pattern = re.compile(r'average delay: (\d+.\d{3})')
        stats_line_pattern = re.compile(
            r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
        )
        with self.launch_topic_command(arguments=['delay', '/cmd_vel']) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    average_delay_line_pattern,
                    stats_line_pattern
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

        head_line = topic_command.output.splitlines()[0]
        computed_delay = (time.time() - start_time)
        average_delay = float(average_delay_line_pattern.match(head_line).groups(0))
        assert math.isclose(average_delay, computed_delay, rel_tol=1e-3)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_hz(self):
        average_rate_line_pattern = re.compile(r'average rate: (\d+.\d{3})')
        stats_line_pattern = re.compile(
            r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
        )
        with self.launch_topic_command(arguments=['hz', '/chatter']) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    average_rate_line_pattern,
                    stats_line_pattern
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

        head_line = topic_command.output.splitlines()[0]
        average_rate = float(average_rate_pattern.match(head_line).groups(0))
        assert math.isclose(average_rate, 1., rel_tol=1e-3)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_filtered_topic_hz(self):
        average_rate_line_pattern = re.compile(r'average rate: (\d+.\d{3})')
        stats_line_pattern = re.compile(
            r'\s*min: \d+.\d{3}s max: \d+.\d{3}s std dev: \d+.\d{5}s window: \d+'
        )
        with self.launch_topic_command(
            arguments=[
                'hz',
                '--filter',
                'int(m.data.rpartition(\":\")[-1]) % 2 == 0',
                '/chatter'
            ]
        ) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    average_rate_line_pattern,
                    stats_line_pattern
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)

        head_line = topic_command.output.splitlines()[0]
        average_rate = float(average_rate_pattern.match(head_line).groups(0))
        assert math.isclose(average_rate, 0.5, rel_tol=1e-3)

    @launch_testing.markers.retry_on_failure(times=5)
    def test_topic_bw(self):
        with self.launch_topic_command(arguments=['bw', '/basic']) as topic_command:
            assert topic_command.wait_for_output(
                functools.partial(expect_output, [
                    'Subscribed to [/basic]',
                    re.compile(r'average: 2\d\.\d{2}B/s'),
                    re.compile(
                        r'\s*mean: 2\d\.\d{2}B/s min: 2\d\.\d{2}B/s max: 2\d\.\d{2}B/s window: \d+'
                    )
                ]),
                timeout=10
            )
        assert topic_command.wait_for_shutdown(timeout=5)
