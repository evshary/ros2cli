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
import itertools
import os
import sys

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

from launch_ros.actions import Node

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

from rmw_implementation import get_available_rmw_implementations


@pytest.marker.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation, ready_fn):
    path_to_complex_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'complex_node.py'
    )
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
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
                        # Add test fixture actions.
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_complex_node_script]
                            node_name='complex_node',
                            additional_env=additional_env
                        ),
                        Node(
                            node_executable=sys.executable,
                            arguments=[path_to_complex_node_script]
                            node_name='_hidden_complex_node',
                            additional_env=additional_env
                        ),
                        OpaqueFunction(function=lambda context: ready_fn())
                    ]
                )
            ]
        ),
    ])
    return launch_description, locals()


class TestROS2NodeCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.manager
        def launch_node_command(self, arguments):
            node_command_action = ExecuteProcess(
                cmd=['ros2', 'node', *arguments],
                name='ros2node-cli', output='screen',
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                }
            )
            with launch_testing.tools.launch_process(
                launch_service, proc_info, proc_output, node_command_action,
                output_filter=launch_testing.tools.basic_output_filter(
                    # ignore launch_ros and ros2cli daemon nodes
                    filtered_patterns=['.*launch_ros.*', '.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as node_command:
                yield node_command
        cls.launch_node_command = launch_node_command

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list(self):
        with self.launch_node_command(arguments=['list']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=['/complex_node'],
            output=node_command.output
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_hidden(self):
        with self.launch_node_command(arguments=['list', '-a']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                '/_hidden_complex_node',
                '/complex_node'
            ],
            output=node_command.output
        )

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_count(self):
        with self.launch_node_command(arguments=['list', '-c']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = node_command.output.splitlines()
        assert len(output_lines) == 1
        # Fixture nodes that are not hidden plus launch_ros node.
        assert int(output_lines[0]) == 2

    @launch_testing.markers.retry_on_failure(times=5)
    def test_list_count_hidden(self):
        with self.launch_node_command(arguments=['list', '-c', '-a']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = node_command.output.splitlines()
        assert len(output_lines) == 1
        # All fixture nodes plus launch_ros and ros2cli daemon nodes.
        assert int(output_lines[0]) == 4

    @launch_testing.markers.retry_on_failure(times=5)
    def test_info_node(self, rmw_implementation):
        with self.launch_node_command(arguments=['info', '/complex_node']) as node_command:
            assert node_command.wait_for_shutdown(timeout=10)
        assert node_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=itertools.chain([
                '/complex_node',
                '  Subscribers:',
                '',
                '  Publishers:',
                '    /my_ns/chatter: std_msgs/msg/String',
                '    /my_ns/parameter_events: rcl_interfaces/msg/ParameterEvent',
                '    /my_ns/rosout: rcl_interfaces/msg/Log',
                '  Services:',
            ], itertools.repeat(re.compile(
                r'\s*/my_ns/my_talker/.*parameter.*: rcl_interfaces/srv/.*Parameter.*'
            ), 6)),
            output=node_command.output
        )
