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

import os
import sys

import unittest

from launch import LaunchDescription
from launch.actions import OpaqueFunction

import launch_testing
import launch_testing.asserts
import launch_testing.util

@pytest.marker.rostest
@launch_testing.markers.keep_alive
def generate_test_description(ready_fn):
    return LaunchDescription([OpaqueFunction(function=lambda context: ready_fn())])

some_messages_from_std_msgs = [
    'std_msgs/msg/Bool',
    'std_msgs/msg/Float32',
    'std_msgs/msg/Float64',
]

class TestROS2MsgCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.manager
        def launch_msg_command(self, arguments):
            msg_command_action = ExecuteProcess(
                cmd=['ros2', 'msg', *arguments],
                name='ros2msg-cli', output='screen',
                additional_env={'PYTHONUNBUFFERED': '1'}
            )
            with launch_testing.tools.launch_process(
                launch_service, proc_info, proc_output, msg_command_action
            ) as msg_command:
                yield msg_command
        cls.launch_msg_command = launch_msg_command

    def test_list(self):
        with self.launch_msg_command(arguments=['list']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 0
        output_lines = msg_command.output.splitlines()
        assert all(msg in output_lines for msg in some_messages_from_std_msgs)
        assert all(re.match(r'.*/msg/.*', line) is not None for line in output_lines)

    def test_package(self):
        with self.launch_msg_command(arguments=['package', 'std_msgs']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 0
        output_lines = msg_command.output.splitlines()
        assert all(msg in output_lines for msg in some_messages_from_std_msgs)
        assert all(re.match(r'std_msgs/msg/.*', line) is not None for line in output_lines)

    def test_not_a_package(self):
        with self.launch_msg_command(arguments=['package', 'not_a_package']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown package name'],
            output=msg_command.output
        )

    def test_packages(self):
        with self.launch_msg_command(arguments=['packages']) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 0
        assert 'std_msgs' in msg_command.output.splitlines()

    def test_show(self):
        with self.launch_msg_command(
            arguments=['show', 'std_msgs/msg/String']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 0
        assert launch_testing.tools.expect_output(
            expected_output=['string data'],
            output=msg_command.output
        )

    def test_show_not_a_message_typename(self):
        with self.launch_msg_command(
            arguments=['show', 'std_msgs/msg/NotAMessageTypeName']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown message name'],
            output=msg_command.output
        )

    def test_show_not_a_message_ns(self):
        with self.launch_msg_command(
            arguments=['show', 'std_msgs/foo/String']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown message name'],
            output=msg_command.output
        )

    def test_show_not_a_package(self):
        with self.launch_msg_command(
            arguments=['show', 'not_a_package/msg/String']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown package name'],
            output=msg_command.output
        )

    def test_show_not_a_message_type(self):
        with self.launch_msg_command(
            arguments=['show', 'not_a_message_type']
        ) as msg_command:
            assert msg_command.wait_for_shutdown(timeout=10)
        assert msg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['The passed message type is invalid'],
            output=msg_command.output
        )
