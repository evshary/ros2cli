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
import re
import sys

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

import launch_testing
import launch_testing.asserts
import launch_testing.util


some_services_from_std_srvs = [
    'std_srvs/srv/Empty',
    'std_srvs/srv/SetBool',
    'std_srvs/srv/Trigger',
]


@pytest.marker.rostest
@launch_testing.markers.keep_alive
def generate_test_description(ready_fn):
    return LaunchDescription([OpaqueFunction(function=lambda context: ready_fn())])


class TestROS2SrvCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.manager
        def launch_srv_command(self, arguments):
            srv_command_action = ExecuteProcess(
                cmd=['ros2', 'srv', *arguments],
                name='ros2srv-cli', output='screen',
                additional_env={'PYTHONUNBUFFERED': '1'}
            )
            with launch_testing.tools.launch_process(
                launch_service, proc_info, proc_output, srv_command_action
            ) as srv_command:
                yield srv_command
        cls.launch_srv_command = launch_srv_command

    def test_list(self):
        with self.launch_srv_command(arguments=['list']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 0
        output_lines = srv_command.output.splitlines()
        assert all(srv in output_lines for srv in some_messages_from_std_srvs)
        assert all(re.match(r'.*/srv/.*', line) is not None for line in output_lines)

    def test_package(self):
        with self.launch_srv_command(arguments=['package', 'std_srvs']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 0
        output_lines = srv_command.output.splitlines()
        assert all(srv in output_lines for srv in some_messages_from_std_srvs)
        assert all(re.match(r'std_srvs/srv/.*', line) is not None for line in output_lines)

    def test_not_a_package(self):
        with self.launch_srv_command(arguments=['package', 'not_a_package']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown package name'],
            output=srv_command.output
        )

    def test_packages(self):
        with self.launch_srv_command(arguments=['packages']) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        assert 'std_srvs' in srv_command.output.splitlines()

    def test_show(self):
        with self.launch_srv_command(
            arguments=['show', 'std_srvs/srv/SetBool']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                'bool data', '---', 'bool success', 'string message'
            ],
            output=srv_command.output
        )

        with self.launch_srv_command(
            arguments=['show', 'std_srvs/srv/Trigger']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=['---', 'bool success', 'string message'],
            output=srv_command.output
        )

    def test_show_not_a_service_typename(self):
        with self.launch_srv_command(
            arguments=['show', 'std_srvs/srv/NotAServiceTypeName']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown service name'],
            output=srv_command.output
        )

    def test_show_not_a_service_ns(self):
        with self.launch_srv_command(
            arguments=['show', 'std_srvs/foo/Empty']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown service name'],
            output=srv_command.output
        )

    def test_show_not_a_package(self):
        with self.launch_srv_command(
            arguments=['show', 'not_a_package/srv/Empty']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown package name'],
            output=srv_command.output
        )

    def test_show_not_a_service_type(self):
        with self.launch_srv_command(
            arguments=['show', 'not_a_service_type']
        ) as srv_command:
            assert srv_command.wait_for_shutdown(timeout=10)
        assert srv_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=[
                'The passed service type is invalid'
            ],
            output=srv_command.output
        )
