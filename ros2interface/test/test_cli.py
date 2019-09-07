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

import itertools
import os
import re
import sys

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools


some_messages_from_std_msgs = [
    'std_msgs/msg/Bool',
    'std_msgs/msg/Float32',
    'std_msgs/msg/Float64',
]

some_services_from_std_srvs = [
    'std_srvs/srv/Empty',
    'std_srvs/srv/SetBool',
    'std_srvs/srv/Trigger',
]

some_actions_from_test_msgs = [
    'test_msgs/action/Fibonacci'
]

some_interfaces = (
    some_messages_from_std_msgs +
    some_services_from_std_srvs +
    some_actions_from_test_msgs
)

filter_idl_content = launch_testing.tools.basic_output_filter(
    filtered_prefixes=[
        '//'  # drop file comments
    ],
    filtered_patterns=[
        r'^$',  # drop blank lines
        r'\s*@verbatim\(.*',  # drop annotations
        r'\s*".*"\)'  # # drop annotation continuations
    ]
)


@pytest.marker.rostest
@launch_testing.markers.keep_alive
def generate_test_description(ready_fn):
    return LaunchDescription([OpaqueFunction(function=lambda context: ready_fn())])


class TestROS2MsgCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.manager
        def launch_interface_command(self, arguments):
            interface_command_action = ExecuteProcess(
                cmd=['ros2', 'interface', *arguments],
                name='ros2interface-cli', output='screen',
                additional_env={'PYTHONUNBUFFERED': '1'}
            )
            with launch_testing.tools.launch_process(
                launch_service, proc_info, proc_output, interface_command_action
            ) as interface_command:
                yield interface_command
        cls.launch_interface_command = launch_interface_command

    def test_list(self):
        with self.launch_interface_command(arguments=['list']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        filter_ = launch_testing.tools.basic_output_filter(
            filtered_prefixes=['Messages:', 'Services:', 'Actions:']
        )
        assert launch_testing.tools.expect_output(
            expected_output=itertools.repeat(
                re.compile(r'    [A-z0-9_]+/(msg|srv|action)/[A-z0-9_]+')
            ),
            output=filter_(interface_command.output)
        )
        output_lines = interface_command.output.splitlines()
        assert all(ifc in output_lines for ifc in some_interfaces)

    def test_list_messages(self):
        with self.launch_interface_command(arguments=['list', '-m']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=itertools.chain(
                ['Messages:'], itertools.repeat(re.compile(r'    [A-z0-9_]+/msg/[A-z0-9_]+'))
            ),
            output=interface_command.output
        )
        output_lines = interface_command.output.splitlines()
        assert all(msg in output_lines for msg in some_messages_from_std_msgs)

    def test_list_services(self):
        with self.launch_interface_command(arguments=['list', '-s']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=itertools.chain(
                ['Services:'], itertools.repeat(re.compile(r'    [A-z0-9_]+/srv/[A-z0-9_]+'))
            ),
            output=interface_command.output
        )
        output_lines = interface_command.output.splitlines()
        assert all(srv in output_lines for srv in some_services_from_std_srvs)

    def test_list_actions(self):
        with self.launch_interface_command(arguments=['list', '-a']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=itertools.chain(
                ['Actions:'], itertools.repeat(re.compile(r'    [A-z0-9_]+/action/[A-z0-9_]+'))
            ),
            output=interface_command.output
        )
        output_lines = interface_command.output.splitlines()
        assert all(action in output_lines for action in some_actions_from_test_msgs)

    def test_package_on_nonexistent_package(self):
        with self.launch_interface_command(
            arguments=['package', 'not_a_package']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown package not_a_package'],
            output=interface_command.output
        )

    def test_package_on_std_msgs(self):
        with self.launch_interface_command(
            arguments=['package', 'std_msgs']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=itertools.repeat(re.compile(r'std_msgs/msg/[A-z0-9_]+')),
            output=interface_command.output
        )
        output_lines = interface_command.output.splitlines()
        assert all(msg in output_lines for msg in some_messages_from_std_msgs)

    def test_package_on_std_srvs(self):
        with self.launch_interface_command(
            arguments=['package', 'std_srvs']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=itertools.repeat(re.compile(r'std_srvs/srv/[A-z0-9_]+')),
            output=interface_command.output
        )
        output_lines = interface_command.output.splitlines()
        assert all(srv in output_lines for srv in some_services_from_std_srvs)

    def test_package_on_test_msgs(self):
        with self.launch_interface_command(
            arguments=['package', 'std_srvs']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=itertools.repeat(re.compile(r'test_msgs/(msg|srv|action)/[A-z0-9_]+')),
            output=interface_command.output
        )
        output_lines = interface_command.output.splitlines()
        assert all(action in output_lines for action in some_actions_from_test_msgs)

    def test_packages(self):
        with self.launch_interface_command(arguments=['packages']) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' in output_lines
        assert 'std_srvs' in output_lines
        assert 'test_msgs' in output_lines

    def test_packages_with_messages(self):
        with self.launch_interface_command(
            arguments=['packages', '-m']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' in output_lines
        assert 'std_srvs' not in output_lines
        assert 'test_msgs' in output_lines

    def test_packages_with_services(self):
        with self.launch_interface_command(
            arguments=['packages', '-s']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' not in output_lines
        assert 'std_srvs' in output_lines
        assert 'test_msgs' in output_lines

    def test_packages_with_actions(self):
        with self.launch_interface_command(
            arguments=['packages', '-a']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = interface_command.output.splitlines()
        assert 'std_msgs' not in output_lines
        assert 'std_srvs' not in output_lines
        assert 'test_msgs' in output_lines

    def test_show_message(self):
        with self.launch_interface_command(
            arguments=['show', 'std_msgs/msg/String']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                'module std_msgs {',
                '  module msg {',
                '    struct String {',
                '      string data;',
                '    };',
                '  };',
                '};'
            ]
            output=filter_idl_content(interface_command.output)
        )

    def test_show_service(self):
        with self.launch_interface_command(
            arguments=['show', 'std_srvs/srv/SetBool']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                'module std_srvs {',
                '  module srv {',
                '    struct SetBool_Request {',
                '      boolean data;',
                '    };',
                '    struct SetBool_Response {',
                '      boolean success;',
                '      string message;',
                '    };',
                '  };',
                '};'
            ]
            output=filter_idl_content(interface_command.output)
        )

    def test_show_action(self):
        with self.launch_interface_command(
            arguments=['show', 'test_msgs/action/Fibonacci']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_output=[
                'module test_msgs {',
                '  module action {',
                '    @verbatim (language="comment", text=',
                '      "goal definition")',
                '    struct Fibonacci_Goal {',
                '      int32 order;',
                '    };',
                '    struct Fibonacci_Result {',
                '      @verbatim (language="comment", text=',
                '        "result definition")',
                '      sequence<int32> sequence;',
                '    };',
                '    struct Fibonacci_Feedback {',
                '      @verbatim (language="comment", text=',
                '        "feedback")',
                '      sequence<int32> sequence;',
                '    };',
                '  };',
                '};'
            ]
            output=filter_idl_content(interface_command.output)
        )

    def test_show_not_a_package(self):
        with self.launch_interface_command(
            arguments=['show', 'not_a_package/msg/NotAMessageTypeName']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=['Unknown package not_a_package'],
            output=interface_command.output
        )

    def test_show_not_an_interface(self):
        with self.launch_interface_command(
            arguments=['show', 'std_msgs/msg/NotAMessageTypeName']
        ) as interface_command:
            assert interface_command.wait_for_shutdown(timeout=2)
        assert interface_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_output=["Could not find the interface '.+NotAMessageTypeName.idl'"],
            output=interface_command.output
        )
