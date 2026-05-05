# Copyright (c) 2026 Carologistics
# SPDX-License-Identifier: Apache-2.0
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

import unittest

from cx_bringup_test_utils import (
    make_shutdown_test_class,
    wait_for_activated,
    wait_for_fact_asserted,
    wait_for_fact_retracted,
    wait_for_output,
    wait_for_plugin_created,
    wait_for_rule_fire,
)
import launch
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import launch_ros
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.markers

# test/test_ros_msgs_client_plugin.py
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    manager_config_file = PathJoinSubstitution(
        [
            FindPackageShare('cx_bringup'),
            TextSubstitution(text='params/plugin_examples/ros_msgs_client.yaml'),
        ]
    )

    cx_node = launch_ros.actions.Node(
        package='cx_clips_env_manager',
        executable='cx_node',
        output='screen',
        emulate_tty=True,
        parameters=[manager_config_file, {'autostart_node': True}],
    )

    test_service_node = launch_ros.actions.Node(
        package='cx_bringup',
        executable='test_service.py',
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription(
        [
            test_service_node,
            cx_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {'cx_node': cx_node, 'test_service_node': test_service_node}


class TestRosMsgsClientPluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_ros_msgs_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'ros_msgs', 'cx::RosMsgsPlugin', cx_node)

    def test_client_init_rule_fires(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'ros-msgs-client-init', cx_node)

    def test_client_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output,
            '(ros-msgs-client (service "ros_cx_client") (type "std_srvs/srv/SetBool"))',
            cx_node,
        )

    def test_client_opened(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Opening client on /ros_cx_client', cx_node)

    def test_request_rule_fires(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'ros-msgs-request-true', cx_node, timeout=15.0)

    def test_request_sent(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Request sent with id 1', cx_node, timeout=15.0)

    def test_request_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(proc_output, '(request 1)', cx_node, timeout=15.0)

    def test_response_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output,
            '(ros-msgs-response (service "ros_cx_client") (request-id 1)',
            cx_node,
            timeout=15.0,
        )

    def test_response_received(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'set-bool-client-response-received', cx_node, timeout=15.0)

    def test_response_content_correct(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            'Received response from ros_cx_client with: TRUE (The request was true!)',
            cx_node,
            timeout=15.0,
        )

    def test_response_fact_retracted(self, cx_node, proc_output):
        wait_for_fact_retracted(
            proc_output,
            '(ros-msgs-response (service "ros_cx_client") (request-id 1)',
            cx_node,
            timeout=15.0,
        )

    def test_no_service_unavailable_warning(self, cx_node, proc_output):
        """Verify the service was available and no abort warning was logged."""
        # wait for response to confirm success path was taken
        wait_for_output(proc_output, 'Received response from ros_cx_client', cx_node, timeout=15.0)
        # then verify no abort warning appeared
        for line in proc_output:
            text = line.text.decode('utf-8')
            self.assertNotIn(
                'service ros_cx_client not available, abort request',
                text,
                f'Service was unavailable: {text}',
            )


@launch_testing.post_shutdown_test()
class TestRosMsgsClientPluginShutdown(make_shutdown_test_class()):
    pass
