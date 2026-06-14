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

# test/test_ros_msgs_service_plugin.py
import pytest
import rclpy
from std_srvs.srv import SetBool


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    manager_config_file = PathJoinSubstitution(
        [
            FindPackageShare('cx_bringup'),
            TextSubstitution(text='params/plugin_examples/ros_msgs_service.yaml'),
        ]
    )

    cx_node = launch_ros.actions.Node(
        package='cx_clips_env_manager',
        executable='cx_node',
        output='screen',
        emulate_tty=True,
        parameters=[manager_config_file, {'autostart_node': True}],
    )

    return launch.LaunchDescription(
        [
            cx_node,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {'cx_node': cx_node}


class TestRosMsgsServicePluginOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_ros_msgs_service')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_set_bool(self, data, timeout=60.0):
        client = self.node.create_client(SetBool, '/ros_cx_service')
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout), '/ros_cx_service not available'
        )
        req = SetBool.Request()
        req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_ros_msgs_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'ros_msgs', 'cx::RosMsgsPlugin', cx_node)

    def test_service_init_rule_fires(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'ros-msgs-service-init', cx_node)

    def test_service_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output,
            '(ros-msgs-service (service "/ros_cx_service") (type "std_srvs/srv/SetBool"))',
            cx_node,
        )

    def test_service_receives_and_responds(self, cx_node, proc_output):
        """Call service and verify both the response and the logged output."""
        # first ensure the service is ready
        wait_for_output(proc_output, 'Opening service /ros_cx_service', cx_node)

        # make the call
        resp = self._call_set_bool(True)
        self.assertTrue(resp.success)

        # now verify the output appeared after the call
        wait_for_output(proc_output, 'Received a request', cx_node, timeout=60.0)
        wait_for_output(proc_output, 'TRUE', cx_node, timeout=60.0)

    def test_service_receives_and_responds_again(self, cx_node, proc_output):
        """Call service and verify both the response and the logged output."""
        # make the call
        resp = self._call_set_bool(False)
        self.assertTrue(resp.success)

        # now verify the output appeared after the call
        wait_for_output(proc_output, 'Received a request', cx_node, timeout=60.0)
        wait_for_output(proc_output, 'FALSE', cx_node, timeout=60.0)


@launch_testing.post_shutdown_test()
class TestRosMsgsServicePluginShutdown(make_shutdown_test_class()):
    pass
