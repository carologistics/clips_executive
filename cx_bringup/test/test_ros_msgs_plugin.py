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

# test/test_ros_msgs_plugin.py
import pytest
import rclpy
from std_msgs.msg import String


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    manager_config_file = PathJoinSubstitution(
        [
            FindPackageShare('cx_bringup'),
            TextSubstitution(text='params/plugin_examples/ros_msgs.yaml'),
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


class TestRosMsgsPluginOutput(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_ros_msgs')
        cls.pub = cls.node.create_publisher(String, '/ros_cx_in', 10)
        cls.received_messages = []
        cls.sub = cls.node.create_subscription(
            String, '/ros_cx_out', lambda msg: cls.received_messages.append(msg.data), 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _publish_and_spin(self, text, timeout=5.0):
        import time

        msg = String()
        msg.data = text
        self.pub.publish(msg)
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_ros_msgs_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'ros_msgs', 'cx::RosMsgsPlugin', cx_node)

    def test_publisher_init_rule_fires(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'ros-msgs-pub-init', cx_node)

    def test_subscriber_init_rule_fires(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'ros-msgs-sub-init', cx_node)

    def test_publisher_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output,
            '(ros-msgs-publisher (topic "ros_cx_out") (type "std_msgs/msg/String"))',
            cx_node,
        )

    def test_subscriber_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output,
            '(ros-msgs-subscription (topic "ros_cx_in") (type "std_msgs/msg/String"))',
            cx_node,
        )

    def test_publisher_opened(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Publishing on /ros_cx_out', cx_node)

    def test_subscriber_opened(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Listening for String messages on /ros_cx_in', cx_node)

    def test_message_received_and_hello_world_published(self, cx_node, proc_output):
        """Publish a message to /ros_cx_in and verify the full pipeline."""
        # wait for setup to be complete first
        wait_for_output(proc_output, 'Activated [clips_manager]', cx_node)

        # publish a message
        self._publish_and_spin('hello')

        # verify the message was received and processed
        wait_for_fact_asserted(
            proc_output, '(ros-msgs-message (topic "ros_cx_in")', cx_node, timeout=15.0
        )
        wait_for_rule_fire(proc_output, 'ros-msgs-pub-hello', cx_node, timeout=15.0)
        wait_for_output(proc_output, 'Sending Hello World Message!', cx_node, timeout=15.0)
        wait_for_rule_fire(proc_output, 'ros-msgs-receive', cx_node, timeout=15.0)
        wait_for_output(proc_output, 'Recieved via ros_cx_in: hello', cx_node, timeout=15.0)
        wait_for_fact_retracted(
            proc_output, '(ros-msgs-message (topic "ros_cx_in")', cx_node, timeout=15.0
        )

        # verify the response was published on /ros_cx_out
        import time

        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if 'Hello World!' in self.received_messages:
                break
        self.assertIn(
            'Hello World!',
            self.received_messages,
            'No Hello World! message received on /ros_cx_out',
        )


@launch_testing.post_shutdown_test()
class TestRosMsgsPluginShutdown(make_shutdown_test_class()):
    pass
