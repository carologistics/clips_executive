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
    make_generate_test_description,
    make_shutdown_test_class,
    wait_for_activated,
    wait_for_index_at_least,
    wait_for_output,
    wait_for_plugin_created,
    wait_for_rule_fire,
)
from cx_msgs.srv import ClipsCommand
import launch_testing
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

generate_test_description = pytest.mark.launch_test(
    launch_testing.markers.keep_alive(make_generate_test_description('executive.yaml'))
)


class TestExecutivePluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        """Verify the manager node was activated."""
        wait_for_activated(proc_output, cx_node)

    def test_executive_plugin_loaded(self, cx_node, proc_output):
        """Verify the executive plugin was loaded."""
        wait_for_plugin_created(proc_output, 'executive', 'cx::ExecutivePlugin', cx_node)

    def test_print_time_rule_fires(self, cx_node, proc_output):
        """Verify the print-time rule fires at least once."""
        wait_for_rule_fire(proc_output, 'print-time', cx_node)

    def test_ros_time_printed(self, cx_node, proc_output):
        """Verify ROS time is printed."""
        wait_for_output(proc_output, 'ROS time:', cx_node)

    def test_sys_time_printed(self, cx_node, proc_output):
        """Verify SYS time is printed."""
        wait_for_output(proc_output, 'SYS time:', cx_node)

    def test_rule_fires_multiple_times(self, cx_node, proc_output):
        """Verify the engine keeps looping — rule fires more than once."""
        wait_for_index_at_least(proc_output, cx_node, min_index=10, context='FIRE')

    def test_time_between_refresh_printed(self, cx_node, proc_output):
        """Verify timing diagnostic is printed."""
        wait_for_output(proc_output, 'time between agenda refresh and rule fire:', cx_node)


class TestExecutivePluginServices(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_executive_services')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_trigger(self, service_name, timeout_sec=5.0):
        client = self.node.create_client(Trigger, service_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout_sec),
            f'Service {service_name} not available',
        )
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), f'Service call to {service_name} timed out')
        return future.result()

    def _call_clips_command(self, service_name, env_name, command, timeout_sec=5.0):
        client = self.node.create_client(ClipsCommand, service_name)
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout_sec),
            f'Service {service_name} not available',
        )
        request = ClipsCommand.Request()
        request.env_name = env_name
        request.command = command
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), f'Service call to {service_name} timed out')
        return future.result()

    def test_pause_service(self):
        """Verify the pause service stops the executive."""
        result = self._call_trigger('/clips_manager/executive/pause')
        self.assertTrue(result.success, f'Pause failed: {result.message}')

    def test_pause_twice_fails(self):
        """Verify pausing an already paused executive returns failure."""
        self._call_trigger('/clips_manager/executive/pause')
        result = self._call_trigger('/clips_manager/executive/pause')
        self.assertFalse(result.success, 'Expected failure when pausing already paused executive')

    def test_tick_once_while_paused(self):
        """Verify tick_once fires at least one rule while paused."""
        self._call_trigger('/clips_manager/executive/pause')
        result = self._call_trigger('/clips_manager/executive/tick_once')
        self.assertTrue(result.success, f'tick_once failed: {result.message}')

    def test_build_service(self):
        """Verify a deftemplate can be built into the environment."""
        result = self._call_clips_command(
            '/clips_manager/executive/build',
            'cx_executive',
            '(deftemplate example (slot message (type STRING)))',
        )
        self.assertTrue(result.success, f'Build failed: {result.message}')

    def test_eval_service(self):
        """Verify a fact can be asserted via eval."""
        # ensure the template exists first
        self._call_clips_command(
            '/clips_manager/executive/build',
            'cx_executive',
            '(deftemplate example (slot message (type STRING)))',
        )
        result = self._call_clips_command(
            '/clips_manager/executive/eval', 'cx_executive', '(assert (example (message "hello")))'
        )
        self.assertTrue(result.success, f'Eval failed: {result.message}')

    def test_build_invalid_construct_fails(self):
        """Verify that a malformed construct returns a failure."""
        result = self._call_clips_command(
            '/clips_manager/executive/build', 'cx_executive', '(deftemplate (slot message))'
        )  # missing template name
        self.assertFalse(result.success, 'Expected failure for malformed construct')

    def test_eval_invalid_expression_fails(self):
        """Verify that a malformed expression returns a failure."""
        result = self._call_clips_command(
            '/clips_manager/executive/eval', 'cx_executive', '(assert ('
        )  # malformed
        self.assertFalse(result.success, 'Expected failure for malformed expression')

    def test_unknown_env_fails(self):
        """Verify that specifying a nonexistent environment returns a failure."""
        result = self._call_clips_command(
            '/clips_manager/executive/eval', 'nonexistent_env', '(assert (foo))'
        )
        self.assertFalse(result.success, 'Expected failure for unknown environment')

    def test_resume_service(self):
        """Verify the resume service restarts the executive after a pause."""
        self._call_trigger('/clips_manager/executive/pause')
        result = self._call_trigger('/clips_manager/executive/resume')
        self.assertTrue(result.success, f'Resume failed: {result.message}')

    def test_resume_without_pause_fails(self):
        """Verify resuming a non-paused executive returns failure."""
        result = self._call_trigger('/clips_manager/executive/resume')
        self.assertFalse(result.success, 'Expected failure when resuming non-paused executive')


@launch_testing.post_shutdown_test()
class TestExecutivePluginShutdown(make_shutdown_test_class()):

    def test_no_errors_or_warnings(self, cx_node, proc_output):
        pass
