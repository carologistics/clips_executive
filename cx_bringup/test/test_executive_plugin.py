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
import launch_testing
import launch_testing.markers

# test/test_executive_plugin.py
import pytest

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


@launch_testing.post_shutdown_test()
class TestExecutivePluginShutdown(make_shutdown_test_class()):
    pass
