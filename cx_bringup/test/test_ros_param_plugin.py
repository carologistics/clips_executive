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
    wait_for_output,
    wait_for_plugin_created,
)
import launch_testing
import launch_testing.markers

# test/test_ros_param_plugin.py
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return make_generate_test_description('ros_param.yaml')()


class TestRosParamPluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_ros_param_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'ros_param', 'cx::RosParamPlugin', cx_node)

    def test_get_existing_list_param(self, cx_node, proc_output):
        """Verify getting an existing list param returns its value."""
        wait_for_output(proc_output, '(ros-param-get-value "environments"', cx_node)
        wait_for_output(proc_output, '("cx_ros_param")', cx_node)

    def test_get_existing_bool_param(self, cx_node, proc_output):
        """Verify getting an existing bool param returns its value."""
        wait_for_output(
            proc_output, '(ros-param-get-value "cx_ros_param.log_clips_to_file"', cx_node
        )
        wait_for_output(proc_output, 'TRUE', cx_node)

    def test_get_existing_float_param(self, cx_node, proc_output):
        """Verify getting an existing float param returns its value."""
        wait_for_output(proc_output, '(ros-param-get-value "bond_heartbeat_period"', cx_node)
        wait_for_output(proc_output, '0.0', cx_node)

    def test_get_nonexistent_param_returns_default(self, cx_node, proc_output):
        """Verify getting a non-existent param returns the default value."""
        wait_for_output(proc_output, '(ros-param-get-value "does_not_exist"', cx_node)
        wait_for_output(proc_output, '(1 2 3)', cx_node)

    def test_get_existing_string_param(self, cx_node, proc_output):
        """Verify getting an existing string param returns its value."""
        wait_for_output(proc_output, '(ros-param-get-value "unused_param"', cx_node)
        wait_for_output(proc_output, 'i am not used anywhere', cx_node)


@launch_testing.post_shutdown_test()
class TestRosParamPluginShutdown(make_shutdown_test_class()):
    pass
