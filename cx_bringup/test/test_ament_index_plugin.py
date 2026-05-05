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
    wait_for_output,
)
import launch_testing
import launch_testing.actions
import launch_testing.markers

# test/test_ament_index_plugin.py
import pytest

generate_test_description = pytest.mark.launch_test(
    launch_testing.markers.keep_alive(make_generate_test_description('ament_index.yaml'))
)


class TestAmentIndexPluginOutput(unittest.TestCase):

    def test_ament_index_get_package_prefix(self, cx_node, proc_output):
        wait_for_output(proc_output, 'ament-index-get-package-prefix "cx_bringup"', cx_node)
        wait_for_output(proc_output, '/install/cx_bringup', cx_node)

    def test_ament_index_get_package_share_directory(self, cx_node, proc_output):
        wait_for_output(proc_output, 'ament-index-get-package-share-directory "rclcpp"', cx_node)
        wait_for_output(proc_output, 'share/rclcpp', cx_node)

    def test_ament_index_get_packages_with_prefixes(self, cx_node, proc_output):
        wait_for_output(proc_output, 'ament-index-get-packages-with-prefixes', cx_node)
        wait_for_output(proc_output, 'Package: ', cx_node)

    def test_ament_index_get_resource(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'ament-index-get-resource "vendor_packages" "clips_vendor"', cx_node
        )
        wait_for_output(proc_output, 'Content: opt/clips_vendor', cx_node)

    def test_ament_index_get_search_paths(self, cx_node, proc_output):
        wait_for_output(proc_output, 'ament-index-get-search-paths', cx_node)
        wait_for_output(proc_output, 'ros-kilted', cx_node)


@launch_testing.post_shutdown_test()
class TestConfigPluginShutdown(make_shutdown_test_class()):
    pass
