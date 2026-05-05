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
    wait_for_fact_asserted,
    wait_for_output,
    wait_for_plugin_created,
    wait_for_rule_fire,
)
import launch_testing
import launch_testing.markers

# test/test_file_load_plugin.py
import pytest

generate_test_description = pytest.mark.launch_test(
    launch_testing.markers.keep_alive(make_generate_test_description('file_load.yaml'))
)


class TestFileLoadPluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'files', 'cx::FileLoadPlugin', cx_node)

    def test_plugin_initialized(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Initializing plugin for environment cx_file_load', cx_node)

    def test_batch_executed(self, cx_node, proc_output):
        wait_for_output(proc_output, 'batch', cx_node)

    def test_hello_world_rule_fired(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'hello-world', cx_node)

    def test_hello_world_printed(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Hello world', cx_node)

    def test_hello_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(proc_output, '(hello)', cx_node)


@launch_testing.post_shutdown_test()
class TestFileLoadPluginShutdown(make_shutdown_test_class()):
    pass
