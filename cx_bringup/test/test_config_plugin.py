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

# test/test_config_plugin.py
import re
import unittest

from cx_bringup_test_utils import (
    make_generate_test_description,
    make_shutdown_test_class,
    wait_for_activated,
    wait_for_output,
    wait_for_rule_fire,
)
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

generate_test_description = pytest.mark.launch_test(
    launch_testing.markers.keep_alive(make_generate_test_description('config.yaml'))
)


def wait_for_confval(proc_output, path_suffix, cx_node, timeout=10.0, **kwargs):
    """Wait for a confval fact matching path suffix and optional field values.

    All matchers are combined into a single regex to ensure they match the same line.
    path_suffix: the end of the path to match (e.g. 'environments')
    kwargs: field=value pairs, e.g. list_value='"cx_config"', type='STRING'
    """
    # match any path ending with the given suffix
    pattern = rf'confval \(path ".*{re.escape(path_suffix)}"\)'
    for key, val in kwargs.items():
        field = key.replace('_', '-')
        pattern += rf'.*\({field} {re.escape(str(val))}\)'
    proc_output.assertWaitFor(
        expected_output=re.compile(pattern), process=cx_node, timeout=timeout
    )


class TestConfigPluginOutput(unittest.TestCase):

    def test_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_rule_fired(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'load-bringup-config', cx_node)

    def test_config_file_loaded(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Loading from file', cx_node)
        wait_for_output(proc_output, 'config.yaml', cx_node)


def test_environments_confval_asserted(self, cx_node, proc_output):
    wait_for_confval(proc_output, 'environments', cx_node, list_value='"cx_config"')


def test_plugins_confval_asserted(self, cx_node, proc_output):
    wait_for_confval(
        proc_output, 'cx_config/plugins', cx_node, list_value='"ament_index" "config" "files"'
    )


def test_plugin_type_confval_asserted(self, cx_node, proc_output):
    wait_for_confval(
        proc_output, 'config/plugin', cx_node, type='STRING', value='"cx::ConfigPlugin"'
    )


def test_watch_confval_asserted(self, cx_node, proc_output):
    wait_for_confval(proc_output, 'cx_config/watch', cx_node, list_value='"facts" "rules"')


def test_log_clips_to_file_confval_asserted(self, cx_node, proc_output):
    wait_for_confval(
        proc_output, 'cx_config/log_clips_to_file', cx_node, type='BOOL', value='TRUE'
    )


@launch_testing.post_shutdown_test()
class TestConfigPluginShutdown(make_shutdown_test_class()):
    pass
