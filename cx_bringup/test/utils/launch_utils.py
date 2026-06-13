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

# test/launch_test_utils.py
import re
import time
import unittest

import launch_testing
import launch_testing.asserts


def _assert_in_output(proc_output, expected, cx_node, timeout=60.0):
    """Works for both active (assertWaitFor) and post-shutdown (assertInStream) handlers."""
    if hasattr(proc_output, 'assertWaitFor'):
        proc_output.assertWaitFor(expected_output=expected, process=cx_node, timeout=timeout)
    else:
        launch_testing.asserts.assertInStream(
            proc_output, expected_output=expected, process=cx_node
        )


def make_shutdown_test_class():
    """Create a post-shutdown test class with common checks."""

    class BaseShutdownTest(unittest.TestCase):

        def test_exit_code(self, cx_node, proc_info):
            """Verify clean shutdown."""
            launch_testing.asserts.assertExitCodes(proc_info, process=cx_node)

        def test_no_errors_or_warnings(self, cx_node, proc_output):
            """Verify no warnings or errors appeared during the entire run."""
            for line in proc_output:
                text = line.text.decode('utf-8').strip()
                if text:
                    self.assertNotIn('[ERROR]', text, f'Unexpected error: {text}')
                    self.assertNotIn('[WARN]', text, f'Unexpected warning: {text}')

    return BaseShutdownTest


def make_generate_test_description(config_yaml):
    """Create a generate_test_description function for a given yaml config."""
    import launch
    from launch.substitutions import PathJoinSubstitution, TextSubstitution
    import launch_ros
    from launch_ros.substitutions import FindPackageShare
    import launch_testing.actions
    import launch_testing.markers

    def generate_test_description():
        manager_config_file = PathJoinSubstitution(
            [
                FindPackageShare('cx_bringup'),
                TextSubstitution(text=f'params/plugin_examples/{config_yaml}'),
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

    return generate_test_description


def wait_for_rule_fire(proc_output, rule_name, cx_node, timeout=60.0):
    """Wait for a CLIPS rule to fire (any fire index)."""
    _assert_in_output(
        proc_output, re.compile(rf'FIRE\s+\d+ {re.escape(rule_name)}'), cx_node, timeout
    )


def wait_for_fact_asserted(proc_output, fact_content, cx_node, timeout=60.0):
    """Wait for a fact to be asserted (any fact index)."""
    _assert_in_output(
        proc_output, re.compile(rf'==>\s+f-\d+\s+{re.escape(fact_content)}'), cx_node, timeout
    )


def wait_for_fact_retracted(proc_output, fact_content, cx_node, timeout=60.0):
    """Wait for a fact to be retracted (any fact index)."""
    _assert_in_output(
        proc_output, re.compile(rf'<==\s+f-\d+\s+{re.escape(fact_content)}'), cx_node, timeout
    )


def wait_for_activated(proc_output, cx_node, timeout=60.0):
    """Wait for the clips manager to reach active state."""
    _assert_in_output(proc_output, 'Activated [clips_manager]', cx_node, timeout)


def wait_for_plugin_created(proc_output, plugin_name, plugin_type, cx_node, timeout=60.0):
    """Wait for a plugin to be created."""
    _assert_in_output(
        proc_output, f'Created plugin: {plugin_name} of type {plugin_type}', cx_node, timeout
    )


def wait_for_output(proc_output, text, cx_node=None, timeout=60.0):
    """Wait for arbitrary text in process output."""
    _assert_in_output(proc_output, text, cx_node, timeout)


def wait_for_confval(proc_output, path_suffix, cx_node, timeout=60.0, **kwargs):
    """Wait for a confval fact matching path suffix and optional field values on the same line."""
    pattern = rf'confval \(path ".*{re.escape(path_suffix)}"\)'
    for key, val in kwargs.items():
        field = key.replace('_', '-')
        pattern += rf'.*\({field} {re.escape(str(val))}\)'
    _assert_in_output(proc_output, re.compile(pattern), cx_node, timeout)


def wait_for_index_at_least(proc_output, cx_node, min_index, context=None, timeout=60.0):
    """Wait for f-<n> where n >= min_index, optionally filtered by context string."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        for line in proc_output:
            text = line.text.decode('utf-8')
            if context and context not in text:
                continue
            match = re.search(r'f-(\d+)', text)
            if match and int(match.group(1)) >= min_index:
                return
        time.sleep(0.1)
    raise AssertionError(
        f'Timed out waiting for f-<n> with n >= {min_index}'
        + (f' in context "{context}"' if context else '')
    )
