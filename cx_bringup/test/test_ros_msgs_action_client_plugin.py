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
)
import launch_testing
import launch_testing.markers

# test/test_fibonacci_action_plugin.py
import pytest

generate_test_description = pytest.mark.launch_test(
    launch_testing.markers.keep_alive(
        make_generate_test_description('ros_msgs_action_client.yaml')
    )
)


class TestFibonacciActionPluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_both_environments_created(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Initialized new CLIPS environment: cx_fibonacci_server', cx_node
        )
        wait_for_output(
            proc_output, 'Initialized new CLIPS environment: cx_ros_msgs_action_client', cx_node
        )

    def test_server_created(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Created server for /ros_cx_fibonacci', cx_node)

    def test_client_created(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Created client for /ros_cx_fibonacci', cx_node)

    def test_first_goal_sent(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Sending goal fibonacci(5)', cx_node)

    def test_first_goal_accepted(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Accepting goal', cx_node)

    def test_partial_results_received(self, cx_node, proc_output):
        wait_for_output(proc_output, 'partial sequence: (0 1 1)', cx_node, timeout=60.0)
        wait_for_output(proc_output, 'partial sequence: (0 1 1 2)', cx_node, timeout=60.0)

    def test_first_goal_completed(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Final fibonacci sequence: (0 1 1 2 3 5)', cx_node, timeout=60.0
        )

    def test_both_server_and_client_see_final_result(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            '[cx_fibonacci_server] [INFO] Final fibonacci sequence: (0 1 1 2 3 5)',
            cx_node,
            timeout=60.0,
        )
        wait_for_output(
            proc_output, 'Final fibonacci sequence: (0 1 1 2 3 5)', cx_node, timeout=60.0
        )

    def test_second_goal_sent(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Request fibonacci(10), will cancel before finish', cx_node, timeout=60.0
        )

    def test_second_goal_accepted(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Accepting goal, uuid: ', cx_node, timeout=60.0)

    def test_cancel_requested(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Canceling current goal', cx_node, timeout=60.0)

    def test_cancel_accepted_by_server(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Accepting cancelation of goal', cx_node, timeout=60.0)

    def test_cancel_confirmed_by_server(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            '[cx_fibonacci_server] [INFO] Canceling fibonacci sequence: (0 1 1 2 3 5 8)',
            cx_node,
            timeout=60.0,
        )

    def test_cancel_confirmed_by_client(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Canceled fibonacci sequence: (0 1 1 2 3 5 8)', cx_node, timeout=60.0
        )


@launch_testing.post_shutdown_test()
class TestFibonacciActionPluginShutdown(make_shutdown_test_class()):
    pass
