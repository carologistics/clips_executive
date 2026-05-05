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
    wait_for_fact_retracted,
    wait_for_output,
    wait_for_plugin_created,
    wait_for_rule_fire,
)
import launch_testing
import launch_testing.markers

# test/test_set_bool_srv_plugin.py
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return make_generate_test_description('set_bool_srv.yaml')()


class TestSetBoolSrvPluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_both_environments_created(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Initialized new CLIPS environment: cx_set_bool_srv', cx_node)
        wait_for_output(
            proc_output, 'Initialized new CLIPS environment: cx_set_bool_client', cx_node
        )

    def test_set_bool_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'set_bool', 'cx::CXStdSrvsSetBoolPlugin', cx_node)

    def test_client_init_rule_fires(self, cx_node, proc_output):
        wait_for_output(
            proc_output, '[cx_set_bool_client] [INFO] FIRE    1 set-bool-client-init', cx_node
        )

    def test_service_init_rule_fires(self, cx_node, proc_output):
        wait_for_output(
            proc_output, '[cx_set_bool_srv] [INFO] FIRE    1 set-bool-service-init', cx_node
        )

    def test_client_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output, '(std-srvs-set-bool-client (service "set_bool_srv"))', cx_node
        )

    def test_service_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output, '(std-srvs-set-bool-service (name "set_bool_srv"))', cx_node
        )

    def test_client_created(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Created client for /set_bool', cx_node)

    def test_service_created(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Created service for /ros_cx_srv', cx_node)

    def test_false_request_sent(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Send a request with data: FALSE', cx_node, timeout=15.0)

    def test_true_request_sent(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Additionally, send a request with data: TRUE', cx_node, timeout=15.0
        )

    def test_service_receives_false_request(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Received request on set_bool_srv. Data: FALSE', cx_node, timeout=15.0
        )

    def test_service_receives_true_request(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Received request on set_bool_srv. Data: TRUE', cx_node, timeout=15.0
        )

    def test_true_response_processed(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            'Response (success: TRUE) for data: TRUE: "Received the request: TRUE"',
            cx_node,
            timeout=15.0,
        )

    def test_false_response_processed(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            'Response (success: FALSE) for data: FALSE: "Received the request: FALSE"',
            cx_node,
            timeout=15.0,
        )


@launch_testing.post_shutdown_test()
class TestSetBoolSrvPluginShutdown(make_shutdown_test_class()):

    def test_client_cleanup_on_shutdown(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'set-bool-client-cleanup', cx_node)
        wait_for_fact_retracted(
            proc_output, '(std-srvs-set-bool-client (service "set_bool_srv"))', cx_node
        )

    def test_service_cleanup_on_shutdown(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'set-bool-srv-cleanup', cx_node)
