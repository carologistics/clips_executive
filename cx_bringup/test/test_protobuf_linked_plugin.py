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
)
import launch_testing
import launch_testing.markers

# test/test_protobuf_linked_plugin.py
import pytest

generate_test_description = pytest.mark.launch_test(
    launch_testing.markers.keep_alive(make_generate_test_description('protobuf_linked.yaml'))
)


class TestProtobufLinkedPluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_both_environments_created(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'Initialized new CLIPS environment: cx_protobuf_server', cx_node
        )
        wait_for_output(
            proc_output, 'Initialized new CLIPS environment: cx_protobuf_client', cx_node
        )

    def test_protobuf_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'protobuf', 'cx::BringupProtobufPlugin', cx_node)

    def test_protobuf_contexts_initialized(self, cx_node, proc_output):
        proc_output.assertWaitFor(
            expected_output=__import__('re').compile(r'Initialised context!'),
            process=cx_node,
            timeout=10.0,
        )

    def test_server_init_rule_fires(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            '[cx_protobuf_server] [INFO] FIRE    1 protobuf-init-example-client-server',
            cx_node,
        )

    def test_client_init_rule_fires(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            '[cx_protobuf_client] [INFO] FIRE    1 protobuf-init-example-client-server',
            cx_node,
        )

    def test_server_type_registered(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Register Type: TRUE', cx_node)

    def test_client_connects_to_server(self, cx_node, proc_output):
        wait_for_output(proc_output, 'Connect to server: 1', cx_node)

    def test_client_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(proc_output, '(client 1)', cx_node)

    def test_client_connected_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(proc_output, '(protobuf-client-connected 1)', cx_node)

    def test_server_client_connected_fact_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output, '(protobuf-server-client-connected 1', cx_node, timeout=15.0
        )

    def test_client_sends_message(self, cx_node, proc_output):
        wait_for_output(
            proc_output,
            '[cx_protobuf_client] [INFO] FIRE    1 peer-send-msg',
            cx_node,
            timeout=15.0,
        )

    def test_server_receives_message(self, cx_node, proc_output):
        wait_for_fact_asserted(
            proc_output, '(protobuf-msg (type "SearchRequest")', cx_node, timeout=15.0
        )

    def test_server_reads_message(self, cx_node, proc_output):
        wait_for_output(
            proc_output, 'query: "hello" page_number: 1 results_per_page: 1', cx_node, timeout=15.0
        )

    def test_search_request_content_correct(self, cx_node, proc_output):
        wait_for_output(proc_output, 'query: "hello"', cx_node, timeout=15.0)

    def test_message_cleaned_up(self, cx_node, proc_output):
        wait_for_fact_retracted(
            proc_output, '(protobuf-msg (type "SearchRequest")', cx_node, timeout=15.0
        )


@launch_testing.post_shutdown_test()
class TestProtobufLinkedPluginShutdown(make_shutdown_test_class()):
    pass
