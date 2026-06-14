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

# test/test_protobuf_plugin.py
import pytest

generate_test_description = pytest.mark.launch_test(
    launch_testing.markers.keep_alive(make_generate_test_description('protobuf.yaml'))
)


class TestProtobufPluginOutput(unittest.TestCase):

    def test_node_activated(self, cx_node, proc_output):
        wait_for_activated(proc_output, cx_node)

    def test_protobuf_plugin_loaded(self, cx_node, proc_output):
        wait_for_plugin_created(proc_output, 'protobuf', 'cx::ProtobufPlugin', cx_node)

    def test_peers_initialized(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'protobuf-init-example-peer', cx_node)

    def test_comm(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'protobuf-send-msg', cx_node)
        wait_for_rule_fire(proc_output, 'protobuf-read-msg', cx_node)

    def test_peer_facts_asserted(self, cx_node, proc_output):
        wait_for_fact_asserted(proc_output, '(peer 1 4444 4445)', cx_node)
        wait_for_fact_asserted(proc_output, '(peer 1 4445 4444)', cx_node)

    def test_messages_received(self, cx_node, proc_output):
        wait_for_output(proc_output, 'page_number: 4445 results_per_page: 4444', cx_node)
        wait_for_output(proc_output, 'page_number: 4444 results_per_page: 4445', cx_node)


@launch_testing.post_shutdown_test()
class TestProtobufPluginShutdown(make_shutdown_test_class()):

    def test_peers_closed_on_shutdown(self, cx_node, proc_output):
        wait_for_rule_fire(proc_output, 'protobuf-close-peer', cx_node)

    def test_protobuf_context_destroyed_on_shutdown(self, cx_node, proc_output):
        """Verify the protobuf context is destroyed cleanly."""
        wait_for_output(proc_output, 'Destroying clips context!', cx_node)
