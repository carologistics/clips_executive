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

# test_clips_cli.py
import os
from unittest.mock import MagicMock, patch

from cx_clips_cli.cli import (
    ClipsCLI,
    ClipsHistoryCompleter,
    _log_color_for_logical_name,
    is_complete,
)
import pytest

# ── is_complete ────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    'text,expected',
    [
        ('(facts)', True),
        (')(', False),
        ('(assert (foo bar))', True),
        ('(defrule r\n  =>\n  (assert (x)))', True),  # multiline
        ('(assert (foo', False),  # unbalanced
        ('', False),  # empty
        ('   ', False),  # whitespace only
        ('no parens', False),  # doesn't start with (
        ('(assert "hello (world")', True),  # string contains parens — should not count
        ('(assert "unclosed string)', False),  # debatable; documents current behaviour
    ],
)
def test_is_complete(text, expected):
    assert is_complete(text) == expected


# ── _log_color_for_logical_name ───────────────────────────────────────────────


@pytest.mark.parametrize(
    'name,expected',
    [
        ('stderr', 'log-error'),
        ('STDERR', 'log-error'),  # case-insensitive
        ('stdwrn', 'log-warn'),
        ('red', 'red'),
        ('blue', 'blue'),
        ('unknown', ''),
        ('', ''),
    ],
)
def test_log_color_for_logical_name(name, expected):
    assert _log_color_for_logical_name(name) == expected


# ── ClipsHistoryCompleter ─────────────────────────────────────────────────────


class FakeHistory:

    def __init__(self, entries):
        self._entries = entries

    def get_strings(self):
        return self._entries


class FakeDocument:

    def __init__(self, text):
        self.text_before_cursor = text


def test_completer_returns_matching_entries():
    history = FakeHistory(['(facts)', '(assert (foo))', '(agenda)'])
    completer = ClipsHistoryCompleter(history)
    results = list(completer.get_completions(FakeDocument('(a'), None))
    labels = [c.display for c in results]
    labels = [c.display[0][1] for c in results]
    assert '(assert (foo))' in labels
    assert '(agenda)' in labels
    assert '(facts)' not in labels
    assert '(assert (foo))' in labels
    assert '(agenda)' in labels
    assert '(facts)' not in labels


def test_completer_no_duplicates():
    history = FakeHistory(['(facts)', '(facts)'])
    completer = ClipsHistoryCompleter(history)
    results = list(completer.get_completions(FakeDocument('(f'), None))
    assert len(results) == 1


def test_completer_empty_input_returns_nothing():
    history = FakeHistory(['(facts)'])
    completer = ClipsHistoryCompleter(history)
    results = list(completer.get_completions(FakeDocument(''), None))
    assert results == []


# ── ClipsCLI (no live ROS) ────────────────────────────────────────────────────


@pytest.fixture
def mock_cli(tmp_path):
    """Build a ClipsCLI with all ROS internals stubbed out."""
    node = MagicMock()
    # make create_client / create_subscription return silent mocks
    node.create_client.return_value = MagicMock()
    node.create_subscription.return_value = MagicMock()

    executor = MagicMock()
    cli = ClipsCLI(
        node=node,
        cx_node_name='test_node',
        plugin_name='executive',
        executor=executor,
        dark_mode=False,
        log_dir=tmp_path,
    )
    return cli


def test_format_timestamp_pattern(mock_cli):
    import re

    ts = mock_cli._format_timestamp()
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2}', ts), f'Bad timestamp: {ts}'


def test_get_ros_log_dir_from_env(tmp_path):
    with patch.dict(os.environ, {'ROS_LOG_DIR': str(tmp_path)}):
        node, executor = MagicMock(), MagicMock()
        node.create_client.return_value = MagicMock()
        node.create_subscription.return_value = MagicMock()
        cli = ClipsCLI(node, 'n', 'p', executor, False)
        assert cli._get_ros_log_dir() == tmp_path


# ── send() picks the right client ─────────────────────────────────────────────


@pytest.mark.parametrize(
    'expr,expect_build',
    [
        ('(facts)', False),
        ('(defrule my-rule => (assert (x)))', True),
        ('(deftemplate foo (slot bar))', True),
        ('(defglobal ?*x* = 1)', True),
        ('(assert (foo))', False),
    ],
)
def test_send_routes_to_correct_client(mock_cli, expr, expect_build):
    mock_cli.env_name = 'main'
    mock_cli._call_command = MagicMock(return_value=(True, 'ok'))

    mock_cli.send(expr)

    called_client = mock_cli._call_command.call_args[0][0]
    if expect_build:
        assert called_client is mock_cli.build_client
    else:
        assert called_client is mock_cli.eval_client


# ── log streaming ─────────────────────────────────────────────────────────────


def test_start_stop_log_stream(mock_cli, tmp_path):
    ok, msg = mock_cli.start_log_stream()
    assert ok
    assert mock_cli._continuous_streaming
    assert mock_cli._log_file_handle is not None

    ok, msg = mock_cli.stop_log_stream()
    assert ok
    assert not mock_cli._continuous_streaming
    assert mock_cli._log_file_handle.closed


def test_start_stream_twice_fails(mock_cli):
    mock_cli.start_log_stream()
    ok, msg = mock_cli.start_log_stream()
    assert not ok
    mock_cli.stop_log_stream()


def test_stop_stream_when_not_started_fails(mock_cli):
    ok, msg = mock_cli.stop_log_stream()
    assert not ok


def test_write_to_log_file_persists(mock_cli, tmp_path):
    mock_cli.start_log_stream()
    mock_cli._write_to_log_file('[env/stdout] ', 'hello world')
    mock_cli.stop_log_stream()

    content = mock_cli._log_file_path.read_text()
    assert 'hello world' in content


# ── _on_log_msg filtering ─────────────────────────────────────────────────────


def _make_log_msg(env_name, logical_name, line):
    msg = MagicMock()
    msg.env_name = env_name
    msg.logical_name = logical_name
    msg.line = line
    return msg


def test_on_log_msg_appends_formatted_text(mock_cli):
    mock_cli.env_name = 'main'
    mock_cli._on_log_msg(_make_log_msg('main', 'stdout', 'hello'))
    texts = [t for _, t in mock_cli._log_formatted_text]
    assert any('hello' in t for t in texts)


def test_on_log_msg_filters_other_envs(mock_cli):
    mock_cli.env_name = 'main'
    mock_cli._on_log_msg(_make_log_msg('other', 'stdout', 'should be dropped'))
    assert mock_cli._log_formatted_text == []


def test_on_log_msg_stderr_gets_error_style(mock_cli):
    mock_cli.env_name = ''
    mock_cli._on_log_msg(_make_log_msg('any', 'stderr', 'bad thing'))
    styles = [s for s, _ in mock_cli._log_formatted_text]
    assert any('log-error' in s for s in styles)
