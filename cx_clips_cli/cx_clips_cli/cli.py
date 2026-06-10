#!/usr/bin/env python3

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

import datetime
import os
from pathlib import Path
import threading

from cx_clips_cli.float_item import FloatItem
from cx_clips_cli.float_utils import open_selection_float
from cx_clips_cli.formatted_utils import FormattedTextArea
from cx_clips_cli.keymap_utils import (
    bind_custom_key,
    create_key_map,
    make_command_palette,
    resolve_keys,
)
from cx_clips_cli.style_utils import is_dark_terminal, make_style
from cx_msgs.msg import Log
from cx_msgs.srv import ClipsCommand, ListClipsEnvs, SetTopicLogging
from prompt_toolkit.application import Application, get_app
from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
from prompt_toolkit.buffer import Buffer
from prompt_toolkit.completion import Completer, Completion
from prompt_toolkit.enums import EditingMode
from prompt_toolkit.filters import has_focus, vi_insert_mode, vi_navigation_mode
from prompt_toolkit.formatted_text import FormattedText, to_formatted_text
from prompt_toolkit.history import FileHistory, InMemoryHistory
from prompt_toolkit.input.vt100_parser import KeyPress
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.key_binding.vi_state import InputMode
from prompt_toolkit.layout import Layout
from prompt_toolkit.layout.containers import FloatContainer, HSplit, VSplit, Window
from prompt_toolkit.layout.controls import BufferControl, FormattedTextControl
from prompt_toolkit.layout.processors import AppendAutoSuggestion, ConditionalProcessor
from prompt_toolkit.lexers import PygmentsLexer
from pygments.lexers.lisp import CommonLispLexer
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# keywords to decide if build is used over eval
BUILD_KEYWORDS = (
    'defrule',
    'deftemplate',
    'deffacts',
    'defglobal',
    'deffunction',
    'defmodule',
    'defclass',
    'defmessage-handler',
)


# text to be displayed on startup and 'help' command
def get_help_text(key_map):
    def _format(action):
        chords = key_map.get(action, [])
        if not chords:
            return '?'
        return ' + '.join(chords[0])

    return f"""
Command Line Interface (CLI) for the ROS CX.

The CLI has three areas: the log window (top), the CLI prompt (middle), and the status bar (bottom).
Operates vi-style.
  The CLI prompt uses vi-style modal editing with two modes.
  INSERT mode: the prompt is active, type and submit CLIPS expressions.
    Press ESC to switch to NORMAL mode.
  NORMAL mode: use shortcuts from the command palette.
    Press i to return to INSERT mode.
    Press {_format('command_palette')} to open the command palette with all available shortcuts.

Log window
  Displays CLIPS output related to expressions sent via CLI.
  Filters other CLIPS output from the ROS CX unless disabled (see command palette).
  Automatically scrolls to newest lines unless disabled (see command palette).
  While focused (status ON LOG): {_format('log_search_fwd')} and {_format('log_search_bwd')} search forward and backward, {_format('log_next_match')} and {_format('log_prev_match')} cycle through matches.
  Contents of the log window can be saved to a file (see command palette).

CLI prompt
  Type and submit CLIPS commands to the currently active environment in INSERT mode.
  Tip: press ESC to enter normal mode and access shortcuts via space+m.
"""


async def open_command_palette(
    cli, app, active_floats, input_window, log_area, key_map, command_palette
):
    """Creates a float selector menu for the available commands."""
    items = [FloatItem(a.label, a.hint, a.keys) for a in make_command_palette(key_map)]

    def on_select(item: FloatItem):
        def feed_keys():
            app.vi_state.input_mode = InputMode.NAVIGATION
            for key in item.keys:
                app.key_processor.feed(KeyPress(resolve_keys(key)))

        import asyncio

        asyncio.get_event_loop().call_soon(feed_keys)

    await open_selection_float(
        app,
        active_floats,
        input_window,
        items,
        on_select,
        key_map=key_map,
        title='command palette',
    )


async def select_env(cli, app, active_floats, input_window, key_map):
    """Creates a float selector menu for the active environments."""
    ok, envs, err = cli.list_envs()
    if not ok or not envs:
        cli.set_status(False, err or 'no environments available')
        app.invalidate()
        return

    items = [FloatItem(e) for e in envs]

    def on_select(item: FloatItem):
        cli.env_name = item.label
        cli.set_status(True, f"switched to '{item.label}'")
        app.invalidate()

    await open_selection_float(
        app,
        active_floats,
        input_window,
        items,
        on_select,
        key_map=key_map,
        title='select environment',
    )


class ClipsHistoryCompleter(Completer):
    """Complete from history entries that start with current input."""

    def __init__(self, history: InMemoryHistory):
        self._history = history

    def get_completions(self, document, complete_event):
        text = document.text_before_cursor
        if not text:
            return
        seen = set()
        for entry in reversed(list(self._history.get_strings())):
            if entry.startswith(text) and entry != text:
                if entry not in seen:
                    seen.add(entry)
                    yield Completion(entry[len(text) :], start_position=0, display=entry)


def _log_color_for_logical_name(logical_name: str) -> str:
    """Return style matching the logical name from the ROS CX."""
    ln = logical_name.lower()
    if ln == 'stderr':
        return 'log-error'
    if ln == 'stdwrn':
        return 'log-warn'
    if ln == 'red':
        return 'red'
    if ln == 'blue':
        return 'blue'
    if ln == 'green':
        return 'green'
    if ln == 'yellow':
        return 'yellow'
    if ln == 'magenta':
        return 'magenta'
    if ln == 'cyan':
        return 'cyan'
    if ln == 'white':
        return 'white'
    if ln == 'bold':
        return 'bold'
    return ''


def is_complete(text: str) -> bool:
    """Return True when parens are balanced and input is non-empty."""
    depth = 0
    in_string = False
    escape = False
    for ch in text:
        if escape:
            escape = False
            continue
        if ch == '\\' and in_string:
            escape = True
            continue
        if ch == '"':
            in_string = not in_string
            continue
        if in_string:
            continue
        if ch == '(':
            depth += 1
        elif ch == ')':
            depth -= 1
    stripped = text.strip()
    return depth == 0 and stripped.startswith('(') and len(stripped) > 0


# ---------------------------------------------------------------------------
# Main CLI class
# ---------------------------------------------------------------------------


class ClipsCLI:
    def __init__(
        self,
        node: Node,
        cx_node_name: str,
        plugin_name: str,
        executor: rclpy.executors.SingleThreadedExecutor,
        dark_mode: bool,
        log_dir: Path | None = None,
    ):
        self.node = node
        self.executor = executor
        self.cx_node_name = cx_node_name.strip('/')
        self.plugin_name = plugin_name
        self.dark_mode = dark_mode
        self.env_name = ''
        self.known_envs: list[str] = []
        self.pending_action: str | None = None

        # Log streaming state
        self.filter_enabled = True  # whether we print received log msgs
        self.ros_logging_enabled = False  # whether the CLIPS engine sends logs at all
        self._log_sub = None  # ROS subscription handle
        self._log_lock = threading.Lock()  # guards console writes from the sub thread

        self.last_ok: bool | None = None  # None = neutral
        self.last_msg: str = ''
        self._log_formatted_text: list = []  # List of (style, text) tuples
        self._log_dirty = False
        self._log_area: FormattedTextArea | None = None
        self._auto_scroll = True  # Enable auto-scroll by default

        # Log file streaming state
        self._log_dir = log_dir or self._get_ros_log_dir()
        self._log_file_path: Path | None = None
        self._log_file_handle = None
        self._file_write_lock = threading.Lock()
        self._continuous_streaming = False

        self.history = self._make_history()
        self._setup_clients()

    def _make_history(self):
        try:
            history_file = self._log_dir / 'cx_clips_cli_history'
            history_file.parent.mkdir(parents=True, exist_ok=True)
            return FileHistory(str(history_file))
        except Exception as e:
            log.warning(f"Could not open history file, falling back to in-memory: {e}")
            return InMemoryHistory()

    def _get_ros_log_dir(self) -> Path:
        """Get ROS log directory from ROS_LOG_DIR or default."""
        ros_log_dir = os.environ.get('ROS_LOG_DIR')
        if ros_log_dir:
            return Path(ros_log_dir)

        # Default ROS log directory
        home = Path.home()
        ros_home = os.environ.get('ROS_HOME', str(home / '.ros'))
        return Path(ros_home) / 'log'

    def _prefix(self) -> str:
        return f'{self.cx_node_name}/{self.plugin_name}'

    def _setup_clients(self):
        p = self._prefix()
        self.eval_client = self.node.create_client(ClipsCommand, f'{p}/eval')
        self.build_client = self.node.create_client(ClipsCommand, f'{p}/build')
        self.pause_client = self.node.create_client(Trigger, f'{p}/pause')
        self.resume_client = self.node.create_client(Trigger, f'{p}/resume')
        self.tick_client = self.node.create_client(Trigger, f'{p}/tick_once')
        self.list_client = self.node.create_client(ListClipsEnvs, f'{self.cx_node_name}/list_envs')
        self.set_logging_client = self.node.create_client(
            SetTopicLogging, f'{self.cx_node_name}/set_topic_logging'
        )

        # Subscribe to the log topic unconditionally; printing is gated by self.filter_enabled
        log_topic = f'{self.cx_node_name}/clips_log'
        self._log_sub = self.node.create_subscription(
            Log,
            log_topic,
            self._on_log_msg,
            10,
        )

    # ------------------------------------------------------------------
    # Log message handler (called from the executor spin thread)
    # ------------------------------------------------------------------
    def _on_log_msg(self, msg: Log):
        if self.env_name and msg.env_name and msg.env_name != self.env_name:
            return

        with self._log_lock:
            # Determine color based on logical name
            color = _log_color_for_logical_name(msg.logical_name)
            prefix = f'[{msg.env_name}/{msg.logical_name}] '

            # Add prefix
            self._log_formatted_text.append(('class:log-prefix', prefix))

            # Add log line with appropriate color
            if color:
                self._log_formatted_text.append((f'class:{color}', msg.line + '\n'))
            else:
                self._log_formatted_text.append(('', msg.line + '\n'))

            self._log_dirty = True

            # Stream to file if enabled
            self._write_to_log_file(prefix, msg.line)

    # ------------------------------------------------------------------
    # Log file operations
    # ------------------------------------------------------------------

    def _format_timestamp(self) -> str:
        """Format timestamp in the format: YYYY-MM-DD-HH-MM-SS"""
        return datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

    def _create_latest_symlink(self, log_file_ts: Path, log_file: Path) -> bool:
        """Create a symlink to _latest.log pointing to the current log file."""
        try:
            latest_link = log_file.parent / f"{log_file.stem}_latest.log"

            # Remove existing symlink if it exists
            if latest_link.exists() or latest_link.is_symlink():
                latest_link.unlink()

            # Create new symlink
            latest_link.symlink_to(log_file_ts.name)
            return True
        except Exception as e:
            log.error(f"Failed to create symlink: {e}")
            return False

    def _write_to_log_file(self, prefix: str, line: str):
        """Write a log line to the active log file (thread-safe)."""
        if not self._log_file_handle or self._log_file_handle.closed:
            return

        try:
            with self._file_write_lock:
                self._log_file_handle.write(f'{prefix}{line}\n')
                self._log_file_handle.flush()
        except Exception as e:
            log.error(f"Failed to write to log file: {e}")

    def start_log_stream(self, file_path: str | Path | None = None) -> tuple[bool, str]:
        """
        Start continuous streaming of logs to a file in ROS log directory.
        If file_path is None, generates a timestamp-based filename.
        Creates a symlink to _latest.log for easy access.
        Returns (success, message).
        """
        if self._continuous_streaming:
            return False, 'Continuous streaming already active'

        timestamp = self._format_timestamp()
        if file_path is None:
            # Generate filename with timestamp in ROS log directory
            f"_{self.env_name}" if self.env_name else ''
            file_path = self._log_dir / f"cx_clips_cli_{timestamp}.log"
            file_path_no_ts = self._log_dir / f"cx_clips_cli.log"
        else:
            file_path = Path(file_path, f"cx_clips_cli_{timestamp}.log")
            file_path_no_ts = Path(file_path, f"cx_clips_cli.log")
            # If relative path, put it in the log directory
            if not file_path.is_absolute():
                file_path = self._log_dir / file_path

        try:
            # Create parent directories if needed
            file_path.parent.mkdir(parents=True, exist_ok=True)

            # Close any existing file handle
            if self._log_file_handle and not self._log_file_handle.closed:
                self._log_file_handle.close()

            # Open file in append mode
            self._log_file_handle = open(file_path, 'a', encoding='utf-8')
            self._log_file_path = file_path
            self._continuous_streaming = True

            # Create symlink to _latest.log
            self._create_latest_symlink(file_path, file_path_no_ts)

            return True, f"Log streaming started: {file_path}"
        except Exception as e:
            return False, f"Failed to open log file: {e}"

    def stop_log_stream(self) -> tuple[bool, str]:
        """Stop continuous streaming to file. Returns (success, message)."""
        if not self._continuous_streaming:
            return False, 'Continuous streaming not active'

        try:
            if self._log_file_handle and not self._log_file_handle.closed:
                self._log_file_handle.close()

            self._continuous_streaming = False
            file_path = self._log_file_path
            return True, f"Log streaming stopped: {file_path}"
        except Exception as e:
            return False, f"Failed to close log file: {e}"

    def save_log_buffer(self, file_path: str | Path | None = None) -> tuple[bool, str]:
        """
        Save the current log buffer to a file in ROS log directory.
        If file_path is None, generates a timestamp-based filename.
        Creates a symlink to _latest.log for easy access.
        Returns (success, message).
        """
        if not self._log_area:
            return False, 'Log area not initialized'

        if file_path is None:
            # Generate filename with timestamp in ROS log directory
            timestamp = self._format_timestamp()
            f"_{self.env_name}" if self.env_name else ''
            file_path = self._log_dir / f"cx_clips_cli_snapshot_{timestamp}.log"
            file_path_no_ts = self._log_dir / f"cx_clips_cli_snapshot.log"
        else:
            file_path = Path(file_path)
            file_path_no_ts = Path(file_path, f"cx_clips_cli_snapshot.log")
            # If relative path, put it in the log directory
            if not file_path.is_absolute():
                file_path = self._log_dir / file_path

        try:
            # Create parent directories if needed
            file_path.parent.mkdir(parents=True, exist_ok=True)

            # Write plain text buffer to file
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(self._log_area.text)

            # Create symlink to _latest.log
            self._create_latest_symlink(file_path, file_path_no_ts)

            return True, f"Log saved to: {file_path}"
        except Exception as e:
            return False, f"Failed to save log file: {e}"

    def toggle_log_stream(self, file_path: str | Path | None = None) -> str:
        """Toggle continuous log streaming on/off."""
        if self._continuous_streaming:
            ok, msg = self.stop_log_stream()
        else:
            ok, msg = self.start_log_stream(file_path)
        return ok, msg

    # ------------------------------------------------------------------
    # ROS service helpers
    # ------------------------------------------------------------------

    def _append_log(self, text: str, style: str = ''):
        """Append text to the log area."""
        if self._log_area is None:
            return

        with self._log_lock:
            self._log_formatted_text.append((style, text + '\n'))
            self._log_dirty = True

    def _spin_until_complete(self, future, timeout_sec=5.0):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=timeout_sec)
        if not future.done():
            raise TimeoutError('Service call timed out')

    def _call_trigger(self, client) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=2.0):
            return False, 'Service not available'
        future = client.call_async(Trigger.Request())
        self._spin_until_complete(future)
        result = future.result()
        return result.success, result.message

    def _call_command(self, client, command: str) -> tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=2.0):
            return False, 'Service not available'
        req = ClipsCommand.Request()
        req.env_name = self.env_name
        req.command = command
        future = client.call_async(req)
        self._spin_until_complete(future)
        result = future.result()
        return result.success, result.message

    def list_envs(self) -> tuple[bool, list[str], str]:
        if not self.list_client.wait_for_service(timeout_sec=2.0):
            return False, [], 'Service not available'
        future = self.list_client.call_async(ListClipsEnvs.Request())
        self._spin_until_complete(future)
        result = future.result()
        if result.success:
            self.known_envs = list(result.envs)
        return result.success, list(result.envs), result.error

    def set_ros_logging(self, enabled: bool) -> tuple[bool, str]:
        """Call the set_clips_logging service for the active environment."""
        if not self.set_logging_client.wait_for_service(timeout_sec=2.0):
            return False, 'Service not available'
        req = SetTopicLogging.Request()
        req.env_name = self.env_name
        req.enabled = enabled
        future = self.set_logging_client.call_async(req)
        self._spin_until_complete(future)
        result = future.result()
        if result.success:
            self.ros_logging_enabled = enabled
        return result.success, ''

    def send(self, text: str) -> tuple[bool, str]:
        stripped = text.strip()
        is_build = any(stripped.startswith(f'({kw}') for kw in BUILD_KEYWORDS)
        client = self.build_client if is_build else self.eval_client
        return self._call_command(client, stripped)

    # ------------------------------------------------------------------
    # Log toggle helpers
    # ------------------------------------------------------------------

    def disable_filter(self) -> str:
        """Enable local printing and optionally activate ROS-side logging."""
        # Also ensure the CLIPS engine is publishing if the service exists
        if self.set_logging_client.wait_for_service(timeout_sec=1.0):
            ok, msg = self.set_ros_logging(True)
            if not ok:
                return f"ROS log filter toggle failed: {msg}"
        else:
            return 'ROS log filter toggle failed: service not reachable'
        self.filter_enabled = False
        return 'Log filter off'

    def enable_filter(self) -> str:
        """Disable local printing (does NOT stop the CLIPS engine from publishing)."""
        # Also ensure the CLIPS engine is publishing if the service exists
        if self.set_logging_client.wait_for_service(timeout_sec=1.0):
            ok, msg = self.set_ros_logging(False)
            if not ok:
                return f"ROS log filter toggle failed: {msg}"
        else:
            return 'ROS log filter toggle failed: service not reachable'
        self.filter_enabled = True
        return 'Log filter on'

    def toggle_log_filter(self) -> str:
        if self.filter_enabled:
            return self.disable_filter()
        else:
            return self.enable_filter()

    def toggle_auto_scroll(self) -> str:
        """Toggle auto-scroll on/off."""
        self._auto_scroll = not self._auto_scroll
        return f"Auto-scroll {'on' if self._auto_scroll else 'off'}"

    def set_status(self, ok: bool, msg: str = ''):
        self.last_ok = ok
        self.last_msg = msg
        if msg:
            style = 'class:log-ok' if ok else 'class:log-error'
            self._append_log(msg, style)

    # ------------------------------------------------------------------
    # Bottom toolbar
    # ------------------------------------------------------------------
    def bottom_toolbar(self, key_map):
        env_display = self.env_name if self.env_name else 'default'

        sep = ('class:bl-separator', '  |  ')
        mode_label = 'INSERT'
        app = get_app()
        navigation_tip = [
            ('', ' '),
            ('class:bl-highlight', 'esc'),
            ('class:bl-text', '  normal mode'),
            ('class:bl-separator', '  |  '),
        ]
        if vi_navigation_mode():
            mode_label = 'NORMAL'
            navigation_tip = [
                ('', ' '),
                ('class:bl-highlight', ' i '),
                ('class:bl-text', '  insert mode'),
                sep,
            ]
        if app.layout.current_window == self._log_area.window:
            mode_label = 'ON LOG'
            navigation_tip = [
                ('', ' '),
                ('class:bl-highlight', 'esc'),
                ('class:bl-text', '  normal mode'),
                sep,
            ]
        if app.layout.current_window == app._search_input_window:
            mode_label = 'SEARCH'
            navigation_tip = [
                ('', ' '),
                ('class:bl-highlight', 'esc'),
                ('class:bl-text', '  log window '),
                sep,
            ]
        mode = [('class:bl-highlight' + ' bold', f'{mode_label}'), sep]

        ctx = [
            ('class:bl-text', 'node: ' + self.cx_node_name),
            sep,
            ('class:bl-text', 'plugin: ' + self.plugin_name),
            sep,
            ('class:bl-highlight', 'env:' + env_display),
        ]

        def pill(label, shortcut, active):
            prefix = [('class::bl-separator', '   ')]
            # if vi_navigation_mode():
            #     prefix = [('class:bl-highlight', f' {shortcut}  ')]
            if active:
                return prefix + [('class:bl-success' + ' bold', label)]
            return prefix + [('class:bl-failure' + ' bold', label)]

        indicators = (
            pill('filter', 'l', self.filter_enabled)
            + pill('scroll', 's', self._auto_scroll)
            + pill('stream', 'T', self._continuous_streaming)
        )

        if self.last_ok is None:
            status = [('class:bl-separator', '  ret '), sep]
        elif self.last_ok:
            status = [('class:bl-success', f'✓ {" ok "}'), sep]
        else:
            status = [('class:bl-failure', f'✗ {"fail"}'), sep]

        line1 = [('', ' ')] + status + mode + indicators

        if vi_navigation_mode():

            def hint(key, label):
                return [('class:bl-highlight', key), ('class:bl-text', f' {label}  ')]

            def _hint_from_key_map(key_map, action):
                chords = key_map.get(action, [])
                if not chords:
                    return '?'
                return ' + '.join(chords[0])

            line1 += (
                [sep]
                + hint(_hint_from_key_map(key_map, 'command_palette'), 'command menu')
                + [sep]
                + hint(_hint_from_key_map(key_map, 'quit'), 'quit')
            )

        line2 = navigation_tip + ctx

        return FormattedText(line1 + [('', '\n')] + line2)


# ---------------------------------------------------------------------------
# Key bindings
# ---------------------------------------------------------------------------


def make_bindings(
    cli: ClipsCLI,
    log_area: 'FormatedTextArea',
    input_window,
    search_input_window,
    active_floats,
    key_map,
    command_palette,
) -> KeyBindings:

    kb = KeyBindings()

    def append(event):
        event.app.current_buffer.cursor_position = len(event.app.current_buffer.text)
        event.app.current_buffer.set_document(
            event.app.current_buffer.document, bypass_readonly=False
        )
        event.app.vi_state.input_mode = InputMode.INSERT

    bind_custom_key(kb, 'append', key_map, append, eager=True, filter=vi_navigation_mode)

    def handle_enter(event):
        buf = event.app.current_buffer
        text = buf.text.strip()
        if not text:
            return
        if not text.startswith('('):
            cli.set_status(False, 'Unknown input — use (expr) or :command')
            event.app.invalidate()
            return
        if is_complete(text):
            cli.history.append_string(text)
            buf.reset()
            ok, msg = cli.send(text)
            cli.set_status(ok, msg)
            event.app.invalidate()
        else:
            buf.insert_text('\n')

    bind_custom_key(kb, 'submit', key_map, handle_enter, filter=vi_insert_mode)

    def pause(event):
        ok, msg = cli._call_trigger(cli.pause_client)
        cli.set_status(ok, msg)
        event.app.invalidate()

    bind_custom_key(kb, 'pause', key_map, pause, filter=vi_navigation_mode)

    def command_palette(event):
        async def run():
            await open_command_palette(
                cli, get_app(), active_floats, input_window, log_area, key_map, command_palette
            )

        event.app.create_background_task(run())

    bind_custom_key(kb, 'command_palette', key_map, command_palette, filter=vi_navigation_mode)

    def resume(event):
        ok, msg = cli._call_trigger(cli.resume_client)
        cli.set_status(ok, msg)
        event.app.invalidate()

    bind_custom_key(kb, 'resume', key_map, resume, filter=vi_navigation_mode)

    def tick(event):
        ok, msg = cli._call_trigger(cli.tick_client)
        cli.set_status(ok, msg)
        event.app.invalidate()

    bind_custom_key(kb, 'tick', key_map, tick, filter=vi_navigation_mode)

    def list_envs(event):
        async def run_selector():
            await select_env(cli, get_app(), active_floats, input_window, key_map)

        event.app.create_background_task(run_selector())

    bind_custom_key(kb, 'select_env', key_map, list_envs, filter=vi_navigation_mode)

    def toggle_log_filter(event):
        cli.toggle_log_filter()
        cli.set_status(True, f"Log {'on' if cli.filter_enabled else 'off'}")
        event.app.invalidate()

    bind_custom_key(kb, 'toggle_filter', key_map, toggle_log_filter, filter=vi_navigation_mode)

    def open_log(event):
        app = get_app()
        app.layout.focus(log_area.window)
        cli.set_status(True, 'Log window active (PgUp/PgDn, g/G, /, n, space+i)')
        event.app.invalidate()

    bind_custom_key(kb, 'open_log', key_map, open_log, filter=vi_navigation_mode)

    def toggle_scroll(event):
        msg = cli.toggle_auto_scroll()
        cli.set_status(True, msg)
        event.app.invalidate()

    bind_custom_key(kb, 'toggle_scroll', key_map, toggle_scroll, filter=vi_navigation_mode)

    def save_log(event):
        ok, msg = cli.save_log_buffer()
        cli.set_status(ok, msg)
        event.app.invalidate()

    bind_custom_key(kb, 'save_log', key_map, save_log, filter=vi_navigation_mode)

    def toggle_stream(event):
        ok, msg = cli.toggle_log_stream()
        cli.set_status(ok, msg)
        event.app.invalidate()

    bind_custom_key(kb, 'toggle_stream', key_map, toggle_stream, filter=vi_navigation_mode)

    def clear_log(event):
        cli._log_formatted_text.clear()
        cli._log_dirty = True
        event.app.invalidate()

    bind_custom_key(kb, 'clear_log', key_map, clear_log, filter=vi_navigation_mode)

    def show_help(event):
        cli.set_status(True, get_help_text(key_map))
        event.app.invalidate()

    bind_custom_key(kb, 'show_help', key_map, show_help, filter=vi_navigation_mode)

    def quit_app(event):
        if cli._continuous_streaming:
            cli.stop_log_stream()
        raise SystemExit(0)

    bind_custom_key(kb, 'quit', key_map, quit_app, filter=vi_navigation_mode)

    def clear_input(event):
        event.app.current_buffer.reset()

    bind_custom_key(kb, 'clear_input', key_map, clear_input, eager=True, filter=vi_insert_mode)

    def start_forward_search(event):
        app = get_app()
        search_buf = app._search_buffer
        search_buf.text = ''
        app._search_direction = 'forward'
        app.vi_state.input_mode = InputMode.INSERT
        app.layout.focus(search_input_window)
        event.app.invalidate()

    bind_custom_key(kb, 'log_search_fwd', key_map, start_forward_search)

    def start_backward_search(event):
        app = get_app()
        search_buf = app._search_buffer
        search_buf.text = ''
        app._search_direction = 'backward'
        app.vi_state.input_mode = InputMode.INSERT
        app.layout.focus(search_input_window)
        event.app.invalidate()

    bind_custom_key(kb, 'log_search_bwd', key_map, start_backward_search)

    def accept_suggestion(event):
        buf = event.app.current_buffer
        suggestion = buf.suggestion
        if suggestion:
            buf.insert_text(suggestion.text)

    bind_custom_key(kb, 'accept_suggestion', key_map, accept_suggestion, filter=vi_insert_mode)

    return kb


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    rclpy.init()
    node = rclpy.create_node('cx_clips_cli')

    node.declare_parameter('cx_node_name', 'clips_manager')
    node.declare_parameter('plugin_name', 'executive')
    node.declare_parameter('dark_mode', is_dark_terminal())
    node.declare_parameter('log_dir', '')
    node.declare_parameter('pygments_style', '')
    node.declare_parameter('style_overrides', '')
    node.declare_parameter('keymap_overrides', '')

    cx_node_name = node.get_parameter('cx_node_name').get_parameter_value().string_value
    plugin_name = node.get_parameter('plugin_name').get_parameter_value().string_value
    dark_mode = node.get_parameter('dark_mode').get_parameter_value().bool_value
    log_dir_param = node.get_parameter('log_dir').get_parameter_value().string_value
    log_dir = Path(log_dir_param) if log_dir_param else None

    pygments_style = node.get_parameter('pygments_style').get_parameter_value().string_value
    style_overrides = node.get_parameter('style_overrides').get_parameter_value().string_value
    key_map_overrides = node.get_parameter('keymap_overrides').get_parameter_value().string_value

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    cli = ClipsCLI(node, cx_node_name, plugin_name, executor, dark_mode, log_dir)

    ok, envs, err = cli.list_envs()
    if ok and envs:
        cli.env_name = envs[0]

    key_map = create_key_map(key_map_overrides)
    command_palette = make_command_palette(key_map)

    # Create the log area
    log_area = FormattedTextArea(
        text='',
        focusable=True,
        wrap_lines=True,
        scrollbar=True,
        dont_extend_height=False,
        key_map=key_map,
    )
    cli._log_area = log_area

    # Create input buffer and area
    input_buffer = Buffer(
        multiline=True,
        name='DEFAULT_BUFFER',
        history=cli.history,
        completer=ClipsHistoryCompleter(cli.history),
        complete_while_typing=True,
        auto_suggest=AutoSuggestFromHistory(),
    )

    input_window = VSplit(
        [
            Window(
                content=FormattedTextControl('CLIPS> '),
                width=7,
                dont_extend_height=True,
            ),
            Window(
                content=BufferControl(
                    buffer=input_buffer,
                    lexer=PygmentsLexer(CommonLispLexer),
                    focusable=True,
                    input_processors=[
                        ConditionalProcessor(
                            AppendAutoSuggestion(),
                            has_focus(input_buffer),
                        )
                    ],
                ),
                height=3,
                dont_extend_height=True,
                wrap_lines=True,
            ),
        ]
    )

    # Create search input
    search_buffer = Buffer()

    def search_accept(event):
        """Handle search query submission."""
        query = search_buffer.text
        if not query:
            log_area.clear_search()
            event.app.layout.focus(log_area.window)
            return

        direction = (
            event.app._search_direction if hasattr(event.app, '_search_direction') else 'forward'
        )
        if log_area.search(query, direction):
            log_area.search_query = query
            # Show match count
            match_count = len(log_area.search_matches)
            if match_count > 0:
                current_match = log_area.search_index + 1
                cli.set_status(True, f"Match {current_match}/{match_count}")
            event.app.layout.focus(log_area.window)
        else:
            cli.set_status(False, f"No matches for '{query}'")
        event.app.invalidate()

    search_kb = KeyBindings()

    @search_kb.add('enter')
    def _(event):
        search_accept(event)

    @search_kb.add('escape')
    def _(event):
        """Cancel search."""
        log_area.clear_search()
        event.app.layout.focus(log_area.window)
        event.app.invalidate()

    search_input_window = Window(
        content=BufferControl(
            buffer=search_buffer,
            key_bindings=search_kb,
            focusable=True,
        ),
        height=1,
        dont_extend_height=True,
        style='class:search-prompt',
    )

    toolbar_window = Window(
        content=FormattedTextControl(lambda: cli.bottom_toolbar(key_map)),
        height=2,
        dont_extend_height=True,
        style='class:bottom-toolbar',
    )

    # Mutable float list — we'll inject the env selector here
    active_floats = []

    layout = Layout(
        FloatContainer(
            content=HSplit(
                [
                    log_area.window,
                    Window(height=1, char='─', style='class:separator'),
                    search_input_window,
                    input_window,
                    toolbar_window,
                ]
            ),
            floats=active_floats,
        ),
        focused_element=input_window,
    )

    style = make_style(dark_mode, pygments_style, style_overrides)

    kb = make_bindings(
        cli, log_area, input_window, search_input_window, active_floats, key_map, command_palette
    )

    app = Application(
        layout=layout,
        key_bindings=kb,
        style=style,
        full_screen=True,
        mouse_support=False,
        editing_mode=EditingMode.VI,
    )

    def check_float_consistency(app):
        if active_floats and vi_navigation_mode():
            active_floats.clear()
            app.layout.focus(input_window)

    app.before_render += check_float_consistency

    # Store references for search handling
    app._search_buffer = search_buffer
    app._input_window = input_window
    app._search_input_window = search_input_window
    app._search_direction = 'forward'
    app.vi_state.input_mode = InputMode.NAVIGATION

    # Periodically invalidate when log is dirty
    async def refresh_log():
        import asyncio

        while True:
            await asyncio.sleep(0.1)
            if cli._log_dirty:
                cli._log_dirty = False
                with cli._log_lock:
                    log_area.text = to_formatted_text(cli._log_formatted_text)
                    if cli._auto_scroll:
                        log_area.scroll_to_bottom()
                app.invalidate()

    import asyncio

    cli.set_status(True, get_help_text(key_map))

    async def run():
        loop = asyncio.get_event_loop()
        loop.create_task(refresh_log())
        await app.run_async(
            pre_run=lambda: setattr(app.vi_state, 'input_mode', InputMode.NAVIGATION)
        )

    asyncio.run(run())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
