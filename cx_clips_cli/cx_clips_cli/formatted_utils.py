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

# ---------------------------------------------------------------------------
# Custom widgets for formatted text support
#
# This feature is adapted from the lira project under MIT license:
# Copyright (c) 2026 Carologistics
# MIT License
#
# Copyright (c) 2020 Python Ecuador
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# https://github.com/pythonecuador/lira/tree/master
# ---------------------------------------------------------------------------

from cx_clips_cli.keymap_utils import bind_custom_key
from prompt_toolkit.application import get_app
from prompt_toolkit.buffer import Buffer
from prompt_toolkit.document import Document
from prompt_toolkit.filters import Condition
from prompt_toolkit.formatted_text import fragment_list_to_text, split_lines, to_formatted_text
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.key_binding.vi_state import InputMode
from prompt_toolkit.layout import ConditionalMargin, ScrollbarMargin
from prompt_toolkit.layout.containers import Window
from prompt_toolkit.layout.controls import BufferControl
from prompt_toolkit.layout.processors import HighlightSelectionProcessor, Processor, Transformation


class FormatTextProcessor(Processor):
    """
    Custom processor to represent formatted text.
    Makes use of :py:class:`FormattedBufferControl`.
    """

    def apply_transformation(self, transformation_input):
        formatted_lines = transformation_input.buffer_control.formatted_lines
        lineno = transformation_input.lineno
        max_lineno = len(formatted_lines) - 1
        if lineno > max_lineno:
            log.warning(
                'Index error when parsing document. max_lineno=%s lineno=%s',
                max_lineno,
                lineno,
            )
            lineno = max_lineno
        line = formatted_lines[lineno]
        return Transformation(to_formatted_text(line))


class FormattedBufferControl(BufferControl):
    """Control to support formatted_text and mouse events."""

    def __init__(self, formatted_text, **kwargs):
        self.formatted_lines = list(split_lines(formatted_text))
        super().__init__(**kwargs)

    def update_formatted_text(self, formatted_text):
        self.formatted_lines = list(split_lines(formatted_text))

    def mouse_handler(self, mouse_event):
        """
        Handle formatted text handlers.
        Taken from ``FormattedTextControl.mouse_handler``.
        """
        response = super().mouse_handler(mouse_event)

        if mouse_event.position.y >= len(self.formatted_lines):
            return response

        self.select(mouse_event)
        return response

    def select(self, mouse_event):
        fragments = self.formatted_lines[mouse_event.position.y]
        # Find position in the fragment list.
        xpos = mouse_event.position.x + 1

        # Find mouse handler for this character.
        count = 0
        for item in fragments:
            count += len(item[1])
            if count >= xpos:
                if len(item) >= 3:
                    handler = item[2]
                    return handler(mouse_event)
                break


class FormattedTextArea:
    """Just like text area, but it accepts formatted content with scrolling and search support."""

    def __init__(
        self,
        text='',
        focusable=False,
        wrap_lines=True,
        width=None,
        height=None,
        scrollbar=False,
        dont_extend_height=True,
        dont_extend_width=False,
        read_only=True,
        initial_position=0,
        key_map=dict(),
    ):
        self.key_map = key_map
        self.read_only = read_only
        formatted_text = to_formatted_text(text)
        plain_text = fragment_list_to_text(formatted_text)
        self.buffer = Buffer(
            document=Document(plain_text, initial_position),
            read_only=Condition(lambda: self.read_only),
        )
        self.control = FormattedBufferControl(
            buffer=self.buffer,
            formatted_text=formatted_text,
            input_processors=[FormatTextProcessor(), HighlightSelectionProcessor()],
            key_bindings=self.get_key_bindings(),
            include_default_input_processors=False,
            focusable=focusable,
            focus_on_click=focusable,
            search_buffer_control=None,
        )
        self.scrollbar = scrollbar
        right_margins = [
            ConditionalMargin(
                ScrollbarMargin(display_arrows=True),
                filter=Condition(lambda: self.scrollbar),
            ),
        ]
        self.window = Window(
            content=self.control,
            width=width,
            height=height,
            wrap_lines=wrap_lines,
            right_margins=right_margins,
            dont_extend_height=dont_extend_height,
            dont_extend_width=dont_extend_width,
            allow_scroll_beyond_bottom=True,
        )

        # Search state
        self.search_query = ''
        self.search_index = 0  # Current match index
        self.search_matches = []  # List of match positions (cursor positions)

    @property
    def document(self):
        return self.buffer.document

    @document.setter
    def document(self, value):
        self.buffer.set_document(value, bypass_readonly=True)

    @property
    def text(self):
        return self.buffer.text

    @text.setter
    def text(self, text):
        formatted_text = to_formatted_text(text)
        self.control.update_formatted_text(formatted_text)
        plain_text = fragment_list_to_text(formatted_text)
        current_position = min(self.document.cursor_position, len(plain_text))
        self.document = Document(plain_text, current_position)

    def scroll_to_bottom(self):
        """Scroll to the end of the buffer."""
        self.buffer.cursor_position = len(self.buffer.text)

    def scroll_to_top(self):
        """Scroll to the beginning of the buffer."""
        self.buffer.cursor_position = 0

    def scroll_up(self, amount=5):
        """Scroll up by moving cursor backwards."""
        self.buffer.cursor_position = max(0, self.buffer.cursor_position - amount * 80)

    def scroll_down(self, amount=5):
        """Scroll down by moving cursor forwards."""
        self.buffer.cursor_position = min(
            len(self.buffer.text), self.buffer.cursor_position + amount * 80
        )

    def search(self, query: str, direction: str = 'forward'):
        """Search for query in buffer. direction='forward' or 'backward'."""
        if not query:
            self.search_matches = []
            self.search_index = 0
            return False

        text = self.buffer.text.lower()
        query_lower = query.lower()

        # Find all matches
        self.search_matches = []
        start = 0
        while True:
            pos = text.find(query_lower, start)
            if pos == -1:
                break
            self.search_matches.append(pos)
            start = pos + 1

        if not self.search_matches:
            return False

        # Move to first/next match
        if direction == 'forward':
            current_pos = self.buffer.cursor_position
            # Find the first match after current position
            for i, match_pos in enumerate(self.search_matches):
                if match_pos > current_pos:
                    self.search_index = i
                    self.buffer.cursor_position = match_pos
                    return True
            # Wrap around to first match
            self.search_index = 0
            self.buffer.cursor_position = self.search_matches[0]
            return True
        else:  # backward
            current_pos = self.buffer.cursor_position
            # Find the last match before current position
            for i in range(len(self.search_matches) - 1, -1, -1):
                if self.search_matches[i] < current_pos:
                    self.search_index = i
                    self.buffer.cursor_position = self.search_matches[i]
                    return True
            # Wrap around to last match
            self.search_index = len(self.search_matches) - 1
            self.buffer.cursor_position = self.search_matches[-1]
            return True

    def next_match(self):
        """Go to next search match."""
        if not self.search_matches:
            return False
        self.search_index = (self.search_index + 1) % len(self.search_matches)
        self.buffer.cursor_position = self.search_matches[self.search_index]
        return True

    def prev_match(self):
        """Go to previous search match."""
        if not self.search_matches:
            return False
        self.search_index = (self.search_index - 1) % len(self.search_matches)
        self.buffer.cursor_position = self.search_matches[self.search_index]
        return True

    def clear_search(self):
        """Clear search state."""
        self.search_query = ''
        self.search_matches = []
        self.search_index = 0

    def get_key_bindings(self):
        keys = KeyBindings()

        def copy_selection(event):
            data = self.buffer.copy_selection()
            if data.text:
                pass

        bind_custom_key(keys, 'log_copy', self.key_map, copy_selection)

        def scroll_up(event):
            self.scroll_up(5)

        bind_custom_key(keys, 'log_scroll_up', self.key_map, scroll_up)

        def scroll_down(event):
            self.scroll_down(5)

        bind_custom_key(keys, 'log_scroll_down', self.key_map, scroll_down)

        def goto_top(event):
            self.scroll_to_top()

        bind_custom_key(keys, 'log_goto_top', self.key_map, goto_top)

        def goto_bottom(event):
            self.scroll_to_bottom()

        bind_custom_key(keys, 'log_goto_bottom', self.key_map, goto_bottom)

        def search_forward(event):
            """Start forward search (vi-style)."""
            app = get_app()
            app.vi_state.input_mode = InputMode.INSERT
            event.app.layout.focus(event.app._search_input_window)

        bind_custom_key(keys, 'log_search_fwd', self.key_map, search_forward)

        def search_backward(event):
            """Start backward search (vi-style)."""
            app = get_app()
            app.vi_state.input_mode = InputMode.INSERT
            event.app.layout.focus(event.app._search_input_window)

        bind_custom_key(keys, 'log_search_bwd', self.key_map, search_backward)

        def proxy_next_match(event):
            self.next_match()

        bind_custom_key(keys, 'log_next_match', self.key_map, proxy_next_match)

        def proxy_prev_match(event):
            self.prev_match()

        bind_custom_key(keys, 'log_prev_match', self.key_map, proxy_prev_match)

        def swallow_escape(event):
            event.app.vi_state.input_mode = InputMode.NAVIGATION
            app = get_app()
            self.clear_search()
            search_buf = app._search_buffer
            search_buf.text = ''
            event.app.layout.focus(app._input_window)
            event.app.invalidate()

        bind_custom_key(keys, 'log_escape', self.key_map, swallow_escape)

        return keys

    def __pt_container__(self):
        return self.window
