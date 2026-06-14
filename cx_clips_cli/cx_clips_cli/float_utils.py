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

from typing import Callable

from cx_clips_cli.float_item import FloatItem
from cx_clips_cli.keymap_utils import bind_custom_key
from prompt_toolkit.application import Application
from prompt_toolkit.formatted_text import FormattedText
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.key_binding.vi_state import InputMode
from prompt_toolkit.layout.containers import Float, Window
from prompt_toolkit.layout.controls import FormattedTextControl


async def open_selection_float(
    app: Application,
    active_floats: list,
    focus_after,
    items: list[FloatItem],
    on_select: Callable[[FloatItem], None],
    key_map: dict,
    title: str = '',
):
    import asyncio

    filter_text = ['']
    current_index = [0]
    done = asyncio.Event()

    def filtered() -> list[FloatItem]:
        q = filter_text[0].lower()
        if not q:
            return items
        return [a for a in items if q in a.label.lower() or q in a.hint.lower()]

    def _close(a):
        if the_float in active_floats:
            active_floats.remove(the_float)
        a.layout.focus(focus_after)
        done.set()
        a.invalidate()

    float_kb = KeyBindings()

    def move_up(event):
        current_index[0] = max(0, current_index[0] - 1)
        event.app.invalidate()

    bind_custom_key(float_kb, 'float_up', key_map, move_up)

    def move_down(event):
        its = filtered()
        current_index[0] = min(len(its) - 1, current_index[0] + 1)
        event.app.invalidate()

    bind_custom_key(float_kb, 'float_down', key_map, move_down)

    def backspace(event):
        filter_text[0] = filter_text[0][:-1]
        current_index[0] = 0
        event.app.invalidate()

    bind_custom_key(float_kb, 'float_backspace', key_map, backspace)

    def accept(event):
        its = filtered()
        if its:
            selected = its[current_index[0]]
            _close(event.app)
            on_select(selected)
        else:
            _close(event.app)

    bind_custom_key(float_kb, 'float_accept', key_map, accept)

    @float_kb.add('<any>')
    def type_filter(event):
        char = event.key_sequence[0].key
        if len(char) == 1 and char.isprintable():
            filter_text[0] += char
            current_index[0] = 0
            event.app.invalidate()

    def cancel(event):
        _close(event.app)

    bind_custom_key(float_kb, 'float_cancel', key_map, cancel, eager=True)

    def get_content():
        its = filtered()
        lines = []
        if title:
            lines += [
                ('class:bottom-toolbar class:bl-highlight reverse' + ' bold', f'  {title}\n')
            ]
            lines += [('class:bottom-toolbar class:bl-separator reverse', '─' * 44 + '\n')]
        if filter_text[0]:
            lines += [('class:bottom-toolbar class:bl-success reverse', f'  / {filter_text[0]}\n')]
        else:
            lines += [('class:bottom-toolbar class:bl-text reverse', '  type to filter...\n')]
        lines += [('class:bottom-toolbar class:bl-separator reverse', '─' * 44 + '\n')]
        if not its:
            lines += [('class:bottom-toolbar class:bl-text reverse', '  no matches\n')]
        else:
            for i, item in enumerate(its):
                selected = i == current_index[0]
                if selected:
                    row = [
                        ('class:bottom-toolbar class:bl-success reverse', f'  {item.label:<28}'),
                        ('class:bottom-toolbar class:bl-success reverse', f'  {item.hint:<12}  '),
                    ]
                else:
                    row = [
                        ('class:bottom-toolbar class:bl-text reverse', f'  {item.label:<28}'),
                        (
                            'class:bottom-toolbar class:bl-separator reverse',
                            f'  {item.hint:<12}  ',
                        ),
                    ]
                lines += row + [('', '\n')]
        return FormattedText(lines)

    height = len(items) + (4 if title else 3)

    float_window = Window(
        content=FormattedTextControl(get_content, focusable=True, key_bindings=float_kb),
        height=height,
        dont_extend_height=True,
        style='class:bottom-toolbar reverse',
    )

    the_float = Float(
        content=float_window, width=50, xcursor=False, ycursor=False, right=2, bottom=3
    )

    app.vi_state.input_mode = InputMode.INSERT
    active_floats.append(the_float)
    app.layout.focus(float_window)
    app.invalidate()

    await done.wait()
