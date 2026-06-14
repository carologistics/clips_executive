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


from cx_clips_cli.float_item import FloatItem
from prompt_toolkit.application import get_app

DEFAULT_KEY_MAP = {
    'append': [['a']],
    'submit': [['enter']],
    'clear_input': [['c-c']],
    'accept_suggestion': [['c-space'], ['c-f'], ['c-right']],
    'pause': [['space', 'p']],
    'resume': [['space', 'r']],
    'tick': [['space', 't']],
    'command_palette': [['space', 'm']],
    'select_env': [['space', 'e']],
    'toggle_filter': [['space', 'f']],
    'open_log': [['space', 'l']],
    'toggle_scroll': [['space', 's']],
    'save_log': [['space', 'S']],
    'toggle_stream': [['space', 'T']],
    'clear_log': [['space', 'd']],
    'show_help': [['space', 'h']],
    'quit': [['space', 'q']],
    'log_search_fwd': [['/']],
    'log_search_bwd': [['?']],
    'log_next_match': [['n']],
    'log_prev_match': [['N']],
    'log_scroll_up': [['pageup']],
    'log_scroll_down': [['pagedown']],
    'log_goto_top': [['g']],
    'log_goto_bottom': [['G']],
    'log_escape': [['escape']],
    'log_copy': [['c-c']],
    'float_up': [['up']],
    'float_down': [['down']],
    'float_backspace': [['backspace']],
    'float_accept': [['enter']],
    'float_cancel': [['escape'], ['c-c']],
}

# commands as shown in the command palette
COMMAND_PALETTE_ACTIONS: list[tuple[str, str]] = [
    ('show_help', 'help'),
    ('pause', 'pause engine'),
    ('resume', 'resume engine'),
    ('tick', 'tick engine (run)'),
    ('open_log', 'focus log window'),
    ('clear_log', 'clear log buffer'),
    ('save_log', 'save log to file'),
    ('toggle_stream', 'toggle stream log to file'),
    ('toggle_filter', 'toggle filter of CX output'),
    ('toggle_scroll', 'toggle log window autoscroll'),
    ('select_env', 'select environment'),
    ('quit', 'quit'),
]

# needed to translate between keymap keys and FloatItems
KEY_NAME_MAP = {
    'space': ' ',
}


def _parse_keymap_file(path: str) -> dict:
    overrides = {}
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if '=' not in line:
                    continue
                action, _, value = line.partition('=')
                action = action.strip()
                # Split alternates on comma, then each chord on whitespace
                chords = [chord.strip().split() for chord in value.split(',') if chord.strip()]
                if chords:
                    overrides[action] = chords
    except FileNotFoundError:
        app = get_app()
        app._cli.set_status(False, f'Keymap override file not found: {path}')
    except Exception as e:
        app._cli.set_status(False, f'Failed to load keymap overrides from {path}: {e}')
    return overrides

    return DEFAULT_KEY_MAP


def create_key_map(overrides_path: str = '') -> dict:
    key_map = {k: [list(chord) for chord in v] for k, v in DEFAULT_KEY_MAP.items()}
    if overrides_path:
        key_map.update(_parse_keymap_file(overrides_path))
    return key_map


def bind_custom_key(kb, action, key_map, handler, **kwargs):
    for chord in key_map[action]:
        kb.add(*chord, **kwargs)(handler)


def resolve_keys(key: str):
    return KEY_NAME_MAP.get(key, key)


def make_command_palette(key_map: dict) -> list[FloatItem]:
    items = []
    for action, label in COMMAND_PALETTE_ACTIONS:
        chords = key_map.get(action, [])
        if not chords:
            continue
        # Always use first binding for display and execution
        first_chord = chords[0]
        hint = ' '.join(first_chord)
        items.append(FloatItem(label, hint, first_chord))
    return items
