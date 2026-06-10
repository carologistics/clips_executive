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

import os

from prompt_toolkit.styles import Style, merge_styles
from prompt_toolkit.styles.pygments import style_from_pygments_cls
from pygments.styles import get_style_by_name

# Default custom-key styles per dark/light theme.
# These are applied after the pygments base and before any user overrides.
_DEFAULTS_DARK = {
    'bottom-toolbar': 'bg:#f5e8d8 fg:#1c1c1c',
    '': '#f8f8f2',
    'pygments.keyword': '#ff79c6 bold',
    'pygments.name': '#f8f8f2',
    'pygments.literal': '#f1fa8c',
    'pygments.string': '#f1fa8c',
    'pygments.comment': '#6272a4 italic',
    'pygments.punctuation': '#ff79c6',
    'pygments.operator': '#ff79c6',
    'bl-separator': 'bg:#822659',
    'bl-text': 'bg:#f0f0f0',
    'bl-highlight': 'bg:#2596be',
    'bl-success': 'bg:#3fb950',
    'bl-failure': 'bg:#ff4550',
    'log-prefix': '#888888',
    'log-error': '#ff4444',
    'log-warn': '#ffaa00',
    'search-prompt': '#00ff00',
    'search-match': 'bg:#ffaa00 #000000 bold',
}

_DEFAULTS_LIGHT = {
    'bottom-toolbar': 'bg:#484b6a fg:#e4e5f1',
    '': '#212121 fg:#ffffff',
    'pygments.keyword': '#7c4dff bold',
    'pygments.name': '#212121',
    'pygments.literal': '#c75000',
    'pygments.string': '#c75000',
    'pygments.comment': '#888888 italic',
    'pygments.punctuation': '#7c4dff',
    'pygments.operator': '#7c4dff',
    'bl-separator': 'bg:#6f4e37',
    'bl-text': 'bg:#000000',
    'bl-highlight': 'bg:#6f42c1',
    'bl-success': 'bg:#1a7f37',
    'bl-failure': 'bg:#b31d28',
    'log-prefix': '#888888',
    'log-error': '#ff4444',
    'log-warn': '#ffaa00',
    'search-prompt': '#00ff00',
    'search-match': 'bg:#ffaa00 #000000 bold',
}


def _load_style_overrides(path: str, dark: bool) -> dict:
    overrides = {}
    section = 'both'  # 'both', 'dark', or 'light'
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if line == '[dark]':
                    section = 'dark'
                    continue
                if line == '[light]':
                    section = 'light'
                    continue
                if '=' not in line:
                    continue
                if (
                    section == 'both'
                    or (section == 'dark' and dark)
                    or (section == 'light' and not dark)
                ):
                    key, _, value = line.partition('=')
                    overrides[key.strip()] = value.strip()
    except FileNotFoundError:
        log.warning(f"Style override file not found: {path}")
    except Exception as e:
        log.warning(f"Failed to load style overrides from {path}: {e}")
    return overrides


def make_style(dark: bool, pygments_style: str = '', overrides_path: str = '') -> Style:
    # 1. Pygments base — use provided name, fall back to theme default
    if pygments_style:
        base_name = pygments_style
    else:
        base_name = 'github-dark' if dark else 'friendly'
    try:
        base = style_from_pygments_cls(get_style_by_name(base_name))
    except Exception:
        log.warning(f"Unknown pygments style '{base_name}', falling back to default")
        base = style_from_pygments_cls(get_style_by_name('github-dark' if dark else 'friendly'))

    # 2. Built-in defaults for custom keys
    defaults = dict(_DEFAULTS_DARK if dark else _DEFAULTS_LIGHT)

    # 3. File overrides — merged on top of defaults
    if overrides_path:
        defaults.update(_load_style_overrides(overrides_path, dark))

    return merge_styles([base, Style.from_dict(defaults)])


def is_dark_terminal() -> bool:
    """Detect terminal color scheme from environment variables."""
    colorfgbg = os.environ.get('COLORFGBG', '')
    if colorfgbg:
        try:
            bg = int(colorfgbg.split(';')[-1])
            return bg < 8
        except ValueError:
            pass

    term_profile = os.environ.get('TERM_PROFILE', '')
    if term_profile:
        dark_hints = ['dark', 'black', 'night', 'monokai', 'dracula', 'solarized-dark']
        light_hints = ['light', 'white', 'day', 'solarized-light']
        lower = term_profile.lower()
        if any(h in lower for h in dark_hints):
            return True
        if any(h in lower for h in light_hints):
            return False

    return True
