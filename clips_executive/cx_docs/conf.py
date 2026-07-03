# Copyright (c) 2024-2026 Carologistics
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

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import sys

from pygments.lexer import RegexLexer
from pygments.token import Comment, Keyword, Name, Number, Operator, Punctuation, String, Text


class CLIPSLexer(RegexLexer):
    name = 'CLIPS'
    aliases = ['clips']
    filenames = ['*.clips']

    tokens = {
        'root': [
            (r'[()]', Punctuation),
            (r'\s+', Text),
            (r';.*$', Comment.Single),
            (r'"[^"]*"', String),
            # numbers
            (r'\b-?\d+\.\d+\b', Number.Float),
            (r'\b-?\d+\b', Number.Integer),
            # CLIPS variables / $ identifiers
            (r'\?[A-Za-z0-9_-]+', Name.Variable),
            (r'\$[?A-Za-z0-9_-]*', Name.Variable),
            # operators
            (r'=>', Name.Tag),
            (r'\|', Operator),
            (r'\$', Operator),
            (r'[()]', Punctuation),
            # keywords
            (r'\b(defrule|deftemplate|deffacts)\b', Keyword.Declaration),
            (r'\b(allowed-values|type|slot|cardinality)\b', Keyword.Reserved),
            (r'\b(assert|modify|retract)\b', Keyword.Name.Function),
            # fallback MUST be last
            (r'[^\s]+', Text),
        ],
    }


sys.path.insert(0, os.path.abspath('.'))

project = 'clips_executive'
project_copyright = '2024, Tarik Viehmann'

author = 'Tarik Viehmann'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration
ros_distro = os.getenv('ROS_DISTRO', 'humble')  # Default to 'humble' if ROS_DISTRO is not set

extensions = [
    'sphinx.ext.extlinks',
    'sphinx.ext.todo',
    'sphinx_copybutton',
]
local = True


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
# html_static_path = ['_static']
# Add static path for custom CSS
html_static_path = ['_static']


# Include custom CSS file
def setup(app):

    app.add_lexer('clips', CLIPSLexer)
    app.add_css_file('custom_figure.css')


if local:
    extlinks = {
        'docsite': ('https://carologistics.github.io/clips_executive/%s', '%s'),
        # ('http://localhost:8000/%s', '%s'),
        'source-master': ('https://github.com/carologistics/clips_executive/blob/master/%s', '%s'),
        'rosdoc': (f'https://docs.ros.org/en/{ros_distro}/p/%s', '%s'),
        'rostut': (f'https://docs.ros.org/en/{ros_distro}/%s', '%s'),
    }
extlinks_detect_hardcoded_links = True
todo_include_todos = True
exclude_patterns = ['docs_build', 'links.rst']
# Read link all targets from file
rst_epilog = ''
with open('links.rst') as f:
    rst_epilog += f.read().replace('{ros_distro}', ros_distro)
