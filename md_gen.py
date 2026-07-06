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

"""
Build Sphinx markdown output from one or more source trees.

This is a helper tool to keep information in sync between rosdoc and readmes.
"""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

# --- Config ------------------------------------------------------------------

# Sphinx source trees to build: src_dir -> build_dir.
# Add as many entries as you need; each is built independently via sphinx-build.
SPHINX_BUILDS: dict[str, str] = {
    'clips_executive/cx_docs/': './build/',
    # "other_component/docs/": "./build/other_component",
}
build_prefix = 'build/markdown/'
# Destination file -> ordered list of generated markdown source files to
# concatenate into it. Destinations can be anywhere; sources can come from
# any of the build dirs above (or anywhere else on disk).

DOC_MAP: dict[str, list[str]] = {
    'cx_plugins/ament_index_plugin/README.md': [build_prefix + 'plugins/ament_index_plugin.md'],
    'cx_plugins/cdb_saver_plugin/README.md': [build_prefix + 'plugins/cdb_saver_plugin.md'],
    'cx_plugins/example_plugin/README.md': [build_prefix + 'plugins/example_plugin.md'],
    'cx_plugins/file_load_plugin/README.md': [build_prefix + 'plugins/file_load_plugin.md'],
    'cx_plugins/ros_msgs_plugin/README.md': [build_prefix + 'plugins/ros_msgs_plugin.md'],
    'cx_plugins/tf2_pose_tracker_plugin/README.md': [
        build_prefix + 'plugins/tf2_pose_tracker_plugin.md'
    ],
    'cx_plugins/cdb_loader_plugin/README.md': [build_prefix + 'plugins/cdb_loader_plugin.md'],
    'cx_plugins/config_plugin/README.md': [build_prefix + 'plugins/config_plugin.md'],
    'cx_plugins/executive_plugin/README.md': [build_prefix + 'plugins/executive_plugin.md'],
    'cx_plugins/protobuf_plugin/README.md': [build_prefix + 'plugins/protobuf_plugin.md'],
    'cx_plugins/ros_param_plugin/README.md': [build_prefix + 'plugins/ros_param_plugin.md'],
    'cx_ros_comm_gen/README.md': [build_prefix + 'plugin_generator/gen_info.md'],
}

BANNER_TEMPLATE = '<!-- AUTO-GENERATED via sphinx-build. Do not edit directly. -->'

# -------------------------------------------------------------------------------


def build_sphinx_sources() -> None:
    print('==> Building Sphinx markdown output...')
    for src_dir, build_dir in SPHINX_BUILDS.items():
        print(f'  building {src_dir} -> {build_dir}')
        subprocess.run(
            ['sphinx-build', '-M', 'markdown', src_dir, build_dir],
            check=True,
        )


def sync_docs() -> int:
    print('==> Syncing generated markdown files to destinations...')
    status = 0

    for dest_file_str, source_names in DOC_MAP.items():
        dest_file = Path(dest_file_str)
        dest_dir = dest_file.parent

        if not dest_dir.is_dir():
            print(
                f'  WARNING: destination directory "{dest_dir}" does not exist, \
                skipping "{dest_file}".'
            )
            status = 1
            continue

        source_files = [Path(name) for name in source_names]
        missing = [f for f in source_files if not f.is_file()]
        if missing:
            missing_list = ', '.join(str(f) for f in missing)
            print(
                f'  WARNING: missing generated file(s) [{missing_list}] for "{dest_file}", \
                skipping this destination.'
            )
            status = 1
            continue

        banner = BANNER_TEMPLATE.format(source=source_files[0])
        sections = [f.read_text(encoding='utf-8').rstrip('\n') for f in source_files]
        content = banner + '\n\n' + '\n\n'.join(sections) + '\n'

        dest_file.write_text(content, encoding='utf-8')

        sources_list = ' '.join(source_names)
        print(f'  [{sources_list}] -> {dest_file}')

    return status


def main() -> int:
    build_sphinx_sources()

    build_dirs = [Path(d) for d in SPHINX_BUILDS.values()]
    for build_dir in build_dirs:
        if not build_dir.exists():
            print(f'ERROR: expected build output dir not found: {build_dir}', file=sys.stderr)
            return 1

    status = sync_docs()
    print('==> Done.')
    return status


if __name__ == '__main__':
    sys.exit(main())
