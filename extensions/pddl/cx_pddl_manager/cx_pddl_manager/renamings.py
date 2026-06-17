# Copyright (c) 2025-2026 Carologistics
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

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Renamings:
    """
    Wraps the name-mangling map produced by PDDLWriter.

    unified-planning may rename actions/objects to produce valid PDDL
    identifiers.  This class provides a safe, typed way to reverse those
    renamings when building plan messages.

    Usage::

        writer = PDDLWriter(problem)
        r = Renamings.from_writer(writer)
        original_name = r.resolve_name(mangled_name)
    """

    _map: dict[str, str] = field(default_factory=dict)

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    @classmethod
    def from_writer(cls, writer) -> Renamings:
        """Build a Renamings instance from a PDDLWriter's nto_renamings map."""
        return cls({k: v.name for k, v in writer.nto_renamings.items()})

    # ------------------------------------------------------------------
    # Resolution helpers
    # ------------------------------------------------------------------

    def resolve_name(self, raw: str) -> str:
        """Return the original action/fluent name, undoing any UP mangling."""
        return self._map.get(raw, raw)

    def resolve_arg(self, arg) -> str:
        """Return the original object name for an action parameter."""
        return self._map.get(str(arg), str(arg))
