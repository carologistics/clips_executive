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

import csv
from pathlib import Path


class StepLogger:
    """Per-step CSV logger for RL env transitions."""

    COLUMNS = ('episode', 'step_in_ep', 'total_step', 'action', 'reward', 'done')

    def __init__(self, log_dir, filename='steps.csv'):
        self._path = Path(log_dir) / filename
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self._path, 'w', buffering=1, newline='')
        self._writer = csv.writer(self._fh)
        self._writer.writerow(self.COLUMNS)

    def record(self, episode, step_in_ep, total_step, action_string, reward, done):
        action = action_string.replace('#', ',') if action_string else ''
        self._writer.writerow(
            [episode, step_in_ep, total_step, action, reward, int(bool(done))]
        )

    def close(self):
        if self._fh and not self._fh.closed:
            self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
