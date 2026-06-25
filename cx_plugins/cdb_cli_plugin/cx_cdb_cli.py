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

import asyncio

from prompt_toolkit import PromptSession, print_formatted_text
from prompt_toolkit.cursor_shapes import CursorShape
from prompt_toolkit.history import InMemoryHistory


class CDBCli:
    def __init__(self):
        self.session: PromptSession[str] = PromptSession(
            multiline=True,
            enable_history_search=True,
            history=InMemoryHistory(),  # TODO
            mouse_support=True,
            cursor=CursorShape.UNDERLINE,
        )

    async def run(self) -> None:
        while True:
            try:
                command = await self.session.prompt_async(
                    '>',  # TODO ENV NAME
                    refresh_interval=0.2,
                )
            except (EOFError, KeyboardInterrupt):
                print_formatted_text('disconnecting...')  # TODO
                break

            command = command.strip()
            if not command:
                continue
            print('TIM: ', command)
            # await self.execute(command)


async def async_main() -> None:
    await CDBCli().run()


def main() -> None:
    asyncio.run(async_main())


if __name__ == '__main__':
    main()
