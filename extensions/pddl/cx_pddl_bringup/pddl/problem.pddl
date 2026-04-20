; Copyright (c) 2025-2026 Carologistics
; SPDX-License-Identifier: Apache-2.0
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

(define (problem bw-1)
    (:domain blocksworld)

    (:objects A B C D - block)

    (:init
        (arm-empty)
        (on-table A)
        (on-table D)
        (on C D)
        (on B A)
        (clear C)
        (clear B)
    )

    (:goal
        (and
            (on D B)
            (on-table C)
            (on B C)
        )
    )
)
