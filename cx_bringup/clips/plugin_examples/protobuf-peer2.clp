; Copyright (c) 2024-2026 Carologistics
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

(defrule protobuf-init-example-peer
  (not (executive-finalize))
  (not (peer ?any-peer-id))
  =>
  (bind ?peer-2 (pb-peer-create-local 127.0.0.1 4445 4444))
  (assert (peer ?peer-2 4445 4444))
  (printout green "127.0.0.1 4445 4444" crlf)
  (assert (started (now)))
)
