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

(defrule protobuf-init-example-client
  (not (executive-finalize))
  (not (client ?any-c-id))
  =>
  (bind ?res (pb-connect 127.0.0.1 4446))
  (printout green "Connect to server: " ?res crlf)
  (assert (client ?res))
  (bind ?success (pb-register-type "SearchRequest"))
  (printout green "Register Type: " ?success crlf)
)

(defrule peer-send-msg
  (client ?c-id)
  (protobuf-client-connected ?c-id)
  (not (protobuf-msg))
  =>
  (bind ?msg (pb-create "SearchRequest"))
  (pb-set-field ?msg "query" "hello")
  (pb-set-field ?msg "page_number" ?c-id)
  (pb-set-field ?msg "results_per_page" ?c-id)
  (pb-send ?c-id ?msg)
  (pb-destroy ?msg)
)

(defrule protobuf-close
  (executive-finalize)
  ?f <- (client ?any-client)
  =>
  (pb-disconnect ?any-client)
  (retract ?f)
)
