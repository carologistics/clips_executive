; Copyright (c) 2026 Carologistics
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

(defrule set-bool-client-init
" Create a simple client using the generated bindings. "
  (not (std-srvs-set-bool-client (service "set_bool_srv")))
  (not (executive-finalize))
=>
  (std-srvs-set-bool-create-client "set_bool_srv")
  (printout info "Created client for /set_bool" crlf)
  (assert (started (now)))
)

(defrule set-bool-client-send-out-request-false
  (std-srvs-set-bool-client (service ?service))
  (not (send-request FALSE ?))
  (started ?start-time)
  (time ?curr-time&:(> (- ?curr-time ?start-time) 1))
  =>
  ;example usage of sending a request
  (printout info "Send a request with data: FALSE" crlf)
  (bind ?new-req (std-srvs-set-bool-request-create))
  (std-srvs-set-bool-request-set-field ?new-req "data" FALSE)
  (bind ?id (std-srvs-set-bool-send-request ?new-req ?service))
  (std-srvs-set-bool-request-destroy ?new-req)
  (if ?id then
    (assert (send-request FALSE ?id))
  )
)

(defrule set-bool-client-send-out-request-true
  (std-srvs-set-bool-client (service ?service))
  ?sr <- (send-request FALSE ?)
  (not (send-request TRUE ?))
  (time ?)
  =>
  ;example usage of sending a request
  (printout info "Additionally, send a request with data: TRUE" crlf)
  (bind ?new-req (std-srvs-set-bool-request-create))
  (std-srvs-set-bool-request-set-field ?new-req "data" TRUE)
  (bind ?id (std-srvs-set-bool-send-request ?new-req ?service))
  (std-srvs-set-bool-request-destroy ?new-req)
  (if ?id then
    (assert (send-request TRUE ?id))
  )
)

(defrule set-bool-client-process-response
  (std-srvs-set-bool-response (service ?serivce) (msg-ptr ?msg) (request-id ?id))
  (send-request ?val ?id)
=>
  (bind ?success (std-srvs-set-bool-response-get-field ?msg "success"))
  (bind ?message (std-srvs-set-bool-response-get-field ?msg "message"))
  (printout blue "Response (success: " ?success") for data: " ?val ": \"" ?message "\"" crlf)
  (std-srvs-set-bool-response-destroy ?msg)
)

(defrule set-bool-client-cleanup
  (executive-finalize)
  (std-srvs-set-bool-client (service ?service))
  =>
  (std-srvs-set-bool-destroy-client ?service)
)
