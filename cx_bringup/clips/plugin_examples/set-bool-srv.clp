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

; This file showcases intefaces with specific ros services

(defrule set-bool-service-init
" Create a simple service using the generated bindings. "
  (not (std-srvs-set-bool-service (name "set_bool_srv")))
=>
  (std-srvs-set-bool-create-service "set_bool_srv")
  (printout info "Created service for /ros_cx_srv" crlf)
)

; this function needs to be defined in order to respond to messages
(deffunction std-srvs-set-bool-service-callback (?service-name ?request ?response)
  (bind ?req-data (std-srvs-set-bool-request-get-field ?request "data"))
  (printout green "Received request on " ?service-name ". Data: " ?req-data crlf)
  (if ?req-data then
    (std-srvs-set-bool-response-set-field ?response "success" TRUE)
    (std-srvs-set-bool-response-set-field ?response "message" (str-cat "Received the request: " ?req-data))
    (assert (send-request))
   else
    (std-srvs-set-bool-response-set-field ?response "success" FALSE)
    (std-srvs-set-bool-response-set-field ?response "message" (str-cat "Received the request: " ?req-data))
  )
)

(defrule set-bool-srv-cleanup
  (executive-finalize)
  (std-srvs-set-bool-service (name ?service))
  =>
  (std-srvs-set-bool-destroy-service ?service)
)
