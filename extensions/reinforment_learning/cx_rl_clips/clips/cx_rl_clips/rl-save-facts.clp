; Copyright (c) 2025 Carologistics
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

(deffunction cx-rl-define-save-logic ()
(bind ?service-clients-pre "")
(bind ?service-clients ?*CX-RL-SERVICE-CLIENTS*)
(bind ?index 1)
(bind ?length (length$ ?service-clients))
(while (< ?index ?length)
    (bind ?service-clients-pre (str-cat ?service-clients-pre " \
  (ros-msgs-client (service \"" ?*CX-RL-NODE-NAME* "/" (nth$ ?index ?service-clients)"\"))"
    ))
    (bind ?index (+ ?index 2))
)
(bind ?services-pre "")
(bind ?services ?*CX-RL-SERVICES*)
(bind ?index 1)
(bind ?length (length$ ?services))
(while (< ?index ?length)
    (bind ?services-pre (str-cat ?services-pre " \
  (ros-msgs-service (service \"" ?*CX-RL-NODE-NAME* "/" (nth$ ?index ?services)"\"))"
    ))
    (bind ?index (+ ?index 2))
)

(bind ?action-servers-pre "")
(bind ?action-servers ?*CX-RL-ACTION-SERVERS*)
(bind ?index 1)
(bind ?length (length$ ?action-servers))
(while (< ?index ?length)
    (bind ?action-servers-pre (str-cat ?action-servers-pre " \
  (" (nth$ ?index ?action-servers) " (name \"" ?*CX-RL-NODE-NAME* "/" (nth$ (+ ?index 1) ?action-servers)"\"))"
    ))
    (bind ?index (+ ?index 2))
)

(bind ?rule (str-cat
"(defrule all-services-actions-created \
	(cx-rl-node (name ?node)) \
" ?services-pre ?service-clients-pre ?action-servers-pre
"  => \
    (assert (saved-facts)) \
    (save-facts reset-save) \
) \
"))
 (build ?rule)
)
(cx-rl-define-save-logic)
