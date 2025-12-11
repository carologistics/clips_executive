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

; batch this file to load all of the pddl definitions

; optionally use default node name if not provided
(if (not (member$ CX-NODE (get-defglobal-list)))
  then
  (load* (str-cat (ament-index-get-package-share-directory "cx_rl_clips") "/clips/cx_rl_clips/node-name.clp"))
)


; load all regular files
(foreach ?f (create$
deftemplates.clp
rl-utils.clp
action-selection-action.clp
exec-action-selection-client.clp
get-free-robot-action.clp
reset-env-action.clp
rl-reset-env.clp
rl-ros-comm-init.clp
rl-execution.clp
)
(load* (str-cat (ament-index-get-package-share-directory "cx_rl_clips") "/clips/cx_rl_clips/" ?f))
)

; batch-load everything that is dynamcially created
; (all service provider feedback functions depend on the node name)
; The procedures fot storing the initial database depend on the required ROS interfaces
(foreach ?f (create$
  create-rl-env-state-srv.clp
  get-action-list-robot-srv.clp
  get-action-list-srv.clp
  get-episode-end-srv.clp
  get-observable-objects-srv.clp
  get-observable-predicates-srv.clp
  get-predefined-observables-srv.clp
  set-rl-mode-srv.clp
  rl-save-facts.clp
)
(batch* (str-cat (ament-index-get-package-share-directory "cx_rl_clips") "/clips/cx_rl_clips/" ?f))
)
