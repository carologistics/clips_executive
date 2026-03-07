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

; batch this file to load all of the pddl definitions

; optionally use default node name if not provided

(foreach ?f
  (create$
    deftemplates.clp
    cx-rl-no-deftemplates.clp
  )
  (batch* (str-cat (ament-index-get-package-share-directory "cx_rl_clips") "/clips/cx_rl_clips/" ?f))
)
