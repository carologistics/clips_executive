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

(deftemplate counter
  (slot value (type INTEGER))
  (slot last-updated (type FLOAT))
)

(deffacts init-counter
  (counter (value 0))
)

(defrule increment-counter
  ?f <- (counter (value ?n) (last-updated ?t))
  (time ?now&:(< ?t ?now))
  =>
  (bind ?new-value (+ ?n 1))
  (if (> ?new-value 20)
   then
    (retract ?f)
    (cx-shutdown)
   else
    (modify ?f (value (+ ?n 1)) (last-updated ?now))
  )
)
