
;---------------------------------------------------------------------------
;  skills-actions.clp - CLIPS executive - execute skill actions
;
;  Created: Wed Sep 20 15:46:48 2017
;  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
;							2021       Ivaylo Doychev
;  Licensed under GPLv2+ license, cf. LICENSE file
;---------------------------------------------------------------------------

(deftemplate skill-action-execinfo
	(slot goal-id (type SYMBOL))
	(slot plan-id (type SYMBOL))
	(slot action-id (type INTEGER))
	;(slot channel (type INTEGER))
	(slot skill-name (type SYMBOL))
	(slot skill-id (type SYMBOL))
	(slot agent-id (type STRING) (default ""))
	(multislot skill-args)
)

(deftemplate skill-action-mapping
	(slot name (type SYMBOL) (default ?NONE))
	(slot map-string (type STRING))
)

(defrule skill-action-init
	(confval (path "/clips_executive/spec") (type STRING) (value ?spec))
	(confval (path ?p&:(eq (str-index (str-cat "/clips_executive/specs/" ?spec "/action-mapping/") ?p) 1))
	         (type STRING) (value ?s))
	=>
	(bind ?prefix (str-cat "/clips_executive/specs/" ?spec "/action-mapping/"))
	(bind ?name (sym-cat (sub-string (+ (str-length ?prefix) 1) (str-length ?p) ?p)))
	(assert (skill-action-mapping (name ?name) (map-string ?s)))
)

(defrule skill-action-start
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (state PENDING)
	                    (action-name ?action-name) (executable TRUE)
	                    (agent-id ?agent-id)
	                    (param-names $?params)
	                    (param-values $?param-values))
	; (skill-action-mapping (name ?action-name))
	(not (skill-action-execinfo (agent-id ?agent-id)))
	=>
	(bind ?skill-id (skill-call ?action-name ?params ?param-values ?agent-id))
	(modify ?pa (state WAITING))
	(bind ?args (create$))
	(loop-for-count (?i (length$ ?params))
		(bind ?args (append$ ?args (nth$ ?i ?params) (nth$ ?i ?param-values)))
	)
	(assert (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id)
	                               (action-id ?id) (skill-id ?skill-id)
	                               (skill-name ?action-name)
	                               (skill-args ?args) (agent-id ?agent-id)))
)

(defrule skill-action-running
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
	                    (action-name ?action-name) (state WAITING) (agent-id ?agent-id))
	?pe <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id)
	                              (action-id ?id) (skill-id ?skill-id) (agent-id ?agent-id))
	(skill (id ?skill-id) (status S_RUNNING))
	=>
	(printout t "Action " ?action-name " is running" crlf)
	(modify ?pa (state RUNNING))
)

(defrule skill-action-final
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
	                    (action-name ?action-name) (state WAITING|RUNNING) (agent-id ?agent-id))
	?pe <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id)
	                              (action-id ?id) (skill-id ?skill-id) (agent-id ?agent-id))
	?sf <- (skill (id ?skill-id) (status S_FINAL) (agent-id ?agent-id))
	=>
	(printout t "Execution of " ?action-name " completed successfully" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
	(retract ?sf ?pe)
)

(defrule skill-action-failed
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
	                    (action-name ?action-name) (state WAITING|RUNNING) (agent-id ?agent-id))
	?pe <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id)
	                              (action-id ?id) (skill-id ?skill-id) (agent-id ?agent-id))
	?sf <- (skill (id ?skill-id) (status S_FAILED) (error-msg ?error) (agent-id ?agent-id))
	=>
	(printout warn "Execution of " ?action-name " FAILED (" ?error ")" crlf)
	(modify ?pa (state EXECUTION-FAILED) (error-msg ?error))
	(retract ?sf ?pe)
)

(defrule skill-action-cancel-if-action-does-not-exist
	?pe <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id)
	                              (action-id ?id) (skill-id ?skill-id) (agent-id ?agent-id))
	(skill (id ?skill-id) (status S_RUNNING) (agent-id ?agent-id))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id)
	                  (agent-id ?agent-id)))
	=>
	(printout warn
		  "Cancelling Skill Execution, corresponding action does not exist" crlf)
	(call-skill-cancel ?agent-id)
	(retract ?pe)
)

(defrule skill-action-retract-execinfo-without-action
	?pe <- (skill-action-execinfo (goal-id ?goal-id) (plan-id ?plan-id)
	                              (action-id ?id) (skill-id ?skill-id) (agent-id ?agent-id))
	(not (skill (status S_RUNNING) (id ?skill-id)))
	(not (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?action-id)))
	=>
	(retract ?pe)
)
