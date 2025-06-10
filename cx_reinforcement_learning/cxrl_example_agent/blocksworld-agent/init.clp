(defrule load-domain
  (not (domain-loaded))
=>
  (bind ?share-dir (ament-index-get-package-share-directory "cxrl_example_agent"))
  (parse-pddl-domain (str-cat ?share-dir "/clips/blocksworld-agent/domain.pddl"))
  (printout t "Domain loaded" crlf)
  (assert (domain-loaded))
)

(deffunction observe-predicates-except-on-table ()
	(do-for-all-facts ((?dp domain-predicate))
		(neq ?dp:name on-table)
	(assert (rl-observable-predicate 	(name ?dp:name) 
										(param-types ?dp:param-types) 
										(param-names ?dp:param-names)))
	)
)

(deffunction observe-all-objects ()
	(do-for-all-facts ((?do domain-object))
		TRUE
	(assert (rl-observable-object	(name ?do:name) 
									(type ?do:type)))
	)
)

(deffunction predefine-observables ()
	(assert (rl-predefined-observable (name on-table) (params block1)))
	(assert (rl-predefined-observable (name on-table) (params block2)))
	(assert (rl-predefined-observable (name on-table) (params block3)))
)

(deffunction rl-generate-observations ()
	(do-for-all-facts ((?df domain-fact))
			TRUE
		(assert (rl-observation (name ?df:name) (param-values ?df:param-values)))
	)
)

(deffunction add-robot ()
  (assert (rl-robot (name robot1)))
)

(deffunction domain-load-local-facts ()
	(assert
		(domain-fact (name clear) (param-values block1))
		(domain-fact (name clear) (param-values block2))
		(domain-fact (name clear) (param-values block3))

		(domain-fact (name on-table) (param-values block1))
		(domain-fact (name on-table) (param-values block2))
		(domain-fact (name on-table) (param-values block3))

		(domain-fact (name can-hold) (param-values robot1))
	)
	(assert 
		(domain-object (name block1) (type block))
		(domain-object (name block2) (type block))
		(domain-object (name block3) (type block))

        (domain-object (name robot1) (type robot))
	)
)

(defrule domain-load-initial-facts
	(domain-loaded)
=>
	(domain-load-local-facts)
	(predefine-observables)
	(observe-predicates-except-on-table)
	(observe-all-objects)
	(add-robot)
	(assert (domain-facts-loaded))
)

(defrule domain-episode-finished-success
    (declare (salience ?*SALIENCE-RL-EPISODE-END-SUCCESS*))
    (not (rl-episode-end))
    (domain-fact (name on) (param-values block2 block1))
    (domain-fact (name on) (param-values block3 block2))
    =>
    (assert (rl-episode-end (success TRUE)))
)   