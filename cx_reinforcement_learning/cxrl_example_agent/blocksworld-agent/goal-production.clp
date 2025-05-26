(defrule goal-stack-formulate
	(declare (salience ?*SALIENCE-HIGH*))
    (domain-object (name ?b1) (type block))
    (domain-object (name ?b2&:(neq ?b1 ?b2)) (type block))
    (domain-fact (name on-table) (param-values ?b1))
    (domain-fact (name on-table) (param-values ?b2))
    (not (goal (class STACK) (params upper ?b1 lower ?b2)))
    =>
    (bind ?id (sym-cat STACK- (gensym*)))
    (assert (goal   (class STACK) 
                    (id ?id)
                    (sub-type SIMPLE)
                    (params upper ?b1 lower ?b2)
            )
    )
    (printout t "Asserted goal " ?id crlf)
)

