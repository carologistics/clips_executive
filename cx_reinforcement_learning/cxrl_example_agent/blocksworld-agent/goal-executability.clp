(defrule goal-executable-stack
    (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
    ?g <-   (goal   (class STACK) (id ?goalid)
                    (mode FORMULATED) (params upper ?upper lower ?lower))
    (not    (goal   (class STACK)
                    (mode SELECTED|EXPANDED|COMMITTED|DISPATCHED)))
    (domain-fact (name clear) (param-values ?lower))
    (domain-fact (name clear) (param-values ?upper))
    (domain-fact (name on-table) (param-values ?upper))
    =>
    (printout t "Goal STACK executable" crlf)
    (assert (rl-action (id ?goalid) (name (sym-cat "STACK#upper#" ?upper "#lower#" ?lower)) (points ?*POINTS-GOAL-STACK*)))
)