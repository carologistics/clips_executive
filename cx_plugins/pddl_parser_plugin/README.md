# cx_config_plugin
This package offers the `cx::PddlParserPlugin` CLIPS plugin to use PDDL with CLIPS.

At the moment this plugin only supports STRIPS-style PDDL domains.
## Usage
Register this plugin with the plugin manager. It requires no additional configuration, an example setup is shown below:

```yaml
clips_manager:
  ros__parameters:
    environments: ["main"]
    main:
      plugins: ["pddl_parser"]

    pddl_parser:
      plugin: "cx::PddlParserPlugin"
```
## CLIPS Features
This plugin adds deftemplates and deffunctions as listed below:
### Deftemplates
```lisp
(deftemplate domain-object-type
  "A type in the domain. The type obj must be super-type of all types."
  (slot name (type SYMBOL))
  (slot super-type (type SYMBOL) (default object))
)

(deftemplate domain-predicate
	"Representation of a predicate specification."
  (slot name (type SYMBOL) (default ?NONE))
  ; If the predicate is sensed, it is not directly changed by an action effect.
  ; Instead, we expect the predicate to be changed externally.
  (slot sensed (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
  ; A value predicate is a predicate that is true for at most one value of the
  ; last argument. In other words, the predicate represents a partial function,
  ; with all but the last predicate argument being the function paramters, and
  ; the last predicate being the function value.
  (slot value-predicate (type SYMBOL) (allowed-values TRUE FALSE)
    (default FALSE))
  (multislot param-names (type SYMBOL))
  (multislot param-types (type SYMBOL))
)

(deftemplate domain-operator
  "An operator of the domain. This only defines the name of the operator,
   other properties such as parameters, precondition, or effects are
   defined in separate templates.
   The wait-sensed slot defines whether to wait for sensed predicates to
   achieve the desired value, or whether to ignore such predicates."
  (slot name (type SYMBOL))
  (multislot param-names)
	(slot wait-sensed (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
  (slot exogenous (type SYMBOL) (allowed-values TRUE FALSE) (default FALSE))
)

(deftemplate domain-operator-parameter
  "A parameter of an operator. The operator and type slots must refer to the
   names of an existing operator and an existing type respectively."
  (slot name)
  (slot operator (type SYMBOL))
  (slot type (type SYMBOL) (default object))
)

(deftemplate domain-effect
  "An effect of an operator. For now, effects are just a set of atomic effects
   which are applied after the action was executed successfully."
  (slot name (type SYMBOL) (default-dynamic (gensym*)))
  (slot part-of (type SYMBOL))
  (slot predicate (type SYMBOL))
  (multislot param-names (default (create$)))
  (multislot param-values (default (create$)))
  (multislot param-constants (default (create$)))
  (slot type (type SYMBOL) (allowed-values POSITIVE NEGATIVE)
    (default POSITIVE))
)

(deftemplate pddl-formula
  "A PDDL formula representation in CLIPS, sourced from the preconditions of
  the PDDL domain description."
  (slot id (type SYMBOL) (default ?NONE))
  (slot part-of (type SYMBOL))

  (slot type (type SYMBOL) (allowed-values conjunction disjunction negation atom))
)


(deftemplate pddl-predicate
  "An instantiated predicate with possible constants that is part of
  a PDDL formula."
  (slot id (type SYMBOL) (default ?NONE))
  (slot part-of (type SYMBOL)) ; reference to parent formula

  ; a PDDL predicate is either an equality or references a domain-predicate
  ; equalities are marked through the symbol 'equality'
  (slot predicate (type SYMBOL))

  (multislot param-names (type SYMBOL))
  (multislot param-constants)
)
```

### Functions
```lisp
; Parse a PDDL domain by representing it through the templated facts above.
(parse-pddl-domain ?formula ?output-id)

; parse a PDDL formula by representing it through the templated facts above.
(parse-pddl-formula ?file)
```
