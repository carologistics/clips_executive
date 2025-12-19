Reinforcement Learning CLIPS Interfaces
#######################################

To facilitate the development of CLIPS-based RL agents, this extensions provides CLIPS code toabstract away the ROS interactions and to provide a symbolic representation of the RL workflow as provided by the environments derived fro mthe ``CXRLGym`` class.
In the following, the CLIPS interface is described as provided by the ``cx_rl_clips`` package.

.. important::

   The ``cx_rl_clips`` package requires the usage of ROS Kilted or above as it heavily relies on service introspection.

Required |CX| Plugins
*********************

In order to integrate the CXRLGym with the |CX|, the following plugins are needed:

- ``cx::ExecutivePlugin`` — manages the overall reasoning and control flow, interleaving ROS feedback with CLIPS reasoning.
- ``cx::RosMsgsPlugin`` provides access to ROS interfaces of the PDDL manager from within CLIPS.
- ``cx::AmentIndexPlugin`` resolves package paths via ``ament_index``. While not required, it is very useful in order to load agents.

Also, as the current configuration is compatible with ROS 2 kilted or above, action server introspection is not supported, hence the following plugins are needed:

- ``cx::CXCxRlInterfacesGetFreeRobotPlugin``
- ``cx::CXCxRlInterfacesExecActionSelectionPlugin``
- ``cx::CXCxRlInterfacesResetCXPlugin``

CLIPS Integration
*****************

CLIPS needs to interact with the CXRLGym environments by providing services and action servers for the relevant endpoints.
In order to reduce manual overhead, predefined rules, functions and deftemplates are available, which are described in the remainder of this document.

Predefined Interfaces
~~~~~~~~~~~~~~~~~~~~~

The idea is to have templates for facts that correspond to the individual endpoints of the PDDL manager.
The workflow then is for users to simply assert a fact of a defined template to trigger the request to the PDDL manager, then using rules to observe the provided slots for the outcome.

Example: Loading a PDDL Problem and Pbtaining the initial State
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following example demonstrates how to initialize the connection to the external
PDDL manager and load a PDDL problem into CLIPS. It shows two rules:

1. One rule sets up service clients for communicating with the PDDL manager node.
2. Another rule creates and registers a PDDL instance, requesting its initial state.

These examples can serve as a starting point for integrating PDDL planning with
CLIPS-based reasoning systems.

A more comprehensive example can be found in the :ref:`Structured PDDL Agent Tutorial <structured_pddl_agent>`.

Creating Service Clients
^^^^^^^^^^^^^^^^^^^^^^^^

The first rule initializes the required ROS service clients for the PDDL manager.
It also asserts a ``pddl-services-loaded``
fact to ensure coherent ordering.

.. code-block:: lisp

  (defrule example-pddl-init
    " Initiate the service clients for the pddl manager "
    =>
    (bind ?services (create$
        add_pddl_instance AddPddlInstance
        get_fluents GetFluents
    ))
    (bind ?index 1)
    (bind ?length (length$ ?services))
    (while (< ?index ?length)
        (bind ?service-name (nth$ ?index ?services))
        (bind ?service-type (nth$ (+ ?index 1) ?services))
        (ros-msgs-create-client
            (str-cat "/pddl_manager" "/" ?service-name)
            (str-cat "cx_pddl_msgs/srv/" ?service-type)
        )
        (bind ?index (+ ?index 2))
    )
    (assert (pddl-services-loaded))
  )


Loading the PDDL Instance
^^^^^^^^^^^^^^^^^^^^^^^^^

Once the service clients are available, the next rule defines and registers a
PDDL instance with the external manager. This includes specifying the domain and
problem files, as well as the directory containing them. After registration, it
triggers fetching of the initial fluents representing the problem’s initial state.


.. code-block:: lisp

  (defrule example-pddl-add-instance
    " Setup PDDL instance and fetch initial facts "
    (pddl-services-loaded)
    =>
    (assert
      (pddl-manager (node "/pddl_manager"))
      (pddl-instance
        (name test)
        (domain "domain.pddl")
        (problem "problem.pddl")
        (directory "<absolute path to directory>")
      )
      (pddl-get-fluents (instance test))
    )
  )



Here, three predefined templates are utilized, which are `pddl-manager`_ (registering the node name), `pddl-instance`_ (creating a PDDL instance from filesand naming it) and `pddl-get-fluents`_ (requesting the initial set of fluents).
The ``pddl-get-fluents`` fact triggers a request to the set of `pddl-fluent`_ facts representing the initial state.

|CX| Configuration
^^^^^^^^^^^^^^^^^^

The example configuration below demonstrates how to pre-load the code from the `cx_pddl_clips` package via batch loading, before loading the example code.
Two files are needed. The template definitionsare bundled in ``deftemplates.clp``, while all required rules are loaded through ``pddl.clp``. This split is deliberate, as it allows to override the templates before loading rules, if needed as discussed in more detail later.

.. code-block:: yaml

   /**:
     ros__parameters:
       autostart_node: true
       environments: ["example_agent"]

       structured_pddl_agent:
         plugins: ["executive", "ros_msgs",
                   "pddl_files",
                   "files"]
         log_clips_to_file: true
         watch: ["facts", "rules"]

       ament_index:
         plugin: "cx::AmentIndexPlugin"

       executive:
         plugin: "cx::ExecutivePlugin"

       ros_msgs:
         plugin: "cx::RosMsgsPlugin"

       pddl_files:
         plugin: "cx::FileLoadPlugin"
         pkg_share_dirs: ["cx_pddl_clips"]
         batch: [
           "clips/cx_pddl_clips/deftemplates.clp",
           "clips/cx_pddl_clips/pddl.clp"

         ]

       files:
         plugin: "cx::FileLoadPlugin"
         pkg_share_dirs: ["cx_pddl_bringup"]
         load: ["clips/structured_agent.clp"]


Template Overrides
~~~~~~~~~~~~~~~~~~

The templates provided by the `cx_pddl_clips` package contain a minimal set of slots to provide the functionality.
In practical applications, it might be convenient to store additional information (e.g., context as to why a particular service call is made or hints about how to interpret the results).
Therefore, deftemplate definitions are decoupled from the rules, allowing to first load the deftemplates, then loading re-definitions of them as needed, and finally loading the rule set.
Potential overrides may add more slots, but need to contain all the original slots, as otherwise the predefined rule base will not work.

In the :ref:`Structured PDDL Agent Tutorial <structured_pddl_agent>`,
this feature is used to extend the notion of PDDL actions.

Provided Deftemplates
~~~~~~~~~~~~~~~~~~~~~

In the remainder of this document, all provided deftemplates of the `cx_pddl_clips` are described.

.. _pddl-action:

pddl-action
^^^^^^^^^^^

Represents a grounded PDDL action in a PDDL instance.
Automatically created when a plan is received.
Can also be asserted directly, if actions are needed in different contexts (e.g., to apply effects).
Used to track parameters, plan order, and scheduled times.

.. code-block:: lisp

  (deftemplate pddl-action
    (slot instance (type SYMBOL))
    (slot id (type SYMBOL))
    (slot plan (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot plan-order-class (type INTEGER))
    (slot planned-start-time (type FLOAT))
    (slot planned-duration (type FLOAT))
  )

.. _pddl-action-condition:

pddl-action-condition
^^^^^^^^^^^^^^^^^^^^^

Represents a condition of a PDDL action.
Requires a :ref:`pddl-action` fact with ``id`` matching the value in slot ``action``.
Obtains the satisfaction state and unsatisfied conditions.

.. code-block:: lisp

  (deftemplate pddl-action-condition
    (slot instance (type SYMBOL))
    (slot action (type SYMBOL))
    (slot condition-type (type SYMBOL) (allowed-values ALL START OVERALL END) (default START))
    (slot state (type SYMBOL) (allowed-values PENDING CHECK-CONDITION CONDITION-SAT CONDITION-UNSAT) (default PENDING))
    (multislot unsatisfied-conditions (type STRING) (default (create$)))
  )

.. _pddl-action-get-effect:

pddl-action-get-effect
^^^^^^^^^^^^^^^^^^^^^^

Request the effect of a grounded PDDL action.
Requires a :ref:`pddl-action` fact with ``id`` matching the value in slot ``action``.
Optionally, the effect can also be directly applied using the slot ``apply``.
If effectsa are also applied, corresponding :ref:`pddl-fluent-change` and ref:`pddl-numeric-fluent-change` facts are asserted, before the state is set to ``DONE``.

.. todo:

   This means that the effect is not applied yet, when the field is DONE. a better approach would be to apply the effects when processing the original requrest, then DONE would properly indicate that the effects are actually arlready applied

.. code-block:: lisp

  (deftemplate pddl-action-get-effect
    (slot action (type SYMBOL))
    (slot effect-type (type SYMBOL) (allowed-values ALL START END) (default ALL))
    (slot apply (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
    (slot state (type SYMBOL) (allowed-values PENDING WAITING START-EFFECT-APPLIED DONE ERROR) (default PENDING))
  )

.. _pddl-action-names:

pddl-action-names
^^^^^^^^^^^^^^^^^

Retrieve the list of action names for a PDDL instance.
This is particularly useful for defining goal filters.

.. code-block:: lisp

  (deftemplate pddl-action-names
    (slot instance (type SYMBOL))
    (multislot action-names (type SYMBOL) (default (create$)))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-clear-goals:

pddl-clear-goals
^^^^^^^^^^^^^^^^

Clears all goal conditions for a PDDL instance via the external manager.
This assumes that a matching goal instance was created via :ref:`pddl-create-goal-instance` beforehand.

.. code-block:: lisp

  (deftemplate pddl-clear-goals
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-create-goal-instance:

pddl-create-goal-instance
^^^^^^^^^^^^^^^^^^^^^^^^^

Create a new managed goal instance for a PDDL instance via the external manager.

.. todo:

   We should probably have a pddl-goal-instance fact similar as to how we have an instance fact (or alernatively, add a goals multifield to each instance)

.. code-block:: lisp

  (deftemplate pddl-create-goal-instance
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-effect-fluent:

pddl-effect-fluent
^^^^^^^^^^^^^^^^^^

Represents a boolean effect of a PDDL action.
Asserted when retrieving an action effect via :ref:`pddl-action-get-effect`.

.. code-block:: lisp

  (deftemplate pddl-effect-fluent
    (slot instance (type SYMBOL))
    (slot action (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot effect-type (type SYMBOL) (allowed-values ALL START END) (default ALL))
  )

.. _pddl-effect-numeric-fluent:

pddl-effect-numeric-fluent
^^^^^^^^^^^^^^^^^^^^^^^^^^

Represents a numeric effect of a PDDL action at a specific time.
Asserted when retrieving an action effect via :ref:`pddl-action-get-effect`.

.. todo:

   numeric effects are not supported fully

.. code-block:: lisp

  (deftemplate pddl-effect-numeric-fluent
    (slot instance (type SYMBOL))
    (slot action (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot value (type FLOAT))
    (slot effect-type (type SYMBOL) (allowed-values ALL START END) (default ALL))
  )


.. _pddl-fluent:

pddl-fluent
^^^^^^^^^^^

Represents a boolean fluent in a PDDL instance.
Do not assert/retract/modify directly; use :ref:`pddl-fluent-change` to keep the external PDDL manager in sync.

.. code-block:: lisp

  (deftemplate pddl-fluent
    (slot instance (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
  )

.. _pddl-fluent-change:

pddl-fluent-change
^^^^^^^^^^^^^^^^^^

Indicates that a boolean fluent should be added or removed.
Acts as a transient layer to synchronize with the external PDDL manager.
This automatically updates the corresponding :ref:`pddl-fluent` fact.

.. code-block:: lisp

  (deftemplate pddl-fluent-change
    (slot instance (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot delete (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
    (slot request-id (type INTEGER))
    (slot state (type SYMBOL) (allowed-values PENDING WAITING ERROR ON-HOLD) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-get-fluents:

pddl-get-fluents
^^^^^^^^^^^^^^^^

Fetch all boolean fluents for a PDDL instance via the external manager.
Asserts the corresponding :ref:`pddl-fluent` facts.

.. code-block:: lisp

  (deftemplate pddl-get-fluents
    (slot instance (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-get-numeric-fluents:

pddl-get-numeric-fluents
^^^^^^^^^^^^^^^^^^^^^^^^

Fetch all numeric fluents for a PDDL instance via the external manager.
Asserts the corresponding :ref:`pddl-numeric-fluent` facts.

.. code-block:: lisp

  (deftemplate pddl-get-numeric-fluents
    (slot instance (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-get-predicates:

pddl-get-predicates
^^^^^^^^^^^^^^^^^^^

Fetch all predicates for a PDDL instance via the external manager.
Asserts the corresponding :ref:`pddl-predicate` facts.

.. code-block:: lisp

  (deftemplate pddl-get-predicates
    (slot instance (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-get-type-objects:

pddl-get-type-objects
^^^^^^^^^^^^^^^^^^^^^

Fetch all objects of a specific type for a PDDL instance via the external manager.
Asserts the corresponding :ref:`pddl-type-objects` fact.

.. code-block:: lisp

  (deftemplate pddl-get-type-objects
    (slot instance (type SYMBOL))
    (slot type (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-goal-fluent:

pddl-goal-fluent
^^^^^^^^^^^^^^^^

Represents a positive boolean goal condition for a PDDL instance. Negative conditions are not supported as of now.

.. todo:

   support negative goal fluents

.. code-block:: lisp

  (deftemplate pddl-goal-fluent
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
  )

.. _pddl-goal-numeric-fluent:

pddl-goal-numeric-fluent
^^^^^^^^^^^^^^^^^^^^^^^^

Represents a numeric goal condition with a specific value in a PDDL instance.

.. code-block:: lisp

  (deftemplate pddl-goal-numeric-fluent
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot value (type FLOAT))
  )

.. _pddl-instance:

pddl-instance
^^^^^^^^^^^^^

Initialize a PDDL instance with the external manager.
Also ensures that no confliction requests are made simultaneously, by always tracking, the current operation.

.. code-block:: lisp

  (deftemplate pddl-instance
    (slot name (type SYMBOL))
    (slot domain (type STRING))
    (slot problem (type STRING))
    (slot directory (type STRING))
    (slot state (type SYMBOL) (allowed-values PENDING LOADED ERROR) (default PENDING))
    (slot busy-with (type SYMBOL) (allowed-values FALSE OBJECTS FLUENTS NUMERIC-FLUENTS ACTION-EFFECTS CREATE-GOAL-INSTANCE CLEAR-GOALS SET-GOALS CHECK-CONDITIONS GET-FLUENTS GET-NUMERIC-FLUENTS GET-PREDICATES GET-TYPE-OBJECTS GET-ACTION-NAMES SET-ACTION-FILTER SET-OBJECT-FILTER SET-FLUENT-FILTER CREATE-GOAL-INSTANCE) (default FALSE))
    (slot error (type STRING))
  )

.. _pddl-manager:

pddl-manager
^^^^^^^^^^^^

Assert a fact of this type to specify the name of the PDDL manager node.

.. code-block:: lisp

  (deftemplate pddl-manager
  " Store information on the external pddl manager.
  "
    (slot node (type STRING) (default "/pddl_manager"))
  )

.. _pddl-numeric-fluent:

pddl-numeric-fluent
^^^^^^^^^^^^^^^^^^^

Represents a numeric fluent in a PDDL instance.
Use `pddl-numeric-fluent-change` to update values safely with the external PDDL manager.

.. code-block:: lisp

  (deftemplate pddl-numeric-fluent
    (slot instance (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot value (type FLOAT))
  )

.. _pddl-numeric-fluent-change:

pddl-numeric-fluent-change
^^^^^^^^^^^^^^^^^^^^^^^^^^

Indicates that a numeric fluent should be added or removed.
Acts as a transient layer to synchronize with the external PDDL manager.

.. code-block:: lisp

  (deftemplate pddl-numeric-fluent-change
    (slot instance (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot value (type FLOAT))
    (slot request-id (type INTEGER))
    (slot state (type SYMBOL) (allowed-values PENDING WAITING ERROR ON-HOLD) (default PENDING))
  )

.. _pddl-object-change:

pddl-object-change
^^^^^^^^^^^^^^^^^^

Indicates that an object should be added or removed from a PDDL instance.
Acts as a transient layer to synchronize with the external PDDL manager.

Note that this does not automatically update the :ref:`pddl-type-objects` facts.

.. code-block:: lisp

  (deftemplate pddl-object-change
    (slot instance (type SYMBOL))
    (slot name (type SYMBOL))
    (slot type (type SYMBOL))
    (slot delete (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
    (slot request-id (type INTEGER))
    (slot state (type SYMBOL) (allowed-values PENDING WAITING ERROR ON-HOLD) (default PENDING))
    (slot error (type STRING))
  )


.. _pddl-plan:

pddl-plan
^^^^^^^^^

Represents a PDDL plan.

Consists of individual `pddl-action` facts with matching plan id.

.. code-block:: lisp

  (deftemplate pddl-plan
    (slot id (type SYMBOL))
    (slot instance (type SYMBOL))
    (slot duration (type FLOAT))
  )

.. _pddl-planning-filter:

pddl-planning-filter
^^^^^^^^^^^^^^^^^^^^

A transient layer for planning filters between the general PDDL interface and domain-specific usage.
Can filter actions, objects, or fluents by defining a whitelist.

.. code-block:: lisp

  (deftemplate pddl-planning-filter
    (slot id (type SYMBOL))
    (slot type (type SYMBOL) (allowed-values ACTIONS OBJECTS FLUENTS))
    (multislot filter (type SYMBOL) (default (create$)))
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
  )

.. _pddl-predicate:

pddl-predicate
^^^^^^^^^^^^^^

Represents a predicate in a PDDL instance.

.. code-block:: lisp

  (deftemplate pddl-predicate
    (slot instance (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot param-types (type SYMBOL) (default (create$)))
    (multislot param-names (type SYMBOL) (default (create$)))
  )


.. _pddl-service-request-meta:

pddl-service-request-meta
^^^^^^^^^^^^^^^^^^^^^^^^^

Facts of these types are used internally and should be ignored.


.. code-block:: lisp

  (deftemplate pddl-service-request-meta
    (slot service (type STRING))
    (slot request-id (type INTEGER))
    (slot meta (type SYMBOL))
  )

.. _pddl-set-goals:

pddl-set-goals
^^^^^^^^^^^^^^

Register goal conditions with the external PDDL manager.
The goal conditions are defined using :ref:`pddl-goal-fluent` and :ref:`pddl-goal-numeric-fluent` facts with matching goal ids.

.. code-block:: lisp

  (deftemplate pddl-set-goals
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )


.. _pddl-type-objects:

pddl-type-objects
^^^^^^^^^^^^^^^^^

Lists all objects of a certain type in a PDDL instance.
Created using :ref:`pddl-get-type-objects`.

.. code-block:: lisp

  (deftemplate pddl-type-objects
    (slot instance (type SYMBOL))
    (slot type (type SYMBOL))
    (multislot objects (type STRING) (default (create$)))
  )
