.. _pddl_clips:

PDDL CLIPS Interfaces
=====================

The PDDL integration is provided in the form of the `PDDL Manager`, a ROS lifecycle node that provides ROS interfaces, which can be called through the |CX| plugins that handle ROS communication.

In order to reduce the manual overhead, the ``cx_pddl_clips`` package provides a CLIPS-based interface for interacting with the PDDL Manager node in ROS 2.
This allows to interact with the PDDL manager by simply asserting and monitoring CLIPS facts, without the need to do direct ROS communication (e.g., populating ROS messages or waiting for service feedback).

CLIPS Integration
-----------------

CLIPS needs to interact with the :ref:`pddl_manager` by creating subscriptions and clients for the endpoints to then utilize the features as desired.
In order to reduce manual overhead, predefined rules and deftemplates are available, which are described in the remainder of this document.

The idea is to have templates for facts that correspond to the individual endpoints of the PDDL manager.
The workflow then is for users to simply assert a fact of a defined template to trigger the request to the PDDL manager, then using rules to observe the provided slots for the outcome.

A comprehensive example for using this integration is given in the :ref:`Tutorial for using the PDDL Manager with cx_pddl_clips <cx_pddl_clips_tutorial>`

Request Execution Model
^^^^^^^^^^^^^^^^^^^^^^^

The CLIPS interface allows users to assert multiple request facts concurrently
(e.g., adding fluents, creating goal instances, or checking action conditions).
However, most interactions with the external PDDL manager must be processed
**sequentially per PDDL instance**.

To guarantee consistency between CLIPS and the PDDL manager, the rule base
ensures that only **one request affecting a given PDDL instance is active at a
time**. Additional requests remain pending until the current operation has
completed.

This coordination is handled using the ``busy-with`` slot of the
:ref:`pddl-instance` fact. While an instance is marked as busy, other rules that
would trigger conflicting service calls are temporarily blocked.


Example:

* A request to add fluents must finish before conditions are checked.
* Retrieving fluents or predicates waits until previous updates have completed.
* Goal configuration steps (creating goal instances, setting goals, applying
  filters) are executed in order.

Parallel Planning
~~~~~~~~~~~~~~~~~

Planning requests are handled differently.

Once a goal instance has been created and the goals are registered, a planning
request (e.g., via :ref:`pddl-plan`) is sent to the planner asynchronously.
Multiple planning tasks may therefore run **in parallel**, allowing the system
to evaluate different goals or planning configurations concurrently.

When a plan is returned, the CLIPS interface automatically creates the
corresponding :ref:`pddl-plan` and :ref:`pddl-action` facts.

Rule Priorities
~~~~~~~~~~~~~~~

Internally, the rule base uses custom saliences (priorities) to enforce a
deterministic ordering of PDDL-related operations (e.g., instance management,
object updates, fluent updates, goal configuration, and planning). This ensures
that dependent operations are always processed in a consistent order while
allowing CLIPS reasoning to interleave with ROS communication.

THe full list of used rule saliences is depicted below:

.. code-block:: clips

   (defglobal
    ?*PRIORITY-PDDL-INSTANCES* = -5100
    ?*PRIORITY-PDDL-GET-ACTION-NAMES* = -5300
    ?*PRIORITY-PDDL-OBJECTS* = -5400
    ?*PRIORITY-PDDL-FLUENTS* = -5500
    ?*PRIORITY-PDDL-APPLY-EFFECT* = -5600
    ?*PRIORITY-PDDL-CLEAR-GOALS* = -5700
    ?*PRIORITY-PDDL-CREATE-GOAL-INSTANCE* = -5800
    ?*PRIORITY-PDDL-SET-ACTION-FILTER* = -5900
    ?*PRIORITY-PDDL-SET-FLUENT-FILTER* = -6000
    ?*PRIORITY-PDDL-SET-OBJECT-FILTER* = -6100
    ?*PRIORITY-PDDL-SET-GOALS* = -6200
    ?*PRIORITY-PDDL-PLAN* = -6300
    ?*PRIORITY-PDDL-CHECK-CONDITION* = -6400
    ?*PRIORITY-PDDL-GET-FLUENTS* = -6500
  )

Template Overrides
~~~~~~~~~~~~~~~~~~

The templates provided by the `cx_pddl_clips` package contain a minimal set of slots to provide the functionality.
In practical applications, it might be convenient to store additional information (e.g., context as to why a particular service call is made or hints about how to interpret the results).
Therefore, deftemplate definitions are decoupled from the rules, allowing to first load the deftemplates, then loading re-definitions of them as needed, and finally loading the rule set.
Potential overrides may add more slots, but need to contain all the original slots, as otherwise the predefined rule base will not work.

In the :ref:`cx_pddl_clips Agent Tutorial <cx_pddl_clips_tutorial>`,
this feature is used to extend the notion of PDDL actions.


Using the |CX| with cx_pddl_clips
---------------------------------

The example configuration below demonstrates how to pre-load the code from the `cx_pddl_clips` package via batch loading, before loading the example code.
In order to integrate the PDDL manager with the |CX|, the following plugins are needed:

- :ref:`cx::ExecutivePlugin <usage_executive_plugin>`: Manages the overall reasoning and control flow, interleaving ROS feedback with CLIPS reasoning.
- :ref:`cx::RosMsgsPlugin <usage_ros_msgs_plugin>`: Provides access to ROS interfaces of the PDDL manager from within CLIPS.
- :ref:`cx::AmentIndexPlugin <usage_ament_index_plugin>`: Resolves package paths via ``ament_index``. While not required, it is very useful in order to load PDDL files.

Also, as the current configuration is compatible with ROS 2 jazzy, action client introspection is not supported, hence the following plugins are needed to trigger planning and obtain the resulting plan:

- ``cx::CXCxPddlInterfacesPlanPlugin``
- ``cx:CXCxPddlInterfacesPlanActionPlugin``
- ``"cx::CXCxPddlInterfacesHierarchicalPlanMethodPlugin``
- ``cx::CXCxPddlInterfacesStnConstraintPlugin``
- ``cx::CXCxPddlInterfacesHierarchicalPlanMethodPlugin``


.. code-block:: yaml

   /**:
     ros__parameters:
       autostart_node: true
       environments: ["cx_pddl_clips_agent"]

       cx_pddl_clips_agent:
         plugins: ["executive", "ros_msgs",
                   "ament_index",
                   "ros_param",
                   "plan_action",
                   "plan_action_msg",
                   "stn_constraint_msg",
                   "hierarchical_plan_method_msg",
                   "pddl_files",
                   "files"]
         log_clips_to_file: true
         watch: ["facts", "rules"]
         redirect_stdout_to_debug: true

       ament_index:
         plugin: "cx::AmentIndexPlugin"

       ros_param:
         plugin: "cx::RosParamPlugin"

       executive:
         plugin: "cx::ExecutivePlugin"

       ros_msgs:
         plugin: "cx::RosMsgsPlugin"

       pddl_files:
         plugin: "cx::FileLoadPlugin"
         pkg_share_dirs: ["cx_pddl_clips", "cx_pddl_bringup"]
         batch: [
           "clips/cx_pddl_clips/deftemplates.clp",
           "clips/cx_pddl_bringup/deftemplate-overrides.clp",
           "clips/cx_pddl_clips/pddl-no-deftemplates.clp"
         ]

       files:
         plugin: "cx::FileLoadPlugin"
         pkg_share_dirs: ["cx_pddl_bringup"]
         load: ["clips/cx_pddl_bringup/cx-pddl-generic-agent.clp"]

       plan_action:
         plugin: "cx::CXCxPddlInterfacesPlanPlugin"
       plan_action_msg:
         plugin: "cx::CXCxPddlInterfacesPlanActionPlugin"
       stn_constraint_msg:
         plugin: "cx::CXCxPddlInterfacesStnConstraintPlugin"
       hierarchical_plan_method_msg:
         plugin: "cx::CXCxPddlInterfacesHierarchicalPlanMethodPlugin"


Example: Loading a PDDL Problem and Obtaining the initial State
---------------------------------------------------------------

The following example rule demonstrates how to initialize the connection to the external
PDDL manager and load a PDDL problem into CLIPS.

.. code-block:: clips

  (defrule example-pddl-add-instance
    " Setup PDDL instance and fetch initial facts "
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

The first step is to initialize the interfaces to a running PDDL manager instance by asserting a respective :ref:`pddl-manager` fact.
To create an instance, a :ref:`pddl-instance` fact is asserted, providing the domain and
problem files, as well as the directory containing them.
In order to fetch the initial fluents of the instance (as provided in the problem file), a :ref:`pddl-get-fluents` fact is asserted.


Provided Deftemplates
---------------------

In the remainder of this document, all provided deftemplates of the `cx_pddl_clips` are described.

.. _pddl-action:

pddl-action
^^^^^^^^^^^

Represents a grounded PDDL action in a PDDL instance.
Automatically created when a plan is received.
Can also be asserted directly, if actions are needed in different contexts (e.g., to apply effects).
Used to track parameters, plan order, and scheduled times.

.. code-block:: clips

  (deftemplate pddl-action
    (slot instance (type SYMBOL))
    (slot id (type SYMBOL)) ; this should be a globally unique ID
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot plan (type SYMBOL))
    (slot order (type INTEGER))
    (multislot predecessors (type INTEGER))
    (slot task-id (type SYMBOL))
    (slot planned-start-time (type FLOAT))
    (slot planned-duration (type FLOAT))
  )

.. _pddl-action-condition:

pddl-action-condition
^^^^^^^^^^^^^^^^^^^^^

Represents a condition of a PDDL action.
Requires a :ref:`pddl-action` fact with ``id`` matching the value in slot ``action``.
Obtains the satisfaction state and unsatisfied conditions.

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

  (deftemplate pddl-manager
  " Store information on the external pddl manager.
  "
    (slot node (type STRING) (default "/pddl_manager"))
  )

.. _pddl-method:

pddl-method
^^^^^^^^^^^

Represents a grounded PDDL method in a PDDL instance belonging to a hierarchical plan.
Automatically created when a plan is received.

.. code-block:: clips

  (deftemplate pddl-method
    (slot instance (type SYMBOL))
    (slot plan (type SYMBOL))
    (slot id (type SYMBOL)) ; this should be a globally unique ID
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot task-id (type SYMBOL))
  )

.. _pddl-numeric-fluent:

pddl-numeric-fluent
^^^^^^^^^^^^^^^^^^^

Represents a numeric fluent in a PDDL instance.
Use `pddl-numeric-fluent-change` to update values safely with the external PDDL manager.

.. code-block:: clips

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

.. code-block:: clips

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

.. code-block:: clips

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

Represents a PDDL plan. The chosen ``plan-type`` determines the format of the resulting plans.

Plans consist of individual :ref:`pddl-action` facts with matching plan id and might additionally have :ref:`pddl-stn-constraint` (for STN plans) or :ref:`pddl-method` facts (for hierarchical plans).

.. code-block:: clips

  (deftemplate pddl-plan
    (slot instance (type SYMBOL))
    (slot id (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot goal-ptr (type EXTERNAL-ADDRESS))
    (slot plan-type (type SYMBOL) (allowed-values CLASSICAL TEMPORAL PARTIAL-ORDER HIERARCHICAL STN) (default CLASSICAL))
    (slot action-type (type SYMBOL) (allowed-values CLASSICAL TEMPORAL STN) (default CLASSICAL))
    (slot goal-handle (type EXTERNAL-ADDRESS))
    (slot output-dir (type STRING))
    (slot state (type SYMBOL) (allowed-values PENDING WAITING PLANNING REQUEST-CANCELING CANCELING CANCELED SUCCESS FAILURE) (default PENDING))
  )



.. _pddl-planning-filter:

pddl-planning-filter
^^^^^^^^^^^^^^^^^^^^

A transient layer for planning filters between the general PDDL interface and domain-specific usage.
Can filter actions, objects, or fluents by defining a whitelist.

.. code-block:: clips

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

.. code-block:: clips

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


.. code-block:: clips

  (deftemplate pddl-service-request-meta
    (slot service (type STRING))
    (slot request-id (type INTEGER))
    (slot meta (type SYMBOL))
  )

.. _pddl-set-goal-from-string:

pddl-set-goal-from-string
^^^^^^^^^^^^^^^^^^^^^^^^^

Register arbitrary goal specifications with the external PDDL manager.
As :ref:`pddl-set-goals` is limited to simple fuent conditions, this string-based
method provides an alternative when more flexibility is needed.
Internally a temporary problem file is created with the current fluents and objects.

The provided string is then placed at the end of the file.
An example string is provided below:

.. code-block:: bash

  "(:goal (and (on a b) (on b c)))"


This method can also be used to specify a goal with custom plan quality metrics:

.. code-block:: bash

  "(:goal ...) (:metric (minimize (fuel-cost)))"

Note that this will re-create the goal instance, any prior goal fluents set for
the goal via :ref:`pddl-set-goals` are lost.

.. code-block:: clips

  (deftemplate pddl-set-goal-from-string
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot goal-description (type STRING))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-set-goals:

pddl-set-goals
^^^^^^^^^^^^^^

Register goal conditions with the external PDDL manager.
The goal conditions are defined using :ref:`pddl-goal-fluent` and :ref:`pddl-goal-numeric-fluent` facts with matching goal ids.

.. code-block:: clips

  (deftemplate pddl-set-goals
    (slot instance (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
    (slot error (type STRING))
  )

.. _pddl-stn-constraint:

pddl-stn-constraint
^^^^^^^^^^^^^^^^^^^

Represents a STN constraint of a STN plan.
Automatically created when a plan is received.
The slots ``from`` and ``to`` correspond to the ids of a given plan action in the respective ``order`` field.
Constraints have the form: ``lower-bound < time(to, to-role) - time(from, from-role) < upper-bound``


.. code-block:: clips

  (deftemplate pddl-stn-constraint
    (slot instance (type SYMBOL))
    (slot plan (type SYMBOL))
    (slot id (type SYMBOL)) ; this should be a globally unique ID
    (slot from (type INTEGER))
    (slot from-role (type SYMBOL) (allowed-values START END))
    (slot to (type INTEGER))
    (slot to-role (type SYMBOL) (allowed-values START END))
    (slot is-lower-bounded (type SYMBOL) (allowed-values TRUE FALSE))
    (slot is-upper-bounded (type SYMBOL) (allowed-values TRUE FALSE))
    (slot lower-bound (type INTEGER))
    (slot upper-bound (type INTEGER))
  )



.. _pddl-type-objects:

pddl-type-objects
^^^^^^^^^^^^^^^^^

Lists all objects of a certain type in a PDDL instance.
Created using :ref:`pddl-get-type-objects`.

.. code-block:: clips

  (deftemplate pddl-type-objects
    (slot instance (type SYMBOL))
    (slot type (type SYMBOL))
    (multislot objects (type STRING) (default (create$)))
  )
