.. _pddl_manager:

PDDL Manager
============

The PDDL Manager node has the following responsibilities:

- Manage multiple PDDL instances concurrently.
- Load PDDL files (optionally templated via Jinja).
- Manage fluents, objects, functions, and goals.
- Check conditions and apply effects of actions.
- Configure planning goals.
- Applying planning filters to narrow down problems to a subset of actions, objects, and fluents for more detailed execution models.
- Trigger planning a for each goal concurrently.
- Return resulting plans in the chosen target format (``SEQUENTIAL_PLAN``, ``TIME_TRIGGERED_PLAN``, ``PARTIAL_ORDER_PLAN``, ``HIERARCHICAL_PLAN``, ``STN_PLAN``)


ROS Parameters
--------------

The following ROS parameters are available:


.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``bond_heartbeat_period``
     - ``0.0``
     - Frequency of bond pulse.
   * - ``planner``
     - ``up_nextflap:NextFLAPImpl``
     - Colon sepearated identifier of the library for importlib and the Class name

Node Parameters
---------------

:autostart_node:

  ============== =======
  Type           Default
  -------------- -------
  bool           false
  ============== =======

  Description
    If set to *true*, automatically activate the lifecycle node on startup.

:bond_heartbeat_period:

  ============== =======
  Type           Default
  -------------- -------
  float          0.0
  ============== =======

  Description
    Heartbeat rate when used to bond with other nodes. A period of 0.0 disables bond usage.

:planner:

  ============== =======
  Type           Default
  -------------- -------
  string         "up_nextflap:NextFLAPImpl"
  ============== =======

  Description
    Colon sepearated identifier of the library for importlib and the Class name.
    Nextflap is provided alongside this extension. When choosing other planners, make sure the respective package is installed on your system.

Basic Workflow
--------------

The PDDL Manager is designed around a clear separation between the *domain
model* (what is true about the world and what actions are available) and
*goals* (what a particular planning request wants to achieve and
which subset of the model it operates on).

PDDL Instances and Associated Goals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The entry point for any use of the manager is the creation of an `instance`, which handles a PDDL domain, it's evolution over time and all associated planning goals.
An instance is setup by loading a PDDL domain fila (and
optionally a problem file) via ``/add_pddl_instance``.
The manager parses file using Jinja2, supporting parameterized and modular domain templates.
If an initial state is supplied (via a problem file), it becomes the current state of the domain represented by the instance. Otherwise the current state is empty.

The instance further manages *goals*, which are  named planning sub-problem attached to a PDDL
instance.
Every PDDL instance starts with a default goal instance called
``base`` that contains the specification supplied through the problem file (or nothing if no file is provided).
Additional goal instances are created via
``/create_goal_instance`` or directly from a string via ``/set_goal_from_string``.

The instance can be updated continuously during execution to reflect the
current state of the world via services:

- ``/add_fluents`` and ``/rm_fluents`` assert or retract boolean predicates.
- ``/set_functions`` updates the value of numeric fluents.
- ``/add_objects`` and ``/rm_objects`` extend or reduce the object universe.

Note that planning requests automatically operates on the
latest world state.

Planning
^^^^^^^^

A planning request is sent as a ROS 2 action goal to ``/plan``.  The
request carries three pieces of information: the PDDL instance name, the
goal instance name, and the desired output plan kind.

Because planning runs in a subprocess, multiple goal instances,even
across different PDDL instances, can be planned concurrently without
blocking each other or blocking service calls that update the world state.

Condition Checking and Effect Inspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Beyond planning, the manager exposes two services that support reactive
execution without requiring a separate planning call.

``/check_action_condition`` evaluates whether the preconditions of a named
action (with concrete object arguments) are satisfied in the current base
problem state.  It returns a boolean result and, on failure, the list of
unsatisfied conditions as human-readable strings.  This is useful for an
executor that wants to verify applicability before dispatching an action to
the physical system.

``/get_action_effects`` computes and returns the effects that a named action
would have if applied in the current state.  Effects are returned as two
separate lists — boolean fluent effects and numeric function effects — each
annotated with the time point at which they take effect (``START`` or
``END``) and, for numeric effects, the operator (``=``, ``+``, ``-``).
This allows an executor to *predict* and *apply* effects locally without
sending another service call to the manager after each action completes.


PDDL Manager Interfaces
-----------------------

The PDDL Manager exposes a wide range of ROS 2 interfaces.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Service / Topic
     - Corresponding Deftemplate(s)
   * - :abbr:`/add_pddl_instance (Add a new PDDL instance to the manager)`
     - :ref:`pddl-instance`
   * - :abbr:`/check_action_condition (Check if the preconditions of a PDDL action are satisfied)`
     - :ref:`pddl-action-condition`
   * - :abbr:`/get_action_effects (Retrieve the effects of a PDDL action)`
     - :ref:`pddl-action-get-effect`
   * - :abbr:`/get_action_names (Get the list of available action names for a PDDL instance)`
     - :ref:`pddl-action-names`
   * - :abbr:`/get_fluents (Fetch all current boolean fluents of a PDDL instance)`
     - :ref:`pddl-get-fluents`
   * - :abbr:`/add_fluents (Add multiple fluents to a PDDL instance)`
     - :ref:`pddl-fluent-change`
   * - :abbr:`/rm_fluents (Remove multiple fluents from a PDDL instance)`
     - :ref:`pddl-fluent-change`
   * - :abbr:`/add_objects (Add objects to a PDDL instance)`
     - :ref:`pddl-object-change`
   * - :abbr:`/rm_objects (Remove objects from a PDDL instance)`
     - :ref:`pddl-object-change`
   * - :abbr:`/set_functions (Set values for numeric functions in the PDDL instance)`
     - :ref:`pddl-numeric-fluent-change`
   * - :abbr:`/get_functions (Retrieve values of numeric functions)`
     - :ref:`pddl-get-numeric-fluents`
   * - :abbr:`/set_goal_from_string (Create a goal instance for planning and populate it from a string)`
     - :ref:`pddl-set-goal-from-string`
   * - :abbr:`/set_goals (Register goal conditions with the PDDL manager)`
     - :ref:`pddl-set-goals`
   * - :abbr:`/clear_goals (Clear all goal conditions of a PDDL instance)`
     - :ref:`pddl-clear-goals`
   * - :abbr:`/set_action_filter (Apply a filter to restrict planning to specific actions)`
     - :ref:`pddl-planning-filter`
   * - :abbr:`/set_object_filter (Apply a filter to restrict planning to specific objects)`
     - :ref:`pddl-planning-filter`
   * - :abbr:`/set_fluent_filter (Apply a filter to restrict planning to specific fluents)`
     - :ref:`pddl-planning-filter`
   * - :abbr:`/create_goal_instance (Create a new goal instance for planning)`
     - :ref:`pddl-create-goal-instance`
   * - :abbr:`/get_predicates (Fetch all predicates of a PDDL instance)`
     - :ref:`pddl-get-predicates`
   * - :abbr:`/get_type_objects (Fetch all objects of a certain type in a PDDL instance)`
     - :ref:`pddl-get-type-objects`
   * - :abbr:`/plan (Send a planning goal and retrieve the resulting plan)`
     - :ref:`pddl-plan`, :ref:`pddl-action`
   * - :abbr:`/instance_update (Publishes updates about instance changes in the manager)`
     - :ref:`pddl-manager`

Plan Converions
---------------

In the Unified Planning Framework, the output format of a planning result depends on the planner used.
For example, the NextFLAP planner returns partial-order plans when solving classical PDDL problems, while other planners may produce sequential plans instead.

The Unified Planning Framework also provides utilities to convert plans between different representations, such as converting between sequential and partial-order plans, or between time-triggered (temporal) plans and Simple Temporal Network (STN) representations.

The PDDL Manager requires users to explicitly specify their desired output format and converts retrieved plans accordingly.
However, not all conversions are meaningful or supported. For instance, converting a classical plan into a hierarchical plan is generally not appropriate, since no abstract tasks or methods are available in that setting.
Users should therefore carefully consider the chosen PDDL model, planner, and requested output format to ensure compatibility and meaningful results.

In addition to the conversions provided by the Unified Planning Framework, the PDDL Manager implements further conversions from time-triggered plans or STNs into partial-order plans.
This conversion is based on a relaxation of the underlying STN: temporal constraints are removed, and the remaining structure is interpreted as causal links.
