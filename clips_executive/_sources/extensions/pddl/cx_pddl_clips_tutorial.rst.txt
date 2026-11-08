.. _cx_pddl_clips_tutorial:

Tutorial: cx_pddl_clips Agent
=============================

**Goal:** Use CLIPS to plan and execute PDDL actions with monitoring,
timing information, and support for different plan representations using the
interfaces provided by ``cx_pddl_clips``.

**Tutorial level:** Advanced

**Time:** 45–60 minutes


.. contents:: Contents
   :depth: 2
   :local:


Overview
--------

This tutorial demonstrates how to implement a PDDL-based planning and
execution agent using CLIPS and the interfaces provided by
``cx_pddl_clips``.

The presented agent does not only request a plan from the PDDL manager, but
also implements a complete execution loop:

* selecting actions from generated plans,
* checking action conditions,
* tracking execution times,
* applying action effects,
* comparing planned and actual execution durations.

The agent supports multiple planning representations:

* classical sequential plans,
* temporal plans,
* partial-order plans,
* hierarchical plans,
* STN plans (inspection only).

You will learn how to:

1. Extend PDDL fact templates using CLIPS overrides,
2. Configure a CLIPS environment with the required PDDL plugins,
3. Create PDDL planning instances from CLIPS,
4. Execute and monitor generated plans,
5. Handle different plan representations.


Prerequisites
-------------

This tutorial assumes that |CX| is installed and that you are familiar with
creating and configuring a custom package using |CX|.


Package Layout
--------------

The relevant directory layout for the PDDL CLIPS agent is shown below.
The example code is part of the ``cx_pddl_bringup`` package.

.. code-block:: text

   cx_pddl_bringup
   ├── CMakeLists.txt
   ├── package.xml
   ├── config
   │   └── cx_pddl_clips_agent.yaml
   ├── clips
   │   └── cx_pddl_bringup
   │       ├── cx-pddl-clips-agent.clp
   │       └── deftemplate-overrides.clp
   └── pddl
       ├── domain.pddl
       └── problem.pddl


Directory Layout
----------------

The package contains several directories with different responsibilities:

* ``config/``

  Contains configuration files used to configure the CLIPS agent and load
  the required plugins.

* ``clips/``

  Contains the CLIPS rules implementing the PDDL planning and execution
  logic.

* ``pddl/``

  Contains the PDDL domain and problem definitions.


Configuration
-------------

The file ``config/cx_pddl_clips_agent.yaml`` configures the CLIPS agent,
loads the required plugins, and specifies which CLIPS files are loaded.

.. code-block:: yaml

  /**:
    ros__parameters:
      autostart_node: true

      pddl:
        package_dir: "cx_pddl_bringup"
        plan_type: "TEMPORAL"
        domain: "domain.pddl"
        problem: "problem.pddl"

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


The configuration creates a CLIPS environment with several plugins:

* ``ament_index`` Provides access to package locations from CLIPS.

* ``executive`` Provides the CLIPS execution loop.

* ``ros_msgs`` Enables communication with ROS interfaces.

* ``pddl_files`` Loads reusable PDDL-related CLIPS definitions.

* ``files`` Loads the tutorial-specific agent implementation.

* ``plan_action`` Provides access to the PDDL planning action interface.

* ``plan_action_msg`` Provides the message definitions required by the planning action.

* ``stn_constraint_msg`` Provides access to STN constraint messages.

* ``hierarchical_plan_method_msg`` Provides access to hierarchical planning method messages.


Defining the PDDL Action Template
---------------------------------

The tutorial agent overrides the default ``pddl-action`` and ``pddl-plan``
templates provided by ``cx_pddl_clips``.

The override is used to demonstrate the mechanism.
The templates of ``cx_pddl_clips`` were deliberately kept minimal with an intended
way for users to extend them as needed.

The extended templates are located in:

.. code-block:: text

   clips/cx_pddl_bringup/deftemplate-overrides.clp


The ``pddl-action`` template adds additional state information as well as slots for the actual start time and duration:

.. code-block:: clips

  (deftemplate pddl-action
    (slot instance (type SYMBOL))
    (slot id (type SYMBOL))
    (slot name (type SYMBOL))
    (multislot params (type SYMBOL) (default (create$)))
    (slot plan (type SYMBOL))
    (slot order (type INTEGER))
    (multislot predecessors (type INTEGER))
    (slot task-id (type SYMBOL))
    (slot planned-start-time (type FLOAT))
    (slot planned-duration (type FLOAT))
    (slot actual-start-time (type FLOAT))
    (slot actual-duration (type FLOAT))
    (slot state (type SYMBOL)
      (allowed-values IDLE SELECTED EXECUTING DONE))
  )


The ``pddl-plan`` template is extended with a ``plan-start`` slot:

.. code-block:: clips

  (deftemplate pddl-plan
    (slot instance (type SYMBOL))
    (slot id (type SYMBOL))
    (slot goal (type SYMBOL))
    (slot goal-ptr (type EXTERNAL-ADDRESS))
    (slot plan-type
      (type SYMBOL)
      (allowed-values
        CLASSICAL
        TEMPORAL
        PARTIAL-ORDER
        HIERARCHICAL
        STN)
      (default CLASSICAL))
    (slot action-type
      (type SYMBOL)
      (allowed-values CLASSICAL TEMPORAL STN))
    (slot goal-handle (type EXTERNAL-ADDRESS))
    (slot output-dir (type STRING))
    (slot state
      (type SYMBOL)
      (allowed-values
        PENDING
        WAITING
        PLANNING
        REQUEST-CANCELING
        CANCELING
        CANCELED
        SUCCESS
        FAILURE)
      (default PENDING))
    (slot plan-start (type FLOAT) (default 0.0))
  )

The template override is loaded after the initial template definition
provided by the PDDL interface, but before any rules depending on the
template are defined. The override retains all original slots and only adds
additional ones required by this tutorial.

This approach ensures that the rule set provided by the interface can still be
loaded without modification, while allowing the tutorial agent to extend the
template with additional execution-related information.

Agent Code Logic
----------------

The file:

.. code-block:: text

   clips/cx_pddl_bringup/cx-pddl-generic-agent.clp

contains the CLIPS rules implementing the planning and execution logic.

The agent follows this general workflow:

1. Initialize communication with the PDDL manager.
2. Create a PDDL planning instance.
3. Request a plan.
4. Select actions according to the generated plan representation.
5. Check whether actions are executable.
6. Execute actions and apply effects.
7. Print execution statistics.


Initializing the PDDL Manager
-----------------------------

The first rule initializes the connection to the PDDL manager.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-pddl-init
  =>
    (assert
      (pddl-manager
        (node "/pddl_manager")))
  )


The ``pddl-manager`` fact provides the interface used by the remaining rules
to communicate with the planning component.


Creating a PDDL Instance
------------------------

After the PDDL manager becomes available, the agent creates a planning
instance.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-pddl-add-instance
  " Setup PDDL instance with an active goal to plan for "
    (pddl-manager (ros-comm-init TRUE))
  =>
    (bind ?type (ros-param-get-value "pddl.plan_type" "TEMPORAL"))
    (bind ?domain (ros-param-get-value "pddl.domain" "domain.pddl"))
    (bind ?problem (ros-param-get-value "pddl.problem" "problem.pddl"))
    (bind ?package (ros-param-get-value "pddl.package_dir" "cx_pddl_bringup"))
    (bind ?share-dir (ament-index-get-package-share-directory ?package))
    (assert
      (pddl-instance
        (name test)
        (domain ?domain)
        (problem ?problem)
        (directory (str-cat ?share-dir "/pddl"))
      )
      (pddl-get-fluents (instance test))
      (pddl-plan (id test-plan) (instance test) (goal base) (plan-type (sym-cat ?type)))
    )
  )

The planning instance uses the domain and problem files specified through ROS
parameters.

The available parameters are:

.. code-block:: yaml

   pddl.domain
   pddl.problem
   pddl.package_dir
   pddl.plan_type


When a plan is generated, the planner creates ``pddl-action`` facts
representing the individual actions.

The task is now to select actions, check their conditions to ensure a consistent domain model, execute the action and applying effects, until the plan is executed.

Selecting Actions
-----------------

The agent supports different plan representations. Each representation has a
different rule for selecting the next executable action.


Classical Plans
^^^^^^^^^^^^^^^

For classical plans, actions are executed according to their order.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-select-action-sequential
  " Start executing the first action of the resulting plan "
    ?plan <- (pddl-plan (id ?plan-id) (plan-type HIERARCHICAL|CLASSICAL)
                        (plan-start ?p-start) (state SUCCESS) (action-type CLASSICAL))
    (not (pddl-action (state EXECUTING|SELECTED)))
    ?pa <- (pddl-action (plan ?plan-id) (order ?o) (state IDLE))
    (not (pddl-action (plan ?plan-id) (state IDLE) (order ?oo&:(< ?oo ?o))))
  =>
    (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
    (modify ?pa (state SELECTED))
  )

Temporal Plans
^^^^^^^^^^^^^^

Temporal plans use the planned start time of each action.
A helper ``plan-timeline`` fact stores the current execution progress in a temporal
plan.

.. code-block:: clips

  (deftemplate plan-timeline
    (slot plan-id (type SYMBOL))
    (slot current-time (type FLOAT) (default 0.0))
  )

  (defrule cx-pddl-bringup-generic-agent-create-timeline
    (pddl-plan (id ?plan-id) (plan-start ?st) (action-type TEMPORAL))
    (pddl-action (plan ?plan-id))
    (not (plan-timeline (plan-id ?plan-id)))
    =>
    (assert (plan-timeline (plan-id ?plan-id) (current-time ?st)))
  )

  (defrule cx-pddl-bringup-generic-agent-select-action-temporal
  " Start executing the first action of the resulting plan based on start time"
    ?plan <- (pddl-plan (id ?plan-id) (plan-type TEMPORAL|HIERARCHICAL) (plan-start ?p-start) (state SUCCESS) (action-type TEMPORAL))
    ?pa <- (pddl-action (id ?a-id) (plan ?plan-id) (planned-start-time ?t) (state IDLE))
    (not (pddl-action (id ?a-id2&:(neq ?a-id ?a-id2)) (plan ?plan-id) (planned-start-time ?t2&:(< ?t2 ?t)) (state IDLE)))
    ?pt <- (plan-timeline (plan-id ?plan-id) (current-time ?st&:(<= ?t ?st)))
  =>
    (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
    (modify ?pa (state SELECTED))
    (modify ?pt (current-time ?t))
  )


Partial-Order Plans
^^^^^^^^^^^^^^^^^^^

For partial-order plans, actions become available once all predecessor
constraints have been satisfied.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-select-action-partial-order
  " Start executing the first action of the resulting plan based on order"
    ?plan <- (pddl-plan (id ?plan-id) (plan-type PARTIAL-ORDER) (plan-start ?p-start) (state SUCCESS) (action-type CLASSICAL))
    (not (pddl-action (state EXECUTING|SELECTED)))
    ?pa <- (pddl-action (plan ?plan-id) (order ?own) (predecessors) (planned-start-time ?t) (state IDLE))
  =>
    (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
    (modify ?pa (state SELECTED))
  )



STN Plans
^^^^^^^^^

STN plans are currently not directly supported for execution.

Instead, the agent detects the plan type and prints the generated actions and
constraints.

This allows inspection of the temporal constraints generated by the planner
without providing an execution mechanism.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-detect-STN-no-execution-available
  " For STN plans, there is no execution available "
    ?plan <- (pddl-plan (id ?plan-id) (plan-type STN) (plan-start ?p-start) (state SUCCESS) (action-type STN))
  =>
    (printout magenta "STN plan detected, no method for execution available" crlf)
    (printout magenta "Actions:" crlf)
    (do-for-all-facts ((?pa pddl-action)) TRUE
       (printout green "action " ?pa:name " "
         ?pa:params " (STN id: " ?pa:order ")" crlf
       )
    )
    (printout magenta "Constraints:" crlf)
    (do-for-all-facts ((?stn-c pddl-stn-constraint)) TRUE
       (if (and ?stn-c:is-lower-bounded ?stn-c:is-upper-bounded) then
         (printout green "action " ?stn-c:from " " ?stn-c:from-role
           " --[ " ?stn-c:lower-bound ", " ?stn-c:upper-bound " ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
       )
       (if (and ?stn-c:is-lower-bounded (not ?stn-c:is-upper-bounded)) then
         (printout green "action " ?stn-c:from " " ?stn-c:from-role
           " --[ " ?stn-c:lower-bound ", INF ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
       )
       (if (and (not ?stn-c:is-lower-bounded) ?stn-c:is-upper-bounded) then
         (printout green "action " ?stn-c:from " " ?stn-c:from-role
           " --[ -INF, " ?stn-c:upper-bound " ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
       )
       (if (and (not ?stn-c:is-lower-bounded) (not ?stn-c:is-upper-bounded)) then
         (printout green "action " ?stn-c:from " " ?stn-c:from-role
           " --[ -INF, INF ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
       )
    )
  )


Action Execution
----------------

After an action has been selected, the agent checks whether the action can be
executed.

The execution loop is intentionally generic. It does not directly control a
robot, but provides the required interface points where a real execution
system can be connected.


Checking Action Conditions
^^^^^^^^^^^^^^^^^^^^^^^^^^

Before executing an action, the agent requests the current execution
condition.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-check-action
  " Before executing an action check the condition to make sure it is feasible "
    (pddl-action (id ?id) (state SELECTED) (name ?name) (params $?params))
    (not (pddl-action-condition (action ?id)))
  =>
    (assert (pddl-action-condition (instance test) (action ?id)))
  )



The PDDL interface evaluates the condition and updates the corresponding
``pddl-action-condition`` fact.


Starting Execution
^^^^^^^^^^^^^^^^^^

When the action condition is satisfied, the action is started.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-executable-action
  " Condition is satisfied, go ahead with execution "
    (pddl-plan (id ?plan-id) (plan-start ?t))
    (pddl-action-condition (action ?action-id) (state CONDITION-SAT))
    ?pa <- (pddl-action (id ?action-id) (plan ?plan-id) (name ?name) (params $?params) (state SELECTED))
  =>
    (modify ?pa (state EXECUTING) (actual-start-time (- (now) ?t)))
  )


The execution start time is stored to allow comparison between planned and
actual execution timing.


Finishing Execution
^^^^^^^^^^^^^^^^^^^

After the planned action duration has elapsed, the action is completed.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-execution-done
  " After the duration has elapsed, the action is done "
    (time ?now)
    (pddl-plan (id ?plan-id) (plan-start ?t))
    ?pa <- (pddl-action (id ?id) (plan ?plan-id) (state EXECUTING) (planned-duration ?d) (name ?name)
      (actual-start-time ?s&:(< (+ ?s ?d ?t) ?now)))
  =>
    (bind ?duration (- (now) (+ ?s ?t)))
    (printout info "Executed action " ?name " in " ?duration " seconds" crlf)
    (modify ?pa (state DONE) (actual-duration ?duration))
    (assert (pddl-action-get-effect (action ?id) (apply TRUE)))
  )

  (defrule cx-pddl-bringup-generic-agent-rm-get-effect-on-done
    ?f <- (pddl-action-get-effect (state DONE))
    =>
    (retract ?f)
  )

After completion, the PDDL manager applies the action effects and updates the
planning state.


Partial-Order Execution Updates
-------------------------------

Partial-order plans can contain actions with predecessor constraints.

After an action finishes, the agent removes the satisfied predecessor
relationship from following actions.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-relax-partial-order
    (pddl-plan (id ?plan-id) (plan-type PARTIAL-ORDER))
    (pddl-action (order ?o) (plan ?plan-id) (state DONE))
    (pddl-action (plan ?plan-id) (id ?a-id) (state IDLE) (predecessors $? ?o $?))
    (not (pddl-action-get-effect (action ?a-id)))
    =>
    (do-for-all-facts ((?pa pddl-action))
      (and
        (eq ?pa:plan ?plan-id)
        (eq ?pa:state IDLE)
        (member$ ?o ?pa:predecessors)
      )
      (bind ?pos (member$ ?o ?pa:predecessors))
      (modify ?pa (predecessors (delete$ ?pa:predecessors ?pos ?pos)))
    )
  )



This allows actions that no longer have unsatisfied dependencies to become
available for execution.


Temporal Timeline Updates
-------------------------

Temporal plans can contain multiple actions scheduled at the same execution
time.

The timeline is advanced only after all actions at the current timestamp have
finished.

.. code-block:: clips

  (defrule cx-pddl-bringup-generic-agent-update-timeline
  "When all parallel actions at a particular time are done, move the timeline forward."
    (pddl-plan (id ?plan-id) (action-type TEMPORAL))
    ?pt <- (plan-timeline (plan-id ?plan-id) (current-time ?st))
    (pddl-action (id ?id) (plan ?plan-id) (state ~DONE) (planned-start-time ?st1&:(> ?st1 ?st)))
    (not (pddl-action (id ?o-id1) (plan ?plan-id) (state ~DONE) (planned-start-time ?st2&:(<= ?st2 ?st))))
    (not (pddl-action (id ?o-id2&:(neq ?id ?o-id2)) (plan ?plan-id) (state ~DONE) (planned-start-time ?st3&:(< ?st3 ?st1))))
    =>
    (modify ?pt (current-time ?st1))
  )


This ensures that temporal plans respect the ordering, while using timing information on a best-effort basis.

Execution Summary
-----------------

Once all actions have completed, the agent prints a comparison between
planned and actual execution times.

.. code-block:: clips


  (defrule cx-pddl-bringup-generic-agent-print-exec-times
  " Once everything is done, print out planned vs actual times "
    (pddl-action)
    (not (pddl-action (state ~DONE)))
    (not (printed))
  =>
    (printout blue "Execution done" crlf)
    (do-for-all-facts ((?pa pddl-action)) TRUE
       (printout green "action " ?pa:name " "
         ?pa:params " " ?pa:planned-start-time "|" ?pa:planned-duration
         " vs actual " ?pa:actual-start-time "|" ?pa:actual-duration crlf
       )
    )
    (assert (printed))
  )

PDDL Planning Model
-------------------

The planning model used in this tutorial is stored in:

.. code-block:: text

   cx_pddl_bringup
   └── pddl
       ├── domain.pddl
       └── problem.pddl


The **domain file** defines:

* available objects,
* predicates describing the world state,
* planning actions,
* action conditions,
* action effects.


The **problem file** defines:

* objects used in the scenario,
* initial state,
* desired goal state.


The tutorial uses a simple Blocks World domain where a robot rearranges
blocks into a desired configuration.

Actions are modeled as durative actions, allowing the temporal planner to
generate execution schedules.


Running the PDDL Agent
----------------------

The tutorial uses the generic launch file provided by
``cx_pddl_bringup``.

The launch file starts:

* the PDDL manager,
* the CLIPS manager,
* the configured PDDL CLIPS agent.


Run the example on a simple blocksworld domain using:

.. code-block:: bash

   ros2 launch cx_pddl_bringup cx_pddl_launch.py \
     pddl_plan_type:='TEMPORAL' \
     pddl_domain:='pddl/domain.pddl' \
     pddl_problem:='pddl/problem.pddl'


A warning about unavailable services may appear during startup. This is
expected because the CLIPS node can start before the PDDL manager has fully
initialized. Requests are retried automatically.

Also, additional warnings will indicate the successful override of the deftemplate definitions.

The cx_pddl_bringup package additional supplies the depots domain from the IPC:

.. code-block:: bash

   ros2 launch cx_pddl_bringup cx_pddl_launch.py \
     pddl_plan_type:='PARTIAL-ORDER' \
     pddl_domain:='pddl/depots_classical_domain.pddl' \
     pddl_problem:='pddl/depots_classical_problem.hddl'

   ros2 launch cx_pddl_bringup cx_pddl_launch.py \
     pddl_plan_type:='TEMPORAL' \
     pddl_domain:='pddl/depots_temporal_domain.pddl' \
     pddl_problem:='pddl/depots_temporal_problem.hddl'

Summary
-------

You now have a CLIPS-based PDDL agent capable of:

* Creating PDDL planning instances,
* Loading goals from PDDL problem definitions,
* Requesting plans from the PDDL manager,
* Executing classical, temporal, partial-order, and hierarchical plans,
* Monitoring action conditions,
* Applying action effects,
* Tracking planned and actual execution times,
* Inspecting STN plans.

This provides a flexible foundation for integrating symbolic planning,
reasoning, and execution monitoring into robotic systems using the |CX|
framework.
