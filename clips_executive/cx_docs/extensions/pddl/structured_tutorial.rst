.. _structured_pddl_agent:

Tutorial: Structured PDDL Agent
===============================

**Goal:** Use CLIPS to plan and execute structured PDDL actions with monitoring and timing.

**Tutorial level:** Advanced

**Time:** 45–60 minutes

.. contents:: Contents
   :depth: 2
   :local:


Overview
--------

This tutorial extends the basic PDDL Manager integration by introducing structured action management within CLIPS.
The structured agent not only requests a plan but also **executes and monitors actions** step by step, checking preconditions and tracking execution times.

You will learn how to:

1. Initialize and manage PDDL-related ROS services and actions,
2. Define a structured `pddl-action` fact template,
3. Plan using the PDDL Manager,
4. Execute actions while checking feasibility conditions,
5. Monitor execution progress and print planned vs. actual times.


Prerequisites
-------------

Before starting, make sure you’ve completed the :doc:`Interfacing with a PDDL Manager <pddl_interface_tutorial>` tutorial.

You need a workspace with:

* The ``cx_pddl_bringup`` package,
* The ``cx_pddl_clips`` package,
* A working PDDL Manager instance running via:

  .. code-block:: bash

     ros2 launch cx_pddl_bringup pddl_manager_launch.py


Configuration
-------------

The CLIPS agent and its plugins are configured via ``structured_pddl_agent.yaml``.

.. code-block:: yaml

  /**:
    ros__parameters:
      autostart_node: true
      environments: ["structured_pddl_agent"]

      structured_pddl_agent:
        plugins: ["executive", "ros_msgs",
                  "ament_index",
                  "plan_temporal_action",
                  "timed_plan_action_msg",
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
        pkg_share_dirs: ["cx_pddl_clips", "cx_pddl_bringup"]
        batch: [
          "clips/deftemplates.clp",
          "clips/deftemplate-overrides.clp",
          "clips/pddl-no-deftemplates.clp"
        ]

      files:
        plugin: "cx::FileLoadPlugin"
        pkg_share_dirs: ["cx_pddl_bringup"]
        load: ["clips/structured_agent.clp"]

      plan_temporal_action:
        plugin: "cx::CXCxPddlMsgsPlanTemporalPlugin"

      timed_plan_action_msg:
        plugin: "cx::CXCxPddlMsgsTimedPlanActionPlugin"

The ``pddl_files`` plugin loads reusable templates and overrides for PDDL interaction,
while ``files`` loads the main CLIPS rule base implementing the structured agent.


Defining the PDDL Action Template
---------------------------------

The structured agent overrides the default ``pddl-action`` template to include
execution state, planned timing, and runtime tracking.

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
    (slot actual-start-time (type FLOAT))
    (slot actual-duration (type FLOAT))
    (slot state (type SYMBOL) (allowed-values IDLE SELECTED EXECUTING DONE))
  )

Each ``pddl-action`` fact represents one step in the plan, including its
desired start time, duration, and execution state.


Initializing the PDDL Manager
-----------------------------

The rule ``structured-agent-pddl-init`` creates all required service clients
and initializes the temporal plan action client.

.. code-block:: lisp

  (defrule structured-agent-pddl-init
    " Initiate the service clients for the pddl manager "
    (not (pddl-services-loaded))
    =>
    (bind ?services (create$
        add_fluents AddFluents
        add_pddl_instance AddPddlInstance
        get_fluents GetFluents
        set_goals SetGoals
        check_action_condition CheckActionCondition
        create_goal_instance CreateGoalInstance
        get_action_effects GetActionEffects
        rm_fluents RemoveFluents
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
    (cx-pddl-msgs-plan-temporal-create-client (str-cat "/pddl_manager" "/temp_plan"))
    (assert (pddl-services-loaded))
  )


Uploading the PDDL Instance
---------------------------

Once initialized, the rule ``structured-agent-pddl-add-instance`` sets up the
domain and problem to be planned for and defines the goal fluents.

.. code-block:: lisp

  (defrule structured-agent-pddl-add-instance
    " Setup PDDL instance with an active goal to plan for "
    (not (pddl-loaded))
    (pddl-services-loaded)
    =>
    (bind ?share-dir (ament-index-get-package-share-directory "cx_pddl_bringup"))
    (assert
      (pddl-manager (node "/pddl_manager"))
      (pddl-instance
        (name test)
        (domain "domain.pddl")
        (problem "problem.pddl")
        (directory (str-cat ?share-dir "/pddl"))
      )
      (pddl-get-fluents (instance test))
      (pddl-create-goal-instance (instance test) (goal active-goal))
      (pddl-goal-fluent (instance test) (goal active-goal) (name on) (params a b))
      (pddl-goal-fluent (instance test) (goal active-goal) (name on) (params b c))
      (pddl-set-goals (instance test) (goal active-goal))
    )
  )


Planning
--------

The rule ``structured-agent-pddl-plan`` triggers the temporal planner once
the ``set-goals`` request is completed.

.. code-block:: lisp

  (defrule structured-agent-pddl-plan
    " Start the planner once the set-goals request is done "
    (cx-pddl-msgs-plan-temporal-client (server ?server&:(eq ?server "/pddl_manager/temp_plan")))
    (not (planned))
    (pddl-set-goals (state DONE))
    =>
    (printout green "Start planning" crlf)
    (bind ?goal (cx-pddl-msgs-plan-temporal-goal-create))
    (cx-pddl-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" "test")
    (cx-pddl-msgs-plan-temporal-goal-set-field ?goal "goal_instance" "active-goal")
    (cx-pddl-msgs-plan-temporal-send-goal ?goal ?server)
    (assert (planned))
  )


Processing the Plan Result
--------------------------

Once the PDDL Manager produces a plan, it is parsed into individual ``pddl-action`` facts.

.. code-block:: lisp

  (defrule structured-agent-pddl-plan-result
    " Retrieve the resulting plan "
    ?wr-f <- (cx-pddl-msgs-plan-temporal-wrapped-result
      (server "/pddl_manager/temp_plan") (code SUCCEEDED) (result-ptr ?res-ptr)
    )
    =>
    (bind ?plan-found (cx-pddl-msgs-plan-temporal-result-get-field ?res-ptr "success"))
    (printout green "planning done" crlf)
    (bind ?id 0)
    (if ?plan-found then
      (bind ?plan (cx-pddl-msgs-plan-temporal-result-get-field ?res-ptr "actions"))
      (foreach ?action ?plan
        (bind ?name (sym-cat (cx-pddl-msgs-timed-plan-action-get-field ?action "name")))
        (bind ?args (cx-pddl-msgs-timed-plan-action-get-field ?action "args"))
        (bind ?ps-time (cx-pddl-msgs-timed-plan-action-get-field ?action "start_time"))
        (bind ?p-duration (cx-pddl-msgs-timed-plan-action-get-field ?action "duration"))
        (assert (pddl-action
          (id ?id)
          (instance test)
          (name ?name)
          (params ?args)
          (planned-start-time ?ps-time)
          (planned-duration ?p-duration))
        )
        (printout t ?ps-time "(" ?p-duration ")   " ?name ?args crlf)
        (bind ?id (+ ?id 1))
      )
    else
      (printout red "plan not found!" crlf)
    )
    (cx-pddl-msgs-plan-temporal-result-destroy ?res-ptr)
  )


Action Execution Loop
---------------------

The structured agent continuously selects, executes, and validates actions.

**Selecting the first action:**

.. code-block:: lisp

  (defrule structured-agent-select-action
    " Start executing the first action of the resulting plan "
    (not (plan-start ?t))
    ?pa <- (pddl-action (planned-start-time ?t) (state IDLE))
    (not (pddl-action (planned-start-time ?ot&:(< ?ot ?t))))
    =>
    (modify ?pa (state SELECTED))
    (assert (plan-start (now)))
  )

**Checking preconditions before execution:**

.. code-block:: lisp

  (defrule structured-agent-check-action
    " Before executing an action check the condition to make sure it is feasible "
    (pddl-action (id ?id) (state SELECTED) (name ?name) (params $?params))
    (not (pddl-action-condition (action ?id)))
    =>
    (assert (pddl-action-condition (instance test) (action ?id)))
  )

**Executing and completing actions:**

.. code-block:: lisp

  (defrule structured-agent-executable-action
    (plan-start ?t)
    (pddl-action-condition (action ?action-id) (state CONDITION-SAT))
    ?pa <- (pddl-action (id ?action-id) (name ?name) (params $?params) (state SELECTED))
    =>
    (modify ?pa (state EXECUTING) (actual-start-time (- (now) ?t)))
  )

  (defrule structured-agent-execution-done
    (time ?now)
    (plan-start ?t)
    ?pa <- (pddl-action (id ?id) (state EXECUTING) (planned-duration ?d)
      (actual-start-time ?s&:(< (+ ?s ?d ?t) ?now)))
    =>
    (modify ?pa (state DONE) (actual-duration (- (now) (+ ?s ?t))))
    (assert (pddl-action-get-effect (action ?id) (apply TRUE)))
  )

**Selecting the next action:**

.. code-block:: lisp

  (defrule structured-agent-select-next-action
    (not (pddl-action (state EXECUTING|SELECTED)))
    (not (pddl-action-get-effect (state ~DONE)))
    (not (pddl-fluent-change))
    ?pa <- (pddl-action (planned-start-time ?t) (state IDLE))
    (not (pddl-action (state IDLE) (planned-start-time ?ot&:(< ?ot ?t))))
    =>
    (modify ?pa (state SELECTED))
  )


Finalizing Execution
--------------------

Once all actions have completed, CLIPS prints a timing summary.

.. code-block:: lisp

  (defrule structured-agent-print-exec-times
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


Running the Structured Agent
----------------------------

As usual, build and source your workspace:

.. code-block:: bash

   cd ~/ros2/cx_tutorial_ws/
   colcon build --symlink-install
   source install/setup.bash

Then launch the agent:

.. code-block:: bash

   ros2 launch cx_bringup cx_launch.py manager_config:=structured_pddl_agent.yaml package:=cx_pddl_bringup

You should see the CLIPS output showing initialization, goal setup, planning,
and the full action execution process with timing data.


Summary
-------

You now have a **fully structured CLIPS PDDL agent** capable of:

* Setting up planning problems,
* Requesting and parsing plans,
* Executing and monitoring actions,
* Comparing planned vs. actual execution durations.

This provides a powerful base for integrating reasoning and execution control
in robotic systems using the |CX| framework.
