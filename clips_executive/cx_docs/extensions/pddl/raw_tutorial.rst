.. _raw_pddl_agent:

Tutorial: Interfacing with a PDDL Manager directly
==================================================

**Goal:** Use CLIPS to load a PDDL problem, set goals, and request a plan from the PDDL Manager.

**Tutorial level:** Intermediate

**Time:** 30 minutes

.. contents:: Contents
   :depth: 2
   :local:


Background
----------

The |CX| (CLIPS Executive) can not only monitor ROS topics, but also interact with ROS services and actions.
This tutorial demonstrates how CLIPS can interface with a PDDL Manager node to set up planning problems and request solutions.

The process includes:

1. Initializing service clients for PDDL operations,
2. Uploading a domain and problem definition,
3. Setting goals,
4. Requesting a plan,
5. Receiving and printing the resulting plan.


Prerequisites
-------------

This tutorial assumes completion of the :doc:`Continuous Monitoring <continuous_monitoring>` tutorial and a workspace containing the
``cx_tutorial_agents`` and ``cx_pddl_bringup`` packages.

Additionally, you need to have the PDDL Manager available.
If not already installed, you can obtain it by cloning the ``ros2-cx_pddl`` repository:

.. code-block:: bash

   cd ~/ros2/cx_tutorial_ws/src
   git clone https://github.com/fawkesrobotics/ros2-cx_pddl.git
   cd ..
   colcon build --symlink-install
   source install/setup.bash


Running the PDDL Manager
^^^^^^^^^^^^^^^^^^^^^^^^

Before starting CLIPS, launch the PDDL Manager:

.. code-block:: bash

   ros2 launch cx_pddl_bringup pddl_manager_launch.py

This node provides the necessary services and actions that CLIPS will interact with.


Interfacing with the PDDL Manager
---------------------------------

The objective is to let CLIPS automatically:

1. Connect to all PDDL-related ROS services,
2. Load a domain and problem from the ``cx_pddl_bringup`` package,
3. Retrieve the available fluents,
4. Set new goals,
5. Request a plan using the temporal planning action.

The configuration is handled through both a YAML file and a CLIPS rule base.


1. Obtaining the Files
^^^^^^^^^^^^^^^^^^^^^^

Navigate to the ``params`` and ``clips`` directories of the ``cx_pddl_bringup`` package and download the tutorial configuration and CLIPS script:

.. code-block:: bash

   cd ~/ros2/cx_tutorial_ws/src/cx_pddl_bringup/params
   wget -O raw_pddl_agent.yaml https://raw.githubusercontent.com/fawkesrobotics/ros2-clips-executive/master/tutorials/cx_pddl_bringup/params/raw_pddl_agent.yaml

   cd ~/ros2/cx_tutorial_ws/src/cx_pddl_bringup/clips
   wget -O raw_agent.clp https://raw.githubusercontent.com/fawkesrobotics/ros2-clips-executive/master/tutorials/cx_pddl_bringup/clips/raw_agent.clp

This adds:

* ``params/raw_pddl_agent.yaml`` – configuration file for the CLIPS environment.
* ``clips/raw_agent.clp`` – CLIPS rule base implementing the PDDL workflow.


2. Configuring the |CX|
^^^^^^^^^^^^^^^^^^^^^^^

The environment configuration in ``raw_pddl_agent.yaml`` loads multiple plugins required for ROS interaction and PDDL message handling.

.. code-block:: yaml

  /**:
    ros__parameters:
      autostart_node: true
      environments: ["raw_pddl_agent"]

      raw_pddl_agent:
        plugins: ["executive", "ros_msgs",
                  "ament_index",
                  "plan_temporal_action",
                  "timed_plan_action_msg",
                  "files"]
        log_clips_to_file: true
        watch: ["facts", "rules"]

      executive:
        plugin: "cx::ExecutivePlugin"

      ros_msgs:
        plugin: "cx::RosMsgsPlugin"

      ament_index:
        plugin: "cx::AmentIndexPlugin"

      plan_temporal_action:
        plugin: "cx::CXCxPddlMsgsPlanTemporalPlugin"

      timed_plan_action_msg:
        plugin: "cx::CXCxPddlMsgsTimedPlanActionPlugin"

      files:
        plugin: "cx::FileLoadPlugin"
        pkg_share_dirs: ["cx_pddl_bringup"]
        load: ["clips/raw_agent.clp"]

The following components are important:

* ``ExecutivePlugin`` continuously runs the CLIPS inference engine.
* ``RosMsgsPlugin`` enables communication with ROS services.
* ``AmentIndexPlugin`` provides access to package share directories.
* ``CXCxPddlMsgsPlanTemporalPlugin`` and ``CXCxPddlMsgsTimedPlanActionPlugin`` define the interfaces for PDDL planning actions.
* ``FileLoadPlugin`` loads the ``raw_agent.clp`` rule base.


3. Initializing the PDDL Services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The first rule in ``raw_agent.clp`` creates clients for all required services provided by the PDDL Manager.

.. code-block:: lisp

  (defrule pddl-init
      (not (pddl-services-loaded))
      =>
      (bind ?services (create$
          add_fluents AddFluents
          add_pddl_instance AddPddlInstance
          get_fluents GetFluents
          set_goals SetGoals
          create_goal_instance CreateGoalInstance
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

Once all clients are available, an additional rule creates the client for the planning action:

.. code-block:: lisp

  (defrule pddl-init-plan-client
      (not (pddl-planning-client-created))
      (pddl-services-loaded)
      =>
      (cx-pddl-msgs-plan-temporal-create-client (str-cat "/pddl_manager" "/temp_plan"))
      (assert (pddl-planning-client-created))
  )


4. Uploading a PDDL Problem
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once all services are available, the rule ``pddl-add-instance`` sends a request to load a PDDL domain and problem:

.. code-block:: lisp

  (defrule pddl-add-instance
      (ros-msgs-client (service ?service&:(eq ?service (str-cat "/pddl_manager" "/add_pddl_instance"))) (type ?type))
      (not (pddl-loaded))
      (pddl-planning-client-created)
      (time ?)
      =>
      (bind ?new-req (ros-msgs-create-request ?type))
      (ros-msgs-set-field ?new-req "name" "test")
      (bind ?share-dir (ament-index-get-package-share-directory "cx_pddl_bringup"))
      (ros-msgs-set-field ?new-req "directory" (str-cat ?share-dir "/pddl"))
      (ros-msgs-set-field ?new-req "domain_file" "domain.pddl")
      (ros-msgs-set-field ?new-req "problem_file" "problem.pddl")
      (bind ?id (ros-msgs-async-send-request ?new-req ?service))
      (if ?id then
        (assert (pddl-loaded))
      )
      (ros-msgs-destroy-message ?new-req)
  )

This command sends the ``domain.pddl`` and ``problem.pddl`` files from the ``cx_pddl_bringup/pddl`` directory to the PDDL Manager.


5. Retrieving Fluents and Setting Goals
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

After successfully uploading the PDDL instance, CLIPS retrieves the current fluents and defines new goals.

The rule ``pddl-get-fluents`` requests the fluents:

.. code-block:: lisp

  (defrule pddl-get-fluents
      (ros-msgs-client (service ?s&:(eq ?s (str-cat "/pddl_manager" "/get_fluents"))) (type ?type))
      (pddl-loaded-confirmed)
      (not (pddl-fluents-requested))
      =>
      (bind ?new-req (ros-msgs-create-request ?type))
      (ros-msgs-set-field ?new-req "pddl_instance" "test")
      (bind ?id (ros-msgs-async-send-request ?new-req ?s))
      (if ?id then
          (printout t "Requested Fluents" crlf)
          (assert (pddl-fluents-requested))
      )
      (ros-msgs-destroy-message ?new-req)
  )

Once fluents are available, CLIPS sets new goals using ``pddl-set-goal``.
Here, two goals are created — ``(on a b)`` and ``(on b c)``.

.. code-block:: lisp

  (defrule pddl-set-goal
      (pddl-fluents-requested)
      (not (pddl-goals-set))
      (ros-msgs-client (service ?s&:(eq ?s (str-cat "/pddl_manager" "/set_goals"))) (type ?type))
      (pddl-goal-instance-set)
      =>
      (bind ?new-req (ros-msgs-create-request ?type))
      (bind ?fluent-msg (ros-msgs-create-message "cx_pddl_msgs/msg/Fluent"))
      ...
      (ros-msgs-set-field ?new-req "goal_instance" "active-goal")
      (bind ?id (ros-msgs-async-send-request ?new-req ?s))
      (if ?id then
          (printout t "Requested to set goals" crlf)
      )
      (ros-msgs-destroy-message ?new-req)
  )


6. Planning
^^^^^^^^^^^

After setting up the goal, CLIPS triggers the planning action:

.. code-block:: lisp

  (defrule pddl-plan
      (cx-pddl-msgs-plan-temporal-client (server ?server&:(eq ?server "/pddl_manager/temp_plan")))
      (not (planned))
      (pddl-goals-set)
      =>
      (printout green "Start planning" crlf)
      (bind ?goal (cx-pddl-msgs-plan-temporal-goal-create))
      (cx-pddl-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" "test")
      (cx-pddl-msgs-plan-temporal-goal-set-field ?goal "goal_instance" "active-goal")
      (cx-pddl-msgs-plan-temporal-send-goal ?goal ?server)
      (assert (planned))
  )

Upon receiving the result, the rule ``pddl-plan-result`` prints out the actions of the generated plan.

.. code-block:: lisp

  (defrule pddl-plan-result
      ?wr-f <- (cx-pddl-msgs-plan-temporal-wrapped-result (server "/pddl_manager/temp_plan") (code SUCCEEDED) (result-ptr ?res-ptr))
      =>
      (bind ?plan-found (cx-pddl-msgs-plan-temporal-result-get-field ?res-ptr "success"))
      (if ?plan-found then
          (bind ?plan (cx-pddl-msgs-plan-temporal-result-get-field ?res-ptr "actions"))
          (foreach ?action ?plan
            (bind ?name (sym-cat (cx-pddl-msgs-timed-plan-action-get-field ?action "name")))
            (bind ?args (cx-pddl-msgs-timed-plan-action-get-field ?action "args"))
            (bind ?ps-time (cx-pddl-msgs-timed-plan-action-get-field ?action "start_time"))
            (bind ?p-duration (cx-pddl-msgs-timed-plan-action-get-field ?action "duration"))
            (printout t ?ps-time "(" ?p-duration ")   " ?name ?args crlf)
          )
      else
          (printout red "plan not found!" crlf)
      )
  )


7. Build and Run
^^^^^^^^^^^^^^^^

As before, build the workspace and source it:

.. code-block:: bash

   cd ~/ros2/cx_tutorial_ws/
   colcon build --symlink-install
   source install/setup.bash

Then, start the PDDL agent:

.. code-block:: bash

   ros2 launch cx_bringup cx_launch.py manager_config:=raw_pddl_agent.yaml package:=cx_pddl_bringup

The CLIPS output should show the initialization of services, loading of the PDDL files, goal creation, and the resulting plan.

Example output:

.. code-block:: bash

   [info] Opening client for /pddl_manager/add_pddl_instance
   [info] PDDL instance added
   [info] Requested Fluents
   [info] Goals set successfully
   [info] Start planning
   [info] planning done
   [info] 0.0(1.0)   move a b
   [info] 1.0(1.0)   move b c


Summary
-------

You created a CLIPS agent that automatically interacts with a PDDL Manager using ROS services and actions.
The environment established service clients, uploaded a PDDL problem, set goals, and requested a temporal plan — all within the CLIPS reasoning loop.


Next Steps
----------

You can now extend this example to:

* Dynamically select PDDL instances and goals,
* React to planning outcomes within CLIPS,
* Integrate the resulting plan execution with other ROS 2 nodes.

For further information, refer to the :docsite:`CXCxPddlMsgs plugins <clips_executive/plugins>` documentation.
