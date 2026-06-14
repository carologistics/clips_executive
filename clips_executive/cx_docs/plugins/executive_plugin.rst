.. _usage_executive_plugin:

Executive Plugin
################

Source code on :source-master:`GitHub <cx_plugins/executive_plugin>`.

.. admonition:: Plugin Class

  cx::ExecutivePlugin

This plugin provides continuous and ad-hoc execution of CLIPS environments.
Additionally, it enables reasoning about the current ROS and system time.

Configuration
*************

.. _refresh_rate:

:`refresh_rate`:

  ============ =======
  Type         Default
  ------------ -------
  int          10
  ============ =======

  Description
    Target rate (in Hz) with which agendas are refreshed and run is called.
    This is done sequentially for the registered environments.

:`publish_on_refresh`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    Whether to publish an empty message with each agenda refresh on the `clips_executive/refresh_agenda` topic.
    Mainly useful for debug purposes to measure the actual refresh rate.

.. _assert_time:

:`assert_time`:

  ============ =======
  Type         Default
  ------------ -------
  bool         true
  ============ =======

  Description
    Whether the latest ROS time as fact into the CLIPS environment on each iteration.

:`autostart`:

  ============ =======
  Type         Default
  ------------ -------
  bool         true
  ============ =======

  Description
    Whether to start periodic execution when loading the plugin.

Features
********

The executive plugin periodically runs all managed CLIPS environments at the configured :ref:`refresh_rate`.
Each tick performs the following steps:

1. Optionally asserts the current time as ``(time (now))`` into the environment.
2. Refreshes all agendas.
3. If a :ref:`focus_stack <focus_stack>` is configured for an environment, iterates through the stack in order, running each module's agenda (respecting a possibly set :ref:`rule_limit <rule_limit>`) before moving to the next.
   Otherwise, runs the ``MAIN`` module's agenda.
4. Repeats steps 2-3 until no rules fire, ensuring cross-module interactions are fully resolved.
5. Optionally publishes the total number of rules fired on the :ref:`refresh_agenda_topic` topic.

Execution can be paused and resumed at any time via the :ref:`pause_service` and :ref:`resume_service` services,
or a single tick can be triggered manually via the :ref:`tick_once_service` service, regardless of the current pause state.

CLIPS commands can be sent directly to a managed environment via dedicated ``eval`` and ``build`` services.
``eval`` evaluates an expression using CLIPS ``Eval``, while ``build`` defines a new construct using CLIPS ``Build``.
If no environment is specified, the command is executed in the first managed environment.

.. note::

  When using a focus stack, the current module is reset to ``MAIN`` after each module's agenda is run.
  This ensures that time assertion and other framework operations always operate in a predictable context.

.. note::

  ``Eval`` is intended for invoking arbirary commands (e.g. ``(assert (foo))``, ``(facts)``),
  while ``Build`` is intended for defining constructs (e.g. ``(defrule ...)``, ``(deftemplate ...)``).

Facts
~~~~~

If :ref:`assert_time` is set to ``true``, it asserts the ordered fact `time` with the current ROS time as float.

.. code-block:: lisp

  (time ?ros-time-float)

Functions
~~~~~~~~~

This plugin adds deffunctions to retrieve the current time.

.. code-block:: lisp

  (bind ?ros-time (now))         ; returns a FLOAT holding get_clock()->now().seconds()
  (bind ?sys-time (now-systime)) ; returns a FLOAT of system time


Other
~~~~~

Lastly, the ``time`` fact is unwatched after the first assertion.


Usage Example
*************

A minimal working example is provided by the :docsite:`cx_bringup` package. Run it via:

.. code-block:: bash

    ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/executive.yaml

It prints the current ROS and system time in each iteration and compares the time at the start of the iteration with the time at the time the rule is fired.

The executive can be stopped by calling the ``pause`` service:

.. code-block:: bash

    ros2 service call /executive/pause std_srvs/srv/Trigger {}

Individual ticks (a run of the inference engine until termination) can be invoked via the ``tick_once`` service:

.. code-block:: bash

    ros2 service call /executive/tick_once std_srvs/srv/Trigger {}

The ``eval`` and ``build`` functions allow for ad-hoc interaction with an environment:

.. code-block:: bash

    ros2 service call /executive/build cx_msgs/srv/ClipsCommand \
      '{env_name: "cx_executive", command: "(deftemplate example (slot message (type STRING)))"}'
    ros2 service call /executive/eval cx_msgs/srv/ClipsCommand \
      '{env_name: "cx_executive", command: "(assert (example (message \"hello\")))"}'

Finally, the ``resume`` service is used in order to resume automatic execution:

.. code-block:: bash

    ros2 service call /executive/resume std_srvs/srv/Trigger {}



Configuration
~~~~~~~~~~~~~

File :source-master:`cx_bringup/params/plugin_examples/executive.yaml`.

.. code-block:: yaml

  clips_manager:
    ros__parameters:
      environments: ["cx_executive"]
      cx_executive:
        plugins: ["executive", "files"]
        log_clips_to_file: true
        watch: ["facts", "rules"]

      executive:
        plugin: "cx::ExecutivePlugin"
        publish_on_refresh: false
        assert_time: true
        refresh_rate: 10
      files:
        plugin: "cx::FileLoadPlugin"
        pkg_share_dirs: ["cx_bringup"]
        load: [
          "clips/plugin_examples/executive.clp"]

Code
~~~~

File :source-master:`cx_bringup/clips/plugin_examples/executive.clp`.

.. code-block:: lisp

  (defrule print-time
    (time ?now)
    =>
    (printout info "time between agenda refresh and rule fire: " (- (now) ?now) crlf)
    (printout info "ROS time: " (now) crlf)
    (printout info "sys time: " (now-systime) crlf)
  )
