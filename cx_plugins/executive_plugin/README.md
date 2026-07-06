<!-- AUTO-GENERATED via sphinx-build. Do not edit directly. -->

<a id="usage-executive-plugin"></a>

# Executive Plugin

Source code on [GitHub](https://github.com/carologistics/clips_executive/blob/master/cx_plugins/executive_plugin).

This plugin provides continuous and ad-hoc execution of CLIPS environments.
Additionally, it enables reasoning about the current ROS and system time.

## Configuration

<a id="refresh-rate"></a>
* **refresh_rate:**
  | Type   |   Default |
  |--------|-----------|
  | int    |        10 |

  Description
  : Target rate (in Hz) with which agendas are refreshed and run is called.
    This is done sequentially for the registered environments.

<a id="publish-on-refresh"></a>
* **publish_on_refresh:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | false     |

  Description
  : Whether to publish an empty message with each agenda refresh on the clips_executive/refresh_agenda topic.
    Mainly useful for debug purposes to measure the actual refresh rate.

<a id="assert-time"></a>
* **assert_time:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | true      |

  Description
  : Whether the latest ROS time as fact into the CLIPS environment on each iteration.
* **autostart:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | true      |

  Description
  : Whether to start periodic execution when loading the plugin.

## Features

The executive plugin periodically runs all managed CLIPS environments at the configured [refresh_rate](#refresh-rate).
Each tick performs the following steps:

1. Optionally asserts the current time as `(time (now))` into the environment.
2. Refreshes all agendas.
3. If a [focus_stack](../getting_started/configuration.md#focus-stack) is configured for an environment, iterates through the stack in order, running each module’s agenda (respecting a possibly set [rule_limit](../getting_started/configuration.md#rule-limit)) before moving to the next.
   Otherwise, runs the `MAIN` module’s agenda.
4. Repeats steps 2-3 until no rules fire, ensuring cross-module interactions are fully resolved.
5. Optionally publishes the total number of rules fired on a topic.

Execution can be paused and resumed at any time via services,
or a single tick can be triggered manually, regardless of the current pause state.

CLIPS commands can be sent directly to a managed environment via dedicated `eval` and `build` services.
`eval` evaluates an expression using CLIPS `Eval`, while `build` defines a new construct using CLIPS `Build`.
If no environment is specified, the command is executed in the first managed environment.

#### NOTE
When using a focus stack, the current module is reset to `MAIN` after each module’s agenda is run.
This ensures that time assertion and other framework operations always operate in a predictable context.

#### NOTE
`Eval` is intended for invoking arbirary commands (e.g. `(assert (foo))`, `(facts)`),
while `Build` is intended for defining constructs (e.g. `(defrule ...)`, `(deftemplate ...)`).

### ROS Interfaces

All interaces are prefixed by tha name of the CX node, followed by the name of the executive plugin, e.g., `clips_manager/executive`.

-  */ns/node/plugin/refresh_agenda*: Topic where a mesage is published after each tick, indicating the number of rules that fired (requires [publish_on_refresh](#publish-on-refresh)).
-  */ns/node/plugin/pause*: Service for stopping periodic ticks according to the configured [refresh_rate](#refresh-rate).
-  */ns/node/plugin/resume*: Service that for starting the periodic ticks.
-  */ns/node/plugin/tick_once*: Serivce for ticking once.
-  */ns/node/plugin/eval*: Service for processing a string as an `eval` command in a given environment.
-  */ns/node/plugin/build*: Service for processing a string as an `build` command in a given environment.

### Facts

If [assert_time](#assert-time) is set to `true`, it asserts the ordered fact time with the current ROS time as float.

```clips
(time ?ros-time-float)
```

### Functions

This plugin adds deffunctions to retrieve the current time.

```clips
(bind ?ros-time (now))         ; returns a FLOAT holding get_clock()->now().seconds()
(bind ?sys-time (now-systime)) ; returns a FLOAT of system time
```

### Other

Lastly, the `time` fact is unwatched after the first assertion.

## Usage Example

A minimal working example is provided by the [cx_bringup](https://carologistics.github.io/clips_executive/cx_bringup) package. Run it via:

```bash
ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/executive.yaml
```

It prints the current ROS and system time in each iteration and compares the time at the start of the iteration with the time at the time the rule is fired.

The executive can be stopped by calling the `pause` service:

```bash
ros2 service call /executive/pause std_srvs/srv/Trigger {}
```

Individual ticks (a run of the inference engine until termination) can be invoked via the `tick_once` service:

```bash
ros2 service call /executive/tick_once std_srvs/srv/Trigger {}
```

The `eval` and `build` functions allow for ad-hoc interaction with an environment:

```bash
ros2 service call /executive/build cx_msgs/srv/ClipsCommand \
  '{env_name: "cx_executive", command: "(deftemplate example (slot message (type STRING)))"}'
ros2 service call /executive/eval cx_msgs/srv/ClipsCommand \
  '{env_name: "cx_executive", command: "(assert (example (message \"hello\")))"}'
```

Finally, the `resume` service is used in order to resume automatic execution:

```bash
ros2 service call /executive/resume std_srvs/srv/Trigger {}
```

### Configuration

File [cx_bringup/params/plugin_examples/executive.yaml](https://github.com/carologistics/clips_executive/blob/master/cx_bringup/params/plugin_examples/executive.yaml).

```yaml
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
```

### Code

File [cx_bringup/clips/plugin_examples/executive.clp](https://github.com/carologistics/clips_executive/blob/master/cx_bringup/clips/plugin_examples/executive.clp).

```clips
(defrule print-time
  (time ?now)
  =>
  (printout info "time between agenda refresh and rule fire: " (- (now) ?now) crlf)
  (printout info "ROS time: " (now) crlf)
  (printout info "sys time: " (now-systime) crlf)
)
```
