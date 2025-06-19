.. _plugins:

CLIPS Plugins
=============

The |CX| can dynamically load `pluginlib`_ plugins, which act as an interface for users to apply the CLIPS C++ API (See the |APG|), e.g., to inject user-defined functions into CLIPS environments.

Plugins are specializations of of the :docsite:`cx_plugin` base class and are handled as follows:
 - Each plugin is initialized exactly once before it is loaded into environments by calling it's ``initialize()`` function.
 - When an environment is loaded, all specified plugins are **loaded in order** of the respective plugins parameter.
 - Whenever an environment needs to load a plugin, it's ``clips_env_init()`` function is called once. Loading the same plugin again before unloading it first, results in an error (and the function is not called again).
 - Whenever an environment needs to unload a plugin, it's ``clips_env_destroyed()`` is called once. Unloading the same plugin again before loading it first, results in an error (and the function is not called again).
 - Before an environment is destroyed, all plugins are **unloaded in reverse order** of loading.
 - On destruction of a plugin, the `finalize()` function is called exactly once.
 - Before a plugin is destroyed, it is unloaded from all environments.

See also the tutorial on :ref:`Writing a Plugin <writing_a_plugin>` to learn how to write your own plugins.

The |CX| provides several plugins out-of-the-box that are described here.

In the following, the virtual functions and their purpose are described.

initialize()
++++++++++++

This function is called exactly once when a plugin is loaded, before it actually provides it's features to CLIPS environments.

Typical uses include
1) Reading of parameters from specified in the parent node.
2) Initialization of environment-agnostic class members.

clips_env_init()
++++++++++++++++

Called once for every environment the plugin is loaded into.

Typically this is used to inject user-defined functions, define templates etc.

It should return `true`, if the initialization of the environment was successful. If it returns `false`, then the plugin manager will call `clips_env_destroyed` to allow proper cleanup.

Environment Reset
-----------------

However, be aware that each environment is reset afterwards on startup.

This in particular means that all asserted facts and instances are deleted and it makes no sense to directly assert facts in this function.

If your plugin should provide initial facts, it should therefore use `deffacts` instead, which would assert the facts on reset.

Environment Context and Multithreading
--------------------------------------

Each environment also holds an instance of the `CLIPSEnvContext` class from the **cx_utils** package that can be retrieved via static functions:

.. code-block:: c++

    cx::CLIPSEnvContext::get_context(clips::Environment *env)
    cx::CLIPSEnvContext::get_context(std::shared_ptr<clips::Environment> env)

This instance contains the name and the environment as well as a mutex to guard the environment.

**Operations on CLIPS environments are not thread-safe**, hence each environment interaction needs to be guarded by this mutex.
This is mainly relevant for plugins handling asynchronous operations.
Directly accessing the environment in **clips_env_init** or **clips_env_destroyed** is safe, because the plugin manager already guards the environments with the mutex (do **not** attempt to lock the mutex in this context or it will block).

Similarly, if a plugin provides a C++ function to a CLIPS environment, it's body will be scoped through whatever context that invoked the function (e.g., the CLIPS environment manager through invoking the inference engine via ``(run)``).

clips_env_destroyed()
+++++++++++++++++++++

Called once for every environment that needs to unload a previously loaded plugin's feature.
Also is called when a `clips_env_init` call returns `false`.

Typically this is used to undefine user-defined functions, templates, etc.

finalize()
++++++++++

This function is called exactly once when a plugin is finally unloaded again, hence all resources should be freed for a graceful destruction of the object.

Pitfalls and Considerations
+++++++++++++++++++++++++++

Below are some lessons learned when developingthe core plugins of the |CX|. They might be a useful read for some.

Injected functions should not be blocking
-----------------------------------------

In order to ensure running a responsive CLIPS application, make sure the injected functions are executing fast. Long-lasting operations should rather be dispatched by a function and then asynchronously handled once they finish. This allows the inference engine to continue operating while heavy computations or time-consuming sub-routines are processed.

Keep your locking scopes as tight as possible
---------------------------------------------

When writing complex plugins with asynchronous operations, be sure to scope your guarded regions well.

A common pitfall may occur when plugins also need to guard data structures from concurrent access using some mutex, while also handling CLIPS access.

Consider this example from **cx_ros_msgs_plugin** which allows interactions with ROS topics:

#. The asynchronous subscription callbacks adds messages and meta-data to an unordered map, which needs to be guarded by a mutex `map_mtx_` as multiple write operations could occur at the same time when multi-threaded executors and re-entrant callback groups are used. Additionally, the messages are asserted as facts (holding a reference to the message) in the callback.
#. The **ros-msgs-get-field** UDF allows to retrieve fields of messages. As fields may contain messages, this again might need to store meta-data, hence it also needs to lock `map_mtx_`.

A bad implementation using a scoped lock for the entire scope of the callback and the entire scope of **ros-msgs-get-field** could cause a deadlock if the ros-msgs-get-field function is called on the left-hand side of a rule, e.g., like this:

.. code-block:: lisp

    (defrule deadlock-example
      (ros-msgs-subscription (topic ?sub))
      ?msg-f <- (ros-msgs-message (topic ?sub) (msg-ptr ?inc-msg))
      (test (and (= 0 (ros-msgs-get-field ?inc-msg "velocity"))))
    ...

The assertion of the fact in the callback triggers the conditional check in the rule which therefore tries to lock `map_mtx_` blocked by the callback function itself. Hence, make sure to keep the scopes of any mutex as tight as possible!

Be aware of hidden locks
------------------------

It can be tricky to interface between ROS callbacks and CLIPS environments, especially if locks are guarding said callbacks that are not visible to the end user.
As CLIPS environment access must also guarded by locks (as access is not thread-safe), this can easily create deadlocks in situations, where other functions can be called from clips that also try to acquire the lock held by a callback.

Example: The feedback callback for action clients is guarded by a mutex that is also used by client goal handles to access members in a thread-safe manner (such as get_status()).

A CLIPS rule that calls ClientGoalHandle::get_status() will therefore attempt to lock such a mutex while the clips lock is being held by the thread running the clips environment.
If a callback is received right before, then the clips environment will stall as the function call is stuck (mutex is held by the callback function), while the callback function is stuck because it tries to acquire the lock for the clips environment (because it wants to pass the callback content to the clips environment).

In these cases special care must be taken, e.g., by deferring CLIPS access out of scope of the mutexes guarding the callbacks.

Available Plugins
+++++++++++++++++

.. toctree::
   :maxdepth: 2

   cx_ament_index.rst
   executive_plugin.rst
   config_plugin.rst
   file_load_plugin.rst
   protobuf_plugin.rst
   ros_msgs_plugin.rst
   tf2_pose_tracker_plugin.rst
