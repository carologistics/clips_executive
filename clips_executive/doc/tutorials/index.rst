.. _tutorials:
Tutorials
=========

.. todo::

   Add tutorials here

Controlling a Turtle via CLIPS
++++++++++++++++++++++++++++++


.. _Writing a Plugin:

Writing a Plugin
++++++++++++++++

Callbacks and Mutexes
~~~~~~~~~~~~~~~~~~~~~
It can be tricky to interface between ROS callbacks and CLIPS environments, especially if locks are guarding said callbacks that are not visible to the end user.
As CLIPS environment access must also guarded by locks (as access is not thread-safe), this can easily create deadlocks in situations, where other functions can be called from clips that also try to acquire the lock held by a callback.

Example: The feedback callback for action clients is guarded by a mutex that is also used by client goal handles to access members in a thread-safe manner (such as get_status()).

A CLIPS rule that calls ClientGoalHandle::get_status() will therefore attempt to lock such a mutex while the clips lock is being held by the thread running the clips environment.
If a callback is received right before, then the clips environment will stall as the function call is stuck (mutex is held by the callback function), while the callback function is stuck because it tries to acquire the lock for the clips environment (because it wants to pass the callback content to the clips environment).

In these cases special care must be taken, e.g., by deferring CLIPS access out of scope of the mutexes guarding the callbacks.

.. toctree::
   :maxdepth: 2

   hello_world.rst
   ros_monitoring.rst
   writing_a_plugin.rst
