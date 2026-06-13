Usage
#####

The CLI requires a running ROS CX with at least one environment, where the executive plugin is loaded.


Starting the CLI
++++++++++++++++

The CLI is supplied as an executable that can be directly started via ``ros2 run``. Per default it assumes a CX node called `clips_manager` and an executive plugin with name `executive`.

Launch the ROS CX accordingly in one terminal, here we utilize the example configuration from the ``executive plugin`` itself:

.. code-block:: bash

    ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/executive.yaml

As this example fits the default parameters, the CLI can be started without additional parameters:

.. code-block:: bash

   ros2 run cx_clips_cli cli

The list of all available parameters is ref:'provided here <cli_ros_params>'.

Navigation
++++++++++

After starting the CLI, you will see the log window printing a helpful message, a command prompt and a status bar on the bottom.

The CLI uses vi-style modal editing, where the result of pressing buttons depend on the mode the CLI currently is in.
When starting the CLI, it is in **NORMAL** mode as indicated by the first entry of the status bar.
You can inspect available commands py pressing :kbd:`Space m`, which opens a menu.
Close the menu using :kbd:`Esc` or by selecting an option and pressing :kbd:`Enter`.

You can switch to **INSERT** mode by pressing :kbd:`i`, meaning you can type CLIPS expressions and send them by pressing enter.

Press :kbd:`Esc` to return to **NORMAL** mode.

To inspect the log window press :kbd:`Space l` to enter **ON LOG** mode, where you can navigate using arrow keys, jump to the top (bottom) via :kbd:`g` (:kbd:`G`) perform  forward (backward) search via :kbd:`/` (:kbd:`?`), jump to the next (previous) search result via :kbd:`n` (:kbd:`N`) and return to **NORMAL** mode py pressing :kbd:`Esc`.

From here you can quit the CLI by pressing :kbd:`Space q`.


.. _cli_ros_params:

ROS Parameters
++++++++++++++

In order to customize the CLI, simply pass ROS parameters to it as follows:

.. code-block:: bash

    ros2 run cx_clips_cli cli --ros-args -p cx_node_name:=clips_manager -p plugin_name:=executive

The following ROS parameters are available and they will be further explained :ref:`later <customization.rst>`.


.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``cx_node_name``
     - ``clips_manager``
     - Name of the running CX manager node.
   * - ``plugin_name``
     - ``executive``
     - Name of the CLIPS executive plugin within that node.
   * - ``dark_mode``
     - auto-detected
     - Force dark (``true``) or light (``false``) colour theme.
       Auto-detection reads ``COLORFGBG`` and ``TERM_PROFILE``.
   * - ``log_dir``
     - ``$ROS_LOG_DIR``
     - Directory for log snapshots and stream files.
       Defaults to the ROS log directory.
   * - ``pygments_style``
     - ``github-dark`` / ``friendly``
     - Any `pygments`_ style to use
       as the syntax-highlighting base.
   * - ``style_overrides``
     - *(none)*
     - Path to a file with colour overrides (see
       :ref:`style-customization`).
   * - ``keymap_overrides``
     - *(none)*
     - Path to a file with key binding overrides (see
       :ref:`keymap-customization`).
