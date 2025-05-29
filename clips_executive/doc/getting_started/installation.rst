Installation
############
This document assumes you have a basic ROS 2 installation. If not, refer to the official `ROS documentation`_.

Binary Installation
+++++++++++++++++++

The easiest way to install the |CX| is via your package manager:

.. note::

   The |CX| is currently in the process of being indexed, hence binary packages might not be available yet.

.. code-block:: bash

   sudo apt intall ros-<ros2-distro>-clips-executive


Installation from Source
+++++++++++++++++++++++++

.. todo::

   dependency.repos should not be necessary

All dependencies are listed in the :source-master:`dependency.repos` file of the |CX| project.
This enables setting up a dependency workspace using `vcstool`.

The following steps will set up two workspaces, one for dependencies of the |CX| and one for the main project. Adjust target locations as needed.

.. code-block:: bash

    mkdir -p ~/ros2/{clips_executive_ws,deps_clips_executive_ws}/src
    git clone -b tviehmann/major-cleanup https://github.com/fawkesrobotics/ros2-clips-executive.git ~/ros2/clips_executive_ws/src/ros2-clips-executive
    cd ~/ros2/deps_clips_executive_ws/src
    vcs import < ~/ros2/clips_executive_ws/src/ros2-clips-executive/dependency.repos
    cd ~/ros2/deps_clips_executive_ws/
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF

Then source the workspace with dependencies and build the CLIPS-Executive:

.. code-block:: bash

    source ~/ros2/deps_clips_executive_ws/install/setup.bash
    cd ~/ros2/clips_executive_ws/
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=OFF
    source ~/ros2/clips_executive_ws/install/setup.bash
