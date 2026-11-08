Installation
============

This document assumes you have a basic ROS 2 installation. If not, refer to the official `ROS documentation`_.

Binary Installation
-------------------

The easiest way to install the |CX| is via your package manager:

.. note::

   The |CX| is currently in the process of being indexed, hence binary packages might not be available yet.

.. code-block:: bash

   sudo apt intall ros-<ros2-distro>-clips-executive

.. _source_installation:


Installation from Source
------------------------

The following steps will setup a workspace for the project. Adjust target locations as needed.

.. code-block:: bash

    # Workspace setup
    mkdir -p ~/ros2/clips_executive_ws/src
    git clone https://github.com/carologistics/clips_executive.git ~/ros2/clips_executive_ws/src/clips_executive

The repository contains the core packages and a set of optional :ref:`extensions <extensions>`.
Extensions are disabled by default, so a plain build only needs the core dependencies.
If you want to use the extensions, follow the steps in :ref:`building_with_extensions` instead of the ones below.

Core only
^^^^^^^^^

Install dependencies using rosdep:

.. code-block:: bash

    cd ~/ros2/clips_executive_ws
    rosdep install --from-paths src -y --ignore-src

Lastly, build and source the workspace:

.. code-block:: bash

    # Building and sourcing
    cd ~/ros2/clips_executive_ws/
    colcon build --symlink-install
    source ~/ros2/clips_executive_ws/install/setup.bash

.. _building_with_extensions:

Building with extensions
^^^^^^^^^^^^^^^^^^^^^^^^

The extensions are disabled by a ``COLCON_IGNORE`` file in each extension directory.
They additionally require Python packages from PyPI (e.g. ``torch``) that are not available as system packages.
Recent PyPI releases of these packages pull in a very new ``setuptools``, which removes ``pkg_resources`` and breaks setuptools.

To prevent this, either constrain ``setuptools`` to a compatible range while installing or remvoe the pip-installed version and re-install the sysem-wide one:

1. Enable the extensions you want by removing their ignore marker. To enable all of them:

   .. code-block:: bash

       cd ~/ros2/clips_executive_ws/src/clips_executive
       rm extensions/*/COLCON_IGNORE

   To enable only a single extension, remove only ``extensions/<extension>/COLCON_IGNORE``.

2. Install the dependencies with rosdep. ``torch`` requires a newer ``setuptools`` than the system provides,
   so choose how to deal with that:

   .. rubric:: Constrain pip's setuptools

   Limit pip to a ``setuptools`` version that still ships ``pkg_resources``, which ``vcs`` and ``colcon`` need:

   .. code-block:: bash

       echo "setuptools>=77.0.3,<80" > ~/pip-constraints.txt
       export PIP_CONSTRAINT=~/pip-constraints.txt
       export PIP_BREAK_SYSTEM_PACKAGES=1

       cd ~/ros2/clips_executive_ws
       rosdep install --from-paths src -y --ignore-src


   .. rubric:: Restore system setuptools

   Let pip install what it wants, then remove its ``setuptools`` and restore the system one:

   .. code-block:: bash

       export PIP_BREAK_SYSTEM_PACKAGES=1

       cd ~/ros2/clips_executive_ws
       rosdep install --from-paths src -y --ignore-src

       pip uninstall -y --break-system-packages setuptools
       sudo apt-get install -y --reinstall python3-setuptools python3-pkg-resources

   .. note::

      The default ``torch`` wheel includes several GB of GPU libraries.
      If you only need CPU support, install ``torch`` and ``torchvision`` beforehand with
      ``pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu``.
      rosdep then sees them as already satisfied.

3. Build and source the workspace:

   .. code-block:: bash

       cd ~/ros2/clips_executive_ws
       colcon build --symlink-install
       source install/setup.bash
