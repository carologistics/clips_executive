.. _usage_cdb_loader_plugin:

CDB Loader Plugin
=================

Source code on :source-master:`GitHub <cx_plugins/cdb_loader_plugin>`.

.. admonition:: Plugin Class

  cx::CDBLoaderPlugin

This plugin is part of the |CDB| suite of the |CX|.

This plugin allows to restore the |CX| using execution traces from a `Postgresql`_ database, as recorded by the :ref:`CDB Saver Plugin <usage_cdb_saver_plugin>`.


Configuration
-------------

.. cdb_loader_host:

:`host`:

  ============ =======
  Type         Default
  ------------ -------
  string       "localhost"
  ============ =======

  Description
    Hostname of the PostgreSQL server.

.. _cdb_loader_port:

:`port`:

  ============ =======
  Type         Default
  ------------ -------
  int          5432
  ============ =======

  Description
    PostgreSQL server port.

.. _cdb_loader_username:

:`username`:

  ============ =======
  Type         Default
  ------------ -------
  string       "anonymous"
  ============ =======

  Description
    PostgreSQL username used by the loader plugin.
    The user must have permission to access databases.

.. _cdb_loader_password:

:`password`:

  ============ =======
  Type         Default
  ------------ -------
  string       ""
  ============ =======

  Description
    PostgreSQL password used by the loader plugin.
    This value is stored in the database as part of the recorded CLIPS executive parameters
    and should therefore not be considered secure.

.. _cdb_loader_restore_run:

:`restore_run`:

  ============ =======
  Type         Default
  ------------ -------
  string       ""
  ============ =======

  Description
    Restore the state after a complete run.

    Positive values are zero-based:
      - "0" = first recorded run
      - "1" = second recorded run

    Negative values count backwards from the end:
      - "-1" = latest recorded run
      - "-2" = run before the latest recorded run

    This value is parsed as a long long integer by the loader.

:`restore_tick`:

  ============ =======
  Type         Default
  ------------ -------
  string       ""
  ============ =======

  Description
    Restore by tick index.

    Positive values are zero-based:
      - "0" = first recorded tick
      - "1" = second recorded tick

    Negative values count backwards from the end:
      - "-1" = last recorded tick
      - "-2" = tick before the last recorded tick

    Mutually exclusive with ``restore_run`` and ``restore_time``.

:`restore_time`:

  ============ =======
  Type         Default
  ------------ -------
  string       ""
  ============ =======

  Description
    Restore by time.

    A value starting with ``T+`` is interpreted relative to the start of the first recorded run.
    Otherwise it is interpreted as an absolute timestamp.

    The loader selects the run whose end time is closest to the requested time and restores its end tick.

:`drop_external_addresses`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, facts containing external addresses are dropped during restore.
    If false, they are restored with placeholder external addresses.

:`restore_external_addresses_as_nullptr`:

  ============ =======
  Type         Default
  ------------ -------
  bool         true
  ============ =======

  Description
    If true, external addresses are restored as nullptr values.
    If false, they are restored as invalid external address values.
    Only applies when ``drop_external_addresses`` is false.

:`modules.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of modules to include. Empty string matches everything.

:`modules.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of modules to exclude. Empty string matches nothing.

:`deffacts.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, deffacts are not restored.

:`deffacts.manual_assert`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, facts from deffacts are manually asserted.
    Useful because cx::CDBLoaderPlugin skips reset.

:`deffacts.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of deffacts names to include.

:`deffacts.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of deffacts names to exclude.

:`facts.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, facts are not restored.

:`facts.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of fact deftemplates to include.

:`facts.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of fact deftemplates to exclude.

:`defrules.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, defrules are not restored.

:`defrules.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of defrule names to include.

:`defrules.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of defrule names to exclude.

:`deftemplates.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, deftemplates are not restored.

:`deftemplates.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of deftemplate names to include.

:`deftemplates.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of deftemplate names to exclude.

:`defglobals.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, defglobals are not restored.

:`defglobals.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of defglobal names to include.

:`defglobals.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of defglobal names to exclude.

:`deffunctions.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, deffunctions are not restored.

:`deffunctions.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of deffunction names to include.

:`deffunctions.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of deffunction names to exclude.

:`activations.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, fired activations are not restored.

:`activations.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of activation or rule names to include.

:`activations.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of activation or rule names to exclude.


Features
--------

The CDB loader plugin registers callbacks with each CLIPS environment to log all changes into a running Postgresql database.

Database Setup
^^^^^^^^^^^^^^

A running Postgresql instance is required with a user and password matching the credentials as specified in the :ref:`parameters <cdb_loader_username>`.

The user must have permission to create databases.

An example setup for a user ``myuser`` with password ``mypassword`` is shown below:

.. code-block:: bash

    sudo -u postgres psql
    postgres=# CREATE USER cx_user WITH PASSWORD 'password' CREATEDB;
    postgres=# \q

Each database is named using a :ref:`configurable prefix <cdb_loader_db_prefix>`, followed by ``_``, the respective environment name and an optional :ref:`time stamp <cdb_loader_db_timestamp_suffix>`.

.. warning::

   The database is always created fresh, dropping any other database with the same name.

   Not using timestamps as suffix **may lead to data loss**.


Database Credentials
^^^^^^^^^^^^^^^^^^^^

All parameters of the CLIPS executive are stored in the database to ensure consistency when loading a recorded environment.
This includes database connection parameters such as the database username and password.

.. warning::

   The database credentials are stored in the database as part of the recorded environment.

   **They are not stored securely** and should not be used for sensitive or production credentials.


Performance Considerations
^^^^^^^^^^^^^^^^^^^^^^^^^^

Database latency or slow write operations can slow down the execution of the CLIPS executive, as it synchronizes runtime data to the database.

It is generally recommended to use a PostgreSQL instance running on the same system as the CLIPS executive.


Defmodule Support
^^^^^^^^^^^^^^^^^

There are some minor limitations when using this plugin in combination with modules.

#. The loader plugin must be loaded before any additional modules are defined to keep track of module relationships.
#. Deleting of defmodule is not supported in CLIPS but can be done using the `clear` command.
   However, this plugin can not handle this edge case.

CLIPS COOL Support
^^^^^^^^^^^^^^^^^^

the CLIPS COOL fragment for object-oriented reasoning is not supported.


Usage Example
-------------

A minimal working example is provided by the :docsite:`cx_bringup` package. Run it via:

.. code-block:: bash

    ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/cdb_loader.yaml

In this example the environment simply counts upwards from 0 to 20.

You can connect to the database and inspect it.
For example, in order to see all fact changes:

.. code-block:: bash

  psql -h localhost -U cx_user -d cdb_cx_cdb -c "SELECT * from fact_values;"

Configuration
^^^^^^^^^^^^^

File :source-master:`cx_bringup/params/plugin_examples/executive.yaml`.

.. code-block:: yaml

  /**:
    ros__parameters:
      environments: ["cx_cdb"]
      cx_cdb:
        plugins: ["cdb_loader", "executive", "files"]
        log_clips_to_file: true
        watch: ["facts", "rules"]

      cdb_loader:
        plugin: "cx::CDBLoaderPlugin"
        hostname: "localhost"
        port: 5432
        username: "cx_user"
        password: "password"
        db_prefix: "cdb"
        # for demonstration purposes this is turned off.
        db_timestamp_suffix: false

      executive:
        plugin: "cx::ExecutivePlugin"
        assert_time: true
        refresh_rate: 10
      files:
        plugin: "cx::FileLoadPlugin"
        pkg_share_dirs: ["cx_bringup"]
        load: [
          "clips/plugin_examples/cdb-loader.clp"]

Code
^^^^

File :source-master:`cx_bringup/clips/plugin_examples/cdb-loader.clp`.

.. code-block:: clips

  (deftemplate counter
    (slot value (type INTEGER))
    (slot last-updated (type FLOAT))
  )

  (deffacts init-counter
    (counter (value 0))
  )

  (defrule increment-counter
    ?f <- (counter (value ?n) (last-updated ?t))
    (time ?now&:(< ?t ?now))
    =>
    (bind ?new-value (+ ?n 1))
    (if (> ?new-value 20)
     then
      (retract ?f)
      (cx-shutdown)
     else
      (modify ?f (value (+ ?n 1)) (last-updated ?now))
    )
  )
