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

.. _cdb_loader_host:

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
  string       "cx_user"
  ============ =======

  Description
    PostgreSQL username used by the loader plugin.
    The user must have permission to access databases.

.. _cdb_loader_password:

:`password`:

  ============ =======
  Type         Default
  ------------ -------
  string       "password"
  ============ =======

  Description
    PostgreSQL password used by the loader plugin.
    This value is stored in the database as part of the recorded CLIPS executive parameters
    and should therefore not be considered secure.


.. _cdb_loader_restore_method:

:`password`:

  ============ =======
  Type         Default
  ------------ -------
  string       "run"
  ============ =======

  Description
    Method to use when restoring. Allowed values are ["run", "tick", "time"].
    See also :ref:`restore_run <cdb_loader_restore_run>`, :ref:`restore_tick <cdb_loader_restore_tick>`,
    :ref:`restore_time <cdb_loader_restore_time>`.


.. _cdb_loader_restore_run:

:`restore_run`:

  ============ =======
  Type         Default
  ------------ -------
  int          0
  ============ =======

  Description
    Restore the state after a complete run.

    Positive values are zero-based:
      - "0" = first recorded run
      - "1" = second recorded run

    Negative values count backwards from the end:
      - "-1" = latest recorded run
      - "-2" = run before the latest recorded run

.. _cdb_loader_restore_tick:

:`restore_tick`:

  ============ =======
  Type         Default
  ------------ -------
  int          0
  ============ =======

  Description
    Restore by tick index.

    Positive values are zero-based:
      - "0" = first recorded tick
      - "1" = second recorded tick

    Negative values count backwards from the end:
      - "-1" = last recorded tick
      - "-2" = tick before the last recorded tick

.. _cdb_loader_restore_time:

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

.. _cdb_loader_drop_external_addresses:

:`drop_external_addresses`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, facts containing external addresses are not restored.

.. _cdb_loader_restore_as_nullptr:

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


Additionally, the loading of the ``modules``, ``deffacts``, ``facts``, ``deftemplates``, ``defrules``, ``deffunctions``, ``activations``, ``defglobals`` constructs can be managed via the following parameters.

.. _cdb_loader_construct_skip:

:`<construct>.skip`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, none of the given construct are not restored.


.. _cdb_loader_construct_load_if:

:`<construct>.load_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list of constructs to include. Empty string matches everything.


.. _cdb_loader_consruct_skip_if:

:`<construct>.skip_if_match`:

  ============ =======
  Type         Default
  ------------ -------
  string[]     [""]
  ============ =======

  Description
    Regex list to exclude from the set of all constructs. Empty string matches nothing.


.. _cdb_loader_deffacts_manual_assert:

Lastly, for deffacts, an extra config value exists:

:`deffacts.manual_assert`:

  ============ =======
  Type         Default
  ------------ -------
  bool         false
  ============ =======

  Description
    If true, facts from deffacts are manually asserted.
    Useful because cx::CDBLoaderPlugin skips reset.


Features
--------

The CDB loader plugin restores a previously recorded CLIPS environment from a PostgreSQL database to a specific point in time, primarily for inspection and debugging purposes.

.. note::
   The restored environment must provide the same CLIPS functions that were available during the original recording run.
   In practice, this means loading the same plugins before loading this plugin.
   The loader emits a warning if differences between the recorded and current configuration are detected.

Limitations
^^^^^^^^^^^

Some information cannot be restored and is therefore lost:

- **External addresses:** External objects are restored as ``nullptr`` external addresses by default.
- **Fact IDs:** The relative assertion order of non-retracted facts is preserved, but relative fact IDs of retracted facts are lost.
- **Fact references to missing facts:** The relationship is preserved, but the relative fact ID of the referenced fact is lost.
- **Defglobal initial values:** Restored defglobals reset to ``nil`` on ``(reset)`` instead of their original initial value.
- **Rules fired before** ``CDBSaverPlugin`` **was loaded:** Only rule firings recorded while ``CDBSaverPlugin`` was active can be restored.
- **MAIN defmodule redefinition time:** MAIN is always restored with its final redefinition, regardless of the requested restore tick.

External Addresses
^^^^^^^^^^^^^^^^^^

External addresses are outside the scope of the recording. By default, they are restored as ``nullptr`` external addresses.

.. warning::
   Keeping external address values is not recommended if the restored environment is intended to continue running,
   because **using these addresses as function input can crash CLIPS**.

If facts and defglobals containing external addresses should not be restored at all, enable ref:`drop_external_addresses <cdb_loader_drop_externeal_addresses`

Continuing Execution After Restoration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Restoring a CLIPS environment is mainly intended for inspection. After loading, the plugin asserts:

.. code-block:: clips

   (cdb-restored)

This fact can be used in custom rules to repair or reinitialize parts of the environment after restoration.
Restored ``defrules``, ``deffunctions``, ``deftemplates``, and ``defglobals`` can be overridden by loading ``cx::FileLoadPlugin`` after this plugin.

Module Filtering
^^^^^^^^^^^^^^^^

Modules can be included or excluded using regexes matched against module names.
The default include regex matches everything; the default exclude regex matches nothing.

Configurable Restore Types
^^^^^^^^^^^^^^^^^^^^^^^^^^

The following CLIPS object types can each be individually skipped:

- ``deffacts``, ``facts``, ``defrules``, ``deftemplates``, ``defglobals``, ``deffunctions``, ``activations``

By default, ``deffacts`` are loaded but not asserted automatically, since ``(reset)`` is skipped during restoration.
To assert them manually, enable :ref:`deffacts.manual_assert <cdb_loader_deffacts_manual_assert`.

Each type supports regex-based filtering via ``load_if_match`` and ``skip_if_match`` parameter lists.
For facts, regexes are matched against the fact's ``deftemplate`` name.
Prefix a regex with ``/`` to perform a contains match instead of an exact match.



Usage Example
-------------

A minimal working example is provided by the :docsite:`cx_bringup` package. Run the example code from the CDB saver plugin to obtain a database to restore:

.. code-block:: bash

    ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/cdb_saver.yaml

Then run the CDB loader plugin:

    ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/cdb_loader.yaml

This loads the run mid execution.


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
