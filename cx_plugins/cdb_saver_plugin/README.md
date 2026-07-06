# cx_cdb_saver_plugin

This package offers the `cx::CDBSaverPlugin` CLIPS plugin, which records a CLIPS environment to a PostgreSQL database for later use in the CDB toolchain.

## Disclaimer

All parameters of the CLIPS executive are stored in the database to ensure consistency when loading a recorded environment. This includes database connection parameters such as the database username and password.

**The database credentials are therefore not stored securely** and should not be used for sensitive or production credentials.

It is generally recommended to use a PostgreSQL instance running on the same system as the CLIPS executive. Database latency or slow write operations can slow down the execution of the CLIPS executive, because the saver plugin writes runtime data while the environment is running.

The saver plugin can also be loaded after an environment has already started. In that case, it takes a snapshot of the current environment when loaded and then continues to record subsequent changes. However, rule firings that occurred before the plugin was loaded are not recorded.

Because `CDBLoaderPlugin` cannot remove activations that were fired before the saver plugin was loaded, it is generally recommended to load this plugin from the start of the environment.

Deleting a defmodule is not supported but can be done using the `clear` command; This is not supported and generally speaking not recommended.

CLIPS global scoping is strictly hierarchical and one-directional: if module A can access constructs from module B, then module B cannot access constructs from module A. To restore an environment correctly, this module hierarchy must be recorded. This is only possible if the saver plugin is loaded before any additional modules are defined.

## Database Creation

The saver plugin creates a new database for each run. Therefore, the configured PostgreSQL user must have permission to create databases.

Each database is named using the CLIPS environment name and the start time of the run:

``` text
cdb_<environment_name>_<YYYY_MM_DDtHH_MM_SS>
```

## Performance considerations

This plugin can significantly slow down the execution of large projects. The main advantage of a CDB saver recording is that it allows the execution flow to be inspected retroactively. This makes it possible to step through a recorded run and understand what happened and why.

If the main issue is understanding why a rule is not firing, or if the goal is to inspect the current fact base of a running environment, consider using `cx_cdb_cli` or `cx_cdb_analyzer` instead. These tools allow a running environment to be investigated without the performance overhead of continuous recording.

## Usage

Register this plugin with the plugin manager. Its configuration parameters are shown in the example setup below.

```yaml
clips_manager:
  ros__parameters:
    environments: ["main"]
    main:
      plugins: ["cdb_saver"]

    cdb_saver:
      plugin: "cx::CDBSaverPlugin"

      # Hostname of the PostgreSQL server.
      # It is recommended to use a local PostgreSQL instance to avoid latency during saving.
      # Defaults to "localhost".
      hostname: "localhost"

      # PostgreSQL server port.
      # Defaults to 5432.
      port: 5432

      # PostgreSQL username used by the saver plugin.
      # The user must have permission to create databases.
      # Defaults to "anonymous".
      username: "anonymous"

      # PostgreSQL password used by the saver plugin.
      # This value is stored in the database as part of the recorded CLIPS executive parameters
      # and should therefore not be considered secure.
      # Defaults to an empty string.
      password: ""
