<!-- AUTO-GENERATED via sphinx-build. Do not edit directly. -->

<a id="usage-cdb-saver-plugin"></a>

# CDB Saver Plugin

Source code on [GitHub](https://github.com/carologistics/clips_executive/blob/master/cx_plugins/cdb_saver_plugin).

This plugin is part of the **C**LIPS **D**e**B**ugger (CDB) suite of the ROS2 CLIPS-Executive.

This plugin allows to log execution traces of CLIPS into a [Postgresql](https://www.postgresql.org/) database.

## Configuration

<!-- cdb_saver_hostname: -->
* **hostname:**
  | Type   | Default     |
  |--------|-------------|
  | string | “localhost” |

  Description
  : Hostname of the PostgreSQL server.
    It is recommended to use a local PostgreSQL instance to avoid latency during saving.

<a id="cdb-saver-port"></a>
* **port:**
  | Type   |   Default |
  |--------|-----------|
  | int    |      5432 |

  Description
  : PostgreSQL server port.

<a id="cdb-saver-username"></a>
* **username:**
  | Type   | Default   |
  |--------|-----------|
  | string | “cx_user” |

  Description
  : PostgreSQL username used by the saver plugin.
    The user must have permission to create databases.

<a id="cdb-saver-password"></a>
* **password:**
  | Type   | Default    |
  |--------|------------|
  | string | “password” |

  Description
  : PostgreSQL password used by the saver plugin.
    This value is stored in the database as part of the recorded CLIPS executive parameters
    and should therefore not be considered secure.

<a id="cdb-saver-db-prefix"></a>
* **db_prefix:**
  | Type   | Default   |
  |--------|-----------|
  | string | “cdb”     |

  Description
  : Prefix used for the automatically created database name.

<a id="db-timestamp-suffix"></a>
* **db_timestamp_suffix:**
  | Type   | Default   |
  |--------|-----------|
  | bool   | true      |

  Description
  : Whether to append a timestamp suffix to the generated database name.
    This is useful for creating unique databases per run.
    Note that if set to `false` this can lead to data loss as any pre-existing database with matching name is automatically dropped.

## Features

The CDB saver plugin registers callbacks with each CLIPS environment to log all changes into a running Postgresql database.

### Database Setup

A running Postgresql instance is required with a user and password matching the credentials as specified in the [parameters](#cdb-saver-username).

The user must have permission to create databases.

An example setup for a user `myuser` with password `mypassword` is shown below:

```bash
sudo -u postgres psql
postgres=# CREATE USER cx_user WITH PASSWORD 'password' CREATEDB;
postgres=# \q
```

Each database is named using a [configurable prefix](#cdb-saver-db-prefix), followed by `_`, the respective environment name and an optional time stamp.

#### WARNING
The database is always created fresh, dropping any other database with the same name.

Not using timestamps as suffix **may lead to data loss**.

### Database Credentials

All parameters of the CLIPS executive are stored in the database to ensure consistency when loading a recorded environment.
This includes database connection parameters such as the database username and password.

#### WARNING
The database credentials are stored in the database as part of the recorded environment.

**They are not stored securely** and should not be used for sensitive or production credentials.

### Performance Considerations

Database latency or slow write operations can slow down the execution of the CLIPS executive, as it synchronizes runtime data to the database.

It is generally recommended to use a PostgreSQL instance running on the same system as the CLIPS executive.

### Defmodule Support

There are some minor limitations when using this plugin in combination with modules.

1. The saver plugin must be loaded before any additional modules are defined to keep track of module relationships.
2. Deleting of defmodule is not supported in CLIPS but can be done using the clear command.
   However, this plugin can not handle this edge case.

### CLIPS COOL Support

the CLIPS COOL fragment for object-oriented reasoning is not supported.

## Usage Example

A minimal working example is provided by the [cx_bringup](https://carologistics.github.io/clips_executive/cx_bringup) package. Run it via:

```bash
ros2 launch cx_bringup cx_launch.py manager_config:=plugin_examples/cdb_saver.yaml
```

In this example the environment simply counts upwards from 0 to 20.

You can connect to the database and inspect it.
For example, in order to see all fact changes:

```bash
psql -h localhost -U cx_user -d cdb_cx_cdb -c "SELECT * from fact_values;"
```

### Configuration

File [cx_bringup/params/plugin_examples/executive.yaml](https://github.com/carologistics/clips_executive/blob/master/cx_bringup/params/plugin_examples/executive.yaml).

```yaml
/**:
  ros__parameters:
    environments: ["cx_cdb"]
    cx_cdb:
      plugins: ["cdb_saver", "executive", "files"]
      log_clips_to_file: true
      watch: ["facts", "rules"]

    cdb_saver:
      plugin: "cx::CDBSaverPlugin"
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
        "clips/plugin_examples/cdb-saver.clp"]
```

### Code

File [cx_bringup/clips/plugin_examples/cdb-saver.clp](https://github.com/carologistics/clips_executive/blob/master/cx_bringup/clips/plugin_examples/cdb-saver.clp).

```clips
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
```
