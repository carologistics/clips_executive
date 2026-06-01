# cx_cdb_loader_plugin

This package provides the `cx::CDBLoaderPlugin` CLIPS plugin. It restores a CLIPS environment to a specific point in time based on a PostgreSQL database recording created by the `cx::CDBSaverPlugin`.

## Disclaimer

This plugin aims to restore recorded CLIPS environments as comprehensively as possible. However, some information cannot be restored and is therefore lost:

- **External addresses:** External objects cannot be restored. By default, they are restored as `nullptr` external addresses.
- **Fact IDs:** Original fact IDs are not preserved exactly. The relative assertion order of non-retracted facts is preserved, but relative fact IDs of retracted facts are lost.
- **Fact references to missing facts:** If a fact or defglobal referenced a fact that was retracted or is not loaded due to this plugin's configuration, the relationship is preserved, but the relative fact ID of the referenced fact is lost.
- **Defglobal initial values:** Defglobals reset to their initial value on `(reset)`. This initial value is not recorded, so restored defglobals reset to `nil`.
- **Rules fired before `CDBSaverPlugin` was loaded:** CLIPS does not keep a history of fired rules. Therefore, only rule firings that happened while `CDBSaverPlugin` was loaded can be recorded and restored.
- **MAIN defmodule redefinition time:** MAIN is the default CLIPS defmodule and may therefore be redefined once to expose ports. This redefinition is recorded, but the exact tick at which the exported ports became available is not. As a result, when restoring an earlier tick, MAIN is loaded with the definition from its redefinition, even if that redefinition happened after the requested restore tick.

The primary use case of this plugin is inspection and debugging. It allows a recorded environment to be restored to a specific point in time so that it can be investigated using tools such as `cx_cdb_cli` and `cx_cdb_analyzer`.

The restored environment must provide the same CLIPS functions that were available during the original recording run. In practice, this usually means loading the same plugins before loading this plugin. The loader compares the recorded configuration with the current configuration and emits a warning if differences are detected. Proceed with caution if such a warning appears, because the restored environment may be incomplete or behave differently from the recorded run.

## Restoring external addresses

External addresses are outside the scope of the recording. The loader can only restore the information that a value pointed to something external; it cannot restore the external object itself.

By default, external addresses are restored as `nullptr` external addresses. This makes it visible that the value is broken and must not be used as a valid external object.

However, this can change the fact base. For example, during the original recording, the following two facts may have existed:

```clips
f-1     (msg <Pointer-C-0x101010101010101>)
f-2     (msg <Pointer-C-0x202020202020202>)
```

After restoration with `nullptr` external addresses, both facts become structurally identical:

```clips
f-1     (msg <Pointer-C-(nil)>)
```

By default, CLIPS does not store duplicate facts, so only one of them may remain.

If the loader is configured to keep external address values, equality relationships between different external addresses can be preserved for inspection. This can be useful when rule conditions depend on the number of distinct facts or on external address equality.

However, restored external addresses are not valid pointers in the restored process. Keeping external address values is therefore not recommended if the restored environment is intended to continue running, because **using these addresses as function input can crash CLIPS**.

If facts and defglobals containing external addresses should not be restored at all, enable:

```text
drop_external_addresses
```

## Missing fact references

Facts and defglobals can contain fact addresses that refer to facts that have already been retracted or are not loaded due to this plugin's configuration.

For this purpose, the loader temporarily asserts and retracts placeholder facts named:

```text
nullptr-<fact-id>
```

where `<fact-id>` is the fact ID from the original recording run.

In general, `CDBLoaderPlugin` asserts facts in the same relative order as in the recording run. Placeholder facts for missing fact references are an exception: they receive random fact IDs that are higher than the last restored fact. This means that the relative fact ID of the referenced missing fact is lost.

The original relationship is still preserved. If two restored values pointed to the same missing fact in the recording, they will also point to the same placeholder fact after restoration.

## Restoring fired activations

Fired activations can be restored to the extent that they were recorded.

To do this, the loader first restores facts and rules. It then iterates over the activations that would currently be eligible to fire and removes the activations that were already recorded as fired in the original run.

This only works for facts and rules that are restored by the this plugin.

Rules fired before `CDBSaverPlugin` was loaded cannot be removed during restoration, because they were never recorded. For this reason, it is generally recommended to load `CDBSaverPlugin` from the start of the environment.

## Continuing execution after restoration

Restoring a CLIPS environment is mainly intended for inspection. Continuing execution from a restored environment may require additional cleanup or reconstruction logic.

After loading, the plugin asserts the following fact:

```clips
(assert (cdb-restored))
```

You can use this fact in custom rules to repair or reinitialize parts of the environment after restoration.

Additionally, you can override restored `defrules`, `deffunctions`, `deftemplates`, and `defglobals` by loading `cx::FileLoadPlugin` after this plugin and providing redefinitions.

In many cases, a custom plugin may also be necessary to restore project-specific external state correctly.

## Module filtering

The loader can filter which parts of the recorded environment should be restored.

First, modules can be included or excluded using regexes.

By default, CLIPS code is in the module `MAIN`.

The default include regex is "", which matches everything.

The default exclude regex is "", which matches nothing.

## Configurable restore types

To assist with restoration, the plugin provides configuration options for the following CLIPS object types:

- `deffacts`
- `facts`
- `defrules`
- `deftemplates`
- `defglobals`
- `deffunctions`
- `activations`

Each type can be skipped.

By default, `deffacts` are loaded but not asserted automatically. This is because `deffacts` are normally asserted during `(reset)`, but `(reset)` is skipped when using `cx::CDBLoaderPlugin`.

If you want the plugin to assert restored `deffacts` manually, enable:

```text
deffacts.manual_assert
```

## Regex-based filtering

Each configurable type supports the following parameters:

```text
load_if_match
skip_if_match
```

Both parameters contain lists of regex strings. The plugin uses the C++ `std::regex` implementation.

For each object, the loader evaluates all `load_if_match` regexes. If at least one load regex matches, the object is considered for loading. It is only loaded if none of the `skip_if_match` regexes match.

For the following types, regexes are matched against the object name:

- `deffunctions`
- `deftemplates`
- `defglobals`
- `deffacts`
- `defrules`

Activations are matched against the corresponding defrule name.

Facts do not have names, so fact filtering is split into separate matching rules.

For facts, the `deftemplate` regex matches the fact’s `deftemplate`.

Ordered facts such as:

```clips
(assert (init))
```

have a deftemplate internally. Here, it is `init`.

If facts need to be skipped based on their slot values, load a `cx::FileLoadPlugin` after `cx::CDBLoaderPlugin` and provide CLIPS rules that retract the restored facts you do not want to keep.

## Contains matching

By default, regexes are matched exactly.

If a regex string starts with `/`, the regex after the `/` is treated as a contains match.

For example:

```text
hallo
```

matches only if the value is exactly `hallo`.

```text
/hallo
```

matches if the value contains `hallo`.

## Example configuration

```yaml
clips_manager:
  ros__parameters:
    environments: ["main"]

    main:
      plugins:
        - "required_plugin_1"
        - "required_plugin_2"
        - "cdb_loader"
        - "restore_overrides"

    required_plugin_1:
      plugin: "my_package::RequiredPlugin1"

    required_plugin_2:
      plugin: "my_package::RequiredPlugin2"

    cdb_loader:
      plugin: "cx::CDBLoaderPlugin"

      # PostgreSQL database connection used to read the recording.
      # These values must point to the database created by cx::CDBSaverPlugin.
      host: "localhost"
      port: 5432
      database: "cdb_main_2026_05_11t10_50_23"
      user: "postgres"
      password: "postgres"

      # Select what should be restored.
      #
      # Exactly one of the following parameters may be set:
      # - restore_run
      # - restore_tick
      # - restore_time
      #
      # If more than one is set, the configuration is invalid.
      #
      # restore_run and restore_tick are strings because the values are parsed
      # as long long by the loader.

      # Restore the state after a complete run.
      #
      # Positive values are zero-based:
      #   "0"  = first recorded run
      #   "1"  = second recorded run
      #
      # Negative values count backwards from the end:
      #   "-1" = latest recorded run
      #   "-2" = run before the latest recorded run
      #
      restore_run: ""

      # Restore by tick.
      #
      # Positive values are zero-based:
      #   "0"  = first recorded tick
      #   "1"  = second recorded tick
      #
      # Negative values count backwards from the end:
      #   "-1" = last recorded tick
      #   "-2" = tick before the last recorded tick
      #
      # Mutually exclusive with restore_run and restore_time.
      #
      restore_tick: ""

      # Restore by time.
      #
      # A value starting with T+ is interpreted relative to the start time of the
      # first recorded run. A normal timestamp is interpreted as an absolute time.
      #
      # The loader selects the run whose end_time is closest to the requested time
      # and restores that run's end_tick.
      #
      # Mutually exclusive with restore_run and restore_tick.
      #
      # Examples:
      # restore_time: "T+00:50:23"
      # restore_time: "2026-05-11T10:50:23Z"
      # restore_time: "2026-05-11 10:50:23+00"
      #
      restore_time: ""

      # Facts containing external addresses cannot be restored reliably.
      # If true, facts containing external addresses are dropped.
      # If false, they are restored with placeholder external addresses.
      # Defaults to false.
      drop_external_addresses: false

      # If true, external addresses are restored as nullptr external addresses.
      # If false, external address values are restored as invalid external address values.
      # This is only used when drop_external_addresses is false.
      # Defaults to true.
      restore_external_addresses_as_nullptr: true

      # Module filtering.
      modules:
        # Empty string means match everything.
        # Defaults to [""]
        load_if_match: [""]

        # Empty string means match nothing.
        # Defaults to [""]
        skip_if_match: [""]

      deffacts:
        # If true, do not restore deffacts.
        # Defaults to false.
        skip: false

        # If true, manually assert facts from restored deffacts.
        # This is useful because cx::CDBLoaderPlugin skips reset.
        # Defaults to false.
        manual_assert: false

        # Regexes matched against deffacts names.
        # Empty string means match everything.
        # Defaults to [""]
        load_if_match: [""]

        # Regexes matched against deffacts names.
        # Empty string means match nothing.
        # Defaults to [""]
        skip_if_match: [""]

      facts:
        # If true, do not restore facts.
        # Defaults to false.
        skip: false

        # Regexes matched against the fact deftemplate.
        # "" matches everything.
        # Defaults to [""]
        load_if_match: [""]

        # Regexes matched against the fact deftemplate.
        # "" matches nothing.
        # Defaults to [""]
        skip_if_match: [""]

      defrules:
        # If true, do not restore defrules.
        # Defaults to false.
        skip: false

        # Regexes matched against defrule names.
        # Empty string means match everything.
        # Defaults to [""]
        load_if_match: [""]

        # Regexes matched against defrule names.
        # Empty string means match nothing.
        # Defaults to [""]
        skip_if_match: [""]

      deftemplates:
        # If true, do not restore deftemplates.
        # Defaults to false.
        skip: false

        # Regexes matched against deftemplate names.
        # Empty string means match everything.
        # Defaults to [""]
        load_if_match: [""]

        # Regexes matched against deftemplate names.
        # Empty string means match nothing.
        # Defaults to [""]
        skip_if_match: [""]

      defglobals:
        # If true, do not restore defglobals.
        # Defaults to false.
        skip: false

        # Regexes matched against defglobal names.
        # Empty string means match everything.
        # Defaults to [""]
        load_if_match: [""]

        # Regexes matched against defglobal names.
        # Empty string means match nothing.
        # Defaults to [""]
        skip_if_match: [""]

      deffunctions:
        # If true, do not restore deffunctions.
        # Defaults to false.
        skip: false

        # Regexes matched against deffunction names.
        # Empty string means match everything.
        # Defaults to [""]
        load_if_match: [""]

        # Regexes matched against deffunction names.
        # Empty string means match nothing.
        # Defaults to [""]
        skip_if_match: [""]

      activations:
        # If true, do not restore fired activations.
        # Defaults to false.
        skip: false

        # Regexes matched against activation or rule names.
        # Empty string means match everything.
        # Defaults to [""]
        load_if_match: [""]

        # Regexes matched against activation or rule names.
        # Empty string means match nothing.
        # Defaults to [""]
        skip_if_match: [""]

    restore_overrides:
      plugin: "cx::FileLoadPlugin"

      # Load this plugin after cx::CDBLoaderPlugin if you want to override
      # restored defrules, deffunctions, deftemplates, or defglobals.
      #
      # When specifying relative paths, the listed package share directories
      # are used to resolve them.
      # Relative paths are resolved in the order of the listed packages.
      # Defaults to an empty list.
      pkg_share_dirs: ["cx_bringup"]

      # Specify files to load using the CLIPS load* command when the plugin is loaded.
      # Supports absolute paths or relative paths using the share directories specified above.
      # Defaults to an empty list.
      load: ["restore_overrides.clp"]

      # Specify files to load using the CLIPS batch* command when the plugin is loaded.
      # Supports absolute paths or relative paths using the share directories specified above.
      # Defaults to an empty list.
      batch: []

      # Specify files to load using the CLIPS batch* command when the plugin is unloaded.
      # This may be used to clean up loaded content to enable dynamic reloading of plugins at runtime.
      # Supports absolute paths or relative paths using the share directories specified above.
      # Defaults to an empty list.
      cleanup_batch: []
```
