# cx_cdb_loader_plugin

This package provides the `cx::CDBLoaderPlugin` CLIPS plugin. It restores a CLIPS environment to a specific point in time based on a PostgreSQL database recording created by the `cx::CDBSaverPlugin`.

## Disclaimer

This plugin can only restore an environment if the same CLIPS functions are available that were used during the original recording run. In practice, this means that the same plugins used during recording must also be loaded before this plugin. Otherwise, the loader will emit a warning, proceed with caution.

Restoring a CLIPS environment is inherently volatile. The primary use case of this plugin is to inspect the CLIPS environment from a recorded run at a specific point in time.

Facts that point to external addresses are outside the scope of the recording and therefore cannot be restored reliably. Depending on the value of the `drop_external_addresses` configuration option, these facts are either dropped or restored with invalid pointers. By default, external addresses are restored as invalid pointers.

If you want to continue execution from a restored environment, the plugin asserts the following fact after loading:

```clips
(assert cdb-restored)
```

You can use this fact in custom rules to restore the environment to a working state.

Additionally, you can override `defrules`, `deffunctions`, `deftemplates` and `defglobals` by loading the `cx::FileLoadPlugin` after this plugin and providing redefinitions. In many cases, it may also be necessary to write a custom plugin to ensure that your own plugins are restored correctly.

## Restoring fired activations

Fired activations can be restored.

To do this, the plugin first asserts facts and rules. Then iterates over all matches that would cause a rule firing and removes the ones that were recorded as fired in the recording.

However, this doesn't work for activations that only occour based on facts from other plugins or if a rule get's redefined.

In theory, all these issues can be resolved manually, but doing so can become tedious. Therefore, it is generally good practice to design your rules with environment restoration in mind. You can choose to only restore some modules and skip others.

#### This plugin only aims to restore the environment to the best of it's abillity since CLIPS is a touring complete language that interacts with external ros components using the cx it's impossible to garrantee the ability to run. However this plugin garantees that the environment if restored fully can be investigated using the cx_cdb_cli to find issues, like why didn't my rule fire.

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
.load_if_match
.skip_if_match
```

Both parameters contain lists of regex strings. The plugin uses the C++ `std::regex` implementation.

For each object, the loader evaluates all `load_if_match` regexes. If at least one load regex matches, the object is considered for loading. It is only loaded if none of the `skip_if_match` regexes match.

For the following types, regexes are matched against the object name:

- `deffunctions`
- `deftemplates`
- `defglobals`
- `deffacts`
- `defrules`

Since `facts` do not have names, fact filtering is split into separate matching rules.

For facts, the `deftemplate` regex matches the fact’s `deftemplate`.

Note: facts without a `deftemplate` like `(assert init)` actually have an empty deftemplate under the hood. It's the name of the facts e.g. `init`.

Value expressions match key-value pairs. These expressions must be provided as pairs of regexes:

1. one regex matching the field name
2. one regex matching the field value

If the field is a multifield, the expression is considered a match if any value in the multifield matches the value regex, and dropped if the same one matches the skip if regex.

## Module filtering

Modules can be included or excluded using regexes.

An empty string `""` is the default include regex and is always treated as “match everything”.

For exclude regexes, an empty string `""` is treated as “match nothing”.

## Contains matching

By default, regexes are matched exactly according to the configured matching behavior.

If a regex string starts with `/`, the regex after the `/` is treated as a “contains” match.

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
      # The exact values must match the database where the CDB saver stored the recording.
      host: "localhost"
      port: 5432
      database: "cdb_main_2026_05_11t10_50_23"
      user: "postgres"
      password: "postgres"

      restore_tick: -1 #Last tick
      #restore_time: "T+50:23" # Restores 50 minutes and 23 seconds into the run mututally exclusive with restore tick

      # Facts containing external addresses cannot be restored reliably.
      # If true, facts with external addresses are dropped.
      # If false, they are restored with invalid pointers.
      # Defaults to false.
      drop_external_addresses: false

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
        # Empty string means match everything.
        # Defaults to [""]
        deftemplate_load_if_match: [""]

        # Regexes matched against the fact deftemplate.
        # Empty string means match nothing.
        # Defaults to [""]
        deftemplate_skip_if_match: [""]

        # Field-value based include filters.
        # Each entry contains a pair of regexes:
        # - field: regex matched against the field name
        # - value: regex matched against the field value
        #
        # For multifields, the value regex matches if any multifield value matches.
        #
        # Defaults to an empty list.
        load_if_value_match:
          - field: "name"
            value: "/robot"

        # Field-value based exclude filters.
        # Each entry contains a pair of regexes:
        # - field: regex matched against the field name
        # - value: regex matched against the field value
        #
        # Defaults to an empty list.
        skip_if_value_match:
          - field: "type"
            value: "ros-subscription"

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
      # restored defrules, deffunctions, or defglobals.
      #
      # When specifying relative paths, the listed package share directories
      # are used to resolve them.
      # Attempts to resolve relative paths in the order of the listed packages.
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
