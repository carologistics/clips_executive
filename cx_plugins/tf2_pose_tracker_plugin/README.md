# cx_tf2_pose_tracker_plugin
This package offers the `cx::Tf2PoseTrackerPlugin' that allows to perform periodic lookups to update poses obtained from a transform tree.

## Usage
Register this plugin with the plugin manager.
It's configuration parameters are depicted in this example setup below.

```yaml
clips_manager:
  ros__parameters:
    environments: ["main"]
    main:
      plugins: ["tf2_pose_tracker"]

    tf2_pose_tracker:
      plugin: "cx::Tf2PoseTrackerPlugin"
      # Whether to spin up a new node for tf lookups or use the main CX node
      # Defaults to true
      spin_thread: true
```
