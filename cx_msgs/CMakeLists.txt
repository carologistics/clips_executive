cmake_minimum_required(VERSION 3.16...3.28)
project(cx_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(SERVICES
  "srv/DestroyClipsEnv.srv"
  "srv/CreateClipsEnv.srv"
  "srv/ListClipsEnvs.srv"
  "srv/LoadClipsPlugin.srv"
  "srv/ListClipsPlugins.srv"
  "srv/UnloadClipsPlugin.srv"
)

# set(ACTIONS

# )
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SERVICES}

  # ${actions}
  DEPENDENCIES std_msgs std_srvs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
