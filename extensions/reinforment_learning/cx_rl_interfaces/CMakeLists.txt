cmake_minimum_required(VERSION 3.8)
project(cx_rl_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(SERVICES
  "srv/SetRLMode.srv"
  "srv/GetActionID.srv"
  "srv/GetActionListRobot.srv"
  "srv/GetObservableObjects.srv"
  "srv/GetObservablePredicates.srv"
  "srv/GetPredefinedObservables.srv"
  "srv/CreateRLEnvState.srv"
  "srv/GetActionList.srv"
  "srv/ExecActionSelection.srv"
)

set(ACTIONS
  "action/GetFreeRobot.action"
  "action/ResetCX.action"
  "action/ActionSelection.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SERVICES}

  ${ACTIONS}
  DEPENDENCIES std_msgs std_srvs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
