cmake_minimum_required(VERSION 3.16...3.31)
project(cx_rl_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

set(MESSAGES
  "msg/Action.msg"
  "msg/Observation.msg"
)

set(SERVICES
  "srv/GetObservations.srv"
  "srv/EndTraining.srv"
  "srv/ExecActionSelection.srv"
  "srv/GetActionID.srv"
  "srv/GetActionListRobot.srv"
  "srv/GetEpisodeEnd.srv"
  "srv/GetObservableActions.srv"
  "srv/GetObservableObjects.srv"
  "srv/GetObservablePredicates.srv"
  "srv/GetPredefinedActions.srv"
  "srv/GetPredefinedObservables.srv"
  "srv/GetStatus.srv"
)

set(ACTIONS
  "action/ActionSelection.action"
  "action/GetFreeRobot.action"
  "action/ResetEnv.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MESSAGES}

  ${SERVICES}

  ${ACTIONS}
  DEPENDENCIES std_msgs std_srvs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
