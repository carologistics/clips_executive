#ifndef CX_FEATURES__REINFORCEMENTLEARNINGEATURE_HPP_
#define CX_FEATURES__REINFORCEMENTLEARNUNGFEATURE_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "cx_rl_interfaces/srv/set_rl_mode.hpp"
#include "cx_rl_interfaces/srv/get_goal_list_robot.hpp"
#include "cx_rl_interfaces/action/get_free_robot.hpp"
#include "cx_rl_interfaces/action/goal_selection.hpp"
#include "cx_rl_interfaces/srv/get_domain_objects.hpp"
#include "cx_rl_interfaces/srv/get_domain_predicates.hpp"
#include "cx_rl_interfaces/srv/create_rl_env_state.hpp"
#include "cx_rl_interfaces/srv/get_goal_list.hpp"
#include "cx_rl_interfaces/srv/reset_cx.hpp"
#include "cx_rl_interfaces/srv/exec_goal_selection.hpp"
#include "std_msgs/msg/string.hpp"

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class ReinforcementLearningPlugin : public ClipsPlugin, public rclcpp::Node {
public:
  ReinforcementLearningPlugin();
  ~ReinforcementLearningPlugin();

  void initialize() override;

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::map<std::string, LockSharedPtr<clips::Environment>> envs_;
  std::thread spin_thread_;
  LockSharedPtr<clips::Environment> clips_env;
  rclcpp::TimerBase::SharedPtr timer_;

  int number_robots_;
  float reset_wait_time_;
  float reset_max_time_;
  float step_wait_time_;
  float step_max_time_;
  float speedup_;


  rclcpp::Service<cx_rl_interfaces::srv::SetRLMode>::SharedPtr set_rl_mode_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetGoalListRobot>::SharedPtr get_goal_list_executable_for_robot_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetGoalList>::SharedPtr get_goal_list_executable_service;
  rclcpp_action::Server<cx_rl_interfaces::action::GetFreeRobot>::SharedPtr get_free_robot_server;
  rclcpp::Service<cx_rl_interfaces::srv::GetDomainObjects>::SharedPtr get_domain_objects_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetDomainPredicates>::SharedPtr get_domain_predicates_service;
  rclcpp::Service<cx_rl_interfaces::srv::CreateRLEnvState>::SharedPtr create_rl_env_state_service;
  rclcpp::Service<cx_rl_interfaces::srv::ResetCX>::SharedPtr reset_cx_service;
  std::vector<rclcpp_action::Server<cx_rl_interfaces::action::GoalSelection>::SharedPtr> goal_selection_action_servers;
  rclcpp::Client<cx_rl_interfaces::srv::ExecGoalSelection>::SharedPtr request_goal_selection_client;


  
  void setRLMode(const std::shared_ptr<cx_rl_interfaces::srv::SetRLMode::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::SetRLMode::Response> response);
  void getGoalListExecutableForRobot(const std::shared_ptr<cx_rl_interfaces::srv::GetGoalListRobot::Request> request, 
                  std::shared_ptr<cx_rl_interfaces::srv::GetGoalListRobot::Response> response);
  void getGoalListExecutable(const std::shared_ptr<cx_rl_interfaces::srv::GetGoalList::Request> request, 
                  std::shared_ptr<cx_rl_interfaces::srv::GetGoalList::Response> response);
  void getDomainObjects(const std::shared_ptr<cx_rl_interfaces::srv::GetDomainObjects::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::GetDomainObjects::Response> response);
  void getDomainPredicates(const std::shared_ptr<cx_rl_interfaces::srv::GetDomainPredicates::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::GetDomainPredicates::Response> response);
  void createRLEnvState(const std::shared_ptr<cx_rl_interfaces::srv::CreateRLEnvState::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::CreateRLEnvState::Response> response);
  void resetCX(const std::shared_ptr<cx_rl_interfaces::srv::ResetCX::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::ResetCX::Response> response);

  rclcpp_action::GoalResponse getFreeRobotHandleGoal(const rclcpp_action::GoalUUID & uuid, 
                  std::shared_ptr<const cx_rl_interfaces::action::GetFreeRobot::Goal> goal);
  rclcpp_action::CancelResponse getFreeRobotHandleCancel(
                  const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GetFreeRobot>> goal_handle);
  void getFreeRobotHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GetFreeRobot>> goal_handle);
  void getFreeRobot(const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GetFreeRobot>> goal_handle);

  rclcpp_action::GoalResponse goalSelectionHandleGoal(const rclcpp_action::GoalUUID & uuid, 
                  std::shared_ptr<const cx_rl_interfaces::action::GoalSelection::Goal> goal);
  rclcpp_action::CancelResponse goalSelectionHandleCancel(
                  const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GoalSelection>> goal_handle);
  void goalSelectionHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GoalSelection>> goal_handle);
  void goalSelection(const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GoalSelection>> goal_handle);

  void request_goal_selection_callback();

  bool exec_in_selection;


  std::vector<std::string> splitActionToGoalParams(std::string action);
  std::string getClipsSlotValuesAsString(clips::CLIPSValue &val);
  std::string createGoalParamString(clips::CLIPSValue &params);
  std::string getClipsEnvStateString();
  std::vector<std::string> getExecutableGoals();
  void execGoalSelection(std::string goal_id);
  bool checkGoalIDForRobot(std::string robot, std::string goalid);
  void assertRLGoalSelectionFact(std::string goal_id);

  std::map<std::string, std::vector<std::string>> paramTypeDomainObjectsMap;
  std::map<std::string, std::vector<std::string>> executableGoalsForRobots;
  std::vector<std::string> executableGoals;

  std::vector<std::string> getDomainObjectsFromCX(std::string type);

  bool in_reset;



};

} // namespace cx
#endif // !CX_FEATURES__REINFORCEMENTLEARNINGFEATURE_HPP_
