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
#include "cx_rl_interfaces/srv/get_free_robot.hpp"
#include "cx_rl_interfaces/action/goal_selection.hpp"
#include "cx_rl_interfaces/srv/get_domain_objects.hpp"
#include "cx_rl_interfaces/srv/get_domain_predicates.hpp"
#include "cx_rl_interfaces/srv/create_rl_env_state.hpp"
#include "cx_rl_interfaces/srv/get_goal_list.hpp"
#include "cx_rl_interfaces/srv/reset_cx.hpp"

#include "cx_core/ClipsFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

namespace cx {

class ReinforcementLearningFeature : public ClipsFeature, public rclcpp::Node {
public:
  ReinforcementLearningFeature();
  ~ReinforcementLearningFeature();

  void initialise(const std::string &feature_name) override;

  bool clips_context_init(const std::string &env_name,
                          LockSharedPtr<CLIPS::Environment> &clips) override;
  bool clips_context_destroyed(const std::string &env_name) override;

  std::string getFeatureName() const;

private:
  std::map<std::string, LockSharedPtr<CLIPS::Environment>> envs_;
  std::thread spin_thread_;
  LockSharedPtr<CLIPS::Environment> clips_env;

  rclcpp::Service<cx_rl_interfaces::srv::SetRLMode>::SharedPtr set_rl_mode_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetGoalListRobot>::SharedPtr get_goal_list_executable_for_robot_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetGoalList>::SharedPtr get_goal_list_executable_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetFreeRobot>::SharedPtr get_free_robot_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetDomainObjects>::SharedPtr get_domain_objects_service;
  rclcpp::Service<cx_rl_interfaces::srv::GetDomainPredicates>::SharedPtr get_domain_predicates_service;
  rclcpp::Service<cx_rl_interfaces::srv::CreateRLEnvState>::SharedPtr create_rl_env_state_service;
  rclcpp::Service<cx_rl_interfaces::srv::ResetCX>::SharedPtr reset_cx_service;
  std::vector<rclcpp_action::Server<cx_rl_interfaces::action::GoalSelection>::SharedPtr> goal_selection_action_servers;


  
  void setRLMode(const std::shared_ptr<cx_rl_interfaces::srv::SetRLMode::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::SetRLMode::Response> response);
  void getGoalListExecutableForRobot(const std::shared_ptr<cx_rl_interfaces::srv::GetGoalListRobot::Request> request, 
                  std::shared_ptr<cx_rl_interfaces::srv::GetGoalListRobot::Response> response);
  void getGoalListExecutable(const std::shared_ptr<cx_rl_interfaces::srv::GetGoalList::Request> request, 
                  std::shared_ptr<cx_rl_interfaces::srv::GetGoalList::Response> response);
  void getFreeRobot(const std::shared_ptr<cx_rl_interfaces::srv::GetFreeRobot::Request> request, 
                  std::shared_ptr<cx_rl_interfaces::srv::GetFreeRobot::Response> response);
  void getDomainObjects(const std::shared_ptr<cx_rl_interfaces::srv::GetDomainObjects::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::GetDomainObjects::Response> response);
  void getDomainPredicates(const std::shared_ptr<cx_rl_interfaces::srv::GetDomainPredicates::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::GetDomainPredicates::Response> response);
  void createRLEnvState(const std::shared_ptr<cx_rl_interfaces::srv::CreateRLEnvState::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::CreateRLEnvState::Response> response);
  void resetCX(const std::shared_ptr<cx_rl_interfaces::srv::ResetCX::Request> request,
                  std::shared_ptr<cx_rl_interfaces::srv::ResetCX::Response> response);

  rclcpp_action::GoalResponse goalSelectionHandleGoal(const rclcpp_action::GoalUUID & uuid, 
                  std::shared_ptr<const cx_rl_interfaces::action::GoalSelection::Goal> goal);
  rclcpp_action::CancelResponse goalSelectionHandleCancel(
                  const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GoalSelection>> goal_handle);
  void goalSelectionHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GoalSelection>> goal_handle);
  void goalSelection(const std::shared_ptr<rclcpp_action::ServerGoalHandle<cx_rl_interfaces::action::GoalSelection>> goal_handle);


  std::vector<std::string> splitActionToGoalParams(std::string action);
  std::string getClipsSlotValuesAsString(std::vector<CLIPS::Value> slot_values);
  std::string createGoalParamString(std::vector<CLIPS::Value> params);
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