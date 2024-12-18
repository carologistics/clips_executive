#include <string>
#include <chrono>
#include <functional>
#include <memory>

#include "cx_reinforcement_learning_plugin/reinforcement_learning_plugin.hpp"
#include "cx_utils/lock_shared_ptr.hpp"
#include "cx_utils/clips_env_context.hpp"
#include <cx_utils/param_utils.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx
{
  using SetRLMode = cx_rl_interfaces::srv::SetRLMode;
  using GetGoalList = cx_rl_interfaces::srv::GetGoalList;
  using GetGoalListRobot = cx_rl_interfaces::srv::GetGoalListRobot;
  using GetFreeRobot = cx_rl_interfaces::action::GetFreeRobot;
  using GetDomainObjects = cx_rl_interfaces::srv::GetDomainObjects;
  using GetDomainPredicates = cx_rl_interfaces::srv::GetDomainPredicates;
  using CreateRLEnvState = cx_rl_interfaces::srv::CreateRLEnvState;
  using GoalSelection = cx_rl_interfaces::action::GoalSelection;
  //using ResetCX = cx_rl_interfaces::srv::ResetCX;
  using ExecGoalSelection = cx_rl_interfaces::srv::ExecGoalSelection;

  using namespace std::chrono_literals;
  using namespace std::placeholders;

  ReinforcementLearningPlugin::ReinforcementLearningPlugin() : Node("reinforcement_learning_feature_node") {}
  ReinforcementLearningPlugin::~ReinforcementLearningPlugin() {}

  void ReinforcementLearningPlugin::initialize()
  {
    RCLCPP_INFO(this->get_logger(), "ReinforcementLearningPlugin init");

    set_rl_mode_service =
        this->create_service<SetRLMode>("set_rl_mode", std::bind(&ReinforcementLearningPlugin::setRLMode, this, _1, _2));

    get_goal_list_executable_for_robot_service =
        this->create_service<GetGoalListRobot>("get_goal_list_executable_for_robot", std::bind(&ReinforcementLearningPlugin::getGoalListExecutableForRobot, this, _1, _2));

    get_goal_list_executable_service =
        this->create_service<GetGoalList>("get_goal_list_executable", std::bind(&ReinforcementLearningPlugin::getGoalListExecutable, this, _1, _2));

    get_domain_objects_service =
        this->create_service<GetDomainObjects>("get_domain_objects", std::bind(&ReinforcementLearningPlugin::getDomainObjects, this, _1, _2));

    get_domain_predicates_service =
        this->create_service<GetDomainPredicates>("get_domain_predicates", std::bind(&ReinforcementLearningPlugin::getDomainPredicates, this, _1, _2));

    create_rl_env_state_service =
        this->create_service<CreateRLEnvState>("create_rl_env_state", std::bind(&ReinforcementLearningPlugin::createRLEnvState, this, _1, _2));

    //reset_cx_service =
    //    this->create_service<ResetCX>("reset_cx", std::bind(&ReinforcementLearningPlugin::resetCX, this, _1, _2));

    get_free_robot_server = rclcpp_action::create_server<GetFreeRobot>(this, "get_free_robot",
                                                                                          std::bind(&ReinforcementLearningPlugin::getFreeRobotHandleGoal, this, _1, _2),
                                                                                          std::bind(&ReinforcementLearningPlugin::getFreeRobotHandleCancel, this, _1),
                                                                                          std::bind(&ReinforcementLearningPlugin::getFreeRobotHandleAccepted, this, _1));

    request_goal_selection_client = 
        this->create_client<ExecGoalSelection>("/request_goal_selection");

    timer_ = this->create_wall_timer(1000ms, std::bind(&ReinforcementLearningPlugin::request_goal_selection_callback, this));

    exec_in_selection = false;

    auto node = parent_.lock();

    cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".number_robots",
      rclcpp::ParameterValue(0));
    cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".reset_wait_time",
      rclcpp::ParameterValue(1.0));
    cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".reset_max_time",
      rclcpp::ParameterValue(1.0));
    cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".step_wait_time",
      rclcpp::ParameterValue(1.0));
    cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".step_max_time",
      rclcpp::ParameterValue(1.0));
    cx::cx_utils::declare_parameter_if_not_declared(
      node, plugin_name_ + ".speedup",
      rclcpp::ParameterValue(1.0));

    node->get_parameter(plugin_name_ + ".number_robots", number_robots_);
    node->get_parameter(plugin_name_ + ".reset_wait_time", reset_wait_time_);
    node->get_parameter(plugin_name_ + ".reset_max_time", reset_max_time_);
    node->get_parameter(plugin_name_ + ".step_wait_time", step_wait_time_);
    node->get_parameter(plugin_name_ + ".step_max_time", step_max_time_);
    node->get_parameter(plugin_name_ + ".speedup", speedup_);


    for (int i = 0; i < number_robots_; i++)
    {
      std::string server_name = "goal_selection_robot" + std::to_string(i + 1);

      goal_selection_action_servers.push_back(rclcpp_action::create_server<GoalSelection>(this, server_name,
                                                                                          std::bind(&ReinforcementLearningPlugin::goalSelectionHandleGoal, this, _1, _2),
                                                                                          std::bind(&ReinforcementLearningPlugin::goalSelectionHandleCancel, this, _1),
                                                                                          std::bind(&ReinforcementLearningPlugin::goalSelectionHandleAccepted, this, _1)));
    }
    in_reset = false;

    spin_thread_ =
        std::thread([this]()
                    { rclcpp::spin(this->get_node_base_interface()); });

    RCLCPP_INFO(this->get_logger(), "ReinforcementLearningPlugin initialized!");
  }

  bool ReinforcementLearningPlugin::clips_env_init(LockSharedPtr<clips::Environment> &env)
  {
    RCLCPP_INFO(this->get_logger(),
                "Initialising context for feature %s",
                plugin_name_.c_str());
    auto context = CLIPSEnvContext::get_context(env.get_obj().get());

    envs_[context->env_name_] = context->env_lock_ptr_;
    clips_env = context->env_lock_ptr_;

    RCLCPP_INFO(this->get_logger(), "Initialized context!");
    return true;
  }

  bool ReinforcementLearningPlugin::clips_env_destroyed(LockSharedPtr<clips::Environment> &env)
  {
    auto context = CLIPSEnvContext::get_context(env.get_obj().get());

    RCLCPP_INFO(this->get_logger(), "Destroying clips context!");
    envs_.erase(context->env_name_);

    return true;
  }

  // SERVICE-FUNCTIONS

  void ReinforcementLearningPlugin::setRLMode(
      const std::shared_ptr<SetRLMode::Request> request,
      std::shared_ptr<SetRLMode::Response> response)
  {
    std::string mode = request->mode;
    RCLCPP_INFO(this->get_logger(), ("Setting reinforcement learning mode to " + mode).c_str());
    
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
    
	auto theFB = clips::CreateFactBuilder(clips_env.get_obj().get(),"rl-mode");
    clips::FBPutSlotSymbol(theFB,"mode", mode.c_str());
	clips::FBAssert(theFB);
	clips::FBDispose(theFB);

    std::string result = "Set mode to " + mode;
    response->confirmation = result;
  }

  void ReinforcementLearningPlugin::getGoalListExecutable(
      const std::shared_ptr<GetGoalList::Request> request,
      std::shared_ptr<GetGoalList::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Generating list of executable goals");
    (void)request;
    std::vector<std::string> goal_list = getExecutableGoals();
    response->goals = goal_list;
  }

  void ReinforcementLearningPlugin::getGoalListExecutableForRobot(
      const std::shared_ptr<GetGoalListRobot::Request> request,
      std::shared_ptr<GetGoalListRobot::Response> response)
  {
    std::string robot = request->robot;

    RCLCPP_INFO(this->get_logger(), ("Generating list of executable goals for " + robot).c_str());

    bool goalsExecutable = false;
    std::vector<std::string> goal_list;
    std::vector<std::string> goal_ids;

    while (!goalsExecutable)
    {
      RCLCPP_INFO(this->get_logger(), ("Searching for all executable goals for " + robot).c_str());
      std::this_thread::sleep_for(10ms);
      std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

      clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "goal");
      clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);

      while (fact)
      {
        clips::CLIPSValue out;
        clips::GetFactSlot(fact,"id", &out);
        std::string goalid = out.lexemeValue->contents;
        clips::GetFactSlot(fact,"assigned-to", &out);
        std::string assigned_to = out.lexemeValue->contents;
        clips::GetFactSlot(fact,"mode", &out);
        std::string mode = out.lexemeValue->contents;
        clips::GetFactSlot(fact,"is-executable", &out);
        std::string is_executable = out.lexemeValue->contents;

        if (mode == "FORMULATED" && is_executable == "TRUE" && assigned_to == robot)
        {
        clips::GetFactSlot(fact,"class", &out);
          std::string goal_class = out.lexemeValue->contents;
        clips::GetFactSlot(fact,"id", &out);
          std::string goal_id = out.lexemeValue->contents;

        clips::GetFactSlot(fact,"params", &out);
          std::string goal_list_entry = goal_class + "#" + goal_id + "#" + createGoalParamString(out);
          goal_list.push_back(goal_list_entry);
          goal_ids.push_back(goal_id);
          goalsExecutable = true;
          RCLCPP_INFO(this->get_logger(), ("Executable goal: " + goal_list_entry).c_str());
        }
        fact = clips::GetNextFactInTemplate(tmpl, fact);
      }
    }
    RCLCPP_INFO(this->get_logger(), ("Finished passing all executable goals for " + robot).c_str());
    executableGoals = goal_ids;
    executableGoalsForRobots[robot] = goal_ids;
    response->goals = goal_list;
  }



  void ReinforcementLearningPlugin::getDomainObjects(
      const std::shared_ptr<GetDomainObjects::Request> request,
      std::shared_ptr<GetDomainObjects::Response> response)
  {
    std::string type = request->type;
    RCLCPP_INFO(this->get_logger(), ("Retrieving Domain objects of type " + type).c_str());
    if (paramTypeDomainObjectsMap.find(type) != paramTypeDomainObjectsMap.end())
    {
      response->objects = paramTypeDomainObjectsMap[type];
    }
    else
    {
      auto domainObjects = getDomainObjectsFromCX(type);
      if (domainObjects.size() > 0)
      {
        paramTypeDomainObjectsMap.insert(std::pair<std::string, std::vector<std::string>>(type, domainObjects));
        response->objects = domainObjects;
      }
      else
      {
        domainObjects.push_back("Not found");
        response->objects = domainObjects;
      }
    }
  }

  void ReinforcementLearningPlugin::getDomainPredicates(
      const std::shared_ptr<GetDomainPredicates::Request> request,
      std::shared_ptr<GetDomainPredicates::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Retrieving domain predicates...");
    (void)request;
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
    std::vector<std::string> predicateNames;
    std::vector<uint8_t> paramCounts;
    std::vector<std::string> paramNames;
    std::vector<std::string> paramTypes;
    clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "domain-predicate");
    clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);

    while (fact)
    {
        clips::CLIPSValue name_val;
        clips::GetFactSlot(fact, "name",&name_val);
        std::string name = getClipsSlotValuesAsString(name_val);

        predicateNames.push_back(name);
        clips::CLIPSValue param_names;
        clips::GetFactSlot(fact, "param-names",&param_names);
        paramCounts.push_back(param_names.multifieldValue->length);
        clips::CLIPSValue param_types;
        clips::GetFactSlot(fact, "param-types",&param_types);

        for (size_t i = 0; i < param_names.multifieldValue->length; i++)
        {
          paramNames.push_back(getClipsSlotValuesAsString(param_names.multifieldValue->contents[i]));
          paramTypes.push_back(getClipsSlotValuesAsString(param_types.multifieldValue->contents[i]));
        }
      fact = clips::GetNextFactInTemplate(tmpl, fact);
    }

    RCLCPP_INFO(this->get_logger(), "Retrieved domain predicates");
    response->predicatenames = predicateNames;
    response->paramcounts = paramCounts;
    response->paramnames = paramNames;
    response->paramtypes = paramTypes;
  }

  void ReinforcementLearningPlugin::createRLEnvState(
      const std::shared_ptr<CreateRLEnvState::Request> request,
      std::shared_ptr<CreateRLEnvState::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Creating environment state...");
    (void)request;
    std::string envStateString = getClipsEnvStateString();
    RCLCPP_INFO(this->get_logger(), "Environment state created...");
    response->state = envStateString;
  }

// TODO: same here, this is not an instantaneous request, this is probably better implemented as an action
/*  
  void ReinforcementLearningPlugin::resetCX(
      const std::shared_ptr<ResetCX::Request> request,
      std::shared_ptr<ResetCX::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Resetting environment...");
    (void)request;
    in_reset = true;
	clips::AssertString(clips_env.get_obj().get(), "(reset-game (stage STAGE-0))");

    auto node = parent_.lock();
    
    float wait_time = reset_wait_time_;

    if (speedup_ != 0.0 || speedup_ != 1.0)
    {
      wait_time = reset_wait_time_ / speedup_;
    }

    bool env_feedback = false;
    int elapsed_time = 0;
    std::string result = "Reset timed out";

    while (!env_feedback && elapsed_time < reset_max_time_)
    {
      std::this_thread::sleep_for(wait_time * 1000ms);
      std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
      clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "reset-game-finished");
      clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);

      if(fact)
      {
        RCLCPP_INFO(this->get_logger(), "Reset completed!");
        clips::Retract(fact);
        env_feedback = true;
        result = "Reset completed";
      }
      elapsed_time += wait_time;
    }
    in_reset = false ;
    response->confirmation = result;
  }
*/
  void
  ReinforcementLearningPlugin::request_goal_selection_callback()
  {
    if (exec_in_selection)
    {
      return;
    }
    bool clips_request_found = false;
    std::unique_lock<std::mutex> guard(*(clips_env.get_mutex_instance()));

    clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "rl-goal-selection-requested");
    clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);

    if(fact)
    {
        RCLCPP_INFO(this->get_logger(), "Goal selection request found");
        clips_request_found = true;
        exec_in_selection = true;
    }
    guard.unlock();
    if (clips_request_found)
    {
      auto request = std::make_shared<ExecGoalSelection::Request>();
      request->state = getClipsEnvStateString();
      request->goals = getExecutableGoals();

      while (!request_goal_selection_client->wait_for_service(1s))
      {
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
      }
      auto result = request_goal_selection_client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        execGoalSelection(result.get()->goalid);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service request_goal_selection");
      }
    }
  }

  // ACTION-FUNCTIONS

  rclcpp_action::GoalResponse ReinforcementLearningPlugin::getFreeRobotHandleGoal(
      const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GetFreeRobot::Goal> goal)
  {
    (void)goal;
    (void)uuid;
    if (in_reset)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ReinforcementLearningPlugin::getFreeRobotHandleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetFreeRobot>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ReinforcementLearningPlugin::getFreeRobotHandleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetFreeRobot>> goal_handle)
  {

    std::thread{std::bind(&cx::ReinforcementLearningPlugin::getFreeRobot, this, _1), goal_handle}.detach();
  }
  
// TODO: not so great, services are supposed to respond quickly. this might take some time
  void ReinforcementLearningPlugin::getFreeRobot(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GetFreeRobot>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Finding free robot...");
    auto feedback = std::make_shared<GetFreeRobot::Feedback>();
    auto result = std::make_shared<GetFreeRobot::Result>();
    std::string freeRobot = "None";

    while (freeRobot == "None")
    {
      std::this_thread::sleep_for(500ms);
      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), "get_free_robot canceled!");
        result->robot = "Canceled";
        goal_handle->canceled(result);
        return;
      }
      std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

      std::vector<std::string> free_robots;
      std::vector<clips::Fact *> goal_facts;
      clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "domain-fact");
      clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);
      while (fact)
      {
          clips::CLIPSValue out;
          clips::GetFactSlot(fact,"name", &out);
        if (getClipsSlotValuesAsString(out).find("robot-waiting") != std::string::npos)
        {
          clips::GetFactSlot(fact,"param-values", &out);
          std::string values = getClipsSlotValuesAsString(out);
          free_robots.push_back(values);
        }
        fact = clips::GetNextFactInTemplate(tmpl, fact);
      }
      if(!free_robots.empty()) {
        tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "goal");
        clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);
        while (fact)
        {
          goal_facts.push_back(fact);
          fact = clips::GetNextFactInTemplate(tmpl, fact);
        }

        for (std::string r : free_robots)
        {
          for (clips::Fact *g : goal_facts)
          {
            clips::CLIPSValue out;
            clips::GetFactSlot(g,"id", &out);
            std::string goalid = getClipsSlotValuesAsString(out);
            clips::GetFactSlot(g,"mode", &out);
            std::string mode = getClipsSlotValuesAsString(out);
            clips::GetFactSlot(g,"is-executable", &out);
            std::string is_executable = getClipsSlotValuesAsString(out);
            clips::GetFactSlot(g,"assigned-to", &out);
            std::string assigned_to = getClipsSlotValuesAsString(out);
            if (mode == "FORMULATED" && is_executable == "TRUE" && assigned_to == r)
            {
              freeRobot = r;
              RCLCPP_INFO(this->get_logger(), ("Free robot: " + freeRobot).c_str());
              result->robot = freeRobot;
              goal_handle->succeed(result);
              return;

            }
          }
        }
      }
      feedback->feedback = "No free robot found, retrying...";
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "No free robot found, retrying");
    }
  }

  rclcpp_action::GoalResponse ReinforcementLearningPlugin::goalSelectionHandleGoal(
      const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GoalSelection::Goal> goal)
  {
    (void)goal;
    (void)uuid;
    if (in_reset)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse ReinforcementLearningPlugin::goalSelectionHandleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalSelection>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void ReinforcementLearningPlugin::goalSelectionHandleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalSelection>> goal_handle)
  {

    std::thread{std::bind(&cx::ReinforcementLearningPlugin::goalSelection, this, _1), goal_handle}.detach();
  }

  void ReinforcementLearningPlugin::goalSelection(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalSelection>> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    std::string goal_id = goal->goalid;
    RCLCPP_INFO(this->get_logger(), ("Selecting goal " + goal_id).c_str());
    auto feedback = std::make_shared<GoalSelection::Feedback>();
    auto result = std::make_shared<GoalSelection::Result>();
    result->goalid = goal_id;

    if (std::find(executableGoals.begin(), executableGoals.end(), goal_id) == executableGoals.end())
    {
      RCLCPP_INFO(this->get_logger(), ("Goal " + goal_id + " not executable!").c_str());
      result->outcome = "Failed";
      result->reward = 0;
      result->info = "Not executable";
      goal_handle->abort(result);
      return;
    }

    assertRLGoalSelectionFact(goal_id);
    feedback->feedback = "Goal selection fact asserted";
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), ("Selection fact for goal " + goal_id + " asserted").c_str());
    
	auto node = parent_.lock();
    
    float wait_time = step_wait_time_;
    if (speedup_ != 0.0 || speedup_ != 1.0)
    {
      wait_time = step_wait_time_ / speedup_;
    }

    bool env_feedback = false;
    int elapsed_time = 0;

    while (!env_feedback && elapsed_time < step_max_time_ * 1000)
    {
      std::this_thread::sleep_for(wait_time * 1000ms);

      if (goal_handle->is_canceling())
      {
        RCLCPP_INFO(this->get_logger(), ("Goal " + goal_id + " canceled!").c_str());
        result->outcome = "Failed";
        result->reward = 0;
        result->info = "Canceled";
        goal_handle->canceled(result);
        return;
      }

      std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

      feedback->feedback = "Waiting for goal " + goal_id + " to finish";
      goal_handle->publish_feedback(feedback);

      clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "rl-finished-goal");
      clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);
      while (fact)
      {
        clips::CLIPSValue out;
        clips::GetFactSlot(fact,"goal-id", &out);
        if (getClipsSlotValuesAsString(out) == goal_id)
        {
          clips::GetFactSlot(fact,"outcome", &out);
          std::string outcome = getClipsSlotValuesAsString(out);
          clips::GetFactSlot(fact,"reward", &out);
          int reward = std::stoi(getClipsSlotValuesAsString(out));
          clips::GetFactSlot(fact,"done", &out);
          std::string done = getClipsSlotValuesAsString(out);

          result->outcome = outcome;
          result->reward = reward;
          result->info = "";
          if (done == "TRUE"){
            result->info = "Done";
          }
          env_feedback = true;
          auto retract_fact = fact;
          fact = clips::GetNextFactInTemplate(tmpl, fact);
          clips::Retract(retract_fact);
          RCLCPP_INFO(this->get_logger(), ("rl-finished-goal found for goal " + goal_id).c_str());
          break;
        }
        fact = clips::GetNextFactInTemplate(tmpl, fact);
      }
      elapsed_time += wait_time;
    }

    if (!env_feedback){
      RCLCPP_INFO(this->get_logger(), ("Goal " + goal_id + " timed out!").c_str());
      result->outcome = "Failed";
      result->reward = 0;
      result->info = "Timed out";
      goal_handle->abort(result);
      return;
    }

    if (rclcpp::ok()){
      RCLCPP_INFO(this->get_logger(), ("Goal " + goal_id + " finished").c_str());
      goal_handle->succeed(result);
    }
  }

  // HELPER-FUNCTIONS

  void
  ReinforcementLearningPlugin::assertRLGoalSelectionFact(std::string goal_id)
  {
    RCLCPP_INFO(this->get_logger(), ("Asserting goal selection fact for goal " + goal_id).c_str());
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

    auto theFB = clips::CreateFactBuilder(clips_env.get_obj().get(),"rl-goal-selection");
    clips::FBPutSlotSymbol(theFB,"goalid", goal_id.c_str());
    clips::FBAssert(theFB);
    clips::FBDispose(theFB);
  }

  std::vector<std::string>
  ReinforcementLearningPlugin::splitActionToGoalParams(std::string action)
  {
    std::vector<std::string> g_splitted;
    std::string::size_type begin = 0;
    for (std::string::size_type end = 0; (end = action.find("#", end)) != std::string::npos; ++end)
    {
      g_splitted.push_back(action.substr(begin, end - begin));
      begin = end + 1;
    }
    return g_splitted;
  }

  std::string
  ReinforcementLearningPlugin::getClipsSlotValuesAsString(clips::CLIPSValue &slot_values)
  {
    std::string value = "";
	if(slot_values.header->type == MULTIFIELD_TYPE) {
    for (std::size_t i = 0; i < slot_values.multifieldValue->length; i++)
    {
      auto v = slot_values.multifieldValue->contents[i];
      switch (v.header->type)
      {
       case FLOAT_TYPE:
        value += std::to_string(v.floatValue->contents);
        break;

      case INTEGER_TYPE:
        value += std::to_string(v.integerValue->contents);
        break;

      default:
        value += v.lexemeValue->contents;
      }
      if (slot_values.multifieldValue->length > 1 && i != (slot_values.multifieldValue->length - 1))
      {
        value += "#";
      }
    }
    } else {
      switch (slot_values.header->type)
      {
       case FLOAT_TYPE:
        value += std::to_string(slot_values.floatValue->contents);
        break;

      case INTEGER_TYPE:
        value += std::to_string(slot_values.integerValue->contents);
        break;

      default:
        value += slot_values.lexemeValue->contents;
      }
    }
    return value;
  }

  std::string ReinforcementLearningPlugin::createGoalParamString(clips::CLIPSValue &params)
  {
    std::string goal_param_string = "";
    size_t length = params.multifieldValue->length;
    for (std::size_t i = 0; i + 1 < length; i += 2)
    {
      // TODO: properly type-cast
      std::string p_name = getClipsSlotValuesAsString(params.multifieldValue->contents[i]);
      std::string p_value = getClipsSlotValuesAsString(params.multifieldValue->contents[i+1]);
      goal_param_string += (p_name + "|" + p_value + "#");
    }

    return goal_param_string;
  }

  std::string ReinforcementLearningPlugin::getClipsEnvStateString()
  {
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
    clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "domain-fact");
    clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);
    std::string envStateString = "{";
    clips::CLIPSValue out;
    std::vector<std::string> slot_names;
    if(fact) {
      clips::FactSlotNames(fact, &out);
      for(size_t i = 0; i < out.multifieldValue->length; i ++) {
         slot_names.push_back(out.multifieldValue->contents[i].lexemeValue->contents);
      }
    }

    while (fact)
    {
      std::string fact_value = "";
      for (std::string s : slot_names)
      {
        fact_value += " Slot " + s + ": ";
        clips::GetFactSlot(fact,"goal-id", &out);
        std::string value = getClipsSlotValuesAsString(out);

        if (s == "name")
        {
          envStateString += "\"" + value + "(";
        }
        if (s == "param-values")
        {
          envStateString += value + ")\",";
        }
        fact_value += " " + value;
      }
      fact = clips::GetNextFactInTemplate(tmpl, fact);
    }
    envStateString = envStateString.substr(0, envStateString.length() - 1) + "}";
    return envStateString;
  }

  std::vector<std::string> ReinforcementLearningPlugin::getExecutableGoals()
  {
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

    clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "goal");
    clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);
    clips::CLIPSValue out;
    std::vector<std::string> goal_list;
    std::vector<std::string> goal_ids;

    while (fact)
    {
      clips::GetFactSlot(fact,"mode", &out);
      std::string mode = getClipsSlotValuesAsString(out);
      clips::GetFactSlot(fact,"is-executable", &out);
      std::string is_executable = getClipsSlotValuesAsString(out);

      if (mode == "FORMULATED" && is_executable == "TRUE")
      {
        clips::GetFactSlot(fact,"class", &out);
        std::string goal_class = getClipsSlotValuesAsString(out);
        clips::GetFactSlot(fact,"id", &out);
        std::string goal_id = getClipsSlotValuesAsString(out);

        clips::GetFactSlot(fact,"params", &out);
        std::string goal_list_entry = goal_class + "#" + goal_id + "#" + createGoalParamString(out);
        goal_list.push_back(goal_list_entry);
        goal_ids.push_back(goal_id);

        RCLCPP_INFO(this->get_logger(), ("Executable goal: " + goal_list_entry).c_str());
      }
      fact = clips::GetNextFactInTemplate(tmpl, fact);
    }
    RCLCPP_INFO(this->get_logger(), "Finished passing all executable goals");

    executableGoals = goal_ids;
    return goal_list;
  }

  void ReinforcementLearningPlugin::execGoalSelection(std::string goal_id)
  {
    RCLCPP_INFO(this->get_logger(), ("Selecting goal " + goal_id).c_str());

    if (std::find(executableGoals.begin(), executableGoals.end(), goal_id) == executableGoals.end())
    {
      RCLCPP_INFO(this->get_logger(), ("Goal " + goal_id + " not executable!").c_str());
      return;
    }
    assertRLGoalSelectionFact(goal_id);
    std::this_thread::sleep_for(500ms);
    exec_in_selection = false;
    RCLCPP_INFO(this->get_logger(), ("Selection fact for goal " + goal_id + " asserted").c_str());
  }

  std::vector<std::string> ReinforcementLearningPlugin::getDomainObjectsFromCX(std::string type)
  {
    RCLCPP_INFO(this->get_logger(), "Retrieving domain objects from CX...");
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
    clips::Deftemplate *tmpl = clips::FindDeftemplate(clips_env.get_obj().get(), "domain-object");
    clips::Fact *fact = clips::GetNextFactInTemplate(tmpl, NULL);
    clips::CLIPSValue out;
    std::vector<std::string> domainObjects;

    while (fact)
    {
      clips::GetFactSlot(fact,"type", &out);
      std::string obj_type = getClipsSlotValuesAsString(out);
      if (obj_type == type)
      {
        clips::GetFactSlot(fact,"name", &out);
        std::string obj_name = getClipsSlotValuesAsString(out);
        domainObjects.push_back(obj_name);
      }
      fact = clips::GetNextFactInTemplate(tmpl, fact);
    }
    RCLCPP_INFO(this->get_logger(), "Domain objects retrieved from CX");
    return domainObjects;
  }

  bool ReinforcementLearningPlugin::checkGoalIDForRobot(std::string robot, std::string goalid)
  {
    auto robot_goals = executableGoalsForRobots[robot];
    return std::find(robot_goals.begin(), robot_goals.end(), goalid) != robot_goals.end();
  }

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::ReinforcementLearningPlugin, cx::ClipsPlugin)
