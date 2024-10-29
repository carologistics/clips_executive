
#include <string>
#include <chrono>
#include <functional>

#include "cx_core/ClipsFeature.hpp"
#include "cx_features/ReinforcementLearningFeature.hpp"
#include "cx_utils/LockSharedPtr.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx
{
  using GetGoalList = cx_rl_interfaces::srv::GetGoalList;
  using GetGoalListRobot = cx_rl_interfaces::srv::GetGoalListRobot;
  using GetFreeRobot = cx_rl_interfaces::srv::GetFreeRobot;
  using GetDomainObjects = cx_rl_interfaces::srv::GetDomainObjects;
  using GetDomainPredicates = cx_rl_interfaces::srv::GetDomainPredicates;
  using CreateRLEnvState = cx_rl_interfaces::srv::CreateRLEnvState;
  using GoalSelection = cx_rl_interfaces::action::GoalSelection;
  using ResetCX = cx_rl_interfaces::srv::ResetCX;

  using namespace std::chrono_literals;
  using namespace std::placeholders;

  ReinforcementLearningFeature::ReinforcementLearningFeature() : Node("reinforcement_learning_feature_node") {}
  ReinforcementLearningFeature::~ReinforcementLearningFeature() {}

  std::string ReinforcementLearningFeature::getFeatureName() const
  {
    return clips_feature_name;
  }

  void ReinforcementLearningFeature::initialise(const std::string &feature_name)
  {
    clips_feature_name = feature_name;

    get_goal_list_executable_for_robot_service =
        this->create_service<GetGoalListRobot>("get_goal_list_executable_for_robot", std::bind(&ReinforcementLearningFeature::getGoalListExecutableForRobot, this, _1, _2));

    get_goal_list_executable_service =
        this->create_service<GetGoalList>("get_goal_list_executable", std::bind(&ReinforcementLearningFeature::getGoalListExecutable, this, _1, _2));

    get_free_robot_service =
        this->create_service<GetFreeRobot>("get_free_robot", std::bind(&ReinforcementLearningFeature::getFreeRobot, this, _1, _2));

    get_domain_objects_service =
        this->create_service<GetDomainObjects>("get_domain_objects", std::bind(&ReinforcementLearningFeature::getDomainObjects, this, _1, _2));

    get_domain_predicates_service =
        this->create_service<GetDomainPredicates>("get_domain_predicates", std::bind(&ReinforcementLearningFeature::getDomainPredicates, this, _1, _2));

    create_rl_env_state_service =
        this->create_service<CreateRLEnvState>("create_rl_env_state", std::bind(&ReinforcementLearningFeature::createRLEnvState, this, _1, _2));

    reset_cx_service =
        this->create_service<ResetCX>("reset_cx", std::bind(&ReinforcementLearningFeature::resetCX, this, _1, _2));

    auto number_robots = parameters["number_robots"].as_int();


    for (int i = 0; i < number_robots; i++)
    {
      std::string server_name = "goal_selection_robot" + std::to_string(i + 1);

      goal_selection_action_servers.push_back(rclcpp_action::create_server<GoalSelection>(this, server_name,
                                                                                          std::bind(&ReinforcementLearningFeature::goalSelectionHandleGoal, this, _1, _2),
                                                                                          std::bind(&ReinforcementLearningFeature::goalSelectionHandleCancel, this, _1),
                                                                                          std::bind(&ReinforcementLearningFeature::goalSelectionHandleAccepted, this, _1)));
    }
    in_reset = false;

    spin_thread_ =
        std::thread([this]()
                    { rclcpp::spin(this->get_node_base_interface()); });
  }

  bool ReinforcementLearningFeature::clips_context_init(
      const std::string &env_name, LockSharedPtr<CLIPS::Environment> &clips)
  {
    RCLCPP_INFO(this->get_logger(),
                "Initialising context for feature %s",
                clips_feature_name.c_str());

    envs_[env_name] = clips;
    clips_env = clips;

    RCLCPP_INFO(this->get_logger(), "Initialised context!");
    return true;
  }

  bool ReinforcementLearningFeature::clips_context_destroyed(
      const std::string &env_name)
  {

    RCLCPP_INFO(this->get_logger(), "Destroying clips context!");
    envs_.erase(env_name);

    return true;
  }

  // SERVICE-FUNCTIONS

  void ReinforcementLearningFeature::getGoalListExecutable(
      const std::shared_ptr<GetGoalList::Request> request,
      std::shared_ptr<GetGoalList::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Generating list of executable goals");
    (void)request;
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

    CLIPS::Fact::pointer fact = clips_env->get_facts();
    std::vector<std::string> goal_list;
    std::vector<std::string> goal_ids;

    while (fact)
    {
      std::string fact_name = fact->get_template()->name();
      if (fact_name == "goal")
      {
        std::string mode = getClipsSlotValuesAsString(fact->slot_value("mode"));
        std::string is_executable = getClipsSlotValuesAsString(fact->slot_value("is-executable"));

        if (mode == "FORMULATED" && is_executable == "TRUE")
        {
          std::string goal_class = getClipsSlotValuesAsString(fact->slot_value("class"));
          std::string goal_id = getClipsSlotValuesAsString(fact->slot_value("id"));

          std::string goal_list_entry = goal_class + "#" + goal_id + "#" + createGoalParamString(fact->slot_value("params"));
          goal_list.push_back(goal_list_entry);
          goal_ids.push_back(goal_id);

          RCLCPP_INFO(this->get_logger(), ("Executable goal: " + goal_list_entry).c_str());
        }
      }
      fact = fact->next();
    }
    RCLCPP_INFO(this->get_logger(), "Finished passing all executable goals");

    executableGoals = goal_ids;
    response->goals = goal_list;
  }

  void ReinforcementLearningFeature::getGoalListExecutableForRobot(
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

      std::this_thread::sleep_for(10ms);
      std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

      CLIPS::Fact::pointer fact = clips_env->get_facts();

      while (fact)
      {
        CLIPS::Fact::pointer gm = NULL;
        if (fact->get_template()->name() == "goal")
        {

          std::string goalid = getClipsSlotValuesAsString(fact->slot_value("id"));

          CLIPS::Fact::pointer gm_fact = clips_env->get_facts();

          while (gm_fact)
          {

            if (gm_fact->get_template()->name() == "goal-meta")
            {

              std::string gm_goalid = getClipsSlotValuesAsString(gm_fact->slot_value("goal-id"));

              if (gm_goalid == goalid)
              {
                gm = gm_fact;
                break;
              }
            }
            gm_fact = gm_fact->next();
          }
          std::string assigned_to = getClipsSlotValuesAsString(gm->slot_value("assigned-to"));
          std::string mode = getClipsSlotValuesAsString(fact->slot_value("mode"));
          std::string is_executable = getClipsSlotValuesAsString(fact->slot_value("is-executable"));

          if (mode == "FORMULATED" && is_executable == "TRUE" && assigned_to == robot)
          {
            std::string goal_class = getClipsSlotValuesAsString(fact->slot_value("class"));
            std::string goal_id = getClipsSlotValuesAsString(fact->slot_value("id"));

            std::string goal_list_entry = goal_class + "#" + goal_id + "#" + createGoalParamString(fact->slot_value("params"));
            goal_list.push_back(goal_list_entry);
            goal_ids.push_back(goal_id);

            RCLCPP_INFO(this->get_logger(), ("Executable goal: " + goal_list_entry).c_str());
          }
        }
        fact = fact->next();
      }
    }

    RCLCPP_INFO(this->get_logger(), ("Finished passing all executable goals for " + robot).c_str());
    executableGoalsForRobots[robot] = goal_ids;
    response->goals = goal_list;
  }

  void ReinforcementLearningFeature::getFreeRobot(
      const std::shared_ptr<GetFreeRobot::Request> request,
      std::shared_ptr<GetFreeRobot::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Finding free robot...");
    (void)request;
    std::string freeRobot = "None";

    while (freeRobot == "None")
    {
      std::this_thread::sleep_for(100ms);
      std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

      CLIPS::Fact::pointer fact = clips_env->get_facts();
      std::vector<std::string> free_robots;
      std::vector<CLIPS::Fact::pointer> goal_facts;
      std::vector<CLIPS::Fact::pointer> goal_meta_facts;
      while (fact)
      {
        std::string fact_name = fact->get_template()->name();
        if (fact_name == "wm-fact" && getClipsSlotValuesAsString(fact->slot_value("key")).find("robot-waiting") != std::string::npos)
        {
          std::string key = getClipsSlotValuesAsString(fact->slot_value("key"));
          size_t pos = key.find("#r#");
          free_robots.push_back(key.substr(pos + 3));
        }
        else if (fact_name == "goal")
        {
          goal_facts.push_back(fact);
        }
        else if (fact_name == "goal-meta")
        {
          goal_meta_facts.push_back(fact);
        }
        fact = fact->next();
      }

      for (std::string r : free_robots)
      {
        for (CLIPS::Fact::pointer g : goal_facts)
        {
          std::string goalid = getClipsSlotValuesAsString(g->slot_value("id"));
          std::string mode = getClipsSlotValuesAsString(g->slot_value("mode"));
          std::string is_executable = getClipsSlotValuesAsString(g->slot_value("is-executable"));
          if (mode == "FORMULATED" && is_executable == "TRUE")
          {
            for (CLIPS::Fact::pointer gm : goal_meta_facts)
            {
              std::string gm_goalid = getClipsSlotValuesAsString(gm->slot_value("goal-id"));
              std::string assigned_to = getClipsSlotValuesAsString(gm->slot_value("assigned-to"));
              if (gm_goalid == goalid && assigned_to == r)
              {
                freeRobot = r;
                RCLCPP_INFO(this->get_logger(), ("Free robot: " + freeRobot).c_str());
                return;
              }
            }
          }
        }
      }
    }

    response->robot = freeRobot;
  }

  void ReinforcementLearningFeature::getDomainObjects(
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

  void ReinforcementLearningFeature::getDomainPredicates(
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
    CLIPS::Fact::pointer fact = clips_env->get_facts();

    while (fact)
    {
      std::string fact_name = fact->get_template()->name();

      if (fact_name == "domain-predicate")
      {
        std::string name = getClipsSlotValuesAsString(fact->slot_value("name"));

        std::vector<CLIPS::Value> param_names = fact->slot_value("param-names");
        std::vector<CLIPS::Value> param_types = fact->slot_value("param-types");

        predicateNames.push_back(name);
        paramCounts.push_back(param_names.size());

        for (size_t i = 0; i < param_names.size(); i++)
        {
          paramNames.push_back(param_names[i].as_string());
          paramTypes.push_back(param_types[i].as_string());
        }
      }
      fact = fact->next();
    }

    RCLCPP_INFO(this->get_logger(), "Retrieved domain predicates");
    response->predicatenames = predicateNames;
    response->paramcounts = paramCounts;
    response->paramnames = paramNames;
    response->paramtypes = paramTypes;
  }

  void ReinforcementLearningFeature::createRLEnvState(
      const std::shared_ptr<CreateRLEnvState::Request> request,
      std::shared_ptr<CreateRLEnvState::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Creating environment state...");
    (void)request;
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
    CLIPS::Fact::pointer fact = clips_env->get_facts();
    std::string envStateString = "{";

    while (fact)
    {
      std::string fact_name = fact->get_template()->name();
      if (fact_name == "domain-fact")
      {
        std::vector<std::string> slot_names = fact->slot_names();
        std::string fact_value = "";
        for (std::string s : slot_names)
        {
          fact_value += " Slot " + s + ": ";
          std::vector<CLIPS::Value> slot_values = fact->slot_value(s);
          std::string value = getClipsSlotValuesAsString(slot_values);

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
      }
      fact = fact->next();
    }
    envStateString = envStateString.substr(0, envStateString.length() - 1) + "}";
    RCLCPP_INFO(this->get_logger(), "Environment state created...");
    response->state = envStateString;
  }

  void ReinforcementLearningFeature::resetCX(
      const std::shared_ptr<ResetCX::Request> request,
      std::shared_ptr<ResetCX::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Resetting environment...");
    (void)request;
    in_reset = true;
    clips_env->assert_fact("(reset-game (stage STAGE-0) (stage-time (time)) )");

    
    double speedup = parameters["speedup"].as_double();
    double max_time = parameters["reset_max_time"].as_double();
    double wait_time = parameters["reset_wait_time"].as_double();
    
    if (speedup != 0.0 || speedup != 1.0)
    {
      wait_time = wait_time / speedup;
    }

    bool env_feedback = false;
    int elapsed_time = 0;
    std::string result = "Reset timed out";

    while (!env_feedback && elapsed_time < max_time)
    {
      std::this_thread::sleep_for(wait_time * 1000ms);
      std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
      CLIPS::Fact::pointer fact = clips_env->get_facts();

      while (fact)
      {
        std::string fact_name = fact->get_template()->name();
        if (fact_name == "reset-game-finished")
        {
          RCLCPP_INFO(this->get_logger(), "Reset completed!");
          fact->retract();
          env_feedback = true;
          result = "Reset completed";
          break;
        }

        fact = fact->next();
      }
      elapsed_time += wait_time;
    }
    in_reset = false ;
    response->confirmation = result;
  }

  // ACTION-FUNCTIONS

  // TODO
  rclcpp_action::GoalResponse ReinforcementLearningFeature::goalSelectionHandleGoal(
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

  // TODO
  rclcpp_action::CancelResponse ReinforcementLearningFeature::goalSelectionHandleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalSelection>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // TODO
  void ReinforcementLearningFeature::goalSelectionHandleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GoalSelection>> goal_handle)
  {

    std::thread{std::bind(&cx::ReinforcementLearningFeature::goalSelection, this, _1), goal_handle}.detach();
  }

  // TODO
  void ReinforcementLearningFeature::goalSelection(
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
    
    
    double speedup = parameters["speedup"].as_double();
    double max_time = parameters["step_max_time"].as_double();
    double wait_time = parameters["step_wait_time"].as_double();
    

    if (speedup != 0.0 || speedup != 1.0)
    {
      wait_time = wait_time / speedup;
    }

    bool env_feedback = false;
    int elapsed_time = 0;
    bool check_for_episode_end = false;

    while (!env_feedback && elapsed_time < max_time * 1000)
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

      CLIPS::Fact::pointer fact = clips_env->get_facts();

      while (fact)
      {
        std::string fact_name = fact->get_template()->name();
        if (fact_name == "rl-finished-goal" && getClipsSlotValuesAsString(fact->slot_value("goal-id")) == goal_id)
        {
          std::string outcome = getClipsSlotValuesAsString(fact->slot_value("outcome"));
          int reward = std::stoi(getClipsSlotValuesAsString(fact->slot_value("reward")));

          result->outcome = outcome;
          result->reward = reward;
          result->info = "";
          env_feedback = true;
          fact->retract();
          RCLCPP_INFO(this->get_logger(), ("rl-finished-goal found for goal " + goal_id).c_str());
          break;
        } else if (check_for_episode_end && fact_name == "rl-episode-end") 
        {
          result->outcome = "Failed";
          result->reward = 0;
          result->info = "Episode end";
          RCLCPP_INFO(this->get_logger(), "Episode end");
          env_feedback = true;
          break;
        }

        fact = fact->next();
      }
      elapsed_time += wait_time;
      
      if (parameters["domain_based_reset"].as_bool()){
        check_for_episode_end = true;
      }
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
  ReinforcementLearningFeature::assertRLGoalSelectionFact(std::string goal_id)
  {
    RCLCPP_INFO(this->get_logger(), ("Asserting goal selection fact for goal " + goal_id).c_str());
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));

    CLIPS::Value v = CLIPS::Value(goal_id, CLIPS::TYPE_SYMBOL);
    CLIPS::Template::pointer tmpl = clips_env->get_template("rl-goal-selection");
  
    CLIPS::Fact::pointer fact = CLIPS::Fact::create(*(clips_env.get_obj()), tmpl);
    fact->set_slot("next-goal-id", v);
    clips_env->assert_fact(fact);
  }

  std::vector<std::string>
  ReinforcementLearningFeature::splitActionToGoalParams(std::string action)
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
  ReinforcementLearningFeature::getClipsSlotValuesAsString(std::vector<CLIPS::Value> slot_values)
  {
    std::string value = "";
    for (std::size_t i = 0; i < slot_values.size(); i++)
    {
      auto v = slot_values[i];
      switch (v.type())
      {
      case CLIPS::TYPE_FLOAT:
        value += std::to_string(v.as_float());
        break;

      case CLIPS::TYPE_INTEGER:
        value += std::to_string(v.as_integer());
        break;

      default:
        value += v.as_string();
      }
      if (slot_values.size() > 1 && i != (slot_values.size() - 1))
      {
        value += "#";
      }
    }
    return value;
  }

  std::string ReinforcementLearningFeature::createGoalParamString(std::vector<CLIPS::Value> params)
  {
    std::string goal_param_string = "";
    for (std::size_t i = 0; i + 1 < params.size(); i += 2)
    {
      std::vector<CLIPS::Value> v_name = {params[i]};
      std::vector<CLIPS::Value> v_value = {params[i + 1]};
      std::string p_name = getClipsSlotValuesAsString(v_name);
      std::string p_value = getClipsSlotValuesAsString(v_value);
      goal_param_string += (p_name + "|" + p_value + "#");
    }

    return goal_param_string;
  }

  std::vector<std::string> ReinforcementLearningFeature::getDomainObjectsFromCX(std::string type)
  {
    RCLCPP_INFO(this->get_logger(), "Retrieving domain objects from CX...");
    std::lock_guard<std::mutex> guard(*(clips_env.get_mutex_instance()));
    std::vector<std::string> domainObjects;
    CLIPS::Fact::pointer fact = clips_env->get_facts();

    while (fact)
    {
      std::string fact_name = fact->get_template()->name();
      if (fact_name == "domain-object")
      {
        std::string obj_type = getClipsSlotValuesAsString(fact->slot_value("type"));
        if (obj_type == type)
        {
          std::string obj_name = getClipsSlotValuesAsString(fact->slot_value("name"));
          domainObjects.push_back(obj_name);
        }
      }
      fact = fact->next();
    }
    RCLCPP_INFO(this->get_logger(), "Domain objects retrieved from CX");
    return domainObjects;
  }

  bool ReinforcementLearningFeature::checkGoalIDForRobot(std::string robot, std::string goalid)
  {
    auto robot_goals = executableGoalsForRobots[robot];
    return std::find(robot_goals.begin(), robot_goals.end(), goalid) != robot_goals.end();
  }

} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::ReinforcementLearningFeature, cx::ClipsFeature)
