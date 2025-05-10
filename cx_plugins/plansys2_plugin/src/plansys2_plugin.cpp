// Copyright (c) 2024-2025 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "cx_plansys2_plugin/plansys2_plugin.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include <cx_utils/clips_env_context.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using namespace std::placeholders;
namespace cx {

Plansys2Plugin::Plansys2Plugin() {}

Plansys2Plugin::~Plansys2Plugin() {}

bool Plansys2Plugin::clips_env_init(LockSharedPtr<clips::Environment> &env) {
  RCLCPP_DEBUG(rclcpp::get_logger(plugin_name_),
               "Initializing context for plugin %s", plugin_name_.c_str());

  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  std::string env_name = context->env_name_;
  domain_client_[env_name] = std::make_unique<plansys2::DomainExpertClient>();
  problem_client_[env_name] = std::make_unique<plansys2::ProblemExpertClient>();
  planner_client_[env_name] = std::make_unique<plansys2::PlannerClient>();

  clips::AddUDF(
      env.get_obj().get(), "psys2-add-domain-instance", "v", 2, 2, ";sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Plugin *>(udfc->context);
        clips::UDFValue name, type;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &name);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &type);

        instance->add_problem_instance(env, name.lexemeValue->contents,
                                       type.lexemeValue->contents);
      },
      "add_problem_instance", this);

  clips::AddUDF(
      env.get_obj().get(), "psys2-add-domain-predicate", "v", 1, 1, ";sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Plugin *>(udfc->context);
        clips::UDFValue predicate;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &predicate);
        instance->add_problem_predicate(env, predicate.lexemeValue->contents);
      },
      "add_problem_predicate", this);

  clips::AddUDF(
      env.get_obj().get(), "psys2-clear-knowledge", "v", 0, 0, "",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Plugin *>(udfc->context);
        instance->clear_problem_expert_knowledge(env);
      },
      "clear_problem_expert_knowledge", this);

  clips::AddUDF(
      env.get_obj().get(), "psys2-call-planner", "v", 3, 3, ";sy;sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        auto *instance = static_cast<Plansys2Plugin *>(udfc->context);
        clips::UDFValue plan;
        clips::UDFValue domain;
        clips::UDFValue problem;
        using namespace clips;
        clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &plan);
        clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &domain);
        clips::UDFNthArgument(udfc, 3, LEXEME_BITS, &problem);
        instance->plan_with_plansys2(env, plan.lexemeValue->contents,
                                     domain.lexemeValue->contents,
                                     problem.lexemeValue->contents);
      },
      "plan_with_plansys2", this);

  RCLCPP_INFO(rclcpp::get_logger(plugin_name_), "Initialised context!");
  return true;
}

bool Plansys2Plugin::clips_env_destroyed(
    LockSharedPtr<clips::Environment> &env) {

  RCLCPP_INFO(rclcpp::get_logger(plugin_name_), "Destroying clips context!");
  clips::RemoveUDF(env.get_obj().get(), "psys2-add-domain-instance");
  clips::RemoveUDF(env.get_obj().get(), "psys2-add-domain-predicate");
  clips::RemoveUDF(env.get_obj().get(), "psys2-clear-knowledge");
  clips::RemoveUDF(env.get_obj().get(), "psys2-call-planner");
  auto context = CLIPSEnvContext::get_context(env.get_obj().get());
  domain_client_.erase(context->env_name_);
  problem_client_.erase(context->env_name_);
  planner_client_.erase(context->env_name_);
  return true;
}

void Plansys2Plugin::add_problem_instance(clips::Environment *env,
                                          const std::string &name,
                                          const std::string &type) {
  const std::string log_name = "PSYS2CLIPS";

  RCLCPP_INFO(rclcpp::get_logger(log_name),
              "Trying to add instance: %s - %s to problem expert", name.c_str(),
              type.c_str());
  auto context = CLIPSEnvContext::get_context(env);
  bool success = problem_client_[context->env_name_]->addInstance(
      plansys2::Instance(name, type));
  if (success) {
    RCLCPP_INFO(rclcpp::get_logger(log_name),
                "Successfully added instance: %s - %s to problem expert",
                name.c_str(), type.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Problem adding instance: %s - %s to problem expert",
                 name.c_str(), type.c_str());
  }
}

void Plansys2Plugin::add_problem_predicate(clips::Environment *env,
                                           const std::string &pred) {
  const std::string log_name = "PSYS2CLIPS";
  RCLCPP_INFO(rclcpp::get_logger(log_name),
              "Trying to add predicate: %s to problem expert", pred.c_str());
  auto context = CLIPSEnvContext::get_context(env);
  bool success = problem_client_[context->env_name_]->addPredicate(
      plansys2::Predicate(pred));
  if (success) {
    RCLCPP_INFO(rclcpp::get_logger(log_name),
                "Successfully added predicate: %s to problem expert",
                pred.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Problem adding predicate: %s to problem expert",
                 pred.c_str());
  }
}

void Plansys2Plugin::clear_problem_expert_knowledge(clips::Environment *env) {
  const std::string log_name = "PSYS2CLIPS";
  RCLCPP_INFO(rclcpp::get_logger(log_name),
              "Clearing problem expert knowledge!");
  auto context = CLIPSEnvContext::get_context(env);
  problem_client_[context->env_name_]->clearKnowledge();
}

void Plansys2Plugin::plan_with_plansys2(clips::Environment *env,
                                        const std::string &goal_id,
                                        const std::string &goal,
                                        const std::string &plan_id) {
  const std::string log_name = "PSYS2CLIPS";
  RCLCPP_INFO(rclcpp::get_logger(log_name), "plan_with_plansys2 called! id: %s",
              plan_id.c_str());

  call_planner(env, goal_id, goal, plan_id);
}

void Plansys2Plugin::call_planner(clips::Environment *env,
                                  const std::string &goal_id,
                                  const std::string &goal,
                                  const std::string &plan_id) {
  const std::string log_name = "PSYS2CLIPS";

  auto context = CLIPSEnvContext::get_context(env);
  if (!problem_client_[context->env_name_]->setGoal(plansys2::Goal(goal))) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Couldn't set goal '%s' with id %s for problem expert",
                 goal.c_str(), goal_id.c_str());
    return;
  }
  auto domain = domain_client_[context->env_name_]->getDomain();
  auto problem = problem_client_[context->env_name_]->getProblem();

  RCLCPP_ERROR(rclcpp::get_logger(log_name),
               "Calling plan for goal '%s' with id %s", goal.c_str(),
               goal_id.c_str());
  auto plan = planner_client_[context->env_name_]->getPlan(domain, problem);
  // std::lock_guard<std::mutex> guard(*(clips.get_mutex_instance()));

  if (!plan.has_value()) {
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "Could not find plan to reach goal '%s' with id %s",
                 goal.c_str(), goal_id.c_str());
    RCLCPP_ERROR(rclcpp::get_logger(log_name),
                 "(pddl-plan-feedback (status PLAN-FAILED) (plan-id %s))",
                 plan_id.c_str());
    clips::AssertString(
        env,
        std::format("(pddl-plan-feedback (status PLAN-FAILED) (plan-id {}))",
                    plan_id)
            .c_str());
    return;
  }

  const std::string space_delimiter = " ";
  unsigned int id = 1;

  clips::AssertString(
      env, std::format("(plan (id {}) (goal-id {}) (type SEQUENTIAL))", plan_id,
                       goal_id)
               .c_str());
  for (const auto &plan_item : plan.value().items) {
    if (plan_item.action != "") {
      const std::string &action = plan_item.action;
      auto pos = action.find(space_delimiter);
      const std::string action_name = action.substr(1, pos - 1);
      const std::string param_values =
          action.substr(pos + space_delimiter.length(), action.length() - 2);

      RCLCPP_INFO(rclcpp::get_logger(log_name),
                  "(plan-action (id %d) (goal-id %s) (plan-id %s) "
                  "(action-name %s) (param-values %s))",
                  id, goal_id.c_str(), plan_id.c_str(), action_name.c_str(),
                  param_values.c_str());

      clips::AssertString(
          env, std::format("(plan-action (id {}) (goal-id {}) (plan-id {}) "
                           "(action-name {}) (param-values {}))",
                           id, goal_id, plan_id, action_name, param_values)
                   .c_str());
      id++;
    }
  }
  clips::AssertString(
      env,
      std::format("(pddl-plan-feedback (status PLANNED) (plan-id {}))", plan_id)
          .c_str());
}

} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::Plansys2Plugin, cx::ClipsPlugin)
