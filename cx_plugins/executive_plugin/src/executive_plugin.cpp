// Copyright (c) 2024-2026 Carologistics
// SPDX-License-Identifier: Apache-2.0
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

#include "cx_executive_plugin/executive_plugin.hpp"

#include <string>

#include "cx_utils/clips_env_context.hpp"
#include "cx_utils/param_utils.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

using std::placeholders::_1;
namespace cx
{
ExecutivePlugin::ExecutivePlugin() {}

ExecutivePlugin::~ExecutivePlugin()
{
  if (agenda_refresh_timer_) {
    agenda_refresh_timer_->cancel();
  }
}
void ExecutivePlugin::finalize()
{
  agenda_refresh_timer_->cancel();
  {
    std::scoped_lock<std::mutex> set_guard(envs_mutex_);
    managed_envs_.clear();
  }
  clips_agenda_refresh_pub_.reset();
}

void ExecutivePlugin::initialize()
{
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
  auto node = parent_.lock();
  if (!node) {
    return;
  }

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".publish_on_refresh", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".assert_time", rclcpp::ParameterValue(true));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".refresh_rate", rclcpp::ParameterValue(10));
  node->get_parameter(plugin_name_ + ".publish_on_refresh", publish_on_refresh_);
  node->get_parameter(plugin_name_ + ".assert_time", assert_time_);
  node->get_parameter(plugin_name_ + ".refresh_rate", refresh_rate_);
  if (publish_on_refresh_) {
    clips_agenda_refresh_pub_ = node->create_publisher<std_msgs::msg::Int64>(
      "clips_executive/refresh_agenda", rclcpp::QoS(10));
  }
  double rate = 1.0 / refresh_rate_;
  // Sets the time between each clips agenda refresh in ns
  publish_rate_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(rate));
  RCLCPP_DEBUG(
    *logger_, "Publishing rate set to: %ldns",
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_rate_).count());

  agenda_refresh_timer_ = node->create_wall_timer(publish_rate_, [this]() {
    std::scoped_lock<std::mutex> set_guard(envs_mutex_);
    int64_t total_fired_all_envs = 0;

    for (ManagedEnv & managed : managed_envs_) {
      auto context = CLIPSEnvContext::get_context(managed.env);
      std::scoped_lock<std::mutex> env_guard(context->env_mtx_);
      const auto & valid_stack = managed.focus_stack;
      clips::Fact * time_fact;
      if (assert_time_) {
        clips::SetCurrentModule(managed.env.get(), clips::FindDefmodule(managed.env.get(), "MAIN"));
        time_fact = clips::AssertString(managed.env.get(), "(time (now))");
        clips::RetainFact(time_fact);
      }

      int64_t total_fired_this_env = 0;
      int64_t fired_this_iteration = 0;

      do {
        fired_this_iteration = 0;
        clips::RefreshAllAgendas(managed.env.get());

        if (valid_stack.empty()) {
          clips::SetCurrentModule(
            managed.env.get(), clips::FindDefmodule(managed.env.get(), "MAIN"));
          fired_this_iteration += clips::Run(managed.env.get(), managed.rule_limit);
        } else {
          for (const auto & module_name : valid_stack) {
            auto mod = clips::FindDefmodule(managed.env.get(), module_name.c_str());
            clips::Focus(mod);
            fired_this_iteration += clips::Run(managed.env.get(), -1);
          }
          clips::SetCurrentModule(
            managed.env.get(), clips::FindDefmodule(managed.env.get(), "MAIN"));
        }

        if (assert_time_) {
          ReleaseFact(time_fact);
          Retract(time_fact);
        }

        total_fired_this_env += fired_this_iteration;
        if (managed.rule_limit > 0 && total_fired_this_env >= managed.rule_limit) {
          RCLCPP_ERROR(
            *logger_, "Env '%s': Rule fire limit of %ld was reached.", context->env_name_.c_str(),
            managed.rule_limit);
          break;
        }
      } while (fired_this_iteration > 0);

      total_fired_all_envs += total_fired_this_env;

      clips::SetCurrentModule(managed.env.get(), clips::FindDefmodule(managed.env.get(), "MAIN"));
    }

    if (publish_on_refresh_) {
      std_msgs::msg::Int64 msg;
      msg.data = total_fired_all_envs;
      clips_agenda_refresh_pub_->publish(msg);
    }
  });
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> ExecutivePlugin::get_node()
{
  return parent_.lock();
}

bool ExecutivePlugin::clips_env_init(std::shared_ptr<clips::Environment> & env)
{
  if (publish_on_refresh_) {
    clips_agenda_refresh_pub_->on_activate();
  }
  {
    clips::AddUDF(
      env.get(), "now", "d", 0, 0, NULL,
      [](clips::Environment * env, clips::UDFContext * udfc, clips::UDFValue * out) {
        ExecutivePlugin * instance = static_cast<ExecutivePlugin *>(udfc->context);
        double currentTime = instance->get_node()->get_clock()->now().seconds();
        out->floatValue = clips::CreateFloat(env, currentTime);
      },
      "clips_now", this);

    clips::AddUDF(
      env.get(), "now-systime", "d", 0, 0, NULL,
      [](clips::Environment * env, clips::UDFContext * /*udfc*/, clips::UDFValue * out) {
        auto now = std::chrono::time_point_cast<std::chrono::duration<double>>(
          std::chrono::system_clock::now());

        out->floatValue = clips::CreateFloat(env, now.time_since_epoch().count());
      },
      "clips_now_systime", NULL);
  }
  auto node = parent_.lock();
  if (!node) {
    return false;
  }

  if (assert_time_) {
    clips::Fact * time_fact = clips::AssertString(env.get(), "(time (now))");
    clips::Eval(env.get(), "(unwatch facts time)", nullptr);
    clips::Retract(time_fact);
  }
  auto context = CLIPSEnvContext::get_context(env);
  std::string env_param_prefix = context->env_name_;
  cx::cx_utils::declare_parameter_if_not_declared(
    node, env_param_prefix + ".focus_stack",
    rclcpp::ParameterValue(std::vector<std::string>{"MAIN"}));
  std::vector<std::string> focus_stack_param;
  node->get_parameter(env_param_prefix + ".focus_stack", focus_stack_param);
  cx::cx_utils::declare_parameter_if_not_declared(
    node, env_param_prefix + ".rule_limit", rclcpp::ParameterValue(-1));
  int64_t rule_limit;
  node->get_parameter(env_param_prefix + ".rule_limit", rule_limit);

  // validate focus stack for this env
  std::vector<std::string> valid_stack;
  for (const auto & module_name : focus_stack_param) {
    if (clips::FindDefmodule(env.get(), module_name.c_str()) != nullptr) {
      valid_stack.push_back(module_name);
    } else {
      RCLCPP_ERROR(
        *logger_, "Focus stack entry '%s' is not a valid module, removing from stack.",
        module_name.c_str());
    }
  }
  {
    std::scoped_lock envs_guard(envs_mutex_);
    managed_envs_.push_back({env, valid_stack, rule_limit});
  }
  return true;
}

bool ExecutivePlugin::clips_env_destroyed(std::shared_ptr<clips::Environment> & env)
{
  {
    clips::RemoveUDF(env.get(), "now");
    clips::RemoveUDF(env.get(), "now-systime");

    clips::Defrule * curr_rule = clips::FindDefrule(env.get(), "time-retract");
    if (curr_rule) {
      clips::Undefrule(curr_rule, env.get());
    }
    clips::Defglobal * curr_glob = clips::FindDefglobal(env.get(), "*PRIORITY-TIME-RETRACT*");
    if (curr_glob) {
      clips::Undefglobal(curr_glob, env.get());
    }
  }

  {
    std::scoped_lock envs_guard(envs_mutex_);
    managed_envs_.erase(
      std::remove_if(
        managed_envs_.begin(), managed_envs_.end(),
        [&env](const ManagedEnv & m) { return m.env.get() == env.get(); }),
      managed_envs_.end());
  }

  return true;
}
}  // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::ExecutivePlugin, cx::ClipsPlugin)
