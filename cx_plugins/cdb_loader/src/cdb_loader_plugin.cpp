// TODO when restoring the rules the salience has to be taken from the field and not the pp since they could point to a defglobal that has changed in the meantime
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

#include <pqxx/pqxx>
#undef RANGES
// RANGES is defined in clips_ns/clips.h, which causes issues with
// pqxx/pqxx. It needs to be included before clips_ns/clips.h to avoid compilation errors.

#include <clips_ns/clips.h>

#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>
#include <nlohmann/json_fwd.hpp>

#include "cx_cdb_loader_plugin/cdb_loader_plugin.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx
{

CDBLoaderPlugin::CDBLoaderPlugin() {}

CDBLoaderPlugin::~CDBLoaderPlugin() {}

void CDBLoaderPlugin::initialize()
{
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = parent_.lock();
  if (!node) {
    return;
  }

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".hostname", rclcpp::ParameterValue("localhost"));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port", rclcpp::ParameterValue(5432));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".username", rclcpp::ParameterValue("anonymous"));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".password", rclcpp::ParameterValue(std::string()));
}

bool CDBLoaderPlugin::clips_env_init(std::shared_ptr<clips::Environment> & env)
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = parent_.lock();
  std::string hostname;
  int port;
  std::string username;
  std::string password;
  node->get_parameter(plugin_name_ + ".hostname", hostname);
  node->get_parameter(plugin_name_ + ".port", port);
  node->get_parameter(plugin_name_ + ".username", username);
  node->get_parameter(plugin_name_ + ".password", password);

  time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y_%m_%dt%H_%M_%S");

  std::string env_name = CLIPSEnvContext::get_context(env)->env_name_;

  std::string db_name = "cdb_" + env_name + "_" + oss.str();

  try {
    // RCLCPP_INFO(
    //   *logger_, "Connecting to database %s at %s:%d with user %s", config.db_name.c_str(),
    //   config.hostname.c_str(), config.port, config.username.c_str());
    // db_handlers_.emplace(env_name, std::make_unique<DBHandler>(config, parent_));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(*logger_, "Failed to connect to database: %s", e.what());
    return false;
  }
  return true;
}

bool CDBLoaderPlugin::clips_env_destroyed(std::shared_ptr<clips::Environment> & env)
{
  CLIPSEnvContext * context = CLIPSEnvContext::get_context(env.get());
  RCLCPP_INFO(
    *logger_, "Destroying CDBLoaderPlugin for environment %s", context->env_name_.c_str());
  return true;
}
}  // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::CDBLoaderPlugin, cx::ClipsPlugin)
