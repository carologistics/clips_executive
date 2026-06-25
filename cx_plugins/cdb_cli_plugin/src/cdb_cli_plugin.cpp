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

// TODO Offer inteface to interact with the env
// TODO Offer interface to lock and release the mutex
// TODO Offer interface to querry to allow edits from form to facts or rules

#include "cdb_cli_plugin/cdb_cli_plugin.hpp"

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace cx
{
CDBCliPlugin::CDBCliPlugin() {}
CDBCliPlugin::~CDBCliPlugin() {}

void CDBCliPlugin::initialize()
{
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = parent_.lock();

  using namespace std::placeholders;
  RCLCPP_INFO(*logger_, "HIER BIN ICH");
  eval_service_ = node->create_service<cx_msgs::srv::EvalClips>(
    "/eval", std::bind(&CDBCliPlugin::eval_clips, this, _1, _2, _3));
  RCLCPP_INFO(*logger_, "HA BIN ICH");
}

bool CDBCliPlugin::clips_env_init(std::shared_ptr<clips::Environment> & env) { return true; }
bool CDBCliPlugin::clips_env_destroyed(std::shared_ptr<clips::Environment> & env) { return false; }

void CDBCliPlugin::eval_clips(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<cx_msgs::srv::EvalClips::Request> request,
  const std::shared_ptr<cx_msgs::srv::EvalClips::Response> response)
{
  (void)request_header;
  response.get()->output = "";
}

}  // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::CDBCliPlugin, cx::ClipsPlugin)
