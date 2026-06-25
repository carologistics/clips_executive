// Copyright (c) 2026 Carologistics
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

#pragma once

#include <cx_msgs/srv/eval_clips.hpp>

#include "cx_msgs/srv/eval_clips.hpp"
#include "cx_plugin/clips_plugin.hpp"

namespace cx
{

class CDBCliPlugin : public ClipsPlugin
{
public:
  CDBCliPlugin();
  ~CDBCliPlugin();

  void initialize() override;

  bool clips_env_init(std::shared_ptr<clips::Environment> & env) override;
  bool clips_env_destroyed(std::shared_ptr<clips::Environment> & env) override;

  void eval_clips(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::EvalClips::Request> request,
    const std::shared_ptr<cx_msgs::srv::EvalClips::Response> response);

private:
  rclcpp::Service<cx_msgs::srv::EvalClips>::SharedPtr eval_service_;
  std::unique_ptr<rclcpp::Logger> logger_;
};

}  // namespace cx
