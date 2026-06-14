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

#ifndef CX_UTILS__CLIPS_ENV_CONTEXT_HPP_
#define CX_UTILS__CLIPS_ENV_CONTEXT_HPP_

#include <memory>
#include <string>

#include "clips_ns/clips.h"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

namespace cx
{
class CLIPSLogger
{
public:
  explicit CLIPSLogger(const char * component, bool log_to_file, bool stdout_to_debug);
  ~CLIPSLogger();
  void log(const char * logical_name, const char * str);
  using PublishFn = std::function<void(const std::string &, const std::string &)>;

  void set_topic_publisher(PublishFn fn);

  void set_topic_logging(bool enabled);

  bool get_topic_logging();

private:
  char * component_;
  const rclcpp::Logger logger_;
  const bool stdout_to_debug_;
  std::shared_ptr<spdlog::logger> clips_logger_;
  std::string buffer_;
  std::string terminal_buffer_;

  PublishFn publish_fn_ = nullptr;
  bool topic_logging_enabled_ = false;
};

class CLIPSEnvContext
{
public:
  CLIPSEnvContext(const std::string & env_name, bool log_to_file, bool stdout_to_debug)
  : env_name_(env_name), logger_(env_name.c_str(), log_to_file, stdout_to_debug)
  {
  }
  std::string env_name_;
  std::mutex env_mtx_;
  CLIPSLogger logger_;

  static CLIPSEnvContext * get_context(clips::Environment * env);
  static CLIPSEnvContext * get_context(std::shared_ptr<clips::Environment> & env);
};

}  // namespace cx

#endif  // CX_UTILS__CLIPS_ENV_CONTEXT_HPP_
