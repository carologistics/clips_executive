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

#ifndef CX_PLUGINS__AMENT_INDEX_PLUGIN_HPP_
#define CX_PLUGINS__AMENT_INDEX_PLUGIN_HPP_

#include <string>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/lock_shared_ptr.hpp"

namespace cx {

class AmentIndexPlugin : public ClipsPlugin {
public:
  AmentIndexPlugin();
  ~AmentIndexPlugin();

  void initialize();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::unique_ptr<rclcpp::Logger> logger_;
  std::unordered_set<std::string> function_names_;
};
} // namespace cx

#endif // !CX_PLUGINS__AMENT_INDEX_PLUGIN_HPP_
