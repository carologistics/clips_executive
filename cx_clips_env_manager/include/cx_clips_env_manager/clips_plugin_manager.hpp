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

#ifndef CX_PLUGINS_CLIPS_PLUGIN_MANAGER_HPP_
#define CX_PLUGINS_CLIPS_PLUGIN_MANAGER_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "cx_plugin/clips_plugin.hpp"

#include "cx_msgs/srv/list_clips_plugin.hpp"
#include "cx_msgs/srv/load_clips_plugin.hpp"
#include "cx_msgs/srv/unload_clips_plugin.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"

namespace cx {
using EnvsMap =
    std::unordered_map<std::string, cx::LockSharedPtr<clips::Environment>>;

class ClipsPluginManager {
public:
  ClipsPluginManager();
  ~ClipsPluginManager();

  using PluginsMap = std::unordered_map<std::string, cx::ClipsPlugin::Ptr>;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                 const std::string &name, LockSharedPtr<EnvsMap> &envs);
  /**
   * @brief Cleanup resources
   */
  void cleanup();

  /**
   * @brief Activate plugin manager
   */
  void activate();
  void activate_env(const std::string &env_name,
                    LockSharedPtr<clips::Environment> &env);

  /**
   * @brief Deactivate plugin manager
   */
  void deactivate();
  void deactivate_env(const std::string &env_name,
                      LockSharedPtr<clips::Environment> &env);

  /**
   * @brief Reset plugin manager
   */
  void reset();

  void plugin_init_context(const std::string &env_name,
                           const std::string &plugin_name);

  void clips_request_plugin(clips::Environment *env, clips::UDFValue *out,
                            const std::string &plugin_name);

  void load_plugin_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Request> request,
      const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Response> response);
  void unload_plugin_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Request> request,
      const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Response>
          response);
  void list_plugin_cb(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Request> request,
      const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Response> response);

private:
  bool load_plugin_for_env(const std::string &plugin,
                           const std::string &env_name,
                           LockSharedPtr<clips::Environment> &env);

  rclcpp::Service<cx_msgs::srv::LoadCLIPSPlugin>::SharedPtr
      load_plugin_service_;
  rclcpp::Service<cx_msgs::srv::UnloadCLIPSPlugin>::SharedPtr
      unload_plugin_service_;
  rclcpp::Service<cx_msgs::srv::ListCLIPSPlugin>::SharedPtr
      list_plugin_service_;

  // Pluginlib class loaders
  pluginlib::ClassLoader<cx::ClipsPlugin> pg_loader_;
  std::vector<std::string> plugin_ids_;

  LockSharedPtr<EnvsMap> envs_;

  std::unordered_map<std::string, std::vector<std::string>> loaded_plugins_;

  PluginsMap plugins_;

  std::string name_;

  rclcpp::Logger logger_{rclcpp::get_logger("CLIPSPluginManager")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
};

} // namespace cx

#endif // !CX_PLUGINS_CLIPS_PLUGIN_MANAGER_HPP_
