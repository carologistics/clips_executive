// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include <map>
#include <memory>
#include <ranges>
#include <string>
#include <utility>

#include "cx_clips_env_manager/clips_plugin_manager.hpp"

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/param_utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::placeholders;

namespace cx {

ClipsPluginManager::ClipsPluginManager()
    : pg_loader_("cx_plugin", "cx::ClipsPlugin") {}

ClipsPluginManager::~ClipsPluginManager() {}

void ClipsPluginManager::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &name, LockSharedPtr<EnvsMap> &envs) {
  parent_ = parent;
  name_ = name;
  envs_ = envs;
  auto node = parent_.lock();
  if (node) {
    RCLCPP_INFO(logger_, "Configuring [%s]...", name_.c_str());

    load_plugin_service_ = node->create_service<cx_msgs::srv::LoadCLIPSPlugin>(
        std::format("{}/load_plugin", name_).c_str(),
        std::bind(&ClipsPluginManager::load_plugin_cb, this, _1, _2, _3));
    unload_plugin_service_ =
        node->create_service<cx_msgs::srv::UnloadCLIPSPlugin>(
            std::format("{}/unload_plugin", name_).c_str(),
            std::bind(&ClipsPluginManager::unload_plugin_cb, this, _1, _2, _3));
    list_plugin_service_ = node->create_service<cx_msgs::srv::ListCLIPSPlugin>(
        std::format("{}/list_plugin", name_).c_str(),
        std::bind(&ClipsPluginManager::list_plugin_cb, this, _1, _2, _3));
  } else {
    RCLCPP_ERROR(logger_, "Invalid parent node reference!");
  }
}

void ClipsPluginManager::activate() {
  std::scoped_lock env_lock(*envs_.get_mutex_instance());
  for (auto &env : *envs_.get_obj().get()) {
    activate_env(env.first, env.second);
  }
}

void ClipsPluginManager::activate_env(const std::string &env_name,
                                      LockSharedPtr<clips::Environment> &env) {

  RCLCPP_INFO(logger_, "Loading plugins for %s.", env_name.c_str());
  std::vector<std::string> plugins;
  auto node = parent_.lock();
  cx::cx_utils::declare_parameter_if_not_declared(
      node, env_name + ".plugins", rclcpp::ParameterValue(plugins));
  node->get_parameter(env_name + ".plugins", plugins);
  for (const std::string &plugin : plugins) {
    load_plugin_for_env(plugin, env_name, env);
  }
}

bool ClipsPluginManager::load_plugin_for_env(
    const std::string &plugin, const std::string &env_name,
    LockSharedPtr<clips::Environment> &env) {
  bool success = false;
  auto node = parent_.lock();
  if (plugins_.contains(plugin)) {
    std::scoped_lock lock(*env.get_mutex_instance());
    success = plugins_[plugin]->clips_env_init(env);
  } else {
    std::string plugin_type;
    cx::cx_utils::declare_parameter_if_not_declared(node, plugin + ".plugin",
                                                    rclcpp::ParameterValue(""));
    node->get_parameter(plugin + ".plugin", plugin_type);
    cx::ClipsPlugin::Ptr plugin_instance =
        pg_loader_.createUniqueInstance(plugin_type);

    plugin_instance->initialize(parent_, plugin);

    RCLCPP_INFO(logger_, "Created plugin: %s of type %s on activation",
                plugin.c_str(), plugin_type.c_str());
    // Insert loaded plugin to the plugins map
    plugins_.insert({plugin, std::move(plugin_instance)});
    std::scoped_lock lock(*env.get_mutex_instance());
    success = plugins_[plugin]->clips_env_init(env);
  }
  if (success) {
    loaded_plugins_[env_name].push_back(plugin);
  }
  return success;
}

void ClipsPluginManager::deactivate() {
  {
    std::scoped_lock env_lock(*envs_.get_mutex_instance());
    for (auto &env : *envs_.get_obj().get()) {
      deactivate_env(env.first, env.second);
    }
  }
  std::vector<std::string> loaded_plugin_types;
  std::string plugin_plugin;
  for (const auto &f : plugins_) {
    auto node = parent_.lock();
    node->get_parameter(f.first + ".plugin", plugin_plugin);
    loaded_plugin_types.push_back(plugin_plugin);
    f.second->finalize();
  }
  plugins_.clear();
  for (const auto &f2 : loaded_plugin_types) {
    pg_loader_.unloadLibraryForClass(f2);
  }
  load_plugin_service_.reset();
  unload_plugin_service_.reset();
  list_plugin_service_.reset();
}

void ClipsPluginManager::deactivate_env(
    const std::string &env_name, LockSharedPtr<clips::Environment> &env) {
  for (const auto &plugin :
       std::ranges::reverse_view(loaded_plugins_[env_name])) {
    std::scoped_lock env_lock(*env.get_mutex_instance());
    plugins_[plugin]->clips_env_destroyed(env);
    RCLCPP_INFO(logger_, "[%s] Deactivated!", plugin.c_str());
  }
}

void ClipsPluginManager::load_plugin_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Request> request,
    const std::shared_ptr<cx_msgs::srv::LoadCLIPSPlugin::Response> response) {
  (void)request_header; // ignoring request id
  std::string env_name = request->env_name;
  std::string plugin_name = request->plugin_name;
  std::scoped_lock env_lock(*envs_.get_mutex_instance());
  if (envs_->contains(env_name)) {
    LockSharedPtr<clips::Environment> &clips = envs_->at(env_name);
    response->success = load_plugin_for_env(plugin_name, env_name, clips);
    if (!response->success) {
      response->error = "error while loading plugin";
    }
  } else {
    response->success = false;
    response->error = "unknown environment";
    return;
  }
}

void ClipsPluginManager::unload_plugin_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Request> request,
    const std::shared_ptr<cx_msgs::srv::UnloadCLIPSPlugin::Response> response) {
  (void)request_header; // ignoring request id
  std::string env_name = request->env_name;
  std::string plugin_name = request->plugin_name;
  std::scoped_lock env_lock(*envs_.get_mutex_instance());
  if (envs_->contains(env_name)) {
    LockSharedPtr<clips::Environment> &clips = envs_->at(env_name);
    std::scoped_lock lock(*clips.get_mutex_instance());
    if (plugins_.contains(plugin_name)) {
      bool success = plugins_[plugin_name]->clips_env_destroyed(clips);
      response->success = success;
      if (!success) {
        response->error = "error while unloading plugin";
      }
    } else {
      response->success = false;
      response->error = "unknown plugin";
      return;
    }
  } else {
    response->success = false;
    response->error = "unknown environment";
    return;
  }
}

void ClipsPluginManager::list_plugin_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Request> request,
    const std::shared_ptr<cx_msgs::srv::ListCLIPSPlugin::Response> response) {
  (void)request_header; // ignoring request id
  std::string env_name = request->env_name;
  if (env_name == "") {
    response->success = true;
    std::vector<std::string> plugins;
    response->plugins = plugins;
    return;
  }
  std::scoped_lock lock(*envs_.get_mutex_instance());
  if (envs_->contains(env_name)) {
    response->success = true;
    std::vector<std::string> plugins;
    response->plugins = plugins;
    return;
  } else {
    response->success = false;
    response->error = "unknown environment";
    return;
  }
}

} // namespace cx
