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

#include <chrono>
#include <format>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/logging.hpp"

#include "cx_clips_env_manager/clips_env_manager.hpp"

#include "cx_utils/clips_env_context.hpp"
#include "cx_utils/param_utils.hpp"

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace cx {
constexpr const char *ROUTER_NAME = "cxlog";

static bool log_router_query(clips::Environment * /*env*/,
                             const char *logical_name, void * /*context*/) {
  // envs
  if (strcmp(logical_name, "l") == 0)
    return true;
  if (strcmp(logical_name, "t") == 0)
    return true;
  if (strcmp(logical_name, "red") == 0)
    return true;
  if (strcmp(logical_name, "green") == 0)
    return true;
  if (strcmp(logical_name, "blue") == 0)
    return true;
  if (strcmp(logical_name, "yellow") == 0)
    return true;
  if (strcmp(logical_name, "magenta") == 0)
    return true;
  if (strcmp(logical_name, "cyan") == 0)
    return true;
  if (strcmp(logical_name, "white") == 0)
    return true;
  if (strcmp(logical_name, "bold") == 0)
    return true;
  if (strcmp(logical_name, "info") == 0)
    return true;
  if (strcmp(logical_name, "debug") == 0)
    return true;
  if (strcmp(logical_name, "warn") == 0)
    return true;
  if (strcmp(logical_name, "error") == 0)
    return true;
  if (strcmp(logical_name, "loginfo") == 0)
    return true;
  if (strcmp(logical_name, "logdebug") == 0)
    return true;
  if (strcmp(logical_name, "logwarn") == 0)
    return true;
  if (strcmp(logical_name, "logerror") == 0)
    return true;
  if (strcmp(logical_name, "stdout") == 0)
    return true;
  if (strcmp(logical_name, clips::STDOUT) == 0)
    return true;
  if (strcmp(logical_name, clips::STDWRN) == 0)
    return true;
  if (strcmp(logical_name, clips::STDERR) == 0)
    return true;
  return false;
}

static void log_router_print(clips::Environment * /*env*/,
                             const char *logical_name, const char *str,
                             void *context) {
  CLIPSLogger *logger = static_cast<CLIPSLogger *>(context);
  logger->log(logical_name, str);
}

static void log_router_exit(clips::Environment * /*env*/, int /*exit_code*/,
                            void * /*context*/) {
  // no particular handling of a closed router necessary
}

using namespace std::placeholders;

CLIPSEnvManager::CLIPSEnvManager(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("clips_manager", options) {
  envs_.init_mutex();
  RCLCPP_INFO(get_logger(), "Initialising [%s]...", get_name());
  // server side never times out from lifecycle manager
  declare_parameter(bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM,
                    true);
  set_parameter(rclcpp::Parameter(
      bond::msg::Constants::DISABLE_HEARTBEAT_TIMEOUT_PARAM, true));

  cx::cx_utils::declare_parameter_if_not_declared(this, "bond_heartbeat_period",
                                                  rclcpp::ParameterValue(0.0));
  get_parameter("bond_heartbeat_period", bond_heartbeat_period);

  bool autostart_node = false;
  cx::cx_utils::declare_parameter_if_not_declared(
      this, "autostart_node", rclcpp::ParameterValue(false));
  get_parameter("autostart_node", autostart_node);
  if (autostart_node) {
    autostart();
  }
  register_rcl_preshutdown_callback();
}

CLIPSEnvManager::~CLIPSEnvManager() {
  {
    std::scoped_lock envs_lock(*(envs_.get_mutex_instance()));
    envs_.get_obj()->clear();
  }
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn CLIPSEnvManager::on_configure(const rclcpp_lifecycle::State &) {

  RCLCPP_INFO(get_logger(), "Configuring [%s]...", get_name());

  create_env_service_ = create_service<cx_msgs::srv::CreateClipsEnv>(
      std::string(get_name()) + "/create_env",
      std::bind(&CLIPSEnvManager::create_env_callback, this, _1, _2, _3));

  destroy_env_service_ = create_service<cx_msgs::srv::DestroyClipsEnv>(
      std::string(get_name()) + "/destroy_env",
      std::bind(&CLIPSEnvManager::destroy_env_callback, this, _1, _2, _3));

  std::shared_ptr<EnvsMap> envs = std::make_shared<EnvsMap>();

  auto node = shared_from_this();
  std::vector<std::string> config_envs;
  cx::cx_utils::declare_parameter_if_not_declared(
      node, "environments", rclcpp::ParameterValue(config_envs));
  get_parameter("environments", config_envs);
  for (const auto &env_name : config_envs) {
    envs->insert({env_name, new_env(env_name)});
  }
  {
    std::scoped_lock envs_lock(*(envs_.get_mutex_instance()));
    envs_.set_obj(envs);
  }

  plugin_manager_.configure(node, "ClipsPluginManager", envs_);

  RCLCPP_INFO(get_logger(), "Configured [%s]!", get_name());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManager::on_activate(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating [%s]...", get_name());
  plugin_manager_.activate();
  std::scoped_lock envs_lock(*(envs_.get_mutex_instance()));
  for (auto &env : *(envs_.get_obj())) {
    std::scoped_lock env_lock(*(env.second.get_mutex_instance()));
    clips::Reset(env.second.get_obj().get());
    clips::RefreshAllAgendas(env.second.get_obj().get());
    clips::Run(env.second.get_obj().get(), -1);
  }
  create_bond();
  RCLCPP_INFO(get_logger(), "Activated [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManager::on_deactivate(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating [%s]...", get_name());
  {
    std::scoped_lock envs_lock(*(envs_.get_mutex_instance()));
    for (auto &env : *(envs_.get_obj())) {
      std::scoped_lock env_lock(*(env.second.get_mutex_instance()));
      clips::AssertString(env.second.get_obj().get(), "(executive-finalize)");

      clips::RefreshAllAgendas(env.second.get_obj().get());
      clips::Run(env.second.get_obj().get(), -1);
    }
  }
  plugin_manager_.deactivate();
  create_env_service_.reset();
  destroy_env_service_.reset();
  destroy_bond();
  RCLCPP_INFO(get_logger(), "Deactivated [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CLIPSEnvManager::on_shutdown(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Shut down [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CLIPSEnvManager::on_error(const rclcpp_lifecycle::State &state) {
  // no action on activate for now
  (void)state;
  RCLCPP_INFO(get_logger(), "Error [%s]...", get_name());
  return CallbackReturn::SUCCESS;
}

void CLIPSEnvManager::create_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::CreateClipsEnv::Response> response) {
  (void)request_header; // the request header is not used in this callback

  bool env_exists = false;
  {
    std::scoped_lock lock(*envs_.get_mutex_instance());
    env_exists = (envs_->find(request->env_name) != envs_->end());
  }
  if (env_exists) {
    RCLCPP_ERROR(get_logger(),
                 "CLIPS environment '%s' already exists--> Should "
                 "be signaled! (e.g. as exception)",
                 request->env_name.c_str());
    response->success = false;
    response->error = "Enviroment " + request->env_name + " already exists!";
  } else {
    LockSharedPtr<clips::Environment> clips = new_env(request->log_name);

    const std::string &env_name = request->env_name;

    if (clips) {
      {
        std::scoped_lock lock(*envs_.get_mutex_instance());
        envs_->insert({env_name, clips});
      }
      plugin_manager_.activate_env(env_name, clips);
      response->success = true;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to initialize CLIPS environment '%s'",
                   request->env_name.c_str());
      response->success = false;
      response->error = "Failed to initialize environment " + request->env_name;
    }
  }
}

void CLIPSEnvManager::destroy_env_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Request> request,
    const std::shared_ptr<cx_msgs::srv::DestroyClipsEnv::Response> response) {

  (void)request_header; // the request header is not used in this callback
  const std::string &env_name = request->env_name;
  response->success = delete_env(env_name);
  if (!response->success) {
    response->error = "unknown Environment name";
  }
}

// --------------- ALL PRIVATE FUNCTION HELPERS ---------------

bool CLIPSEnvManager::delete_env(const std::string &env_name) {
  RCLCPP_WARN(get_logger(), "Deleting '%s' --- Clips Environment...",
              env_name.c_str());

  bool env_exists = false;
  {
    std::scoped_lock lock(*envs_.get_mutex_instance());
    env_exists = (envs_->find(env_name) != envs_->end());
  }
  if (env_exists) {

    LockSharedPtr<clips::Environment> env;
    {
      std::scoped_lock lock(*envs_.get_mutex_instance());
      env = envs_->at(env_name);
    }
    plugin_manager_.deactivate_env(env_name, env);
    {
      std::scoped_lock env_lock(*env.get_mutex_instance());
      clips::DeleteRouter(env.get_obj().get(), (char *)ROUTER_NAME);
      clips::DestroyEnvironment(env.get_obj().get());
    }

    {
      std::scoped_lock lock(*envs_.get_mutex_instance());
      envs_->erase(env_name);
    }

    RCLCPP_WARN(get_logger(), "Deleted '%s' --- Clips Environment!",
                env_name.c_str());

    return true;
  } else {
    RCLCPP_WARN(get_logger(), "Didn't find the provided env: '%s'!",
                env_name.c_str());
    return false;
  }
}

// Helper function to convert a string to lowercase
std::string to_lowercase(const std::string &str) {
  std::string lower_str = str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return lower_str;
}

// Function to map string to clips::WatchItem enum (case-insensitive)
clips::WatchItem get_watch_item_from_string(const std::string &watch_str) {
  std::string lower_str = to_lowercase(watch_str);

  if (lower_str == "all")
    return clips::ALL;
  if (lower_str == "facts")
    return clips::FACTS;
  if (lower_str == "instances")
    return clips::INSTANCES;
  if (lower_str == "slots")
    return clips::SLOTS;
  if (lower_str == "rules")
    return clips::RULES;
  if (lower_str == "activations")
    return clips::ACTIVATIONS;
  if (lower_str == "messages")
    return clips::MESSAGES;
  if (lower_str == "message_handlers")
    return clips::MESSAGE_HANDLERS;
  if (lower_str == "generic_functions")
    return clips::GENERIC_FUNCTIONS;
  if (lower_str == "methods")
    return clips::METHODS;
  if (lower_str == "deffunctions")
    return clips::DEFFUNCTIONS;
  if (lower_str == "compilations")
    return clips::COMPILATIONS;
  if (lower_str == "statistics")
    return clips::STATISTICS;
  if (lower_str == "globals")
    return clips::GLOBALS;
  if (lower_str == "focus")
    return clips::FOCUS;

  throw std::invalid_argument("Unknown watch item: " + watch_str);
}

cx::LockSharedPtr<clips::Environment>
CLIPSEnvManager::new_env(const std::string &env_name) {

  std::shared_ptr<clips::Environment> new_env(clips::CreateEnvironment());

  LockSharedPtr<clips::Environment> clips(std::move(new_env));
  clips::Environment *env = clips.get_obj().get();
  // Only place to init the env mutex
  clips.init_mutex();
  // no locking needed, as env is not shared yet
  // silent clips by default
  clips::Unwatch(env, clips::WatchItem::ALL);
  if (!clips::AllocateEnvironmentData(env, USER_ENVIRONMENT_DATA,
                                      sizeof(CLIPSEnvContext), NULL)) {
    RCLCPP_ERROR(get_logger(), "Error allocating environment data for %s",
                 env_name.c_str());
    clips::Writeln(env, "Error allocating environment data");
    clips::ExitRouter(env, EXIT_FAILURE);
    return clips;
  }
  cx::cx_utils::declare_parameter_if_not_declared(
      this, env_name + ".log_clips_to_file", rclcpp::ParameterValue(true));
  cx::cx_utils::declare_parameter_if_not_declared(
      this, env_name + ".watch",
      rclcpp::ParameterValue(std::vector<std::string>{}));
  cx::cx_utils::declare_parameter_if_not_declared(
      this, env_name + ".redirect_stdout_to_debug",
      rclcpp::ParameterValue(false));
  std::vector<std::string> watch_info;
  get_parameter(env_name + ".watch", watch_info);
  for (const auto &w : watch_info) {
    clips::WatchItem watch_item = get_watch_item_from_string(w);
    clips::Watch(clips.get_obj().get(), watch_item);
  }
  bool log_to_file = false;
  get_parameter(env_name + ".log_clips_to_file", log_to_file);
  bool stdout_to_debug = false;
  get_parameter(env_name + ".redirect_stdout_to_debug", stdout_to_debug);

  auto context = CLIPSEnvContext::get_context(env);
  context->env_name_ = env_name;
  context->env_lock_ptr_ = clips;
  // mem allocated already, so construct object in-place
  new (&context->logger_)
      CLIPSLogger(env_name.c_str(), log_to_file, stdout_to_debug);

  clips::AddRouter(env, (char *)ROUTER_NAME, /*router priority*/
                   40, log_router_query, log_router_print, NULL, NULL,
                   log_router_exit, &context->logger_);

  RCLCPP_INFO(get_logger(), "Initialisied new CLIPS environment: %s",
              env_name.c_str());
  return clips;
}

void CLIPSEnvManager::autostart() {
  using lifecycle_msgs::msg::State;
  autostart_timer_ = this->create_wall_timer(0s, [this]() -> void {
    autostart_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Auto-starting node: %s", this->get_name());
    if (configure().id() != State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_ERROR(get_logger(), "Auto-starting node %s failed to configure!",
                   this->get_name());
      return;
    }
    if (activate().id() != State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_ERROR(get_logger(), "Auto-starting node %s failed to activate!",
                   this->get_name());
    }
  });
}

void CLIPSEnvManager::on_rcl_preshutdown() {
  RCLCPP_INFO(get_logger(), "Running rcl preshutdown (%s)", this->get_name());

  run_cleanups();

  destroy_bond();
}

void CLIPSEnvManager::run_cleanups() {
  /*
   * In case this lifecycle node wasn't properly shut down, do it here.
   * We will give the user some ability to clean up properly here, but it's
   * best effort; i.e. we aren't trying to account for all possible states.
   */
  if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    this->deactivate();
  }

  if (get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    this->cleanup();
  }
}

void CLIPSEnvManager::register_rcl_preshutdown_callback() {
  rclcpp::Context::SharedPtr context = get_node_base_interface()->get_context();

  rcl_preshutdown_cb_handle_ =
      std::make_unique<rclcpp::PreShutdownCallbackHandle>(
          context->add_pre_shutdown_callback(
              std::bind(&CLIPSEnvManager::on_rcl_preshutdown, this)));
}

void CLIPSEnvManager::create_bond() {
  if (bond_heartbeat_period > 0.0) {
    RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.",
                this->get_name());

    bond_ = std::make_unique<bond::Bond>(std::string("bond"), this->get_name(),
                                         shared_from_this());

    bond_->setHeartbeatPeriod(bond_heartbeat_period);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();
  }
}

void CLIPSEnvManager::destroy_bond() {
  if (bond_heartbeat_period > 0.0) {
    RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.",
                this->get_name());

    if (bond_) {
      bond_->breakBond();
      bond_.reset();
    }
  }
}

} // namespace cx

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cx::CLIPSEnvManager)
