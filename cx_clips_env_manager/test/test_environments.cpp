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

#include <gtest/gtest.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "cx_clips_env_manager/clips_env_manager.hpp"
#include "cx_msgs/srv/create_clips_env.hpp"
#include "cx_msgs/srv/destroy_clips_env.hpp"
#include "cx_msgs/srv/list_clips_envs.hpp"
#include "cx_msgs/srv/list_clips_plugins.hpp"
#include "cx_msgs/srv/load_clips_plugin.hpp"
#include "cx_msgs/srv/unload_clips_plugin.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Base fixture — autostart node, no environments or plugins pre-configured
// ---------------------------------------------------------------------------
class CLIPSEnvManagerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("autostart_node", true);
    node_ = std::make_shared<cx::CLIPSEnvManager>(options);
    exec_.add_node(node_->get_node_base_interface());
    exec_.spin_some();
  }

  void TearDown() override
  {
    if (node_) {
      node_->run_cleanups();
      exec_.spin_some();
      exec_.cancel();
      exec_.remove_node(node_->get_node_base_interface());
      exec_.spin_some();
      node_.reset();
    }
  }

  // helpers to reduce boilerplate
  template <typename SrvT>
  typename rclcpp::Client<SrvT>::SharedPtr make_client(const std::string & service)
  {
    auto client = node_->create_client<SrvT>(service);
    EXPECT_TRUE(client->wait_for_service(4s));
    return client;
  }

  std::shared_ptr<cx::CLIPSEnvManager> node_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

// ---------------------------------------------------------------------------
// Fixture that pre-configures one environment with the example plugin
// ---------------------------------------------------------------------------
class CLIPSEnvManagerWithPluginTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("autostart_node", true);
    options.append_parameter_override("environments", std::vector<std::string>{"test_env"});
    options.append_parameter_override("test_env.plugins", std::vector<std::string>{"example1"});
    options.append_parameter_override("test_env.log_clips_to_file", false);
    options.append_parameter_override("example1.plugin", std::string("cx::ExamplePlugin"));
    options.append_parameter_override("example2.plugin", std::string("cx::ExamplePlugin"));

    node_ = std::make_shared<cx::CLIPSEnvManager>(options);
    exec_.add_node(node_->get_node_base_interface());
    exec_.spin_some();
  }

  void TearDown() override
  {
    if (node_) {
      node_->run_cleanups();
      exec_.spin_some();
      exec_.cancel();
      exec_.remove_node(node_->get_node_base_interface());
      exec_.spin_some();
      node_.reset();
    }
  }

  template <typename SrvT>
  typename rclcpp::Client<SrvT>::SharedPtr make_client(const std::string & service)
  {
    auto client = node_->create_client<SrvT>(service);
    EXPECT_TRUE(client->wait_for_service(4s));
    return client;
  }

  std::shared_ptr<cx::CLIPSEnvManager> node_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

// ===========================================================================
// Environment service tests
// ===========================================================================

TEST_F(CLIPSEnvManagerTest, ListEnvsWhenEmpty)
{
  auto client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto future = client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  ASSERT_TRUE(resp->success);
  EXPECT_TRUE(resp->envs.empty());
}

TEST_F(CLIPSEnvManagerTest, CreateEnvAndVerifyList)
{
  node_->declare_parameter("text_env.log_clips_to_file", false);
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = "test_env";
  auto future = create_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);

  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  EXPECT_NE(
    std::find(list_resp->envs.begin(), list_resp->envs.end(), "test_env"), list_resp->envs.end());
}

TEST_F(CLIPSEnvManagerTest, CreateEnvDuplicateFails)
{
  node_->declare_parameter("dup_env.log_clips_to_file", false);
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = "dup_env";

  auto future1 = create_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future1, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future1.get()->success);

  auto future2 = create_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future2, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp2 = future2.get();
  EXPECT_FALSE(resp2->success);
  EXPECT_FALSE(resp2->error.empty());

  // verify only one entry exists
  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  EXPECT_EQ(std::count(list_resp->envs.begin(), list_resp->envs.end(), "dup_env"), 1);
}

TEST_F(CLIPSEnvManagerTest, CreateAndDestroyEnv)
{
  node_->declare_parameter("temp_env.log_clips_to_file", false);
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = "temp_env";
  auto future = create_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);

  auto destroy_client = make_client<cx_msgs::srv::DestroyClipsEnv>("/clips_manager/destroy_env");
  auto destroy_req = std::make_shared<cx_msgs::srv::DestroyClipsEnv::Request>();
  destroy_req->env_name = "temp_env";
  auto destroy_future = destroy_client->async_send_request(destroy_req);
  ASSERT_EQ(
    exec_.spin_until_future_complete(destroy_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(destroy_future.get()->success);

  // verify it is gone
  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  EXPECT_EQ(std::count(list_resp->envs.begin(), list_resp->envs.end(), "temp_env"), 0);
}

TEST_F(CLIPSEnvManagerTest, DestroyNonExistentEnvFails)
{
  auto destroy_client = make_client<cx_msgs::srv::DestroyClipsEnv>("/clips_manager/destroy_env");
  auto req = std::make_shared<cx_msgs::srv::DestroyClipsEnv::Request>();
  req->env_name = "nonexistent_env";
  auto future = destroy_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->error.empty());
}

TEST_F(CLIPSEnvManagerTest, CreateMultipleEnvs)
{
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");

  for (const auto & name : {"env_a", "env_b", "env_c"}) {
    node_->declare_parameter(std::string(name) + ".log_clips_to_file", false);
    auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
    req->env_name = name;
    auto future = create_client->async_send_request(req);
    ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
  }

  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  EXPECT_EQ(list_resp->envs.size(), 3u);
}

// Two envs with redirect_stdout_to_debug=true, log_clips_to_file=true
TEST_F(CLIPSEnvManagerTest, CreateTwoEnvsWithStdoutToDebug)
{
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");

  for (const auto & name : {"env_debug1", "env_debug2"}) {
    // declare per-env param before creating
    node_->declare_parameter(std::string(name) + ".redirect_stdout_to_debug", true);
    node_->declare_parameter(std::string(name) + ".log_clips_to_file", true);
    auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
    req->env_name = name;
    auto future = create_client->async_send_request(req);
    ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
  }

  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  EXPECT_EQ(list_resp->envs.size(), 2u);
  EXPECT_NE(
    std::find(list_resp->envs.begin(), list_resp->envs.end(), "env_debug1"), list_resp->envs.end());
  EXPECT_NE(
    std::find(list_resp->envs.begin(), list_resp->envs.end(), "env_debug2"), list_resp->envs.end());
}

// Two envs with redirect_stdout_to_debug=false, log_clips_to_file=true
TEST_F(CLIPSEnvManagerTest, CreateTwoEnvsWithStdoutToInfo)
{
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");

  for (const auto & name : {"env_info1", "env_info2"}) {
    node_->declare_parameter(std::string(name) + ".redirect_stdout_to_debug", false);
    node_->declare_parameter(std::string(name) + ".log_clips_to_file", true);
    auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
    req->env_name = name;
    auto future = create_client->async_send_request(req);
    ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
  }

  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  EXPECT_EQ(list_resp->envs.size(), 2u);
}

TEST_F(CLIPSEnvManagerTest, CreateEnvWithValidWatchItem)
{
  node_->declare_parameter("watched_env.watch", std::vector<std::string>{"facts"});
  node_->declare_parameter("watched_env.log_clips_to_file", false);

  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = "watched_env";
  auto future = create_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);
}

TEST_F(CLIPSEnvManagerTest, CreateEnvWithInvalidWatchItemFails)
{
  node_->declare_parameter(
    "invalid_watch_env.watch", std::vector<std::string>{"not_a_valid_watch_item"});
  node_->declare_parameter("watched_env.log_clips_to_file", false);

  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = "invalid_watch_env";
  auto future = create_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(future.get()->success);
}

// ===========================================================================
// Pre-configured env + plugin fixture tests
// ===========================================================================

TEST_F(CLIPSEnvManagerWithPluginTest, LoadPluginOnExistingEnv)
{
  // set the plugin type parameter before loading
  node_->set_parameter(rclcpp::Parameter("example1.plugin", "cx::ExamplePlugin"));

  // load the plugin
  auto load_client = make_client<cx_msgs::srv::LoadClipsPlugin>("/clips_manager/load_plugin");
  auto load_req = std::make_shared<cx_msgs::srv::LoadClipsPlugin::Request>();
  load_req->env_name = "test_env";
  load_req->plugin_name = "example1";
  auto load_future = load_client->async_send_request(load_req);
  ASSERT_EQ(exec_.spin_until_future_complete(load_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(load_future.get()->success);

  // verify plugin appears in list
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
  list_req->env_name = "test_env";
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  EXPECT_NE(
    std::find(list_resp->plugins.begin(), list_resp->plugins.end(), "example1"),
    list_resp->plugins.end());
}

TEST_F(CLIPSEnvManagerWithPluginTest, LoadPluginOnNonExistentEnvFails)
{
  node_->set_parameter(rclcpp::Parameter("example1.plugin", "cx::ExamplePlugin"));

  auto load_client = make_client<cx_msgs::srv::LoadClipsPlugin>("/clips_manager/load_plugin");
  auto req = std::make_shared<cx_msgs::srv::LoadClipsPlugin::Request>();
  req->env_name = "nonexistent_env";
  req->plugin_name = "example1";
  auto future = load_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->error.empty());
}

TEST_F(CLIPSEnvManagerWithPluginTest, LoadAndUnloadPlugin)
{
  node_->set_parameter(rclcpp::Parameter("example1.plugin", "cx::ExamplePlugin"));

  // load
  auto load_client = make_client<cx_msgs::srv::LoadClipsPlugin>("/clips_manager/load_plugin");
  auto load_req = std::make_shared<cx_msgs::srv::LoadClipsPlugin::Request>();
  load_req->env_name = "test_env";
  load_req->plugin_name = "example2";
  auto load_future = load_client->async_send_request(load_req);
  ASSERT_EQ(exec_.spin_until_future_complete(load_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(load_future.get()->success);

  // unload
  auto unload_client = make_client<cx_msgs::srv::UnloadClipsPlugin>("/clips_manager/unload_plugin");
  auto unload_req = std::make_shared<cx_msgs::srv::UnloadClipsPlugin::Request>();
  unload_req->env_name = "test_env";
  unload_req->plugin_name = "example2";
  auto unload_future = unload_client->async_send_request(unload_req);
  ASSERT_EQ(exec_.spin_until_future_complete(unload_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(unload_future.get()->success);

  // verify plugin is gone from list
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
  list_req->env_name = "test_env";
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  EXPECT_EQ(std::count(list_resp->plugins.begin(), list_resp->plugins.end(), "example2"), 0);
}

TEST_F(CLIPSEnvManagerWithPluginTest, UnloadNonExistentPluginFails)
{
  auto unload_client = make_client<cx_msgs::srv::UnloadClipsPlugin>("/clips_manager/unload_plugin");
  auto req = std::make_shared<cx_msgs::srv::UnloadClipsPlugin::Request>();
  req->env_name = "test_env";
  req->plugin_name = "nonexistent_plugin";
  auto future = unload_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->error.empty());
}

TEST_F(CLIPSEnvManagerWithPluginTest, ListPluginsEmptyEnvName)
{
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");
  auto req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
  req->env_name = "";
  auto future = list_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->error.empty());
  EXPECT_TRUE(resp->plugins.empty());
}

TEST_F(CLIPSEnvManagerWithPluginTest, ListPluginsNonExistentEnvFails)
{
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");
  auto req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
  req->env_name = "nonexistent_env";
  auto future = list_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->error.empty());
  EXPECT_TRUE(resp->plugins.empty());
}

TEST_F(CLIPSEnvManagerWithPluginTest, PreConfiguredEnvExists)
{
  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto future = list_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  ASSERT_TRUE(resp->success);
  EXPECT_NE(std::find(resp->envs.begin(), resp->envs.end(), "test_env"), resp->envs.end());
}

TEST_F(CLIPSEnvManagerWithPluginTest, PreConfiguredPluginLoaded)
{
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");
  auto req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
  req->env_name = "test_env";
  auto future = list_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto resp = future.get();
  ASSERT_TRUE(resp->success);
  EXPECT_NE(std::find(resp->plugins.begin(), resp->plugins.end(), "example1"), resp->plugins.end());
}

TEST_F(CLIPSEnvManagerWithPluginTest, DestroyEnvWithPluginLoaded)
{
  // destroying an env with a loaded plugin should succeed and clean up gracefully
  auto destroy_client = make_client<cx_msgs::srv::DestroyClipsEnv>("/clips_manager/destroy_env");
  auto req = std::make_shared<cx_msgs::srv::DestroyClipsEnv::Request>();
  req->env_name = "test_env";
  auto future = destroy_client->async_send_request(req);
  ASSERT_EQ(exec_.spin_until_future_complete(future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);

  // env should be gone
  auto list_client = make_client<cx_msgs::srv::ListClipsEnvs>("/clips_manager/list_envs");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  EXPECT_EQ(std::count(list_resp->envs.begin(), list_resp->envs.end(), "test_env"), 0);
}

// Two envs, same plugin loaded in both
TEST_F(CLIPSEnvManagerWithPluginTest, LoadSamePluginInTwoEnvs)
{
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto load_client = make_client<cx_msgs::srv::LoadClipsPlugin>("/clips_manager/load_plugin");
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");

  for (const auto & env_name : {"env_a", "env_b"}) {
    node_->declare_parameter(std::string(env_name) + ".log_clips_to_file", false);
    auto create_req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
    create_req->env_name = env_name;
    auto create_future = create_client->async_send_request(create_req);
    ASSERT_EQ(
      exec_.spin_until_future_complete(create_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(create_future.get()->success);

    auto load_req = std::make_shared<cx_msgs::srv::LoadClipsPlugin::Request>();
    load_req->env_name = env_name;
    load_req->plugin_name = "example1";
    auto load_future = load_client->async_send_request(load_req);
    ASSERT_EQ(exec_.spin_until_future_complete(load_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(load_future.get()->success);

    // verify plugin is listed for this env
    auto list_req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
    list_req->env_name = env_name;
    auto list_future = list_client->async_send_request(list_req);
    ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    auto list_resp = list_future.get();
    ASSERT_TRUE(list_resp->success);
    EXPECT_NE(
      std::find(list_resp->plugins.begin(), list_resp->plugins.end(), "example1"),
      list_resp->plugins.end());
  }
}

// Two envs, different plugin names (same type) each in one env
TEST_F(CLIPSEnvManagerWithPluginTest, LoadDifferentNamedPluginsInTwoEnvs)
{
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto load_client = make_client<cx_msgs::srv::LoadClipsPlugin>("/clips_manager/load_plugin");
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");

  std::vector<std::pair<std::string, std::string>> env_plugin_pairs = {
    {"env_a", "example1"}, {"env_b", "example2"}};

  for (const auto & [env_name, plugin_name] : env_plugin_pairs) {
    auto create_req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
    create_req->env_name = env_name;
    node_->declare_parameter(std::string(env_name) + ".log_clips_to_file", false);
    auto create_future = create_client->async_send_request(create_req);
    ASSERT_EQ(
      exec_.spin_until_future_complete(create_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(create_future.get()->success);

    auto load_req = std::make_shared<cx_msgs::srv::LoadClipsPlugin::Request>();
    load_req->env_name = env_name;
    load_req->plugin_name = plugin_name;
    auto load_future = load_client->async_send_request(load_req);
    ASSERT_EQ(exec_.spin_until_future_complete(load_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(load_future.get()->success);
  }

  // verify each env only has its own plugin
  for (const auto & [env_name, plugin_name] : env_plugin_pairs) {
    auto list_req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
    list_req->env_name = env_name;
    auto list_future = list_client->async_send_request(list_req);
    ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    auto list_resp = list_future.get();
    ASSERT_TRUE(list_resp->success);
    ASSERT_EQ(list_resp->plugins.size(), 1u);
    EXPECT_EQ(list_resp->plugins[0], plugin_name);
  }
}

// One env, two plugins (same type, different names)
TEST_F(CLIPSEnvManagerWithPluginTest, LoadTwoPluginsInOneEnv)
{
  auto create_client = make_client<cx_msgs::srv::CreateClipsEnv>("/clips_manager/create_env");
  auto create_req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  create_req->env_name = "multi_plugin_env";
  node_->declare_parameter("multi_plugin_env.log_clips_to_file", false);
  auto create_future = create_client->async_send_request(create_req);
  ASSERT_EQ(exec_.spin_until_future_complete(create_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(create_future.get()->success);

  auto load_client = make_client<cx_msgs::srv::LoadClipsPlugin>("/clips_manager/load_plugin");

  for (const auto & plugin_name : {"example1", "example2"}) {
    auto load_req = std::make_shared<cx_msgs::srv::LoadClipsPlugin::Request>();
    load_req->env_name = "multi_plugin_env";
    load_req->plugin_name = plugin_name;
    auto load_future = load_client->async_send_request(load_req);
    ASSERT_EQ(exec_.spin_until_future_complete(load_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(load_future.get()->success);
  }

  // verify both plugins are listed
  auto list_client = make_client<cx_msgs::srv::ListClipsPlugins>("/clips_manager/list_plugins");
  auto list_req = std::make_shared<cx_msgs::srv::ListClipsPlugins::Request>();
  list_req->env_name = "multi_plugin_env";
  auto list_future = list_client->async_send_request(list_req);
  ASSERT_EQ(exec_.spin_until_future_complete(list_future, 4s), rclcpp::FutureReturnCode::SUCCESS);
  auto list_resp = list_future.get();
  ASSERT_TRUE(list_resp->success);
  ASSERT_EQ(list_resp->plugins.size(), 2u);
  EXPECT_NE(
    std::find(list_resp->plugins.begin(), list_resp->plugins.end(), "example1"),
    list_resp->plugins.end());
  EXPECT_NE(
    std::find(list_resp->plugins.begin(), list_resp->plugins.end(), "example2"),
    list_resp->plugins.end());
}

// ===========================================================================
// Watch item parsing — pure logic, no node needed
// ===========================================================================

// expose the function for testing via a free declaration
// (it's in the cx namespace but not in a header — add it to the header or
// test via a friend/subclass if needed)

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

/**
using namespace std::chrono_literals;

class CLIPSEnvManagerServiceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("autostart_node", true);

    node_ = std::make_shared<cx::CLIPSEnvManager>(options);
    // make sure node is active
    exec_.add_node(node_->get_node_base_interface());
    exec_.spin_some();
  }

  void TearDown() override
  {
		  if (node_) {
    // 2. stop executor (critical barrier)
    exec_.cancel();
    exec_.spin_some();
    exec_.remove_node(node_->get_node_base_interface());
    exec_.spin_some();
    node_.reset();
  }

  // 3. reset node AFTER executor is fully stopped
  }

  std::shared_ptr<cx::CLIPSEnvManager> node_;
	rclcpp::executors::SingleThreadedExecutor exec_;
};

TEST_F(CLIPSEnvManagerServiceTest, create_env_and_verify_list)
{
  using namespace std::chrono_literals;

  // -----------------------------
  // 1. Create environment client
  // -----------------------------
  auto create_env_client =
    node_->create_client<cx_msgs::srv::CreateClipsEnv>(
      "/clips_manager/create_env");

  ASSERT_TRUE(create_env_client->wait_for_service(4s));

  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = "test_env";

  auto future = create_env_client->async_send_request(req);

  auto status = exec_.spin_until_future_complete(future, 4s);

  ASSERT_EQ(status, rclcpp::FutureReturnCode::SUCCESS);

  auto resp = future.get();
  EXPECT_TRUE(resp->success);

  // -----------------------------
  // 2. List environments
  // -----------------------------
  auto list_client =
    node_->create_client<cx_msgs::srv::ListClipsEnvs>(
      "/clips_manager/list_envs");

  ASSERT_TRUE(list_client->wait_for_service(4s));

  auto list_req =
    std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();

  auto list_future = list_client->async_send_request(list_req);

  auto list_status = exec_.spin_until_future_complete(list_future, 4s);

  ASSERT_EQ(list_status, rclcpp::FutureReturnCode::SUCCESS);

  auto list_resp = list_future.get();

  ASSERT_TRUE(list_resp->success);

  // -----------------------------
  // 3. Validate result
  // -----------------------------
  ASSERT_NE(
    std::find(
      list_resp->envs.begin(),
      list_resp->envs.end(),
      "test_env"),
    list_resp->envs.end());
}
TEST_F(CLIPSEnvManagerServiceTest, create_env_duplicate_fails)
{
  using namespace std::chrono_literals;

  // -----------------------------
  // 1. Create first environment
  // -----------------------------
  auto create_client =
    node_->create_client<cx_msgs::srv::CreateClipsEnv>(
      "/clips_manager/create_env");

  ASSERT_TRUE(create_client->wait_for_service(4s));

  auto req = std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  req->env_name = "dup_env";

  auto future1 = create_client->async_send_request(req);

  auto status1 = exec_.spin_until_future_complete(future1, 4s);

  ASSERT_EQ(status1, rclcpp::FutureReturnCode::SUCCESS);

  auto resp1 = future1.get();
  EXPECT_TRUE(resp1->success);

  // -----------------------------
  // 2. Try to create same env again
  // -----------------------------
  auto future2 = create_client->async_send_request(req);

  auto status2 = exec_.spin_until_future_complete(future2, 4s);

  ASSERT_EQ(status2, rclcpp::FutureReturnCode::SUCCESS);

  auto resp2 = future2.get();

  // -----------------------------
  // 3. Validate rejection
  // -----------------------------
  EXPECT_FALSE(resp2->success);
  EXPECT_FALSE(resp2->error.empty());

  // Optional: verify system state unchanged
  auto list_client =
    node_->create_client<cx_msgs::srv::ListClipsEnvs>(
      "/clips_manager/list_envs");

  ASSERT_TRUE(list_client->wait_for_service(4s));

  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();

  auto list_future = list_client->async_send_request(list_req);

  auto list_status = exec_.spin_until_future_complete(list_future, 4s);

  ASSERT_EQ(list_status, rclcpp::FutureReturnCode::SUCCESS);

  auto list_resp = list_future.get();

  ASSERT_TRUE(list_resp->success);

  // should still only exist once
  auto count = std::count(
    list_resp->envs.begin(),
    list_resp->envs.end(),
    "dup_env");

  EXPECT_EQ(count, 1);
}

TEST_F(CLIPSEnvManagerServiceTest, create_and_destroy_env)
{
  using namespace std::chrono_literals;

  // -----------------------------
  // 1. Create environment
  // -----------------------------
  auto create_client =
    node_->create_client<cx_msgs::srv::CreateClipsEnv>(
      "/clips_manager/create_env");

  ASSERT_TRUE(create_client->wait_for_service(4s));

  auto create_req =
    std::make_shared<cx_msgs::srv::CreateClipsEnv::Request>();
  create_req->env_name = "temp_env";

  auto create_future = create_client->async_send_request(create_req);

  ASSERT_EQ(
    exec_.spin_until_future_complete(create_future, 4s),
    rclcpp::FutureReturnCode::SUCCESS);

  auto create_resp = create_future.get();
  EXPECT_TRUE(create_resp->success);

  // -----------------------------
  // 2. Verify it exists
  // -----------------------------
  auto list_client =
    node_->create_client<cx_msgs::srv::ListClipsEnvs>(
      "/clips_manager/list_envs");

  ASSERT_TRUE(list_client->wait_for_service(4s));

  auto list_req = std::make_shared<cx_msgs::srv::ListClipsEnvs::Request>();

  auto list_future = list_client->async_send_request(list_req);

  ASSERT_EQ(
    exec_.spin_until_future_complete(list_future, 4s),
    rclcpp::FutureReturnCode::SUCCESS);

  auto list_resp = list_future.get();

  ASSERT_TRUE(list_resp->success);
  ASSERT_NE(
    std::find(
      list_resp->envs.begin(),
      list_resp->envs.end(),
      "temp_env"),
    list_resp->envs.end());

   // -----------------------------
   // 3. Destroy environment
   // -----------------------------
   auto destroy_client =
     node_->create_client<cx_msgs::srv::DestroyClipsEnv>(
       "/clips_manager/destroy_env");

   ASSERT_TRUE(destroy_client->wait_for_service(4s));

   auto destroy_req =
     std::make_shared<cx_msgs::srv::DestroyClipsEnv::Request>();
   destroy_req->env_name = "temp_env";

   auto destroy_future = destroy_client->async_send_request(destroy_req);

   ASSERT_EQ(
     exec_.spin_until_future_complete(destroy_future, 4s),
     rclcpp::FutureReturnCode::SUCCESS);

   auto destroy_resp = destroy_future.get();

   EXPECT_TRUE(destroy_resp->success);

   // -----------------------------
   // 4. Verify it is gone
   // -----------------------------
   auto list_future2 = list_client->async_send_request(list_req);

   ASSERT_EQ(
     exec_.spin_until_future_complete(list_future2, 4s),
     rclcpp::FutureReturnCode::SUCCESS);

   auto list_resp2 = list_future2.get();

   ASSERT_TRUE(list_resp2->success);

   EXPECT_EQ(
     std::count(
       list_resp2->envs.begin(),
       list_resp2->envs.end(),
       "temp_env"),
     0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}

*/
