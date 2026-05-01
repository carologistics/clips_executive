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

#include <rclcpp/rclcpp.hpp>

#include "cx_clips_env_manager/clips_env_manager.hpp"

class CLIPSEnvManagerLifecycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("autostart_node", false);

    node_ = std::make_shared<cx::CLIPSEnvManager>(options);
  }

  void TearDown() override { node_.reset(); }

  void spin_some()
  {
    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::shared_ptr<cx::CLIPSEnvManager> node_;
};

TEST_F(CLIPSEnvManagerLifecycleTest, lifecycle_happy_path)
{
  using lifecycle_msgs::msg::State;

  // Initial state
  EXPECT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // CONFIGURE
  auto ret = node_->configure();
  spin_some();

  EXPECT_EQ(ret.id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);

  // ACTIVATE
  ret = node_->activate();
  spin_some();

  EXPECT_EQ(ret.id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_ACTIVE);

  // DEACTIVATE
  ret = node_->deactivate();
  spin_some();

  EXPECT_EQ(ret.id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_INACTIVE);

  // CLEANUP
  ret = node_->cleanup();
  spin_some();

  EXPECT_EQ(ret.id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node_->get_current_state().id(), State::PRIMARY_STATE_UNCONFIGURED);

  // SHUTDOWN
  ret = node_->shutdown();
  spin_some();

  EXPECT_EQ(ret.id(), State::PRIMARY_STATE_FINALIZED);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
