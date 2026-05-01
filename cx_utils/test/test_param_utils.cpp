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

#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cx_utils/param_utils.hpp"

class ParamUtilsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    // fresh node for each test to avoid parameter state bleeding between tests
    node_ = std::make_shared<rclcpp::Node>("test_node_" + std::to_string(test_counter_++));
  }

  rclcpp::Node::SharedPtr node_;
  static int test_counter_;
};

int ParamUtilsTest::test_counter_ = 0;

// --- declare_parameter_if_not_declared (value overload) ---

TEST_F(ParamUtilsTest, DeclareValueNotYetDeclared)
{
  cx::cx_utils::declare_parameter_if_not_declared(node_, "my_param", rclcpp::ParameterValue(42));

  ASSERT_TRUE(node_->has_parameter("my_param"));
  EXPECT_EQ(node_->get_parameter("my_param").as_int(), 42);
}

TEST_F(ParamUtilsTest, DeclareValueAlreadyDeclared)
{
  node_->declare_parameter("my_param", 42);

  // should not throw or change the value
  EXPECT_NO_THROW(
    cx::cx_utils::declare_parameter_if_not_declared(node_, "my_param", rclcpp::ParameterValue(99)));

  EXPECT_EQ(node_->get_parameter("my_param").as_int(), 42);
}

// --- declare_parameter_if_not_declared (type overload) ---

TEST_F(ParamUtilsTest, DeclareTypeNotYetDeclared)
{
  cx::cx_utils::declare_parameter_if_not_declared(node_, "my_param", rclcpp::PARAMETER_STRING);

  ASSERT_TRUE(node_->has_parameter("my_param"));
  // can't get_parameter on an uninitialized param, just check the descriptor type
  auto descriptors = node_->describe_parameter("my_param");
  EXPECT_EQ(descriptors.type, rcl_interfaces::msg::ParameterType::PARAMETER_STRING);
}

TEST_F(ParamUtilsTest, DeclareTypeAlreadyDeclared)
{
  node_->declare_parameter("my_param", rclcpp::PARAMETER_INTEGER);

  EXPECT_NO_THROW(
    cx::cx_utils::declare_parameter_if_not_declared(node_, "my_param", rclcpp::PARAMETER_STRING));

  // type should still be INTEGER, not STRING
  auto descriptors = node_->describe_parameter("my_param");
  EXPECT_EQ(descriptors.type, rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER);
}

// --- get_plugin_type_param ---

TEST_F(ParamUtilsTest, GetPluginTypeParamReturnsValue)
{
  node_->declare_parameter("my_plugin.plugin", std::string("my_package/MyPlugin"));

  std::string result = cx::cx_utils::get_plugin_type_param(node_, "my_plugin");

  EXPECT_EQ(result, "my_package/MyPlugin");
}

TEST_F(ParamUtilsTest, GetPluginTypeParamThrowsWhenNotSet)
{
  // parameter exists (declared by get_plugin_type_param internally) but has no value
  EXPECT_THROW(cx::cx_utils::get_plugin_type_param(node_, "my_plugin"), std::runtime_error);
}
