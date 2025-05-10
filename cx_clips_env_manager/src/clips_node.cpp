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

#include <memory>
#include <string>

#include "cx_clips_env_manager/clips_env_manager.hpp"
#include "rclcpp/rclcpp.hpp"
std::shared_ptr<cx::CLIPSEnvManager> clips_env_manager_node;
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe;

void custom_signal_handler(int /*signum*/) {
  if (exe) {
    exe->cancel();
  }
  if (exe && clips_env_manager_node) {
    exe->remove_node(clips_env_manager_node->get_node_base_interface());
  }
  exe.reset();
  clips_env_manager_node.reset();
  rclcpp::shutdown();
}

int main(int argc, const char **argv) {
  rclcpp::init(argc, argv);
  exe = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::signal(SIGINT, custom_signal_handler);

  clips_env_manager_node = std::make_shared<cx::CLIPSEnvManager>();

  exe->add_node(clips_env_manager_node->get_node_base_interface());
  exe->spin();
  custom_signal_handler(SIGINT);
  return 0;
}
