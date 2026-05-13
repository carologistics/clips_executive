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

#include <pqxx/pqxx>
#undef RANGES
// RANGES is defined in clips_ns/clips.h, which causes issues with
// pqxx/pqxx

#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cx
{
struct DBHandlerConfig
{
  std::string hostname;
  int port;
  std::string username;
  std::string password;
  std::string db_name;
};
class DBHandler
{
  friend class CDBSaverPlugin;

public:
  DBHandler(DBHandlerConfig & config, rclcpp_lifecycle::LifecycleNode::WeakPtr parent);
  ~DBHandler();

  void assert_fact(
    long long id, const std::string & deftemplate, const std::string & fact_json, long long tick);
  void retract_fact(long long id, long long tick);

  void assert_defrule(
    const std::string & name, const std::string & module_name, const std::string & definition,
    const int salience, long long tick);
  void retract_defrule(const std::string & name, const std::string & module_name, long long tick);

  void assert_deffunction(
    const std::string & name, const std::string & module_name, const std::string & definition,
    long long tick);
  void retract_deffunction(
    const std::string & name, const std::string & module_name, long long tick);

  void assert_deffacts(
    const std::string & name, const std::string & module_name, const std::string & definition,
    long long tick);
  void retract_deffacts(const std::string & name, const std::string & module_name, long long tick);

  void assert_defglobal(
    const std::string & name, const std::string & module_name, const std::string & value_json,
    long long tick);
  void retract_defglobal(const std::string & name, const std::string & module_name, long long tick);

  void assert_deftemplate(
    const std::string & name, const std::string & module_name, const std::string & definition,
    long long tick);
  void retract_deftemplate(
    const std::string & name, const std::string & module_name, long long tick);

  void assert_rule_fired(
    const std::string & name, const std::string & module_name, const std::vector<long long> & basis,
    long long tick);

  void load_plugin(
    const std::string & plugin_name, const std::string & config_json, long long tick);
  void unload_plugin(const std::string & plugin_name, long long tick);

  long long start_run(int64_t start_time_ns, long long start_tick);
  void end_run(long long run_number, int64_t end_time_ns, long long end_tick);

  bool init_db(DBHandlerConfig & config);

  inline long long get_tick() { return tick_++; }

private:
  std::shared_ptr<pqxx::connection> connection_;
  DBHandlerConfig config_;

  long long tick_;
  long long current_run_;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
};
}  // namespace cx
