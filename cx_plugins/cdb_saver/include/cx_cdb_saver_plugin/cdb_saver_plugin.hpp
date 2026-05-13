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

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>

#include "cx_cdb_saver_plugin/db_handler.hpp"
#include "cx_plugin/clips_plugin.hpp"
#include "rcl_interfaces/msg/list_parameters_result.hpp"

namespace cx
{

class CDBSaverPlugin : public ClipsPlugin
{
public:
  CDBSaverPlugin();
  ~CDBSaverPlugin();

  void initialize() override;
  void finalize() override;

  static void cdb_assert_callback(clips::Environment *, void *, void *);
  static void cdb_retract_callback(clips::Environment *, void *, void *);

  static void cdb_before_rule_callback(clips::Environment *, clips::Activation *, void *);

  static void cdb_before_run_callback(clips::Environment *, void *);
  static void cdb_after_run_callback(clips::Environment *, void *);

  static void cdb_defglobal_assert_callback(clips::Environment *, clips::Defglobal *, void *);
  static void cdb_defglobal_retract_callback(clips::Environment *, clips::Defglobal *, void *);

  static void cdb_defrule_assert_callback(clips::Environment *, clips::Defrule *, void *);
  static void cdb_defrule_retract_callback(clips::Environment *, clips::Defrule *, void *);

  static void cdb_deftemplate_assert_callback(clips::Environment *, clips::Deftemplate *, void *);
  static void cdb_deftemplate_retract_callback(clips::Environment *, clips::Deftemplate *, void *);

  static void cdb_deffacts_assert_callback(clips::Environment *, clips::Deffacts *, void *);
  static void cdb_deffacts_retract_callback(clips::Environment *, clips::Deffacts *, void *);

  static void cdb_deffunction_assert_callback(clips::Environment *, clips::Deffunction *, void *);
  static void cdb_deffunction_retract_callback(clips::Environment *, clips::Deffunction *, void *);

  bool clips_env_init(std::shared_ptr<clips::Environment> & env) override;
  bool clips_env_destroyed(std::shared_ptr<clips::Environment> & env) override;

  void plugin_load_callback(const std::string & env_name, const std::string & plugin_name);
  void plugin_unload_callback(const std::string & env_name, const std::string & plugin_name);

private:
  static inline nlohmann::json slot_value_to_json(unsigned short type, clips::CLIPSValue * value);
  static inline std::vector<nlohmann::json> multifield_to_json_list(clips::Multifield * theSegment);
  inline nlohmann::json parameter_list_to_json(
    const rcl_interfaces::msg::ListParametersResult & parameters);

  static std::string clips_fact_to_json(clips::Fact * f);
  std::unique_ptr<rclcpp::Logger> logger_;

  std::unordered_map<std::string, std::unique_ptr<DBHandler>> db_handlers_;
};
}  // namespace cx
