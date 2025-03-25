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

#include "cx_plugin/clips_plugin.hpp"
#include "db_handler.hpp"

namespace cx
{

struct fact_assert
{
  long long tick;
  std::string value;
};

class CDBPlugin : public ClipsPlugin
{
public:
  CDBPlugin();
  ~CDBPlugin();

  void initialize() override;

  static void cdb_assert_callback(clips::Environment *, void *, void *);
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

private:
  inline nlohmann::json slot_value_to_json(unsigned short type, clips::CLIPSValue * value);
  inline std::vector<nlohmann::json> multifield_to_json_list(clips::Multifield * theSegment);
  std::string clips_fact_to_json(clips::Fact * f, const char * deftemplate_name);
  std::unique_ptr<rclcpp::Logger> logger_;
  bool started_ = false;
  static long long tick_;
  static inline long long get_tick() { return tick_++; }
  static long long current_run_;

  static std::shared_ptr<DBHandler> db_;
};
}  // namespace cx
