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
// pqxx/pqxx. It needs to be included before clips_ns/clips.h to avoid compilation errors.

#include <nlohmann/json.hpp>
#include <unordered_map>

#include "cx_cdb_loader_plugin/helpers.hpp"
#include "cx_plugin/clips_plugin.hpp"

namespace cx
{

class CDBLoaderPlugin : public ClipsPlugin
{
public:
  CDBLoaderPlugin();
  ~CDBLoaderPlugin();

  void initialize() override;

  bool clips_env_init(std::shared_ptr<clips::Environment> & env) override;
  bool clips_env_destroyed(std::shared_ptr<clips::Environment> & env) override;

  void assert_deftemplates(
    clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
    const std::string & defmodule, Tick restore_tick);
  void assert_deffunctions(
    clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
    const std::string & defmodule, Tick restore_tick);
  void assert_defrules(
    clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
    const std::string & defmodule, Tick restore_tick);
  void assert_deffacts(
    clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
    const std::string & defmodule, Tick restore_tick);
  void assert_defglobals(
    clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
    std::vector<Defglobal> fact_pointing_defglobals, clips::Fact * tmp_fact,
    const std::string & defmodule, Tick restore_tick);
  void update_defglobals(
    clips::Environment * env, std::vector<Defglobal> defglobals, const RegexConfig & config,
    std::unordered_map<long long, clips::Fact *> id_to_fact_ptr,
    std::vector<clips::Fact *> created_nullptr_facts);
  void assert_facts(
    clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
    std::unordered_map<long long, clips::Fact *> id_to_fact_ptr,
    std::vector<clips::Fact *> created_nullptr_facts, Tick restore_tick);

private:
  std::unique_ptr<rclcpp::Logger> logger_;

  std::unordered_map<long long, long long> fact_id_mapping_;
};
}  // namespace cx
