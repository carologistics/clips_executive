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

// clang-format off
#include <pqxx/pqxx>
#undef RANGES
// RANGES is defined in clips_ns/clips.h, which causes issues with
// pqxx/pqxx. It needs to be included before clips_ns/clips.h to avoid compilation errors.

#include <clips_ns/clips.h>
// clang-format on

#include <cx_cdb_loader_plugin/parser.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cx
{

using json = nlohmann::json;
using Tick = long long;

struct TimedFact
{
  Tick tick;
  json value;
};

struct TimedText
{
  Tick tick;
  std::string value;
};

struct TimeLookup
{
  long long run_number;
  std::string start_time;
  std::optional<std::string> end_time;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Fact
{
  long long fact_id;
  std::string defmodule;
  std::string deftemplate_name;
  TimedFact value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Defglobal
{
  std::string name;
  std::string defmodule;
  TimedFact value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Defrule
{
  int rule_id;
  std::string name;
  std::string defmodule;
  TimedText value;
  int salience;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct RuleFiring
{
  int rule_id;
  std::vector<long long> base;
  Tick tick;
};

struct Deffunction
{
  std::string name;
  std::string defmodule;
  TimedText value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Deftemplate
{
  std::string name;
  std::string defmodule;
  TimedText value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Deffacts
{
  std::string name;
  std::string defmodule;
  TimedText value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Defmodule
{
  std::string name;
  std::string value;
  Tick start_tick;
};

struct Plugin
{
  std::string name;
  Tick start_tick;
  std::optional<Tick> end_tick;
  json config;
};

enum class SlotType
{
  Float,
  Integer,
  String,
  Symbol,
  ExternalAddress,
  Multifield,
  FactAddress
};

NLOHMANN_JSON_SERIALIZE_ENUM(
  SlotType, {{SlotType::Float, "FLOAT"},
             {SlotType::Integer, "INTEGER"},
             {SlotType::String, "STRING"},
             {SlotType::Symbol, "SYMBOL"},
             {SlotType::ExternalAddress, "EXTERNAL_ADDRESS"},
             {SlotType::Multifield, "MULTIFIELD"},
             {SlotType::FactAddress, "FACT_ADDRESS"}})

clips::Multifield * json_to_multifield(
  clips::Environment * env, const nlohmann::json & json_array, clips::Fact * tmp_fact,
  const RegexConfig & config, bool & contains_fact);

void append_json_to_multifield_builder(
  clips::Environment * env, clips::MultifieldBuilder * mb, const nlohmann::json & valueJson,
  clips::Fact * tmp_fact, const RegexConfig & config, bool & contains_fact);

void append_json_to_multifield_builder(
  clips::Environment * env, clips::MultifieldBuilder * mb, const nlohmann::json & valueJson,
  std::unordered_map<long long, clips::Fact *> & id_to_fact_ptr,
  std::vector<clips::Fact *> & created_nullptr_facts, const RegexConfig & config);

clips::Multifield * json_to_multifield(
  clips::Environment * env, const nlohmann::json & json,
  std::unordered_map<long long, clips::Fact *> & id_to_fact_ptr,
  std::vector<clips::Fact *> & created_nullptr_facts, const RegexConfig & config);

clips::Fact * get_nullptr_fact(
  clips::Environment * env, long long fact_id,
  std::unordered_map<long long, clips::Fact *> & id_to_fact_ptr,
  std::vector<clips::Fact *> & created_nullptr_facts);

template <typename T>
std::optional<T> optional_field(const pqxx::row & row, const char * column);
TimedText parse_timed_text(const pqxx::row & row);
TimedFact parse_timed_fact(const pqxx::row & row);
std::vector<Defmodule> load_defmodules(pqxx::connection & conn, Tick restore_tick);
std::vector<Deftemplate> load_deftemplates(
  pqxx::connection & conn, const std::string & defmodule, Tick restore_tick);
std::vector<Defglobal> load_defglobals(
  pqxx::connection & conn, const std::string & defmodule, Tick restore_tick,
  bool skip_external_addresses);
std::vector<Deffunction> load_deffunctions(
  pqxx::connection & conn, const std::string & defmodule, Tick restore_tick);
std::vector<Defrule> load_defrules(
  pqxx::connection & conn, const std::string & defmodule, Tick restore_tick);
std::vector<Deffacts> load_deffacts(
  pqxx::connection & conn, const std::string & defmodule, Tick restore_tick);
std::vector<Fact> load_facts(
  pqxx::connection & conn, Tick restore_tick, bool skip_external_addresses);

bool rule_firing_exists_before_tick(
  pqxx::connection & conn, const std::string & defmodule, const std::string & name,
  const std::vector<std::optional<long long>> & basis, long long before_tick);
long long get_end_tick_for_run(pqxx::connection & db, long long run_index);
long long resolve_restore_tick(pqxx::connection & db, long long restore_tick_index);
long long resolve_restore_time(
  pqxx::connection & db, const std::string & restore_time, const rclcpp::Logger & logger);

}  // namespace cx
