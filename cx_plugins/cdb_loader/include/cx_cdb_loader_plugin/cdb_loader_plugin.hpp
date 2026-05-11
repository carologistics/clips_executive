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

#include "cx_plugin/clips_plugin.hpp"

namespace cx
{

using json = nlohmann::json;
using Tick = std::int64_t;

// Store timestamptz as string first.
// Example PostgreSQL output: "2026-05-11 12:30:00+09"
using TimestampText = std::string;

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
  std::int64_t run_number;
  TimestampText start_time;
  std::optional<TimestampText> end_time;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Fact
{
  std::int64_t fact_id;
  std::string deftemplate;
  std::vector<TimedFact> value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Defglobal
{
  std::string name;
  std::string defmodule;
  std::vector<TimedFact> value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Defrule
{
  int rule_id;
  std::string name;
  std::string defmodule;
  std::vector<TimedText> value;
  int salience;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct RuleFiring
{
  int rule_id;
  std::vector<std::int64_t> base;
  Tick tick;
};

struct Deffunction
{
  std::string name;
  std::string defmodule;
  std::vector<TimedText> value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Deftemplate
{
  std::string name;
  std::string defmodule;
  std::vector<TimedText> value;
  Tick start_tick;
  std::optional<Tick> end_tick;
};

struct Deffacts
{
  std::string name;
  std::string defmodule;
  std::vector<TimedText> value;
  Tick start_tick;
  std::optional<Tick> end_tick;
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
  Void,
  Multifield,
  FactAddress
};

NLOHMANN_JSON_SERIALIZE_ENUM(
  SlotType, {{SlotType::Float, "FLOAT"},
             {SlotType::Integer, "INTEGER"},
             {SlotType::String, "STRING"},
             {SlotType::Symbol, "SYMBOL"},
             {SlotType::ExternalAddress, "EXTERNAL_ADDRESS"},
             {SlotType::Void, "VOID"},
             {SlotType::Multifield, "MULTIFIELD"},
             {SlotType::FactAddress, "FACT_ADDRESS"}})

class CDBLoaderPlugin : public ClipsPlugin
{
public:
  CDBLoaderPlugin();
  ~CDBLoaderPlugin();

  void initialize() override;

  bool clips_env_init(std::shared_ptr<clips::Environment> & env) override;
  bool clips_env_destroyed(std::shared_ptr<clips::Environment> & env) override;

private:
  std::unique_ptr<rclcpp::Logger> logger_;

  template <typename T>
  std::optional<T> optional_field(const pqxx::row & row, const char * column);
  std::vector<TimedText> parse_timed_text_history(const pqxx::row & row);
  std::vector<TimedFact> parse_timed_fact_history(const pqxx::row & row);
  std::vector<Deftemplate> load_deftemplates(pqxx::connection & conn);
  std::vector<Defglobal> load_defglobals(pqxx::connection & conn);
  std::vector<Deffunction> load_deffunctions(pqxx::connection & conn);

  clips::Multifield * json_to_multifield(clips::Environment * env, const nlohmann::json & json);

  void append_json_to_multifield_builder(
    clips::Environment * env, clips::MultifieldBuilder * mb, const nlohmann::json & valueJson);
  //
};
}  // namespace cx
