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

#include "cx_cdb_loader_plugin/helpers.hpp"

namespace cx
{
clips::Multifield * json_to_multifield(
  clips::Environment * env, const nlohmann::json & json_array,
  std::unordered_map<long long, clips::Fact *> & id_to_fact_ptr,
  std::vector<clips::Fact *> & created_nullptr_facts)
{
  clips::MultifieldBuilder * mb = clips::CreateMultifieldBuilder(env, json_array.size());

  for (const nlohmann::json & element : json_array) {
    append_json_to_multifield_builder(env, mb, element, id_to_fact_ptr, created_nullptr_facts);
  }

  clips::Multifield * multifield = clips::MBCreate(mb);
  clips::MBDispose(mb);
  return multifield;
}

void append_json_to_multifield_builder(
  clips::Environment * env, clips::MultifieldBuilder * mb, const nlohmann::json & valueJson,
  std::unordered_map<long long, clips::Fact *> & id_to_fact_ptr,
  std::vector<clips::Fact *> & created_nullptr_facts)
{
  switch (valueJson["type"].get<SlotType>()) {
    case SlotType::String:
      clips::MBAppendString(mb, valueJson["value"].get<std::string>().c_str());
      break;
    case SlotType::Symbol:
      clips::MBAppendSymbol(mb, valueJson["value"].get<std::string>().c_str());
      break;
    case SlotType::Integer:
      clips::MBAppendInteger(mb, valueJson["value"].get<long long>());
      break;
    case SlotType::Float:
      clips::MBAppendFloat(mb, valueJson["value"].get<double>());
      break;
    case SlotType::ExternalAddress:
      clips::MBAppendCLIPSExternalAddress(mb, clips::CreateCExternalAddress(env, nullptr));
      break;
    case SlotType::Multifield: {
      clips::Multifield * nested =
        json_to_multifield(env, valueJson, id_to_fact_ptr, created_nullptr_facts);
      clips::MBAppendMultifield(mb, nested);
      break;
    }
    case SlotType::FactAddress:
      if (id_to_fact_ptr.contains(valueJson["value"].get<long long>())) {
        clips::MBAppendFact(mb, id_to_fact_ptr[valueJson["value"].get<long long>()]);
      } else {
        clips::Fact * null_fact = get_nullptr_fact(
          env, valueJson["value"].get<long long>(), id_to_fact_ptr, created_nullptr_facts);
        clips::MBAppendFact(mb, null_fact);
      }
  }
}

clips::Fact * get_nullptr_fact(
  clips::Environment * env, long long fact_id,
  std::unordered_map<long long, clips::Fact *> & id_to_fact_ptr,
  std::vector<clips::Fact *> & created_nullptr_facts)
{
  clips::Fact * null_fact = clips::AssertString(env, std::format("(nullptr-{})", fact_id).c_str());
  id_to_fact_ptr[fact_id] = null_fact;
  created_nullptr_facts.push_back(null_fact);
  return null_fact;
}

template <typename T>
std::optional<T> optional_field(const pqxx::row & row, const char * column)
{
  if (row[column].is_null()) {
    return std::nullopt;
  }

  return row[column].as<T>();
}

std::vector<TimedFact> parse_timed_fact_history(const pqxx::row & row)
{
  std::vector<TimedFact> result;

  if (row["value_history"].is_null()) {
    return result;
  }

  json history = json::parse(row["value_history"].as<std::string>());

  for (const auto & entry : history) {
    result.push_back(TimedFact{.tick = entry.at("tick").get<Tick>(), .value = entry.at("value")});
  }

  return result;
}

std::vector<TimedText> parse_timed_text_history(const pqxx::row & row)
{
  std::vector<TimedText> history;

  if (row["value_history"].is_null()) {
    return history;
  }

  json entries = json::parse(row["value_history"].as<std::string>());

  for (const auto & entry : entries) {
    TimedText item{
      .tick = entry.at("tick").get<Tick>(),
      .value = entry.at("value").is_null() ? "" : entry.at("value").get<std::string>()};

    history.push_back(std::move(item));
  }

  return history;
}

std::vector<Deftemplate> load_deftemplates(pqxx::connection & conn)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec("SELECT * FROM deftemplates_cpp");

  std::vector<Deftemplate> result;
  result.reserve(rows.size());

  for (const pqxx::row & row : rows) {
    Deftemplate item{
      .name = row["name"].as<std::string>(),
      .defmodule = row["module"].as<std::string>(),
      .value = parse_timed_text_history(row),
      .start_tick = row["start_tick"].as<Tick>(),
      .end_tick = optional_field<Tick>(row, "end_tick")};

    result.push_back(std::move(item));
  }

  tx.commit();

  return result;
}

std::vector<Defglobal> load_defglobals(pqxx::connection & conn)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec("SELECT * FROM defglobals_cpp");

  std::vector<Defglobal> result;
  result.reserve(rows.size());

  for (const pqxx::row & row : rows) {
    result.push_back(Defglobal{
      .name = row["name"].as<std::string>(),
      .defmodule = row["module"].as<std::string>(),
      .value = parse_timed_fact_history(row),
      .start_tick = row["start_tick"].as<Tick>(),
      .end_tick = optional_field<Tick>(row, "end_tick")});
  }

  tx.commit();

  return result;
}

std::vector<Deffunction> load_deffunctions(pqxx::connection & conn)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec("SELECT * FROM deffunctions_cpp");

  std::vector<Deffunction> result;
  result.reserve(rows.size());

  for (const pqxx::row & row : rows) {
    result.push_back(Deffunction{
      .name = row["name"].as<std::string>(),
      .defmodule = row["module"].as<std::string>(),
      .value = parse_timed_text_history(row),
      .start_tick = row["start_tick"].as<Tick>(),
      .end_tick = optional_field<Tick>(row, "end_tick")});
  }

  tx.commit();

  return result;
}

std::vector<Deffacts> load_deffacts(pqxx::connection & conn)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec("SELECT * FROM deffacts_cpp");

  std::vector<Deffacts> result;
  result.reserve(rows.size());

  for (const pqxx::row & row : rows) {
    result.push_back(Deffacts{
      .name = row["name"].as<std::string>(),
      .defmodule = row["module"].as<std::string>(),
      .value = parse_timed_text_history(row),
      .start_tick = row["start_tick"].as<Tick>(),
      .end_tick = optional_field<Tick>(row, "end_tick")});
  }

  tx.commit();

  return result;
}

std::vector<Defrule> load_defrules(pqxx::connection & conn)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec("SELECT * FROM defrules_cpp");

  std::vector<Defrule> result;
  result.reserve(rows.size());

  for (const pqxx::row & row : rows) {
    result.push_back(Defrule{
      .rule_id = row["rule_id"].as<int>(),
      .name = row["name"].as<std::string>(),
      .defmodule = row["module"].as<std::string>(),
      .value = parse_timed_text_history(row),
      .salience = row["salience"].as<int>(),
      .start_tick = row["start_tick"].as<Tick>(),
      .end_tick = optional_field<Tick>(row, "end_tick")});
  }

  tx.commit();

  return result;
}

std::vector<Fact> load_facts(pqxx::connection & conn)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec("SELECT * FROM facts_cpp");

  std::vector<Fact> result;
  result.reserve(rows.size());

  for (const pqxx::row & row : rows) {
    result.push_back(Fact{
      .fact_id = row["fact_id"].as<std::int64_t>(),
      .value = parse_timed_fact_history(row),
      .start_tick = row["start_tick"].as<Tick>(),
      .end_tick = optional_field<Tick>(row, "end_tick")});
  }

  tx.commit();

  return result;
}

bool rule_firing_exists_before_tick(
  pqxx::connection & conn, const std::string & defmodule, const std::string & name,
  const std::vector<std::optional<long long>> & basis, long long before_tick)
{
  pqxx::work tx{conn};

  std::ostringstream array_str;
  array_str << "{";
  for (size_t i = 0; i < basis.size(); ++i) {
    if (i > 0) {
      array_str << ",";
    }

    if (basis[i].has_value()) {
      array_str << *basis[i];
    } else {
      array_str << "NULL";
    }
  }
  array_str << "}";

  pqxx::params params;
  params.append(defmodule);
  params.append(name);
  params.append(array_str.str());
  params.append(before_tick);

  bool exists = tx.query_value<bool>(
    R"SQL(
            SELECT EXISTS (
                SELECT 1
                FROM defrules r
                JOIN rule_firing rf
                    ON rf.rule_id = r.rule_id
                WHERE r.module = $1
                  AND r.name = $2
                  AND rf.tick < $4
                  AND ARRAY(
                        SELECT x
                        FROM unnest(COALESCE(rf.base, '{}'::bigint[])) AS u(x)
                        ORDER BY x
                      )
                      =
                      ARRAY(
                        SELECT x
                        FROM unnest($3::bigint[]) AS u(x)
                        ORDER BY x
                      )
            )
        )SQL",
    params);

  tx.commit();
  return exists;
}
}  // namespace cx
