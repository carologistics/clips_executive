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

#include <regex>

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

std::vector<Deftemplate> load_deftemplates(pqxx::connection & conn, Tick restore_tick)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec_params(
    R"sql(
      SELECT *
      FROM deftemplates_cpp
      WHERE start_tick <= $1
        AND (end_tick IS NULL OR end_tick > $1)
      ORDER BY module, name, start_tick
    )sql",
    restore_tick);

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

long long get_end_tick_for_run(pqxx::connection & db, long long run_index)
{
  const bool count_from_end = run_index < 0;
  const long long offset = count_from_end ? (-(run_index)) : run_index;

  const std::string query = count_from_end ? "SELECT run_number, end_tick "
                                             "FROM time_lookup "
                                             "ORDER BY run_number DESC "
                                             "LIMIT 1 OFFSET $1"
                                           : "SELECT run_number, end_tick "
                                             "FROM time_lookup "
                                             "ORDER BY run_number ASC "
                                             "LIMIT 1 OFFSET $1";

  pqxx::work w(db);
  pqxx::result result = w.exec_params(query, offset);
  w.commit();

  if (result.empty()) {
    throw std::runtime_error(
      "Could not resolve restore_run index " + std::to_string(run_index) +
      ": no matching run exists.");
  }

  if (result[0]["end_tick"].is_null()) {
    throw std::runtime_error(
      "Could not resolve restore_run index " + std::to_string(run_index) + ": selected run " +
      result[0]["run_number"].as<std::string>() + " has no end_tick.");
  }

  return result[0]["end_tick"].as<long long>();
}

long long resolve_restore_tick(pqxx::connection & db, long long restore_tick_index)
{
  if (restore_tick_index >= 0) {
    return restore_tick_index;
  }

  pqxx::work w(db);
  pqxx::result result = w.exec("SELECT cdb_max_tick() AS max_tick");
  w.commit();

  if (result.empty() || result[0]["max_tick"].is_null()) {
    throw std::runtime_error("Could not resolve restore_tick: cdb_max_tick() returned no value.");
  }

  const long long max_tick = result[0]["max_tick"].as<long long>();
  const long long resolved_tick = max_tick + restore_tick_index + 1;

  if (resolved_tick < 0) {
    throw std::runtime_error(
      "Could not resolve restore_tick " + std::to_string(restore_tick_index) +
      ": resolved tick would be " + std::to_string(resolved_tick) + ".");
  }

  return resolved_tick;
}

long long resolve_restore_time(
  pqxx::connection & db, const std::string & restore_time, const rclcpp::Logger & logger)
{
  if (restore_time.empty()) {
    throw std::runtime_error("restore_time must not be empty.");
  }

  pqxx::work w(db);
  pqxx::result result;

  if (restore_time.starts_with("T+")) {
    const std::string relative_time = restore_time.substr(2);

    if (relative_time.empty()) {
      throw std::runtime_error("restore_time starts with T+ but contains no interval.");
    }

    result = w.exec_params(
      R"sql(
        WITH target_time AS (
          SELECT
            start_time + $1::interval AS value
          FROM time_lookup
          ORDER BY run_number ASC
          LIMIT 1
        )
        SELECT
          tl.run_number,
          tl.end_time,
          tl.end_tick,
          target_time.value AS requested_time
        FROM time_lookup tl
        CROSS JOIN target_time
        WHERE tl.end_time IS NOT NULL
          AND tl.end_tick IS NOT NULL
        ORDER BY
          ABS(EXTRACT(EPOCH FROM (tl.end_time - target_time.value))) ASC,
          tl.run_number ASC
        LIMIT 1
      )sql",
      relative_time);
  } else {
    static const std::regex timestamp_regex(
      R"(^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}(?:\.[0-9]+)?(?:Z|[+-][0-9]{2}(?::[0-9]{2})?)$)");
    if (!std::regex_match(restore_time, timestamp_regex)) {
      throw std::runtime_error(
        "restore_time format is invalid: '" + restore_time +
        "'. It must be either an absolute timestamp in ISO 8601 format or a relative time starting "
        "with 'T+'.");
    }
    result = w.exec_params(
      R"sql(
        WITH target_time AS (
          SELECT
            ($1::timestamptz AT TIME ZONE 'UTC') AS value
        )
        SELECT
          tl.run_number,
          tl.end_time,
          tl.end_tick,
          target_time.value AS requested_time
        FROM time_lookup tl
        CROSS JOIN target_time
        WHERE tl.end_time IS NOT NULL
          AND tl.end_tick IS NOT NULL
        ORDER BY
          ABS(EXTRACT(EPOCH FROM (tl.end_time - target_time.value))) ASC,
          tl.run_number ASC
        LIMIT 1
      )sql",
      restore_time);
  }

  w.commit();

  if (result.empty()) {
    throw std::runtime_error(
      "Could not resolve restore_time '" + restore_time +
      "': no finished run with end_time and end_tick exists.");
  }

  const long long run_number = result[0]["run_number"].as<long long>();
  const long long end_tick = result[0]["end_tick"].as<long long>();
  const std::string requested_time = result[0]["requested_time"].as<std::string>();
  const std::string selected_end_time = result[0]["end_time"].as<std::string>();

  RCLCPP_INFO(
    logger,
    "Resolved restore_time '%s' to requested timestamp '%s'. "
    "Closest run end is run %lli at '%s', using end_tick %lli.",
    restore_time.c_str(), requested_time.c_str(), run_number, selected_end_time.c_str(), end_tick);

  return end_tick;
}

std::optional<TimedText> latest_timed_text_at_tick(
  const std::vector<TimedText> & values, Tick restore_tick)
{
  const TimedText * best = nullptr;

  for (const TimedText & value : values) {
    if (value.tick <= restore_tick) {
      if (best == nullptr || value.tick > best->tick) {
        best = &value;
      }
    }
  }

  if (best == nullptr) {
    return std::nullopt;
  }

  return *best;
}

std::vector<Defmodule> load_defmodules(pqxx::connection & conn, Tick restore_tick)
{
  pqxx::work tx{conn};

  pqxx::result rows = tx.exec_params(
    R"sql(
      SELECT name, value, start_tick
      FROM defmodules
      WHERE start_tick <= $1
      ORDER BY name, start_tick
    )sql",
    restore_tick);

  std::vector<Defmodule> result;
  result.reserve(rows.size());

  for (const pqxx::row & row : rows) {
    result.push_back(Defmodule{
      .name = row["name"].as<std::string>(),
      .value = row["value"].as<std::string>(),
      .start_tick = row["start_tick"].as<Tick>()});
  }

  tx.commit();

  return result;
}

}  // namespace cx
