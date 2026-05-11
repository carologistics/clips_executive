// TODO when restoring the rules the salience has to be taken from the field and not the pp since they could point to a defglobal that has changed in the meantime
// Copyright (c) 2024-2026 Carologistics
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

#include <pqxx/pqxx>
#undef RANGES
// RANGES is defined in clips_ns/clips.h, which causes issues with
// pqxx/pqxx. It needs to be included before clips_ns/clips.h to avoid compilation errors.

#include <clips_ns/clips.h>

#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>
#include <nlohmann/json_fwd.hpp>

#include "cx_cdb_loader_plugin/cdb_loader_plugin.hpp"
#include "cx_cdb_loader_plugin/schema_sql.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx
{

CDBLoaderPlugin::CDBLoaderPlugin() {}

CDBLoaderPlugin::~CDBLoaderPlugin() {}

void CDBLoaderPlugin::initialize()
{
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = parent_.lock();
  if (!node) {
    return;
  }

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".hostname", rclcpp::ParameterValue("localhost"));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port", rclcpp::ParameterValue(5432));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".username", rclcpp::ParameterValue("anonymous"));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".password", rclcpp::ParameterValue(std::string()));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".database", rclcpp::ParameterValue(std::string()));
}

bool CDBLoaderPlugin::clips_env_init(std::shared_ptr<clips::Environment> & env)
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = parent_.lock();
  std::string hostname;
  int port;
  std::string username;
  std::string password;
  std::string database;
  node->get_parameter(plugin_name_ + ".hostname", hostname);
  node->get_parameter(plugin_name_ + ".port", port);
  node->get_parameter(plugin_name_ + ".username", username);
  node->get_parameter(plugin_name_ + ".password", password);
  node->get_parameter(plugin_name_ + ".database", database);

  time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y_%m_%dt%H_%M_%S");

  std::string env_name = CLIPSEnvContext::get_context(env)->env_name_;

  std::ostringstream conn;
  conn << "dbname=" << database << ' ' << "user=" << username << ' ' << "gssencmode=disable ";
  if (!password.empty()) {
    conn << "password=" << password << ' ';
  }
  conn << "port=" << port << ' ' << "host=" << hostname;
  pqxx::connection db{conn.str()};

  if (!db.is_open()) {
    RCLCPP_ERROR(*logger_, "Failed to connect to database with provided parameters");
    return false;
  }

  // Initializing databse views for easier parsing of composed types
  pqxx::work w(db);
  w.exec(kSchemaSql);
  w.commit();

  std::vector<Deftemplate> deftemplates = load_deftemplates(db);
  for (Deftemplate deftemplate : deftemplates) {
    RCLCPP_ERROR(
      *logger_, "DEFTEMPLATE %s in %s, created at %li ended at %li", deftemplate.name.c_str(),
      deftemplate.defmodule.c_str(), deftemplate.start_tick, deftemplate.end_tick.value_or(-1));
    for (TimedText value : deftemplate.value) {
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.c_str());
      clips::Build(env.get(), value.value.c_str());
    }
  }

  std::vector<Defglobal> defglobals = load_defglobals(db);
  for (Defglobal defglobal : defglobals) {
    RCLCPP_ERROR(
      *logger_, "DEFGLOBAL %s in %s created at %li ended at %li", defglobal.name.c_str(),
      defglobal.defmodule.c_str(), defglobal.start_tick, defglobal.end_tick.value_or(-1));
    // for(TimedFact value : defglobal.value)
    {
      TimedFact value = defglobal.value.back();
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.dump().c_str());
      clips::Build(
        env.get(),
        std::format("(defglobal {} ?*{}* = -1)", defglobal.defmodule, defglobal.name).c_str());
      clips::Defglobal * global = clips::FindDefglobal(
        env.get(), std::format("{}::{}", defglobal.defmodule, defglobal.name).c_str());
      switch (value.value["type"].get<SlotType>()) {
        case SlotType::String:
          clips::DefglobalSetString(global, value.value["value"].get<std::string>().c_str());
          break;
        case SlotType::Symbol:
          clips::DefglobalSetSymbol(global, value.value["value"].get<std::string>().c_str());
          break;
        case SlotType::Void: {
          clips::CLIPSValue v;
          v.voidValue = env.get()->VoidConstant;
          clips::DefglobalSetValue(global, &v);
          break;
        }
        case SlotType::ExternalAddress: {
          const std::string addressStr = value.value["value"].get<std::string>();

          std::uintptr_t rawAddress =
            static_cast<std::uintptr_t>(std::stoull(addressStr, nullptr, 0));

          void * address = reinterpret_cast<void *>(rawAddress);
          clips::DefglobalSetCLIPSExternalAddress(
            global, clips::CreateCExternalAddress(env.get(), address));
          break;
        }
        case SlotType::Integer:
          clips::DefglobalSetInteger(global, value.value["value"].get<long long>());
          break;
        case SlotType::Float:
          clips::DefglobalSetFloat(global, value.value["value"].get<double>());
          break;
        case SlotType::FactAddress:
          //TODO HANDLE FACT MAPPINGS AFTERWARDS
          // clips::DefglobalSetString(
          //   global, value.value["value"].get<std::string>().c_str());
          break;
        case SlotType::Multifield: {
          clips::Multifield * multifield = json_to_multifield(env.get(), value.value["value"]);
          clips::DefglobalSetMultifield(global, multifield);
          break;
        }
      }
    }
    RCLCPP_ERROR(*logger_, "JAWOHL");
    clips::ShowDefglobals(env.get(), clips::STDOUT, NULL);
  }

  std::vector<Deffunction> deffunctions = load_deffunctions(db);
  for (Deffunction deffunction : deffunctions) {
    RCLCPP_ERROR(
      *logger_, "DEFFUNCTION %s in %s created at %li ended at %li", deffunction.name.c_str(),
      deffunction.defmodule.c_str(), deffunction.start_tick, deffunction.end_tick.value_or(-1));
    for (TimedText value : deffunction.value) {
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.c_str());
      clips::Build(env.get(), value.value.c_str());
    }
  }

  return true;
}

template <typename T>
std::optional<T> CDBLoaderPlugin::optional_field(const pqxx::row & row, const char * column)
{
  if (row[column].is_null()) {
    return std::nullopt;
  }

  return row[column].as<T>();
}

std::vector<TimedFact> CDBLoaderPlugin::parse_timed_fact_history(const pqxx::row & row)
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

std::vector<TimedText> CDBLoaderPlugin::parse_timed_text_history(const pqxx::row & row)
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

std::vector<Deftemplate> CDBLoaderPlugin::load_deftemplates(pqxx::connection & conn)
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

std::vector<Defglobal> CDBLoaderPlugin::load_defglobals(pqxx::connection & conn)
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

std::vector<Deffunction> CDBLoaderPlugin::load_deffunctions(pqxx::connection & conn)
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

clips::Multifield * CDBLoaderPlugin::json_to_multifield(
  clips::Environment * env, const nlohmann::json & json_array)
{
  clips::MultifieldBuilder * mb = clips::CreateMultifieldBuilder(env, json_array.size());

  for (const nlohmann::json & element : json_array) {
    append_json_to_multifield_builder(env, mb, element);
  }

  clips::Multifield * multifield = clips::MBCreate(mb);
  clips::MBDispose(mb);
  return multifield;
}

void CDBLoaderPlugin::append_json_to_multifield_builder(
  clips::Environment * env, clips::MultifieldBuilder * mb, const nlohmann::json & valueJson)
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
    case SlotType::ExternalAddress: {
      const std::string addressStr = valueJson["value"].get<std::string>();

      const std::uintptr_t rawAddress =
        static_cast<std::uintptr_t>(std::stoull(addressStr, nullptr, 0));

      void * address = reinterpret_cast<void *>(rawAddress);

      clips::MBAppendCLIPSExternalAddress(mb, clips::CreateCExternalAddress(env, address));

      break;
    }
    case SlotType::Void: {
      clips::CLIPSValue v;
      v.voidValue = env->VoidConstant;

      clips::MBAppend(mb, &v);
      break;
    }
    case SlotType::Multifield: {
      clips::Multifield * nested = json_to_multifield(env, valueJson);

      clips::MBAppendMultifield(mb, nested);
      break;
    }
    case SlotType::FactAddress:
      //TODO
      throw std::runtime_error("FactAddress deserialization into multifield not implemented");
  }
}

bool CDBLoaderPlugin::clips_env_destroyed(std::shared_ptr<clips::Environment> & env)
{
  CLIPSEnvContext * context = CLIPSEnvContext::get_context(env.get());
  RCLCPP_INFO(
    *logger_, "Destroying CDBLoaderPlugin for environment %s", context->env_name_.c_str());
  return true;
}
}  // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::CDBLoaderPlugin, cx::ClipsPlugin)
