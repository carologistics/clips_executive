// TODO when restoring the rules the salience has to be taken from the field and not the pp since they could point to a defglobal that has changed in the meantime
// TODO CHECK IF ARRAYS IN THE TYPES IS BETTER THAN A TABLE REFERENCING THE THE FACT!?
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

// clang-format off
// agenda.h has to be loaded after clips.h
#include <clips_ns/clips.h>
#include <clips_ns/agenda.h>
#include <clips_ns/tmpltutl.h>
// clang-format on

#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>

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
        case SlotType::ExternalAddress: {
          const std::string addressStr = value.value["value"].get<std::string>();

          //TODO SKIP OPTION
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
  }
  //TODO FIND A WAY TO BUILD THEM CORRECTLY maybe a function that returns the CLIPS value correctly
  clips::SetResetGlobals(env.get(), false);

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

  std::vector<Deffacts> deffacts = load_deffacts(db);
  for (Deffacts deffact : deffacts) {
    RCLCPP_ERROR(
      *logger_, "DEFFACTS %s in %s created at %li ended at %li", deffact.name.c_str(),
      deffact.defmodule.c_str(), deffact.start_tick, deffact.end_tick.value_or(-1));
    for (TimedText value : deffact.value) {
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.c_str());
      clips::Build(env.get(), value.value.c_str());
    }
  }

  std::vector<Defrule> defrules = load_defrules(db);
  for (Defrule defrule : defrules) {
    RCLCPP_ERROR(
      *logger_, "DEFRULE %s in %s created at %li ended at %li", defrule.name.c_str(),
      defrule.defmodule.c_str(), defrule.start_tick, defrule.end_tick.value_or(-1));
    for (TimedText value : defrule.value) {
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.c_str());
      clips::Build(env.get(), value.value.c_str());
    }
  }

  std::vector<Fact> facts = load_facts(db);
  for (Fact fact : facts) {
    TimedFact value = fact.value.back();
    if (!value.value["type"].is_null()) {
      clips::Deftemplate * deftemplate =
        clips::FindDeftemplate(env.get(), value.value["deftemplate"].get<std::string>().c_str());
      if (deftemplate == nullptr) {
        clips::CLIPSLexeme * sym =
          clips::CreateSymbol(env.get(), value.value["deftemplate"].get<std::string>().c_str());
        deftemplate = clips::CreateImpliedDeftemplate(env.get(), sym, true);
        RCLCPP_ERROR(
          *logger_, "Could not find deftemplate %s",
          value.value["deftemplate"].get<std::string>().c_str());
      }
      clips::Fact * f = clips::CreateFact(deftemplate);

      clips::CLIPSValue cv;
      cv.multifieldValue = json_to_multifield(env.get(), value.value["value"]);

      clips::PutFactSlot(f, nullptr, &cv);
      f = clips::Assert(f);
      fact_id_mapping_[f->factIndex] = fact.fact_id;
      RCLCPP_WARN(
        *logger_, "ASSERTED FACT %lli of type %s: %s", f->factIndex,
        value.value["deftemplate"].get<std::string>().c_str(), value.value["value"].dump().c_str());

      continue;
    }
    RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.dump().c_str());

    clips::FactBuilder * fb =
      clips::CreateFactBuilder(env.get(), value.value["deftemplate"].get<std::string>().c_str());

    for (nlohmann::json slot : value.value["slots"]) {
      switch (slot["type"].get<SlotType>()) {
        case SlotType::String:
          clips::FBPutSlotString(
            fb, slot["name"].get<std::string>().c_str(), slot["value"].get<std::string>().c_str());
          break;
        case SlotType::Symbol:
          clips::FBPutSlotSymbol(
            fb, slot["name"].get<std::string>().c_str(), slot["value"].get<std::string>().c_str());
          break;
        case SlotType::ExternalAddress: {
          clips::FBPutSlotCLIPSExternalAddress(
            fb, slot["name"].get<std::string>().c_str(), nullptr);
          break;
        }
        case SlotType::Integer:
          clips::FBPutSlotInteger(
            fb, slot["name"].get<std::string>().c_str(), slot["value"].get<long long>());
          break;
        case SlotType::Float:
          clips::FBPutSlotFloat(
            fb, slot["name"].get<std::string>().c_str(), slot["value"].get<double>());
          break;
        case SlotType::FactAddress:
          //TODO HANDLE FACT MAPPINGS AFTERWARDS
          // clips::DefglobalSetString(
          //   global, value.value["value"].get<std::string>().c_str());
          break;
        case SlotType::Multifield: {
          clips::Multifield * multifield = json_to_multifield(env.get(), slot["value"]);
          clips::FBPutSlotMultifield(fb, slot["name"].get<std::string>().c_str(), multifield);
          break;
        }
      }
    }

    clips::Fact * f = clips::FBAssert(fb);
    fact_id_mapping_[f->factIndex] = fact.fact_id;

    clips::FBDispose(fb);
  }

  clips::Defmodule * theModule;
  clips::Activation * theActivation;

  for (theModule = clips::GetNextDefmodule(env.get(), NULL); theModule != NULL;
       theModule = clips::GetNextDefmodule(env.get(), theModule)) {
    clips::SetCurrentModule(env.get(), theModule);
    for (theActivation = clips::GetNextActivation(env.get(), NULL); theActivation != NULL;
         theActivation = clips::GetNextActivation(env.get(), theActivation)) {
      std::string name = theActivation->theRule->header.name->contents;
      std::string module_name =
        theActivation->theRule->header.whichModule->theModule->header.name->contents;

      std::vector<long long> basis;
      for (int i = 0; i < theActivation->basis->bcount; i++) {
        if (
          (get_nth_pm_match(theActivation->basis, i) != NULL) &&
          (get_nth_pm_match(theActivation->basis, i)->matchingItem != NULL)) {
          clips::PatternEntity * matchingItem =
            get_nth_pm_match(theActivation->basis, i)->matchingItem;
          basis.push_back(fact_id_mapping_[(((clips::Fact *)matchingItem))->factIndex]);
        }
      }
      if (rule_firing_exists_before_tick(db, module_name, name, basis, 10000)) {
        RCLCPP_ERROR(
          *logger_,
          "Removing activation of rule %s in module %s because it was already fired before %lli",
          name.c_str(), module_name.c_str(), basis.empty() ? -1 : basis.back());
        clips::RemoveActivation(env.get(), theActivation, true, true);
      }
    }
  }

  clips::Build(env.get(), "(assert cdb-restored)");

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

std::vector<Deffacts> CDBLoaderPlugin::load_deffacts(pqxx::connection & conn)
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

std::vector<Defrule> CDBLoaderPlugin::load_defrules(pqxx::connection & conn)
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

std::vector<Fact> CDBLoaderPlugin::load_facts(pqxx::connection & conn)
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

bool CDBLoaderPlugin::rule_firing_exists_before_tick(
  pqxx::connection & conn, const std::string & defmodule, const std::string & name,
  const std::vector<long long> & bases, long long before_tick)
{
  pqxx::work tx{conn};

  pqxx::params params;
  params.append(defmodule);
  params.append(name);
  params.append(bases);
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
