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

// TODO DEFFACTS MANUAL
// TODO PLUGIN CHECK
// TODO Check weather the db exists porperly
// TODO SALIENCE OVERRIDE

#include <pqxx/pqxx>

#include "cx_cdb_loader_plugin/helpers.hpp"
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
#include "cx_cdb_loader_plugin/parser.hpp"
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

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = parent_.lock();
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

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".restore_run", rclcpp::ParameterValue(std::string()));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".restore_tick", rclcpp::ParameterValue(std::string()));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".restore_time", rclcpp::ParameterValue(std::string()));
}

void CDBLoaderPlugin::assert_deftemplates(
  clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
  const std::string & defmodule, Tick restore_tick)
{
  std::vector<Deftemplate> deftemplates = load_deftemplates(conn, defmodule, restore_tick);
  filter_in_place(
    deftemplates, config.deftemplate_rules,
    [](const Deftemplate & value) -> const std::string & { return value.name; });
  for (Deftemplate deftemplate : deftemplates) {
    RCLCPP_DEBUG(
      *logger_, "ASSERTING DEFTEMPLATE %s in %s, with value from tick %lli: %s",
      deftemplate.name.c_str(), deftemplate.defmodule.c_str(), deftemplate.value.tick,
      deftemplate.value.value.c_str());
    clips::Build(env, deftemplate.value.value.c_str());
  }
}

void CDBLoaderPlugin::assert_deffunctions(
  clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
  const std::string & defmodule, Tick restore_tick)
{
  std::vector<Deffunction> deffunctions = load_deffunctions(conn, defmodule, restore_tick);
  filter_in_place(
    deffunctions, config.deffunction_rules,
    [](const Deffunction & value) -> const std::string & { return value.name; });
  for (Deffunction deffunction : deffunctions) {
    RCLCPP_DEBUG(
      *logger_, "ASSERTING DEFFUNCTION %s in %s with value from tick %lli: %s",
      deffunction.name.c_str(), deffunction.defmodule.c_str(), deffunction.value.tick,
      deffunction.value.value.c_str());
    clips::Build(env, deffunction.value.value.c_str());
  }
}

void CDBLoaderPlugin::assert_defrules(
  clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
  const std::string & defmodule, Tick restore_tick)
{
  std::vector<Defrule> defrules = load_defrules(conn, defmodule, restore_tick);
  filter_in_place(defrules, config.defrule_rules, [](const Defrule & value) -> const std::string & {
    return value.name;
  });
  for (Defrule defrule : defrules) {
    RCLCPP_DEBUG(
      *logger_, "ASSERTING DEFRULE %s in %s with value from tick %lli: %s", defrule.name.c_str(),
      defrule.defmodule.c_str(), defrule.value.tick, defrule.value.value.c_str());
    clips::Build(env, defrule.value.value.c_str());
  }
}

void CDBLoaderPlugin::assert_deffacts(
  clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
  const std::string & defmodule, Tick restore_tick)
{
  std::vector<Deffacts> deffacts = load_deffacts(conn, defmodule, restore_tick);
  filter_in_place(
    deffacts, config.deffacts_rules,
    [](const Deffacts & value) -> const std::string & { return value.name; });
  for (Deffacts deffact : deffacts) {
    RCLCPP_DEBUG(
      *logger_, "ASSERTING DEFFACTS %s in %s with value from tick %lli: %s", deffact.name.c_str(),
      deffact.defmodule.c_str(), deffact.value.tick, deffact.value.value.c_str());
    clips::Build(env, deffact.value.value.c_str());
  }
}

void CDBLoaderPlugin::assert_defglobals(
  clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
  std::vector<Defglobal> fact_pointing_defglobals, clips::Fact * tmp_fact,
  const std::string & defmodule, Tick restore_tick)
{
  std::vector<Defglobal> defglobals =
    load_defglobals(conn, defmodule, restore_tick, config.skip_external_address_values);
  filter_in_place(
    defglobals, config.defglobal_rules,
    [](const Defglobal & value) -> const std::string & { return value.name; });
  for (Defglobal defglobal : defglobals) {
    RCLCPP_DEBUG(
      *logger_, "ASSERTING DEFGLOBAL %s in %s to value from tick %lli: %s", defglobal.name.c_str(),
      defglobal.defmodule.c_str(), defglobal.value.tick, defglobal.value.value.dump().c_str());
    clips::Build(env, std::format("(defglobal {} ?*{}* = nil)", defmodule, defglobal.name).c_str());
    clips::Defglobal * global =
      clips::FindDefglobal(env, std::format("{}::{}", defglobal.defmodule, defglobal.name).c_str());
    switch (defglobal.value.value["type"].get<SlotType>()) {
      case SlotType::String:
        clips::DefglobalSetString(
          global, defglobal.value.value["value"].get<std::string>().c_str());
        break;
      case SlotType::Symbol:
        clips::DefglobalSetSymbol(
          global, defglobal.value.value["value"].get<std::string>().c_str());
        break;
      case SlotType::ExternalAddress: {
        if (config.external_address_values_as_nullptr) {
          clips::DefglobalSetCLIPSExternalAddress(
            global, clips::CreateCExternalAddress(env, nullptr));
        } else {
          const std::string addressStr = defglobal.value.value["value"].get<std::string>();

          std::uintptr_t rawAddress =
            static_cast<std::uintptr_t>(std::stoull(addressStr, nullptr, 0));

          void * address = reinterpret_cast<void *>(rawAddress);

          clips::DefglobalSetCLIPSExternalAddress(
            global, clips::CreateCExternalAddress(env, address));
        }
        break;
      }
      case SlotType::Integer:
        clips::DefglobalSetInteger(global, defglobal.value.value["value"].get<long long>());
        break;
      case SlotType::Float:
        clips::DefglobalSetFloat(global, defglobal.value.value["value"].get<double>());
        break;
      case SlotType::FactAddress:
        clips::DefglobalSetFact(global, tmp_fact);
        fact_pointing_defglobals.push_back(defglobal);
        break;
      case SlotType::Multifield: {
        bool contains_fact = false;
        clips::Multifield * multifield =
          json_to_multifield(env, defglobal.value.value["value"], tmp_fact, config, contains_fact);
        if (contains_fact) {
          fact_pointing_defglobals.push_back(defglobal);
        }
        clips::DefglobalSetMultifield(global, multifield);
        break;
      }
    }
  }
}

void CDBLoaderPlugin::update_defglobals(
  clips::Environment * env, std::vector<Defglobal> defglobals, const RegexConfig & config,
  std::unordered_map<long long, clips::Fact *> id_to_fact_ptr,
  std::vector<clips::Fact *> created_nullptr_facts)
{
  for (Defglobal defglobal : defglobals) {
    clips::Defglobal * global =
      clips::FindDefglobal(env, std::format("{}::{}", defglobal.defmodule, defglobal.name).c_str());
    switch (defglobal.value.value["type"].get<SlotType>()) {
      case SlotType::FactAddress: {
        if (id_to_fact_ptr.contains(defglobal.value.value["value"].get<long long>())) {
          clips::DefglobalSetFact(
            global, id_to_fact_ptr[defglobal.value.value["value"].get<long long>()]);
        } else {
          clips::Fact * null_fact = get_nullptr_fact(
            env, defglobal.value.value["value"].get<long long>(), id_to_fact_ptr,
            created_nullptr_facts);
          RCLCPP_WARN(
            *logger_,
            "Defglobal %s::%s references a fact %lli that is not present in the current "
            "environment, setting it to nullptr",
            defglobal.defmodule.c_str(), defglobal.name.c_str(),
            defglobal.value.value["value"].get<long long>());
          clips::DefglobalSetFact(global, null_fact);
        }
        break;
      }
      case SlotType::Multifield: {
        clips::Multifield * multifield = json_to_multifield(
          env, defglobal.value.value["value"], id_to_fact_ptr, created_nullptr_facts, config);
        clips::DefglobalSetMultifield(global, multifield);
        break;
      }
      default:
        break;
    }
  }
}

void CDBLoaderPlugin::assert_facts(
  clips::Environment * env, pqxx::connection & conn, const RegexConfig & config,
  std::unordered_map<long long, clips::Fact *> id_to_fact_ptr,
  std::vector<clips::Fact *> created_nullptr_facts, Tick restore_tick)
{
  std::vector<Fact> facts = load_facts(conn, restore_tick, config.skip_external_address_values);
  filter_in_place(facts, config.fact_rules, [](const Fact & value) -> const std::string & {
    return value.deftemplate_name;
  });
  for (Fact fact : facts) {
    const std::string qualified_name = std::format("{}::{}", fact.defmodule, fact.deftemplate_name);
    clips::Deftemplate * deftemplate = clips::FindDeftemplate(env, qualified_name.c_str());
    if (!fact.value.value["type"].is_null() && deftemplate == nullptr) {
      clips::Defmodule * defmodule = clips::FindDefmodule(env, fact.defmodule.c_str());
      if (defmodule == nullptr) {
      }

      clips::CLIPSLexeme * sym = clips::CreateSymbol(env, qualified_name.c_str());
      deftemplate = clips::CreateImpliedDeftemplate(env, sym, true);
    }
    clips::Fact * f = clips::CreateFact(deftemplate);
    RCLCPP_DEBUG(*logger_, "PREPARING FACT id %lli as new ID: %lli", fact.fact_id, f->factIndex);
    id_to_fact_ptr[fact.fact_id] = f;
  }
  clips::Fact * f;
  for (Fact fact : facts) {
    f = id_to_fact_ptr[fact.fact_id];
    RCLCPP_DEBUG(
      *logger_, "UPDATING FACT id %lli of deftemplate %s::%s with value from tick %lli: %s",
      f->factIndex, fact.defmodule.c_str(), fact.deftemplate_name.c_str(), fact.value.tick,
      fact.value.value.dump().c_str());
    if (!fact.value.value["type"].is_null()) {
      clips::CLIPSValue cv;
      cv.multifieldValue = json_to_multifield(
        env, fact.value.value["value"], id_to_fact_ptr, created_nullptr_facts, config);

      clips::PutFactSlot(f, nullptr, &cv);
      f = clips::Assert(f);
      fact_id_mapping_[f->factIndex] = fact.fact_id;

      continue;
    }

    for (nlohmann::json slot : fact.value.value["slots"]) {
      clips::CLIPSValue cv;
      switch (slot["type"].get<SlotType>()) {
        case SlotType::String:
          cv.lexemeValue = clips::CreateString(env, slot["value"].get<std::string>().c_str());
          break;
        case SlotType::Symbol:
          cv.lexemeValue = clips::CreateSymbol(env, slot["value"].get<std::string>().c_str());
          break;
        case SlotType::ExternalAddress:
          if (config.external_address_values_as_nullptr) {
            cv.externalAddressValue = clips::CreateCExternalAddress(env, nullptr);
          } else {
            const std::string addressStr = fact.value.value["value"].get<std::string>();

            std::uintptr_t rawAddress =
              static_cast<std::uintptr_t>(std::stoull(addressStr, nullptr, 0));

            void * address = reinterpret_cast<void *>(rawAddress);
            cv.externalAddressValue = clips::CreateCExternalAddress(env, address);
          }
          break;
        case SlotType::Integer:
          cv.integerValue = clips::CreateInteger(env, slot["value"].get<long long>());
          break;
        case SlotType::Float:
          cv.floatValue = clips::CreateFloat(env, slot["value"].get<double>());
          break;
        case SlotType::FactAddress:
          if (id_to_fact_ptr.contains(slot["value"].get<long long>())) {
            cv.factValue = id_to_fact_ptr[slot["value"].get<long long>()];
          } else {
            clips::Fact * null_fact = get_nullptr_fact(
              env, slot["value"].get<long long>(), id_to_fact_ptr, created_nullptr_facts);
            RCLCPP_WARN(
              *logger_,
              "Fact %lli from previous run references a fact %lli that is not present in the "
              "current environment, setting it to nullptr",
              fact.fact_id, slot["value"].get<long long>());
            cv.factValue = null_fact;
          }
          break;
        case SlotType::Multifield:
          cv.multifieldValue =
            json_to_multifield(env, slot["value"], id_to_fact_ptr, created_nullptr_facts, config);
          break;
      }

      clips::PutFactSlot(f, slot["name"].get<std::string>().c_str(), &cv);
    }

    f = clips::Assert(f);
    fact_id_mapping_[f->factIndex] = fact.fact_id;
  }
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

  std::string restore_run_param;
  std::string restore_tick_param;
  std::string restore_time_param;

  node->get_parameter(plugin_name_ + ".restore_run", restore_run_param);
  node->get_parameter(plugin_name_ + ".restore_tick", restore_tick_param);
  node->get_parameter(plugin_name_ + ".restore_time", restore_time_param);

  bool restore_run_is_set = restore_run_param != "";

  bool restore_tick_is_set = restore_tick_param != "";

  bool restore_time_is_set = restore_time_param != "";

  if (
    static_cast<int>(restore_run_is_set) + static_cast<int>(restore_tick_is_set) +
      static_cast<int>(restore_time_is_set) !=
    1) {
    RCLCPP_ERROR(
      *logger_,
      "Exactly one of restore_run, restore_tick and restore_time parameters has to be set");
    return false;
  }

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
  w.exec(kViewSchemaSql);
  w.commit();

  long long restore_tick;
  if (restore_run_is_set) {
    long long run = strtoll(restore_run_param.c_str(), nullptr, 10);
    restore_tick = get_end_tick_for_run(db, run);
  } else if (restore_tick_is_set) {
    restore_tick = resolve_restore_tick(db, strtoll(restore_tick_param.c_str(), nullptr, 10));
  } else {
    restore_tick = resolve_restore_time(db, restore_time_param, *logger_);
  }

  RCLCPP_INFO(*logger_, "Restoring environment %s to tick %lli", env_name.c_str(), restore_tick);

  RegexConfig config = populate_regex_config(node, plugin_name_);

  std::vector<Defmodule> defmodules = load_defmodules(db, restore_tick);

  filter_in_place(
    defmodules, config.module_rules,
    [](const Defmodule & value) -> const std::string & { return value.name; });

  clips::Fact * tmp_fact = clips::AssertString(env.get(), "(cdb-restored)");

  std::vector<Defglobal> fact_pointing_defglobal;
  std::unordered_map<long long, clips::Fact *> id_to_fact_ptr;
  std::vector<clips::Fact *> created_nullptr_facts;
  for (const Defmodule & defmodule : defmodules) {
    clips::Build(env.get(), defmodule.value.c_str());
    if (!config.skip_deftemplates) {
      assert_deftemplates(env.get(), db, config, defmodule.name, restore_tick);
    }
    if (!config.skip_defglobals) {
      assert_defglobals(
        env.get(), db, config, fact_pointing_defglobal, tmp_fact, defmodule.name, restore_tick);
    }
    if (!config.skip_deffunctions) {
      assert_deffunctions(env.get(), db, config, defmodule.name, restore_tick);
    }
    if (!config.skip_defrules) {
      assert_defrules(env.get(), db, config, defmodule.name, restore_tick);
    }
    if (!config.skip_deffacts) {
      assert_deffacts(env.get(), db, config, defmodule.name, restore_tick);
    }
  }

  if (!config.skip_facts) {
    assert_facts(env.get(), db, config, id_to_fact_ptr, created_nullptr_facts, restore_tick);
  }

  if (!config.skip_defglobals) {
    update_defglobals(
      env.get(), fact_pointing_defglobal, config, id_to_fact_ptr, created_nullptr_facts);
  }

  clips::Defmodule * theModule;
  clips::Activation * theActivation;

  for (theModule = clips::GetNextDefmodule(env.get(), NULL); theModule != NULL;
       theModule = clips::GetNextDefmodule(env.get(), theModule)) {
    clips::SetCurrentModule(env.get(), theModule);
    if (!allowed_by_generic_regex_rule(config.module_rules, theModule->header.name->contents)) {
      continue;
    }
    for (theActivation = clips::GetNextActivation(env.get(), NULL); theActivation != NULL;
         theActivation = clips::GetNextActivation(env.get(), theActivation)) {
      if (!allowed_by_generic_regex_rule(
            config.activation_rules, theActivation->theRule->header.name->contents)) {
        continue;
      }
      std::string name = theActivation->theRule->header.name->contents;
      std::string module_name =
        theActivation->theRule->header.whichModule->theModule->header.name->contents;

      bool not_found = false;
      std::vector<std::optional<long long>> basis;
      for (int i = 0; i < theActivation->basis->bcount; i++) {
        if (
          (get_nth_pm_match(theActivation->basis, i) != NULL) &&
          (get_nth_pm_match(theActivation->basis, i)->matchingItem != NULL)) {
          clips::PatternEntity * matchingItem =
            get_nth_pm_match(theActivation->basis, i)->matchingItem;
          long long id = (((clips::Fact *)matchingItem))->factIndex;
          if (!fact_id_mapping_.contains(id)) {
            not_found = true;
            break;
          }
          basis.push_back(fact_id_mapping_[id]);
        } else {
          basis.push_back(std::nullopt);
        }
      }
      if (not_found) {
        continue;
      }
      if (rule_firing_exists_before_tick(db, module_name, name, basis, restore_tick)) {
        RCLCPP_DEBUG(
          *logger_,
          "REMOVING ACTIOVATION of DEFRULE %s::%s with previous basis:", module_name.c_str(),
          name.c_str());
        for (auto partial : basis) {
          if (partial.has_value()) {
            RCLCPP_DEBUG(*logger_, "%lli", partial.value());
          } else {
            RCLCPP_DEBUG(*logger_, "*");
          }
        }
        clips::RemoveActivation(env.get(), theActivation, true, true);
      }
    }
  }

  for (clips::Fact * f : created_nullptr_facts) {
    clips::Retract(f);
  }

  RCLCPP_INFO(
    *logger_, "Finished Restoring environment %s to tick %lli", env_name.c_str(), restore_tick);

  return true;
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
