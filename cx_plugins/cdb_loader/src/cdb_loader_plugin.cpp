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

// TODO when restoring the rules the salience has to be taken from the field and not the pp since they could point to a defglobal that has changed in the meantime
// TODO CHECK IF ARRAYS IN THE TYPES IS BETTER THAN A TABLE REFERENCING THE THE FACT!?
// TODO DEFFACTS MANUAL
// TODO PLUGIN CHECK
// TODO: This license is not consistent with the license used in the project.
//       Delete the inconsistent license and above line and rerun pre-commit to insert a good license.
// TODO EXTERNAL ADDRESS AS VALUE
// TODO Check weather the db exists porperly

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

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".restore_run", rclcpp::ParameterValue(std::string()));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".restore_tick", rclcpp::ParameterValue(std::string()));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".restore_time", rclcpp::ParameterValue(std::string()));
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

  std::vector<Deftemplate> deftemplates = load_deftemplates(db, restore_tick);
  for (Deftemplate deftemplate : deftemplates) {
    // RCLCPP_ERROR(
    //   *logger_, "DEFTEMPLATE %s in %s, created at %lli ended at %lli", deftemplate.name.c_str(),
    //   deftemplate.defmodule.c_str(), deftemplate.start_tick, deftemplate.end_tick.value_or(-1));
    std::optional<TimedText> value = latest_timed_text_at_tick(deftemplate.value, restore_tick);
    if (value.has_value()) {
      clips::Build(env.get(), value.value().value.c_str());
    } else {
      RCLCPP_ERROR(
        *logger_, "No value found for deftemplate %s in %s at tick %lli", deftemplate.name.c_str(),
        deftemplate.defmodule.c_str(), restore_tick);
    }
  }

  ((clips::Fact *)nullptr)->factIndex = -1;

  std::vector<Fact> facts = load_facts(db);
  std::unordered_map<long long, clips::Fact *> id_to_fact_ptr;
  std::vector<clips::Fact *> created_nullptr_facts;
  for (Fact fact : facts) {
    TimedFact value = fact.value.back();
    clips::Deftemplate * deftemplate =
      clips::FindDeftemplate(env.get(), value.value["deftemplate"].get<std::string>().c_str());
    if (!value.value["type"].is_null() && deftemplate == nullptr) {
      clips::CLIPSLexeme * sym =
        clips::CreateSymbol(env.get(), value.value["deftemplate"].get<std::string>().c_str());
      deftemplate = clips::CreateImpliedDeftemplate(env.get(), sym, true);
    }
    if (deftemplate == nullptr) {
      RCLCPP_ERROR(
        *logger_, "Could not find deftemplate %s for fact %lli",
        value.value["deftemplate"].get<std::string>().c_str(), fact.fact_id);
      continue;
    }
    clips::Fact * f = clips::CreateFact(deftemplate);
    id_to_fact_ptr[fact.fact_id] = f;
  }
  clips::Fact * f;
  for (Fact fact : facts) {
    TimedFact value = fact.value.back();
    f = id_to_fact_ptr[fact.fact_id];
    if (!value.value["type"].is_null()) {
      clips::CLIPSValue cv;
      cv.multifieldValue =
        json_to_multifield(env.get(), value.value["value"], id_to_fact_ptr, created_nullptr_facts);

      clips::PutFactSlot(f, nullptr, &cv);
      f = clips::Assert(f);
      fact_id_mapping_[f->factIndex] = fact.fact_id;
      RCLCPP_WARN(
        *logger_, "ASSERTED FACT %lli of type %s: %s", f->factIndex,
        value.value["deftemplate"].get<std::string>().c_str(), value.value["value"].dump().c_str());

      continue;
    }
    RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.dump().c_str());

    for (nlohmann::json slot : value.value["slots"]) {
      clips::CLIPSValue cv;
      switch (slot["type"].get<SlotType>()) {
        case SlotType::String:
          cv.lexemeValue = clips::CreateString(env.get(), slot["value"].get<std::string>().c_str());
          break;
        case SlotType::Symbol:
          cv.lexemeValue = clips::CreateSymbol(env.get(), slot["value"].get<std::string>().c_str());
          break;
        case SlotType::ExternalAddress:
          cv.externalAddressValue = clips::CreateCExternalAddress(env.get(), nullptr);
          break;
        case SlotType::Integer:
          cv.integerValue = clips::CreateInteger(env.get(), slot["value"].get<long long>());
          break;
        case SlotType::Float:
          cv.floatValue = clips::CreateFloat(env.get(), slot["value"].get<double>());
          break;
        case SlotType::FactAddress:
          if (id_to_fact_ptr.contains(slot["value"].get<long long>())) {
            cv.factValue = id_to_fact_ptr[slot["value"].get<long long>()];
          } else {
            clips::Fact * null_fact = get_nullptr_fact(
              env.get(), slot["value"].get<long long>(), id_to_fact_ptr, created_nullptr_facts);
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
            json_to_multifield(env.get(), slot["value"], id_to_fact_ptr, created_nullptr_facts);
          break;
      }

      clips::PutFactSlot(f, slot["name"].get<std::string>().c_str(), &cv);
    }

    f = clips::Assert(f);
    fact_id_mapping_[f->factIndex] = fact.fact_id;
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
        std::format("(defglobal {} ?*{}* = nil)", defglobal.defmodule, defglobal.name).c_str());
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
          clips::DefglobalSetCLIPSExternalAddress(
            global, clips::CreateCExternalAddress(env.get(), nullptr));
          break;
        }
        case SlotType::Integer:
          clips::DefglobalSetInteger(global, value.value["value"].get<long long>());
          break;
        case SlotType::Float:
          clips::DefglobalSetFloat(global, value.value["value"].get<double>());
          break;
        case SlotType::FactAddress:
          if (id_to_fact_ptr.contains(value.value["value"].get<long long>())) {
            clips::DefglobalSetFact(global, id_to_fact_ptr[value.value["value"].get<long long>()]);
          } else {
            clips::Fact * null_fact = get_nullptr_fact(
              env.get(), value.value["value"].get<long long>(), id_to_fact_ptr,
              created_nullptr_facts);
            RCLCPP_WARN(
              *logger_,
              "Defglobal %s::%s references a fact %lli that is not present in the current "
              "environment, setting it to nullptr",
              defglobal.defmodule.c_str(), defglobal.name.c_str(),
              value.value["value"].get<long long>());
            clips::DefglobalSetFact(global, null_fact);
          }
          break;
        case SlotType::Multifield: {
          clips::Multifield * multifield = json_to_multifield(
            env.get(), value.value["value"], id_to_fact_ptr, created_nullptr_facts);
          clips::DefglobalSetMultifield(global, multifield);
          break;
        }
      }
    }
  }
  clips::SetResetGlobals(env.get(), false);

  std::vector<Deffunction> deffunctions = load_deffunctions(db);
  for (Deffunction deffunction : deffunctions) {
    // RCLCPP_ERROR(
    //   *logger_, "DEFFUNCTION %s in %s created at %li ended at %li", deffunction.name.c_str(),
    //   deffunction.defmodule.c_str(), deffunction.start_tick, deffunction.end_tick.value_or(-1));
    for (TimedText value : deffunction.value) {
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.c_str());
      clips::Build(env.get(), value.value.c_str());
    }
  }

  std::vector<Deffacts> deffacts = load_deffacts(db);
  for (Deffacts deffact : deffacts) {
    // RCLCPP_ERROR(
    //   *logger_, "DEFFACTS %s in %s created at %li ended at %li", deffact.name.c_str(),
    //   deffact.defmodule.c_str(), deffact.start_tick, deffact.end_tick.value_or(-1));
    for (TimedText value : deffact.value) {
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.c_str());
      clips::Build(env.get(), value.value.c_str());
    }
  }

  std::vector<Defrule> defrules = load_defrules(db);
  for (Defrule defrule : defrules) {
    // RCLCPP_ERROR(
    //   *logger_, "DEFRULE %s in %s created at %li ended at %li", defrule.name.c_str(),
    //   defrule.defmodule.c_str(), defrule.start_tick, defrule.end_tick.value_or(-1));
    for (TimedText value : defrule.value) {
      RCLCPP_ERROR(*logger_, "VALUE AT %li: %s", value.tick, value.value.c_str());
      clips::Build(env.get(), value.value.c_str());
    }
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
      if (rule_firing_exists_before_tick(db, module_name, name, basis, 10000)) {
        RCLCPP_ERROR(
          *logger_,
          "Removing activation of rule %s in module %s because it was already fired before %lli",
          name.c_str(), module_name.c_str(), basis.empty() ? -1 : basis.back());
        clips::RemoveActivation(env.get(), theActivation, true, true);
      }
    }
  }

  clips::AssertString(env.get(), "(cdb-restored)");
  for (clips::Fact * f : created_nullptr_facts) {
    clips::Retract(f);
  }

  clips::Eval(env.get(), "(facts)", NULL);
  clips::ShowDefglobals(env.get(), clips::STDOUT, NULL);

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
