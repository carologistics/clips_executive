// TODO RULE add the rule to the definiton no lhs rhs seperation possible
// TODO BATCH UPLOAD
// TODO when restoring the rules the salience has to be taken from the field and not the pp since they could point to a defglobal that has changed in the meantime
// TODO retract defglobal callback
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

#include <clips_ns/engine.h>
#include <clips_ns/entities.h>
#include <clips_ns/envrnmnt.h>
#include <clips_ns/factmngr.h>

#include <cstddef>
#include <cstdio>
#include <nlohmann/json_fwd.hpp>
// RANGES is defined in clips_ns/clips.h, which causes issues with
// pqxx/pqxx
#include <ctime>
#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>

#include "cx_cdb_plugin/cdb_plugin.hpp"
#include "cx_cdb_plugin/db_handler.hpp"

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx
{

long long CDBPlugin::tick_ = 0;
long long CDBPlugin::current_run_ = -1;
std::shared_ptr<DBHandler> CDBPlugin::db_ = nullptr;

CDBPlugin::CDBPlugin() {}

CDBPlugin::~CDBPlugin() {}

void CDBPlugin::initialize()
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
    node, plugin_name_ + ".db_name", rclcpp::ParameterValue(std::string()));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".username", rclcpp::ParameterValue("anonymous"));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".password", rclcpp::ParameterValue(std::string()));
}

bool CDBPlugin::clips_env_init(std::shared_ptr<clips::Environment> & env)
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = parent_.lock();
  std::string hostname;
  int port;
  std::string db_name;
  std::string username;
  std::string password;
  node->get_parameter(plugin_name_ + ".hostname", hostname);
  node->get_parameter(plugin_name_ + ".port", port);
  node->get_parameter(plugin_name_ + ".db_name", db_name);
  node->get_parameter(plugin_name_ + ".username", username);
  node->get_parameter(plugin_name_ + ".password", password);
  if (started_) {
    RCLCPP_ERROR(*logger_, "TODO: MULTIPLE INITIALIZATION OF CDB NOT YET SUPPORTED");
    return false;
  }
  started_ = true;
  if (!db_name.empty()) {
    RCLCPP_ERROR(*logger_, "TODO: DB_NAME LOADING NOT YET SUPPORTED");
    return false;
  }

  time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  // db_name set to current time
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y_%m_%dt%H_%M_%S");
  db_name = "cdb_" + oss.str();

  DBHandlerConfig config = {hostname, port, username, password, db_name};
  try {
    RCLCPP_INFO(
      *logger_, "Connecting to database %s at %s:%d with user %s", config.db_name.c_str(),
      config.hostname.c_str(), config.port, config.username.c_str());
    db_ = std::make_shared<DBHandler>(config, true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(*logger_, "Failed to connect to database: %s", e.what());
    return false;
  }

  clips::Defmodule * theModule;
  clips::Defrule * theRule;
  clips::Defglobal * theDefGlobal;
  clips::Deffunction * theDefFunction;
  clips::Deftemplate * theDefTemplate;

  for (theModule = clips::GetNextDefmodule(env.get(), NULL); theModule != NULL;
       theModule = clips::GetNextDefmodule(env.get(), theModule)) {
    clips::SetCurrentModule(env.get(), theModule);
    const std::string module_name = theModule->header.name->contents;
    for (theRule = clips::GetNextDefrule(env.get(), NULL); theRule != NULL;
         theRule = clips::GetNextDefrule(env.get(), theRule)) {
      const std::string name = theRule->header.name->contents;
      const int salience = theRule->salience;
      const std::string definition = theRule->header.ppForm;

      db_->assert_defrule(name, module_name, definition, salience, 0);
    }
    for (theDefGlobal = clips::GetNextDefglobal(env.get(), NULL); theDefGlobal != NULL;
         theDefGlobal = clips::GetNextDefglobal(env.get(), theDefGlobal)) {
      std::string name = theDefGlobal->header.name->contents;
      std::string value_json =
        slot_value_to_json(theDefGlobal->current.header->type, &theDefGlobal->current).dump();
      db_->assert_defglobal(name, module_name, value_json, 0);
    }
    for (theDefFunction = clips::GetNextDeffunction(env.get(), NULL); theDefFunction != NULL;
         theDefFunction = clips::GetNextDeffunction(env.get(), theDefFunction)) {
      std::string name = theDefFunction->header.name->contents;
      std::string definition = theDefFunction->header.ppForm;
      db_->assert_deffunction(name, module_name, definition, 0);
    }
    for (theDefTemplate = clips::GetNextDeftemplate(env.get(), NULL); theDefTemplate != NULL;
         theDefTemplate = clips::GetNextDeftemplate(env.get(), theDefTemplate)) {
      std::string name = theDefTemplate->header.name->contents;
      if (theDefTemplate->header.ppForm == nullptr) {
        continue;
      }
      db_->assert_deftemplate(name, module_name, theDefTemplate->header.ppForm, 0);
    }
  }

  clips::AddDefruleAssertFunction(
    env.get(), "cdb_defrule_assert_callback", &cdb_defrule_assert_callback, 0, this);
  clips::AddDefruleRetractFunction(
    env.get(), "cdb_defrule_retract_callback", &cdb_defrule_retract_callback, 0, this);

  clips::AddDeftemplateAssertFunction(
    env.get(), "cdb_deftemplate_assert_callback", &cdb_deftemplate_assert_callback, 0, this);
  clips::AddDeftemplateRetractFunction(
    env.get(), "cdb_deftemplate_retract_callback", &cdb_deftemplate_retract_callback, 0, this);

  clips::AddDeffunctionAssertFunction(
    env.get(), "cdb_deffunction_assert_callback", &cdb_deffunction_assert_callback, 0, this);
  clips::AddDeffunctionRetractFunction(
    env.get(), "cdb_deffunction_retract_callback", &cdb_deffunction_retract_callback, 0, this);

  clips::AddDeffactsAssertFunction(
    env.get(), "cdb_deffacts_assert_callback", &cdb_deffacts_assert_callback, 0, this);
  clips::AddDeffactsRetractFunction(
    env.get(), "cdb_deffacts_retract_callback", &cdb_deffacts_retract_callback, 0, this);

  clips::AddAssertFunction(env.get(), "cdb_assert_callback", &cdb_assert_callback, 0, this);
  clips::AddBeforeRuleFiresFunction(
    env.get(), "cdb_before_rule_callback", &cdb_before_rule_callback, 0, this);

  clips::AddBeforeRunFiresFunction(
    env.get(), "cdb_before_run_callback", &cdb_before_run_callback, 0, this);
  clips::AddAfterRunFiresFunction(
    env.get(), "cdb_after_run_callback", &cdb_after_run_callback, 0, this);

  clips::AddDefglobalAssertFunction(
    env.get(), "cdb_defglobal_assert", &cdb_defglobal_assert_callback, 0, this);
  clips::AddDefglobalRetractFunction(
    env.get(), "cdb_defglobal_retract", &cdb_defglobal_retract_callback, 0, this);

  return true;
}

void CDBPlugin::cdb_defrule_assert_callback(
  clips::Environment * /*env*/, clips::Defrule * defrule, void * /*context*/)
{
  std::string name = defrule->header.name->contents;
  std::string module_name = defrule->header.whichModule->theModule->header.name->contents;
  std::string definition = defrule->header.ppForm;
  int salience = defrule->salience;
  db_->assert_defrule(name, module_name, definition, salience, get_tick());
}

void CDBPlugin::cdb_defrule_retract_callback(
  clips::Environment * /*env*/, clips::Defrule * defrule, void * /*context*/)
{
  std::string name = defrule->header.name->contents;
  std::string module_name = defrule->header.whichModule->theModule->header.name->contents;
  db_->retract_defrule(name, module_name, get_tick());
}

void CDBPlugin::cdb_deffacts_assert_callback(
  clips::Environment * /*env*/, clips::Deffacts * deffacts, void * /*context*/)
{
  std::string name = deffacts->header.name->contents;
  std::string module_name = deffacts->header.whichModule->theModule->header.name->contents;
  std::string definition = deffacts->header.ppForm;
  db_->assert_deffacts(name, module_name, definition, get_tick());
}

void CDBPlugin::cdb_deffacts_retract_callback(
  clips::Environment * /*env*/, clips::Deffacts * deffacts, void * /*context*/)
{
  std::string name = deffacts->header.name->contents;
  std::string module_name = deffacts->header.whichModule->theModule->header.name->contents;
  std::string definition = deffacts->header.ppForm;
  db_->retract_deffacts(name, module_name, get_tick());
}

void CDBPlugin::cdb_deftemplate_assert_callback(
  clips::Environment * /*env*/, clips::Deftemplate * deftemplate, void * /*context*/)
{
  std::string name = deftemplate->header.name->contents;
  std::string module_name = deftemplate->header.whichModule->theModule->header.name->contents;
  std::string definition = deftemplate->header.ppForm;
  db_->assert_deftemplate(name, module_name, definition, get_tick());
}

void CDBPlugin::cdb_deftemplate_retract_callback(
  clips::Environment * /*env*/, clips::Deftemplate * deftemplate, void * /*context*/)
{
  std::string name = deftemplate->header.name->contents;
  std::string module_name = deftemplate->header.whichModule->theModule->header.name->contents;
  std::string definition = deftemplate->header.ppForm;
  db_->retract_deftemplate(name, module_name, get_tick());
}

void CDBPlugin::cdb_deffunction_assert_callback(
  clips::Environment * /*env*/, clips::Deffunction * deffunction, void * /*context*/)
{
  std::string name = deffunction->header.name->contents;
  std::string module_name = deffunction->header.whichModule->theModule->header.name->contents;
  std::string definition = deffunction->header.ppForm;
  db_->assert_deffunction(name, module_name, definition, get_tick());
}

void CDBPlugin::cdb_deffunction_retract_callback(
  clips::Environment * /*env*/, clips::Deffunction * deffunction, void * /*context*/)
{
  std::string name = deffunction->header.name->contents;
  std::string module_name = deffunction->header.whichModule->theModule->header.name->contents;
  std::string definition = deffunction->header.ppForm;
  db_->retract_deffunction(name, module_name, get_tick());
}

void CDBPlugin::cdb_defglobal_assert_callback(
  clips::Environment * /*env*/, clips::Defglobal * defglobal, void * context)
{
  CDBPlugin * cdb_plugin = static_cast<CDBPlugin *>(context);
  std::string name = defglobal->header.name->contents;
  std::string module_name = defglobal->header.whichModule->theModule->header.name->contents;
  std::string value_json =
    cdb_plugin->slot_value_to_json(defglobal->current.header->type, &defglobal->current).dump();
  db_->assert_defglobal(name, module_name, value_json, get_tick());
}

void CDBPlugin::cdb_defglobal_retract_callback(
  clips::Environment * /*env*/, clips::Defglobal * defglobal, void * /*context*/)
{
  std::string name = defglobal->header.name->contents;
  std::string module_name = defglobal->header.whichModule->theModule->header.name->contents;
  db_->retract_defglobal(name, module_name, get_tick());
}

void CDBPlugin::cdb_before_rule_callback(
  clips::Environment * /*env*/, clips::Activation * act, void * /*context*/)
{
  std::string name = act->theRule->header.name->contents;
  std::string module_name = act->theRule->header.whichModule->theModule->header.name->contents;
  std::vector<std::optional<long long>> basis;
  for (int i = 0; i < act->basis->bcount; i++) {
    if (
      (get_nth_pm_match(act->basis, i) != NULL) &&
      (get_nth_pm_match(act->basis, i)->matchingItem != NULL)) {
      clips::PatternEntity * matchingItem = get_nth_pm_match(act->basis, i)->matchingItem;
      basis.push_back((((clips::Fact *)matchingItem))->factIndex);
    } else {
      basis.push_back(std::nullopt);
    }
  }
  db_->assert_rule_fired(name, module_name, basis, get_tick());
}

void CDBPlugin::cdb_before_run_callback(clips::Environment * /*env*/, void * context)
{
  CDBPlugin * cdb_plugin = static_cast<CDBPlugin *>(context);
  current_run_ = db_->start_run(cdb_plugin->parent_.lock()->now().nanoseconds(), tick_);
}

void CDBPlugin::cdb_after_run_callback(clips::Environment * /*env*/, void * context)
{
  CDBPlugin * cdb_plugin = static_cast<CDBPlugin *>(context);
  db_->end_run(current_run_, cdb_plugin->parent_.lock()->now().nanoseconds(), tick_);
}

nlohmann::json CDBPlugin::slot_value_to_json(unsigned short type, clips::CLIPSValue * value)
{
  nlohmann::json json;
  switch (type) {
    case FLOAT_TYPE: {
      json["value"] = value->floatValue->contents;
      json["type"] = "FLOAT";
      break;
    }
    case INTEGER_TYPE: {
      json["value"] = value->integerValue->contents;
      json["type"] = "INTEGER";
      break;
    }
    case STRING_TYPE: {
      json["value"] = value->lexemeValue->contents;
      json["type"] = "STRING";
      break;
    }
    case SYMBOL_TYPE: {
      json["value"] = value->lexemeValue->contents;
      json["type"] = "SYMBOL";
      break;
    }
    case EXTERNAL_ADDRESS_TYPE: {
      json["type"] = "EXTERNAL_ADDRESS";
      void * address = (value->externalAddressValue->contents);
      std::ostringstream oss;
      oss << static_cast<const void *>(address);
      json["value"] = oss.str();
      break;
    }
    case VOID_TYPE: {
      json["type"] = "VOID";
      break;
    }
    case MULTIFIELD_TYPE: {
      clips::Multifield * theSegment = value->multifieldValue;
      json["value"] = multifield_to_json_list(theSegment);
      json["type"] = "MULTIFIELD";
      break;
    }
    case FACT_ADDRESS_TYPE: {
      RCLCPP_ERROR(*logger_, "HALLO %lli", value->factValue->factIndex);
      break;
    }
    default:
      json["type"] = "UNKNOWN";
      RCLCPP_ERROR(*logger_, "type %i", type);
      throw;
      // NOTE instance is for COOL; BITMAP is internal
  }
  return json;
}

std::vector<nlohmann::json> CDBPlugin::multifield_to_json_list(clips::Multifield * theSegment)
{
  clips::CLIPSValue * theMultifield = theSegment->contents;
  size_t range = theSegment->length;
  std::vector<nlohmann::json> json_list;
  for (size_t j = 0; j < range; j++) {
    json_list.push_back(slot_value_to_json(theMultifield[j].header->type, &theMultifield[j]));
  }
  return json_list;
}

std::string CDBPlugin::clips_fact_to_json(clips::Fact * f, const char * deftemplate_name)
{
  nlohmann::json json;
  clips::Deftemplate * deftemplate = f->whichDeftemplate;
  json["deftemplate"] = deftemplate_name;
  // Logic stolen from factmngr.c void PrintFact(...)
  if (f->whichDeftemplate->implied == false) {
    /*=========================================*/
    /* Print a deftemplate (non-ordered) fact. */
    /*=========================================*/
    std::vector<nlohmann::json> slots;
    struct clips::templateSlot * slotPtr = deftemplate->slotList;
    clips::CLIPSValue * sublist = f->theProposition.contents;
    int i = 0;
    while (slotPtr != NULL) {
      nlohmann::json slot_json;
      const char * slotname = slotPtr->slotName->contents;
      clips::CLIPSValue * value = &sublist[i];
      if (slotPtr->multislot) {
        clips::Multifield * theSegment = value->multifieldValue;
        slot_json["value"] = multifield_to_json_list(theSegment);
        slot_json["type"] = "MULTIFIELD";

      } else {
        unsigned short type = ((clips::TypeHeader *)value->value)->type;
        slot_json = slot_value_to_json(type, value);
      }
      slotPtr = slotPtr->next;
      i++;
      slot_json["name"] = slotname;
      slots.push_back(slot_json);
    }
    json["slots"] = slots;
  } else {
    /*==============================*/
    /* Print an ordered fact (which */
    /* has an implied deftemplate). */
    /*==============================*/
    clips::Multifield * theSegment = f->theProposition.contents[0].multifieldValue;
    json["value"] = multifield_to_json_list(theSegment);
    json["type"] = "MULTIFIELD";
  }
  return json.dump();
}
void CDBPlugin::cdb_assert_callback(clips::Environment *, void * fact, void * context)
{
  CDBPlugin * cdb_plugin = static_cast<CDBPlugin *>(context);
  clips::Fact * f = static_cast<clips::Fact *>(fact);

  clips::Deftemplate * deftemplate = f->whichDeftemplate;
  const char * deftemplate_name = deftemplate->header.name->contents;
  db_->assert_fact(
    f->factIndex, deftemplate_name, cdb_plugin->clips_fact_to_json(f, deftemplate_name).c_str(),
    get_tick());
}

bool CDBPlugin::clips_env_destroyed(std::shared_ptr<clips::Environment> & env)
{
  CLIPSEnvContext * context = CLIPSEnvContext::get_context(env.get());
  RCLCPP_INFO(*logger_, "Destroying plugin for environment %s", context->env_name_.c_str());
  return true;
}
}  // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::CDBPlugin, cx::ClipsPlugin)
