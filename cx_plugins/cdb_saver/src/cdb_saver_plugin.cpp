// TODO BATCH UPLOAD
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

#include <cstddef>
#include <cstdio>
#include <ctime>
#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>
#include <memory>
#include <nlohmann/json_fwd.hpp>

#include "cx_cdb_saver_plugin/cdb_saver_plugin.hpp"
#include "cx_msgs/srv/list_clips_plugins.hpp"
#include "rcl_interfaces/msg/list_parameters_result.hpp"

// To export as plugin
#include "cx_clips_env_manager/clips_env_manager.hpp"
#include "cx_clips_env_manager/clips_plugin_manager.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cx
{

CDBSaverPlugin::CDBSaverPlugin() {}

CDBSaverPlugin::~CDBSaverPlugin() {}

void CDBSaverPlugin::initialize()
{
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = parent_.lock();
  if (!node) {
    return;
  }

  CLIPSEnvManager * env_manager = static_cast<CLIPSEnvManager *>(parent_.lock().get());

  env_manager->get_plugin_manager().add_plugin_load_callback(
    "cdb_plugin_load_callback",
    std::bind(
      &CDBSaverPlugin::plugin_load_callback, this, std::placeholders::_1, std::placeholders::_2));
  env_manager->get_plugin_manager().add_plugin_unload_callback(
    "cdb_plugin_unload_callback",
    std::bind(
      &CDBSaverPlugin::plugin_unload_callback, this, std::placeholders::_1, std::placeholders::_2));

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".hostname", rclcpp::ParameterValue("localhost"));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".port", rclcpp::ParameterValue(5432));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".username", rclcpp::ParameterValue("anonymous"));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name_ + ".password", rclcpp::ParameterValue(std::string()));
}

void CDBSaverPlugin::finalize()
{
  CLIPSEnvManager * env_manager = static_cast<CLIPSEnvManager *>(parent_.lock().get());
  env_manager->get_plugin_manager().remove_plugin_load_callback("cdb_plugin_load_callback");
  env_manager->get_plugin_manager().remove_plugin_unload_callback("cdb_plugin_unload_callback");
}

bool CDBSaverPlugin::clips_env_init(std::shared_ptr<clips::Environment> & env)
{
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = parent_.lock();
  std::string hostname;
  int port;
  std::string username;
  std::string password;
  node->get_parameter(plugin_name_ + ".hostname", hostname);
  node->get_parameter(plugin_name_ + ".port", port);
  node->get_parameter(plugin_name_ + ".username", username);
  node->get_parameter(plugin_name_ + ".password", password);

  time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y_%m_%dt%H_%M_%S");

  std::string env_name = CLIPSEnvContext::get_context(env)->env_name_;

  std::string db_name = "cdb_" + env_name + "_" + oss.str();

  DBHandlerConfig config = {hostname, port, username, password, db_name};
  try {
    RCLCPP_INFO(
      *logger_, "Connecting to database %s at %s:%d with user %s", config.db_name.c_str(),
      config.hostname.c_str(), config.port, config.username.c_str());
    db_handlers_.emplace(env_name, std::make_unique<DBHandler>(config, parent_));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(*logger_, "Failed to connect to database: %s", e.what());
    return false;
  }
  DBHandler * db_handler_ptr = db_handlers_[env_name].get();

  clips::Defmodule * theModule;
  clips::Defrule * theRule;
  clips::Defglobal * theDefGlobal;
  clips::Deffunction * theDefFunction;
  clips::Deftemplate * theDefTemplate;
  clips::Deffacts * theDefFacts;

  for (theModule = clips::GetNextDefmodule(env.get(), NULL); theModule != NULL;
       theModule = clips::GetNextDefmodule(env.get(), theModule)) {
    clips::SetCurrentModule(env.get(), theModule);
    const std::string module_name = theModule->header.name->contents;
    for (theRule = clips::GetNextDefrule(env.get(), NULL); theRule != NULL;
         theRule = clips::GetNextDefrule(env.get(), theRule)) {
      const std::string name = theRule->header.name->contents;
      const int salience = theRule->salience;
      const std::string definition = theRule->header.ppForm;

      db_handler_ptr->assert_defrule(name, module_name, definition, salience, 0);
    }
    for (theDefGlobal = clips::GetNextDefglobal(env.get(), NULL); theDefGlobal != NULL;
         theDefGlobal = clips::GetNextDefglobal(env.get(), theDefGlobal)) {
      std::string name = theDefGlobal->header.name->contents;
      std::string value_json =
        slot_value_to_json(theDefGlobal->current.header->type, &theDefGlobal->current).dump();
      db_handler_ptr->assert_defglobal(name, module_name, value_json, 0);
    }
    for (theDefFunction = clips::GetNextDeffunction(env.get(), NULL); theDefFunction != NULL;
         theDefFunction = clips::GetNextDeffunction(env.get(), theDefFunction)) {
      std::string name = theDefFunction->header.name->contents;
      std::string definition = theDefFunction->header.ppForm;
      db_handler_ptr->assert_deffunction(name, module_name, definition, 0);
    }
    for (theDefTemplate = clips::GetNextDeftemplate(env.get(), NULL); theDefTemplate != NULL;
         theDefTemplate = clips::GetNextDeftemplate(env.get(), theDefTemplate)) {
      std::string name = theDefTemplate->header.name->contents;
      if (theDefTemplate->header.ppForm == nullptr) {
        continue;
      }
      db_handler_ptr->assert_deftemplate(name, module_name, theDefTemplate->header.ppForm, 0);
    }
    for (theDefFacts = clips::GetNextDeffacts(env.get(), NULL); theDefFacts != NULL;
         theDefFacts = clips::GetNextDeffacts(env.get(), theDefFacts)) {
      std::string name = theDefFacts->header.name->contents;
      db_handler_ptr->assert_deffacts(name, module_name, theDefFacts->header.ppForm, 0);
    }
  }

  CLIPSEnvManager * env_manager = static_cast<CLIPSEnvManager *>(parent_.lock().get());
  std::vector<std::string> plugins = env_manager->get_plugin_manager().list_plugins(env_name);
  for (auto plugin : plugins) {
    rcl_interfaces::msg::ListParametersResult parameters =
      parent_.lock()->list_parameters({plugin}, 0);
    std::string config_json = parameter_list_to_json(parameters).dump();
    db_handler_ptr->load_plugin(plugin, config_json, 0);
  }

  clips::AddAssertFunction(
    env.get(), "cdb_assert_callback", &cdb_assert_callback, 0, db_handler_ptr);
  clips::AddRetractFunction(
    env.get(), "cdb_retract_callback", &cdb_retract_callback, 0, db_handler_ptr);

  clips::AddBeforeRuleFiresFunction(
    env.get(), "cdb_before_rule_callback", &cdb_before_rule_callback, 0, db_handler_ptr);

  clips::AddBeforeRunFiresFunction(
    env.get(), "cdb_before_run_callback", &cdb_before_run_callback, 0, db_handler_ptr);
  clips::AddAfterRunFiresFunction(
    env.get(), "cdb_after_run_callback", &cdb_after_run_callback, 0, db_handler_ptr);

  clips::AddDefruleAssertFunction(
    env.get(), "cdb_defrule_assert_callback", &cdb_defrule_assert_callback, 0, db_handler_ptr);
  clips::AddDefruleRetractFunction(
    env.get(), "cdb_defrule_retract_callback", &cdb_defrule_retract_callback, 0, db_handler_ptr);

  clips::AddDeftemplateAssertFunction(
    env.get(), "cdb_deftemplate_assert_callback", &cdb_deftemplate_assert_callback, 0,
    db_handler_ptr);
  clips::AddDeftemplateRetractFunction(
    env.get(), "cdb_deftemplate_retract_callback", &cdb_deftemplate_retract_callback, 0,
    db_handler_ptr);

  clips::AddDeffunctionAssertFunction(
    env.get(), "cdb_deffunction_assert_callback", &cdb_deffunction_assert_callback, 0,
    db_handler_ptr);
  clips::AddDeffunctionRetractFunction(
    env.get(), "cdb_deffunction_retract_callback", &cdb_deffunction_retract_callback, 0,
    db_handler_ptr);

  clips::AddDeffactsAssertFunction(
    env.get(), "cdb_deffacts_assert_callback", &cdb_deffacts_assert_callback, 0, db_handler_ptr);
  clips::AddDeffactsRetractFunction(
    env.get(), "cdb_deffacts_retract_callback", &cdb_deffacts_retract_callback, 0, db_handler_ptr);

  clips::AddDefglobalAssertFunction(
    env.get(), "cdb_defglobal_assert", &cdb_defglobal_assert_callback, 0, db_handler_ptr);
  clips::AddDefglobalRetractFunction(
    env.get(), "cdb_defglobal_retract", &cdb_defglobal_retract_callback, 0, db_handler_ptr);

  return true;
}

void CDBSaverPlugin::plugin_load_callback(
  const std::string & env_name, const std::string & plugin_name)
{
  auto db = db_handlers_.find(env_name);
  if (db == db_handlers_.end()) {
    return;
  }

  rcl_interfaces::msg::ListParametersResult parameters =
    parent_.lock()->list_parameters({plugin_name}, 0);
  std::string config_json = parameter_list_to_json(parameters).dump();
  db->second->load_plugin(plugin_name, config_json, db->second->get_tick());
}

nlohmann::json CDBSaverPlugin::parameter_list_to_json(
  const rcl_interfaces::msg::ListParametersResult & parameters)
{
  nlohmann::json json_list = nlohmann::json::array();
  for (const auto & parameter : parameters.names) {
    nlohmann::json param;
    param["name"] = parameter;
    rclcpp::Parameter value = parent_.lock()->get_parameter(parameter);
    param["type"] = value.get_type_name();
    param["value"] = value.value_to_string();
    json_list.push_back(param);
  }
  return json_list;
}

void CDBSaverPlugin::plugin_unload_callback(
  const std::string & env_name, const std::string & plugin_name)
{
  auto db = db_handlers_.find(env_name);
  if (db == db_handlers_.end()) {
    return;
  }
  db->second->unload_plugin(plugin_name, db->second->get_tick());
}

void CDBSaverPlugin::cdb_assert_callback(clips::Environment * /*env*/, void * fact, void * context)
{
  clips::Fact * f = static_cast<clips::Fact *>(fact);

  clips::Deftemplate * deftemplate = f->whichDeftemplate;
  const char * deftemplate_name = deftemplate->header.name->contents;

  DBHandler * db = static_cast<DBHandler *>(context);
  db->assert_fact(
    f->factIndex, deftemplate_name, clips_fact_to_json(f, deftemplate_name).c_str(),
    db->get_tick());
}

void CDBSaverPlugin::cdb_retract_callback(clips::Environment * /*env*/, void * fact, void * context)
{
  clips::Fact * f = static_cast<clips::Fact *>(fact);
  DBHandler * db = static_cast<DBHandler *>(context);
  db->retract_fact(f->factIndex, db->get_tick());
}

void CDBSaverPlugin::cdb_defrule_assert_callback(
  clips::Environment * /*env*/, clips::Defrule * defrule, void * context)
{
  std::string name = defrule->header.name->contents;
  std::string module_name = defrule->header.whichModule->theModule->header.name->contents;
  std::string definition = defrule->header.ppForm;
  int salience = defrule->salience;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->assert_defrule(name, module_name, definition, salience, db->get_tick());
}

void CDBSaverPlugin::cdb_defrule_retract_callback(
  clips::Environment * /*env*/, clips::Defrule * defrule, void * context)
{
  std::string name = defrule->header.name->contents;
  std::string module_name = defrule->header.whichModule->theModule->header.name->contents;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->retract_defrule(name, module_name, db->get_tick());
}

void CDBSaverPlugin::cdb_deffacts_assert_callback(
  clips::Environment * /*env*/, clips::Deffacts * deffacts, void * context)
{
  std::string name = deffacts->header.name->contents;
  std::string module_name = deffacts->header.whichModule->theModule->header.name->contents;
  std::string definition = deffacts->header.ppForm;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->assert_deffacts(name, module_name, definition, db->get_tick());
}

void CDBSaverPlugin::cdb_deffacts_retract_callback(
  clips::Environment * /*env*/, clips::Deffacts * deffacts, void * context)
{
  std::string name = deffacts->header.name->contents;
  std::string module_name = deffacts->header.whichModule->theModule->header.name->contents;
  std::string definition = deffacts->header.ppForm;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->retract_deffacts(name, module_name, db->get_tick());
}

void CDBSaverPlugin::cdb_deftemplate_assert_callback(
  clips::Environment * /*env*/, clips::Deftemplate * deftemplate, void * context)
{
  std::string name = deftemplate->header.name->contents;
  std::string module_name = deftemplate->header.whichModule->theModule->header.name->contents;
  std::string definition = deftemplate->header.ppForm;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->assert_deftemplate(name, module_name, definition, db->get_tick());
}

void CDBSaverPlugin::cdb_deftemplate_retract_callback(
  clips::Environment * /*env*/, clips::Deftemplate * deftemplate, void * context)
{
  std::string name = deftemplate->header.name->contents;
  std::string module_name = deftemplate->header.whichModule->theModule->header.name->contents;
  std::string definition = deftemplate->header.ppForm;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->retract_deftemplate(name, module_name, db->get_tick());
}

void CDBSaverPlugin::cdb_deffunction_assert_callback(
  clips::Environment * /*env*/, clips::Deffunction * deffunction, void * context)
{
  std::string name = deffunction->header.name->contents;
  std::string module_name = deffunction->header.whichModule->theModule->header.name->contents;
  std::string definition = deffunction->header.ppForm;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->assert_deffunction(name, module_name, definition, db->get_tick());
}

void CDBSaverPlugin::cdb_deffunction_retract_callback(
  clips::Environment * /*env*/, clips::Deffunction * deffunction, void * context)
{
  std::string name = deffunction->header.name->contents;
  std::string module_name = deffunction->header.whichModule->theModule->header.name->contents;
  std::string definition = deffunction->header.ppForm;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->retract_deffunction(name, module_name, db->get_tick());
}

void CDBSaverPlugin::cdb_defglobal_assert_callback(
  clips::Environment * /*env*/, clips::Defglobal * defglobal, void * context)
{
  CDBSaverPlugin * cdb_plugin = static_cast<CDBSaverPlugin *>(context);
  std::string name = defglobal->header.name->contents;
  std::string module_name = defglobal->header.whichModule->theModule->header.name->contents;
  std::string value_json =
    cdb_plugin->slot_value_to_json(defglobal->current.header->type, &defglobal->current).dump();
  DBHandler * db = static_cast<DBHandler *>(context);
  db->assert_defglobal(name, module_name, value_json, db->get_tick());
}

void CDBSaverPlugin::cdb_defglobal_retract_callback(
  clips::Environment * /*env*/, clips::Defglobal * defglobal, void * context)
{
  std::string name = defglobal->header.name->contents;
  std::string module_name = defglobal->header.whichModule->theModule->header.name->contents;
  DBHandler * db = static_cast<DBHandler *>(context);
  db->retract_defglobal(name, module_name, db->get_tick());
}

void CDBSaverPlugin::cdb_before_rule_callback(
  clips::Environment * /*env*/, clips::Activation * act, void * context)
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
  DBHandler * db = static_cast<DBHandler *>(context);
  db->assert_rule_fired(name, module_name, basis, db->get_tick());
}

void CDBSaverPlugin::cdb_before_run_callback(clips::Environment * /*env*/, void * context)
{
  DBHandler * db = static_cast<DBHandler *>(context);
  db->current_run_ = db->start_run(db->parent_.lock()->now().nanoseconds(), db->tick_);
}

void CDBSaverPlugin::cdb_after_run_callback(clips::Environment * /*env*/, void * context)
{
  DBHandler * db = static_cast<DBHandler *>(context);
  db->end_run(db->current_run_, db->parent_.lock()->now().nanoseconds(), db->tick_);
}

nlohmann::json CDBSaverPlugin::slot_value_to_json(unsigned short type, clips::CLIPSValue * value)
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
      json["type"] = "FACT_ADDRESS";
      json["value"] = value->factValue->factIndex;
      break;
    }
    default:
      json["type"] = "UNKNOWN";
      throw "UNKNOWN TYPE";
      // NOTE instance is for COOL; BITMAP is internal
  }
  return json;
}

std::vector<nlohmann::json> CDBSaverPlugin::multifield_to_json_list(clips::Multifield * theSegment)
{
  clips::CLIPSValue * theMultifield = theSegment->contents;
  size_t range = theSegment->length;
  std::vector<nlohmann::json> json_list;
  for (size_t j = 0; j < range; j++) {
    json_list.push_back(slot_value_to_json(theMultifield[j].header->type, &theMultifield[j]));
  }
  return json_list;
}

std::string CDBSaverPlugin::clips_fact_to_json(clips::Fact * f, const char * deftemplate_name)
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

bool CDBSaverPlugin::clips_env_destroyed(std::shared_ptr<clips::Environment> & env)
{
  CLIPSEnvContext * context = CLIPSEnvContext::get_context(env.get());
  RCLCPP_INFO(*logger_, "Destroying CDBSaverPlugin for environment %s", context->env_name_.c_str());

  clips::RemoveAssertFunction(env.get(), "cdb_assert_callback");
  clips::RemoveRetractFunction(env.get(), "cdb_retract_callback");

  clips::RemoveBeforeRuleFiresFunction(env.get(), "cdb_before_rule_callback");

  clips::RemoveBeforeRunFiresFunction(env.get(), "cdb_before_run_callback");
  clips::RemoveAfterRunFiresFunction(env.get(), "cdb_after_run_callback");

  clips::RemoveDefruleAssertFunction(env.get(), "cdb_defrule_assert_callback");
  clips::RemoveDefruleRetractFunction(env.get(), "cdb_defrule_retract_callback");

  clips::RemoveDeftemplateAssertFunction(env.get(), "cdb_deftemplate_assert_callback");
  clips::RemoveDeftemplateRetractFunction(env.get(), "cdb_deftemplate_retract_callback");

  clips::RemoveDeffunctionAssertFunction(env.get(), "cdb_deffunction_assert_callback");
  clips::RemoveDeffunctionRetractFunction(env.get(), "cdb_deffunction_retract_callback");

  clips::RemoveDeffactsAssertFunction(env.get(), "cdb_deffacts_assert_callback");
  clips::RemoveDeffactsRetractFunction(env.get(), "cdb_deffacts_retract_callback");

  clips::RemoveDefglobalAssertFunction(env.get(), "cdb_defglobal_assert");
  clips::RemoveDefglobalRetractFunction(env.get(), "cdb_defglobal_retract");
  db_handlers_.erase(context->env_name_);
  return true;
}
}  // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::CDBSaverPlugin, cx::ClipsPlugin)
