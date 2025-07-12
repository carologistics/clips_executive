// Copyright (c) 2024-2025 Carologistics
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
// pqxx/pqxx
#include "cx_cdb_plugin/cdb_plugin.hpp"
#include "cx_cdb_plugin/db_handler.hpp"
#include <bits/types/struct_tm.h>
#include <bits/types/time_t.h>
#include <clips_ns/agenda.h>
#include <clips_ns/engine.h>
#include <clips_ns/entities.h>
#include <clips_ns/factfun.h>
#include <clips_ns/factmngr.h>
#include <clips_ns/match.h>
#include <clips_ns/utility.h>
#include <ctime>
#include <cx_utils/clips_env_context.hpp>
#include <cx_utils/param_utils.hpp>
#include <iomanip>
#include <memory>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx {
CDBPlugin::CDBPlugin() {}

CDBPlugin::~CDBPlugin() {}

void CDBPlugin::initialize() {
    logger_ =
        std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));

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
        node, plugin_name_ + ".password",
        rclcpp::ParameterValue(std::string()));
}

bool CDBPlugin::clips_env_init(std::shared_ptr<clips::Environment> &env) {
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
        RCLCPP_ERROR(*logger_,
                     "TODO: MULTIPLE INITIALIZATION OF CDB NOT YET SUPPORTED");
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

    CLIPSEnvContext *context = CLIPSEnvContext::get_context(env.get());
    RCLCPP_INFO(*logger_, "Initializing plugin for environment %s",
                context->env_name_.c_str());
    // clips::AddAssertFunction(env.get_obj().get(), "cdb_assert_callback",
    // &cdb_assert_callback, 0, this);
    // clips::AddBeforeRuleFiresFunction(env.get_obj().get(),
    // "cdb_before_rule_callback",  &cdb_before_rule_callback, 0, this);

    DBHandlerConfig config = {hostname, port, username, password, db_name};
    try {
        RCLCPP_INFO(*logger_, "Connecting to database %s at %s:%d with user %s",
                    config.db_name.c_str(), config.hostname.c_str(),
                    config.port, config.username.c_str());
        DBHandler db_handler(config, true);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(*logger_, "Failed to connect to database: %s", e.what());
        return false;
    }
    return true;
}

void CDBPlugin::cdb_before_rule_callback(clips::Environment *env,
                                         clips::Activation *act,
                                         void *context) {
    CDBPlugin *cdb_plugin = static_cast<CDBPlugin *>(context);
    RCLCPP_INFO(*cdb_plugin->logger_, "Rule %s is about to fire",
                act->theRule->header.ppForm);
    for (int i = 0; i < act->basis->bcount; i++) {
        if ((get_nth_pm_match(act->basis, i) != NULL) &&
            (get_nth_pm_match(act->basis, i)->matchingItem != NULL)) {
            clips::PatternEntity *matchingItem =
                get_nth_pm_match(act->basis, i)->matchingItem;
            RCLCPP_INFO(*cdb_plugin->logger_, "Fact: %lli",
                        (((clips::Fact *)matchingItem))->factIndex);
        } else {
            RCLCPP_INFO(*cdb_plugin->logger_, "Fact: *");
        }
        if (i < act->basis->bcount)
            RCLCPP_INFO(*cdb_plugin->logger_, ",");
    }
}

void CDBPlugin::cdb_assert_callback(clips::Environment *env, void *fact,
                                    void *context) {
    CDBPlugin *cdb_plugin = static_cast<CDBPlugin *>(context);
    clips::Fact *f = static_cast<clips::Fact *>(fact);
    clips::StringBuilder *sb = clips::CreateStringBuilder(env, 1024);
    clips::FactPPForm(f, sb, false);
    RCLCPP_INFO(*cdb_plugin->logger_, "Fact %lld asserted with content: %s",
                f->factIndex, sb->contents);
}

bool CDBPlugin::clips_env_destroyed(std::shared_ptr<clips::Environment> &env) {
    CLIPSEnvContext *context = CLIPSEnvContext::get_context(env.get());
    RCLCPP_INFO(*logger_, "Destroying plugin for environment %s",
                context->env_name_.c_str());
    return true;
}
} // namespace cx

PLUGINLIB_EXPORT_CLASS(cx::CDBPlugin, cx::ClipsPlugin)
