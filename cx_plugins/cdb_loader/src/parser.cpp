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

#include "cx_cdb_loader_plugin/parser.hpp"

namespace cx
{

bool matches_regex_rule(const RegexRule & rule, const std::string & value)
{
  if (rule.contains_match) {
    return std::regex_search(value, rule.regex);
  }

  return std::regex_match(value, rule.regex);
}

bool matches_any_regex_rule(const std::vector<RegexRule> & rules, const std::string & value)
{
  for (const RegexRule & rule : rules) {
    if (matches_regex_rule(rule, value)) {
      return true;
    }
  }

  return false;
}

bool allowed_by_generic_regex_rule(const GenericRegexRule & rule, const std::string & value)
{
  return !matches_any_regex_rule(rule.skip_if_match, value) &&
         matches_any_regex_rule(rule.load_if_match, value);
}

RegexRule populate_regex_rule(const std::string & regex_text, bool is_skip)
{
  if (regex_text.empty()) {
    if (is_skip) {
      return RegexRule{std::regex("a^", std::regex::ECMAScript | std::regex::optimize), false};
    }
    return RegexRule{std::regex(".*", std::regex::ECMAScript | std::regex::optimize), false};
  }

  if (!regex_text.empty() && regex_text.front() == '/') {
    return RegexRule{
      std::regex(regex_text.substr(1), std::regex::ECMAScript | std::regex::optimize), true};
  }

  return RegexRule{std::regex(regex_text, std::regex::ECMAScript | std::regex::optimize), false};
}

GenericRegexRule populate_generic_regex_rule(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & parameter_prefix)
{
  cx::cx_utils::declare_parameter_if_not_declared(
    node, parameter_prefix + ".load_if_match",
    rclcpp::ParameterValue(std::vector<std::string>{""}));

  cx::cx_utils::declare_parameter_if_not_declared(
    node, parameter_prefix + ".skip_if_match",
    rclcpp::ParameterValue(std::vector<std::string>{""}));

  std::vector<std::string> load_if_match_params;
  std::vector<std::string> skip_if_match_params;

  node->get_parameter(parameter_prefix + ".load_if_match", load_if_match_params);
  node->get_parameter(parameter_prefix + ".skip_if_match", skip_if_match_params);

  GenericRegexRule rule;

  for (const std::string & regex_text : load_if_match_params) {
    rule.load_if_match.push_back(populate_regex_rule(regex_text, false));
  }

  for (const std::string & regex_text : skip_if_match_params) {
    rule.skip_if_match.push_back(populate_regex_rule(regex_text, true));
  }

  return rule;
}

RegexConfig populate_regex_config(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & plugin_name)
{
  RegexConfig config{};

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".defglobals.skip", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".deftemplates.skip", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".defrules.skip", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".deffacts.skip", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".deffunctions.skip", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".activations.skip", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".facts.skip", rclcpp::ParameterValue(false));

  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".skip_external_address_values", rclcpp::ParameterValue(false));
  cx::cx_utils::declare_parameter_if_not_declared(
    node, plugin_name + ".external_address_values_as_nullptr", rclcpp::ParameterValue(true));

  node->get_parameter(plugin_name + ".defglobals.skip", config.skip_defglobals);
  node->get_parameter(plugin_name + ".deftemplates.skip", config.skip_deftemplates);
  node->get_parameter(plugin_name + ".defrules.skip", config.skip_defrules);
  node->get_parameter(plugin_name + ".deffacts.skip", config.skip_deffacts);
  node->get_parameter(plugin_name + ".deffunctions.skip", config.skip_deffunctions);
  node->get_parameter(plugin_name + ".activations.skip", config.skip_activations);
  node->get_parameter(plugin_name + ".facts.skip", config.skip_facts);

  node->get_parameter(
    plugin_name + ".skip_external_address_values", config.skip_external_address_values);

  node->get_parameter(
    plugin_name + ".external_address_values_as_nullptr", config.external_address_values_as_nullptr);

  config.module_rules = populate_generic_regex_rule(node, plugin_name + ".modules");

  config.defglobal_rules = populate_generic_regex_rule(node, plugin_name + ".modules");

  config.defglobal_rules = populate_generic_regex_rule(node, plugin_name + ".defglobals");

  config.deftemplate_rules = populate_generic_regex_rule(node, plugin_name + ".deftemplates");

  config.defrule_rules = populate_generic_regex_rule(node, plugin_name + ".defrules");

  config.deffacts_rules = populate_generic_regex_rule(node, plugin_name + ".deffacts");

  config.deffunction_rules = populate_generic_regex_rule(node, plugin_name + ".deffunctions");

  config.activation_rules = populate_generic_regex_rule(node, plugin_name + ".activations");

  config.fact_rules = populate_generic_regex_rule(node, plugin_name + ".facts");

  return config;
}

void filter_defmodule_in_place(
  std::vector<Defmodule> & values, const GenericRegexRule & module_rules)
{
  values.erase(
    std::remove_if(
      values.begin(), values.end(),
      [&](const Defmodule & value) {
        return !allowed_by_generic_regex_rule(module_rules, value.name);
      }),
    values.end());
}

}  // namespace cx
