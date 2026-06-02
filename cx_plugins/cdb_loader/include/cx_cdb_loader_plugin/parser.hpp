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

#pragma once

// clang-format off
#include <pqxx/pqxx>
#undef RANGES
// RANGES is defined in clips_ns/clips.h, which causes issues with
// pqxx/pqxx. It needs to be included before clips_ns/clips.h to avoid compilation errors.

#include <clips_ns/clips.h>
// clang-format on

#include <cx_utils/param_utils.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <regex>

namespace cx
{

struct Defmodule;

struct RegexRule
{
  std::regex regex;
  bool contains_match = false;
};

struct GenericRegexRule
{
  std::vector<RegexRule> skip_if_match;
  std::vector<RegexRule> load_if_match;
};

struct RegexConfig
{
  GenericRegexRule module_rules;

  GenericRegexRule defglobal_rules;
  bool skip_defglobals;
  GenericRegexRule deftemplate_rules;
  bool skip_deftemplates;
  GenericRegexRule defrule_rules;
  bool skip_defrules;
  GenericRegexRule deffacts_rules;
  bool skip_deffacts;
  GenericRegexRule deffunction_rules;
  bool skip_deffunctions;
  GenericRegexRule activation_rules;
  bool skip_activations;
  GenericRegexRule fact_rules;
  bool skip_facts;

  bool skip_external_address_values;
  bool external_address_values_as_nullptr;
};

bool matches_regex_rule(const RegexRule & rule, const std::string & value);

bool matches_any_regex_rule(const std::vector<RegexRule> & rules, const std::string & value);

bool allowed_by_generic_regex_rule(const GenericRegexRule & rule, const std::string & value);

RegexRule populate_regex_rule(const std::string & regex_text, bool is_skip);

GenericRegexRule populate_generic_regex_rule(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & parameter_prefix);

RegexConfig populate_regex_config(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & plugin_name);

template <typename T, typename ObjectNameGetter>
void filter_in_place(
  std::vector<T> & values, const GenericRegexRule & object_rules, ObjectNameGetter get_object_name)
{
  values.erase(
    std::remove_if(
      values.begin(), values.end(),
      [&](const T & value) {
        return !allowed_by_generic_regex_rule(object_rules, get_object_name(value));
      }),
    values.end());
}

}  // namespace cx
