// Copyright (c) 2024-2025 Carologistics
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Library General Public License for more details.
//
// Read the full text in the LICENSE.GPL file in the main directory.

/***************************************************************************
 *  pddl_parser_plugin.cpp
 *
 *  Created: 15 September 2021
 *  Copyright  2024  Tarik Viehmann
 ****************************************************************************/

#include <map>
#include <memory>
#include <string>

#include <cx_pddl_parser_plugin/clips_pddl_parser.h>

#include "cx_pddl_parser_plugin/pddl_parser_plugin.hpp"
#include "cx_plugin/clips_plugin.hpp"

#include <cx_utils/clips_env_context.hpp>

// To export as plugin
#include "pluginlib/class_list_macros.hpp"

namespace cx {

PddlParserPlugin::PddlParserPlugin() {}
PddlParserPlugin::~PddlParserPlugin() {}

void PddlParserPlugin::initialize() {
  plugin_path_ =
      ament_index_cpp::get_package_share_directory("cx_pddl_parser_plugin");
  logger_ = std::make_unique<rclcpp::Logger>(rclcpp::get_logger(plugin_name_));
}

bool PddlParserPlugin::clips_env_init(
    std::shared_ptr<clips::Environment> &env) {
  RCLCPP_DEBUG(rclcpp::get_logger(plugin_name_), "Initializing plugin %s",
               plugin_name_.c_str());
  auto context = CLIPSEnvContext::get_context(env.get());
  std::string env_name = context->env_name_;

  pddl_parsers_[env_name] =
      std::make_unique<clips_pddl_parser::ClipsPddlParser>(env.get(),
                                                           context->env_mtx_);
  std::vector<std::string> files{plugin_path_ +
                                 "/clips/cx_pddl_parser_plugin/domain.clp"};
  for (const auto &f : files) {
    if (!clips::BatchStar(env.get(), f.c_str())) {
      RCLCPP_ERROR(*logger_,
                   "Failed to initialize CLIPS environment, "
                   "batch file '%s' failed!, aborting...",
                   f.c_str());
      return false;
    }
  }
  return true;
}

bool PddlParserPlugin::clips_env_destroyed(
    std::shared_ptr<clips::Environment> &env) {
  auto context = CLIPSEnvContext::get_context(env.get());
  std::string env_name = context->env_name_;

  RCLCPP_DEBUG(rclcpp::get_logger(plugin_name_), "Destroying clips context!");

  pddl_parsers_.erase(env_name);
  return true;
}
} // namespace cx
PLUGINLIB_EXPORT_CLASS(cx::PddlParserPlugin, cx::ClipsPlugin)
