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
 *  pddl_parser_plugin.hpp
 *
 *  Copyright  2024  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

#ifndef CX_PLUGINS__CLIPSPDDLPARSERPLUGIN_HPP_
#define CX_PLUGINS__CLIPSPDDLPARSERPLUGIN_HPP_

#include <map>
#include <memory>
#include <string>

#include <cx_pddl_parser_plugin/clips_pddl_parser.h>

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/lock_shared_ptr.hpp"

namespace cx {

class PddlParserPlugin : public ClipsPlugin {
public:
  PddlParserPlugin();
  ~PddlParserPlugin();

  void initialize() override;

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  std::map<std::string, std::unique_ptr<clips_pddl_parser::ClipsPddlParser>>
      pddl_parsers_;

  std::unique_ptr<rclcpp::Logger> logger_;

  std::string plugin_path_;
};

} // namespace cx
#endif // !CX_PLUGINS__CLIPSPDDLPARSERPLUGIN_HPP_
