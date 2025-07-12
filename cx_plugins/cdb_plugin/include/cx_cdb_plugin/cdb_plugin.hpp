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

#ifndef CX_PLUGINS__CDB_PLUGIN_HPP_
#define CX_PLUGINS__CDB_PLUGIN_HPP_

#include "cx_plugin/clips_plugin.hpp"

namespace cx {

class CDBPlugin : public ClipsPlugin {
  public:
    CDBPlugin();
    ~CDBPlugin();

    void initialize() override;

    static void cdb_assert_callback(clips::Environment *, void *, void *);
    static void cdb_before_rule_callback(clips::Environment *,
                                         clips::Activation *, void *);
    bool clips_env_init(std::shared_ptr<clips::Environment> &env) override;
    bool clips_env_destroyed(std::shared_ptr<clips::Environment> &env) override;

  private:
    std::unique_ptr<rclcpp::Logger> logger_;
    bool started_ = false;
};
} // namespace cx

#endif // !CX_PLUGINS__EXAMPLE_PLUGIN_HPP_
