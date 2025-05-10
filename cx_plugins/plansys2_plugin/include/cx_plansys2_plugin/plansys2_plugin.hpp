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

#ifndef CX_PLUGINS__PLANSYS2PLUGIN_HPP_
#define CX_PLUGINS__PLANSYS2PLUGIN_HPP_

#include <string>

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "cx_plugin/clips_plugin.hpp"
#include "cx_utils/lock_shared_ptr.hpp"

namespace cx {

class Plansys2Plugin : public ClipsPlugin {
public:
  Plansys2Plugin();
  ~Plansys2Plugin();

  bool clips_env_init(LockSharedPtr<clips::Environment> &env) override;
  bool clips_env_destroyed(LockSharedPtr<clips::Environment> &env) override;

private:
  // PLANSYS2 CLIENTS
  std::unordered_map<std::string, std::unique_ptr<plansys2::DomainExpertClient>>
      domain_client_;
  std::unordered_map<std::string,
                     std::unique_ptr<plansys2::ProblemExpertClient>>
      problem_client_;
  std::unordered_map<std::string, std::unique_ptr<plansys2::PlannerClient>>
      planner_client_;

  unsigned int plan_id_{0};

private:
  inline unsigned int next_plan_id() { return ++plan_id_; }

  void add_problem_instance(clips::Environment *env, const std::string &name,
                            const std::string &type);
  void add_problem_predicate(clips::Environment *env, const std::string &pred);
  void clear_problem_expert_knowledge(clips::Environment *env);

  void plan_with_plansys2(clips::Environment *env, const std::string &goal_id,
                          const std::string &goal, const std::string &plan_id);

  void call_planner(clips::Environment *env, const std::string &goal_id,
                    const std::string &goal, const std::string &plan_id);
};

} // namespace cx
#endif // !CX_PLUGINS__PLANSYS2PLUGIN_HPP_
