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

#include <cstdint>
#include <memory>
#include <optional>
#include <pqxx/pqxx>
#undef RANGES

namespace cx
{
struct DBHandlerConfig
{
  std::string hostname;
  int port;
  std::string username;
  std::string password;
  std::string db_name;
};
class DBHandler
{
public:
  DBHandler(DBHandlerConfig & config, bool create_db = true);
  ~DBHandler();

  void assert_fact(
    long long id, const std::string & deftemplate, const std::string & fact_json, long long tick);
  void retract_fact(long long id, long long tick);

  void assert_defrule(
    const std::string & name, const std::string & module_name, const std::string & definition,
    const int salience, long long tick);
  void retract_defrule(const std::string & name, const std::string & module_name, long long tick);

  void assert_deffunction(
    const std::string & name, const std::string & module_name, const std::string & definition,
    long long tick);
  void retract_deffunction(
    const std::string & name, const std::string & module_name, long long tick);

  void assert_deffacts(
    const std::string & name, const std::string & module_name, const std::string & definition,
    long long tick);
  void retract_deffacts(const std::string & name, const std::string & module_name, long long tick);

  void assert_defglobal(
    const std::string & name, const std::string & module_name, const std::string & value_json,
    long long tick);
  void retract_defglobal(const std::string & name, const std::string & module_name, long long tick);

  void assert_deftemplate(
    const std::string & name, const std::string & module_name, const std::string & definition,
    long long tick);
  void retract_deftemplate(
    const std::string & name, const std::string & module_name, long long tick);

  void assert_rule_fired(
    const std::string & name, const std::string & module,
    const std::vector<std::optional<long long int>> & basis, long long tick);

  long long start_run(int64_t start_time_ns, long long start_tick);
  void end_run(long long run_number, int64_t end_time_ns, long long end_tick);

  bool init_db(DBHandlerConfig & config);

private:
  std::shared_ptr<pqxx::connection> connection_;
  DBHandlerConfig config_;
};
}  // namespace cx
