// Copyright (c) 2025 Carologistics
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

#include <iomanip>
#include <pqxx/pqxx>
#undef RANGES

namespace cx {
struct DBHandlerConfig {
    std::string hostname;
    int port;
    std::string username;
    std::string password;
    std::string db_name;
};
class DBHandler {
  public:
    DBHandler(DBHandlerConfig config, bool create_db = true);
    ~DBHandler();

    void assert_fact(long long id, std::string fact_json, long long tick);
    void retract_fact(long long id, long long tick);
    void update_fact(long long id, std::string fact_json, long long tick);
    void add_rule(std::string name, std::string module_name, std::string definition);
    void add_funtion(std::string name, std::string module_name, std::string definition);
    void add_defglobal(std::string name, std::string module_name, std::string definition);
    void add_deftemplate(std::string name, std::string module_name, std::string definition);
    void add_rule_fired(std::string name, std::string modulle, std::vector<long long> basis, long long tick);


    bool init_db(DBHandlerConfig config);

  private:
    std::shared_ptr<pqxx::connection> connection_;
    DBHandlerConfig config_;
};
} // namespace cx
