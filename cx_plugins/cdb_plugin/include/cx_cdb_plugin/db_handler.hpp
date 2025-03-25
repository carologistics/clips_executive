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

    bool init_db(DBHandlerConfig config);

  private:
    std::shared_ptr<pqxx::connection> connection_;
    DBHandlerConfig config_;
};
} // namespace cx
