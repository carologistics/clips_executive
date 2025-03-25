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

#include "cx_cdb_plugin/db_handler.hpp"
#include <iostream>
#include <thread>

namespace cx {

DBHandler::DBHandler(DBHandlerConfig config, bool create_db) : config_(config) {
    if (create_db) {
        if (!init_db(config)) {
            throw(std::runtime_error("Failed to initialize database"));
        }
    }
    try {
        std::string connection_settings{
            "dbname=postgres user=" + config.username + " password=" +
            config.password + " port=" + std::to_string(config.port) +
            " host=" + config.hostname};
        connection_ = std::make_shared<pqxx::connection>(connection_settings);

    } catch (const std::exception &e) {
        throw(std::runtime_error("Failed to connect to database: " +
                                 std::string(e.what())));
    }
}

DBHandler::~DBHandler() {
    if (connection_) {
        connection_->close();
    }
}

bool DBHandler::init_db(DBHandlerConfig config) {
    std::ostringstream admin_conn;
    admin_conn << "dbname=postgres gssencmode=disable "
               << "user=" << config.username << ' ';
    if (!config.password.empty()) {
        admin_conn << "password=" << config.password << ' ';
    }
    admin_conn << "port=" << config.port << ' ' << "host=" << config.hostname;

    pqxx::connection admin_connection{admin_conn.str()};

    if (admin_connection.is_open()) {
        pqxx::nontransaction n(admin_connection);
        n.exec("CREATE DATABASE " + config.db_name + " WITH OWNER " +
               config.username + ";");
        // while(!n.exec("SELECT 1 FROM pg_database WHERE datname = '" +
        // config.db_name + "';").empty()) {
        //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }
        admin_connection.close();
    } else {
        return false;
    }
    std::ostringstream conn;
    conn << "dbname=" << config.db_name << ' ' << "user=" << config.username
         << ' ' << "gssencmode=disable ";
    if (!config.password.empty()) {
        conn << "password=" << config.password << ' ';
    }
    conn << "port=" << config.port << ' ' << "host=" << config.hostname;
    pqxx::connection db_connection{conn.str()};

    if (db_connection.is_open()) {
        pqxx::work w{db_connection};
        w.exec("CREATE TYPE timed_fact AS (tick BIGINT, value JSONB);");
        w.exec("CREATE TABLE time_lookup (timestamp TIMESTAMPTZ, tick BIGINT, "
               "run_number BIGINT);");
        w.exec("CREATE TABLE facts (fact_id SERIAL PRIMARY KEY, module TEXT, "
               "name TEXT, value timed_fact[]);");
        w.exec("CREATE TABLE fact_lifetime (fact_id INT REFERENCES "
               "facts(fact_id), start_tick BIGINT, end_tick BIGINT);");
        w.exec("CREATE TABLE rules (rule_id SERIAL PRIMARY KEY, name TEXT, "
               "module TEXT, lhs TEXT, rhs TEXT);");
        w.exec("CREATE TABLE rule_firing (rule_id INT REFERENCES "
               "rules(rule_id), base INT[], tick BIGINT);");
        w.exec(R"(CREATE FUNCTION check_facts() RETURNS TRIGGER AS $$
      DECLARE
          missing_id INT;
      BEGIN
          -- Find any element in NEW.base[] that is not present in facts.fact_id
          SELECT unnest(NEW.base)
            EXCEPT
          SELECT fact_id
          FROM facts
          INTO missing_id;

          IF missing_id IS NOT NULL THEN
              RAISE EXCEPTION 'Fact with ID % does not exist', missing_id;
          END IF;

          RETURN NEW;
      END;
      $$ LANGUAGE plpgsql;)");
        w.exec("CREATE TRIGGER check_facts_trigger BEFORE INSERT OR UPDATE ON "
               "rule_firing FOR EACH ROW EXECUTE FUNCTION check_facts();");
        w.commit();
        db_connection.close();
        return true;
    } else {
        return false;
    }
}

} // namespace cx
