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
#include <sstream>
#include <vector>
#include <stdexcept>

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
        w.exec("CREATE TABLE facts (fact_id SERIAL PRIMARY KEY, "
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

void DBHandler::assert_fact(long long id, std::string fact_json, long long tick) {
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // Check if fact already exists
        pqxx::result result = w.exec_params(
            "SELECT fact_id FROM facts WHERE fact_id = $1",
            id
        );
        
        if (result.empty()) {
            // Insert new fact with initial timed_fact
            w.exec_params(
                "INSERT INTO facts (fact_id, name, value) VALUES ($1, $2, ARRAY[($3, $4)::timed_fact])",
                id, "", tick, fact_json
            );
            
            // Add to fact_lifetime
            w.exec_params(
                "INSERT INTO fact_lifetime (fact_id, start_tick, end_tick) VALUES ($1, $2, NULL)",
                id, tick
            );
        } else {
            // Update existing fact by appending to timed_fact array
            w.exec_params(
                "UPDATE facts SET value = value || ARRAY[($1, $2)::timed_fact] WHERE fact_id = $3",
                tick, fact_json, id
            );
            
            // Update fact_lifetime - set end_tick to NULL (fact is active again)
            w.exec_params(
                "UPDATE fact_lifetime SET end_tick = NULL WHERE fact_id = $1 AND end_tick IS NOT NULL",
                id
            );
        }
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to assert fact: " + std::string(e.what()));
    }
}

void DBHandler::retract_fact(long long id, long long tick) {
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // Update fact_lifetime to mark fact as retracted
        pqxx::result result = w.exec_params(
            "UPDATE fact_lifetime SET end_tick = $1 WHERE fact_id = $2 AND end_tick IS NULL",
            tick, id
        );
        
        if (result.affected_rows() == 0) {
            throw std::runtime_error("Fact with ID " + std::to_string(id) + " not found or already retracted");
        }
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to retract fact: " + std::string(e.what()));
    }
}

void DBHandler::update_fact(long long id, std::string fact_json, long long tick) {
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // Check if fact exists
        pqxx::result result = w.exec_params(
            "SELECT fact_id FROM facts WHERE fact_id = $1",
            id
        );
        
        if (result.empty()) {
            throw std::runtime_error("Fact with ID " + std::to_string(id) + " does not exist");
        }
        
        // Append new timed_fact to the array
        w.exec_params(
            "UPDATE facts SET value = value || ARRAY[($1, $2)::timed_fact] WHERE fact_id = $3",
            tick, fact_json, id
        );
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to update fact: " + std::string(e.what()));
    }
}

void DBHandler::add_rule(std::string name, std::string module_name, std::string definition) {
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // For now, we'll store the complete definition in lhs and leave rhs empty
        // This could be enhanced to parse the rule definition into LHS and RHS
        w.exec_params(
            "INSERT INTO rules (name, module, lhs, rhs) VALUES ($1, $2, $3, $4)",
            name, module_name, definition, ""
        );
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to add rule: " + std::string(e.what()));
    }
}

void DBHandler::add_funtion(std::string name, std::string module_name, std::string definition) {
    // Functions can be stored in the rules table with a special marker or in a separate way
    // For now, we'll store them as rules with a function prefix
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // Store function as a rule with function prefix
        w.exec_params(
            "INSERT INTO rules (name, module, lhs, rhs) VALUES ($1, $2, $3, $4)",
            "FUNCTION:" + name, module_name, definition, ""
        );
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to add function: " + std::string(e.what()));
    }
}

void DBHandler::add_defglobal(std::string name, std::string module_name, std::string definition) {
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // Store defglobal as a rule with defglobal prefix
        w.exec_params(
            "INSERT INTO rules (name, module, lhs, rhs) VALUES ($1, $2, $3, $4)",
            "DEFGLOBAL:" + name, module_name, definition, ""
        );
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to add defglobal: " + std::string(e.what()));
    }
}

void DBHandler::add_deftemplate(std::string name, std::string module_name, std::string definition) {
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // Store deftemplate as a rule with deftemplate prefix
        w.exec_params(
            "INSERT INTO rules (name, module, lhs, rhs) VALUES ($1, $2, $3, $4)",
            "DEFTEMPLATE:" + name, module_name, definition, ""
        );
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to add deftemplate: " + std::string(e.what()));
    }
}

void DBHandler::add_rule_fired(std::string name, std::string modulle, std::vector<long long> basis, long long tick) {
    if (!connection_ || !connection_->is_open()) {
        throw std::runtime_error("Database connection is not open");
    }
    
    try {
        pqxx::work w(*connection_);
        
        // First, get the rule_id for the given rule name and module
        pqxx::result rule_result = w.exec_params(
            "SELECT rule_id FROM rules WHERE name = $1 AND module = $2",
            name, modulle
        );
        
        if (rule_result.empty()) {
            throw std::runtime_error("Rule '" + name + "' in module '" + modulle + "' not found");
        }
        
        int rule_id = rule_result[0][0].as<int>();
        
        // Convert vector to PostgreSQL array format
        std::ostringstream array_str;
        array_str << "{";
        for (size_t i = 0; i < basis.size(); ++i) {
            if (i > 0) array_str << ",";
            array_str << basis[i];
        }
        array_str << "}";
        
        // Insert rule firing record
        w.exec_params(
            "INSERT INTO rule_firing (rule_id, base, tick) VALUES ($1, $2, $3)",
            rule_id, array_str.str(), tick
        );
        
        w.commit();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to add rule firing: " + std::string(e.what()));
    }
}

} // namespace cx
