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
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace cx
{

DBHandler::DBHandler(DBHandlerConfig & config, bool create_db) : config_(config)
{
  if (create_db) {
    if (!init_db(config)) {
      throw(std::runtime_error("Failed to initialize database"));
    }
  }
  try {
    std::string connection_settings{
      "dbname=" + config.db_name + " user=" + config.username + " password=" + config.password +
      " port=" + std::to_string(config.port) + " host=" + config.hostname + " gssencmode=disable"};
    connection_ = std::make_shared<pqxx::connection>(connection_settings);

  } catch (const std::exception & e) {
    throw(std::runtime_error("Failed to connect to database: " + std::string(e.what())));
  }
}

DBHandler::~DBHandler()
{
  if (connection_) {
    connection_->close();
  }
}

static constexpr std::string_view dbSchemaSQL = R"sql(
CREATE TYPE timed_fact AS (
  tick  BIGINT,
  value JSONB
);

CREATE TABLE time_lookup (
  timestamp  TIMESTAMPTZ NOT NULL,
  tick       BIGINT      NOT NULL,
  run_number BIGINT      NOT NULL
);

CREATE TABLE facts (
  fact_id BIGINT PRIMARY KEY,
  name    TEXT NOT NULL DEFAULT '',
  value   timed_fact[] NOT NULL DEFAULT '{}'
);

CREATE TABLE fact_lifetime (
  fact_id    BIGINT PRIMARY KEY REFERENCES facts(fact_id) ON DELETE CASCADE,
  start_tick BIGINT NOT NULL,
  end_tick   BIGINT
);

CREATE TABLE rules (
  rule_id SERIAL PRIMARY KEY,
  name    TEXT NOT NULL,
  module  TEXT NOT NULL,
  lhs     TEXT NOT NULL,
  rhs     TEXT NOT NULL,
  salience  INTEGER NOT NULL,
  UNIQUE (name, module)
);

CREATE TABLE rule_firing (
  rule_id INT      NOT NULL REFERENCES rules(rule_id) ON DELETE CASCADE,
  base    BIGINT[] NOT NULL,
  tick    BIGINT   NOT NULL
);

CREATE FUNCTION assert_fact(p_id bigint, p_fact_json jsonb, p_tick bigint)
RETURNS void AS $$
BEGIN
    INSERT INTO facts (fact_id, name, value)
    VALUES (p_id, '', ARRAY[ROW(p_tick, p_fact_json)::timed_fact])
    ON CONFLICT (fact_id)
    DO UPDATE
    SET value = facts.value || EXCLUDED.value;

    INSERT INTO fact_lifetime (fact_id, start_tick, end_tick)
    VALUES (p_id, p_tick, NULL)
    ON CONFLICT (fact_id) DO NOTHING;
END;
$$ LANGUAGE plpgsql;

-- TODO Disable in real application
CREATE FUNCTION check_facts() RETURNS TRIGGER AS $$
DECLARE
    missing_id BIGINT;
BEGIN
    SELECT x
    INTO missing_id
    FROM unnest(NEW.base) AS x
    WHERE NOT EXISTS (
        SELECT 1
        FROM facts f
        WHERE f.fact_id = x
    )
    LIMIT 1;

    IF missing_id IS NOT NULL THEN
        RAISE EXCEPTION 'Fact with ID % does not exist', missing_id;
    END IF;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER check_facts_trigger
BEFORE INSERT OR UPDATE ON rule_firing
FOR EACH ROW
EXECUTE FUNCTION check_facts();
)sql";

bool DBHandler::init_db(DBHandlerConfig & config)
{
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
    n.exec("CREATE DATABASE " + config.db_name + " WITH OWNER " + config.username + ";");
    // while(!n.exec("SELECT 1 FROM pg_database WHERE datname = '" +
    // config.db_name + "';").empty()) {
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    admin_connection.close();
  } else {
    return false;
  }
  std::ostringstream conn;
  conn << "dbname=" << config.db_name << ' ' << "user=" << config.username << ' '
       << "gssencmode=disable ";
  if (!config.password.empty()) {
    conn << "password=" << config.password << ' ';
  }
  conn << "port=" << config.port << ' ' << "host=" << config.hostname;
  pqxx::connection db_connection{conn.str()};

  if (db_connection.is_open()) {
    pqxx::work w{db_connection};
    w.exec(dbSchemaSQL);
    w.commit();
    db_connection.close();
    return true;
  } else {
    return false;
  }
}

void DBHandler::assert_fact(long long id, const std::string & fact_json, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT assert_fact($1, $2::jsonb, $3)", id, fact_json, tick);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert fact: " + std::string(e.what()));
  }
}

void DBHandler::retract_fact(long long id, long long tick)
{
  try {
    pqxx::work w(*connection_);

    // Update fact_lifetime to mark fact as retracted
    pqxx::result result = w.exec_params(
      "UPDATE fact_lifetime SET end_tick = $1 WHERE fact_id = $2 AND end_tick IS NULL", tick, id);

    if (result.affected_rows() != 1) {
      throw std::runtime_error(
        "fact with id " + std::to_string(id) + " not found or already retracted");
    }

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to retract fact: " + std::string(e.what()));
  }
}

void DBHandler::update_fact(long long id, const std::string & fact_json, long long tick)
{
  try {
    pqxx::work w(*connection_);

    // Append new timed_fact to the array
    pqxx::result result = w.exec_params(
      "UPDATE facts SET value = value || ARRAY[ROW($1, $2::jsonb)::timed_fact] WHERE fact_id = $3",
      tick, fact_json, id);

    if (result.affected_rows() != 1) {
      throw std::runtime_error("Fact with ID " + std::to_string(id) + " does not exist");
    }

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to update fact: " + std::string(e.what()));
  }
}

void DBHandler::add_rule(
  const std::string & name, const std::string & module_name, const std::string & lhs,
  const std::string & rhs, const int salience)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params(
      "INSERT INTO rules (name, module, lhs, rhs, salience) VALUES ($1, $2, $3, $4, $5)", name,
      module_name, lhs, rhs, salience);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to add rule: " + std::string(e.what()));
  }
}

void DBHandler::add_function(
  const std::string & name, const std::string & module_name, const std::string & definition)
{
  try {
    pqxx::work w(*connection_);

    // Store function as a rule with function prefix
    w.exec_params(
      "INSERT INTO rules (name, module, lhs, rhs) VALUES ($1, $2, $3, $4)", "FUNCTION:" + name,
      module_name, definition, "");

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to add function: " + std::string(e.what()));
  }
}

void DBHandler::add_defglobal(
  const std::string & name, const std::string & module_name, const std::string & definition)
{
  try {
    pqxx::work w(*connection_);

    // Store defglobal as a rule with defglobal prefix
    w.exec_params(
      "INSERT INTO rules (name, module, lhs, rhs) VALUES ($1, $2, $3, $4)", "DEFGLOBAL:" + name,
      module_name, definition, "");

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to add defglobal: " + std::string(e.what()));
  }
}

void DBHandler::add_deftemplate(
  const std::string & name, const std::string & module_name, const std::string & definition)
{
  try {
    pqxx::work w(*connection_);

    // Store deftemplate as a rule with deftemplate prefix
    w.exec_params(
      "INSERT INTO rules (name, module, lhs, rhs) VALUES ($1, $2, $3, $4)", "DEFTEMPLATE:" + name,
      module_name, definition, "");

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to add deftemplate: " + std::string(e.what()));
  }
}

void DBHandler::add_rule_fired(
  const std::string & name, const std::string & module, const std::vector<long long> & basis,
  long long tick)
{
  try {
    pqxx::work w(*connection_);

    // Convert vector to PostgreSQL array format
    std::ostringstream array_str;
    array_str << "{";
    for (size_t i = 0; i < basis.size(); ++i) {
      if (i > 0) array_str << ",";
      array_str << basis[i];
    }
    array_str << "}";

    // Insert rule firing record
    pqxx::result result = w.exec_params(
      "INSERT INTO rule_firing (rule_id, base, tick) "
      "SELECT rule_id, $3::bigint[], $4 "
      "FROM rules "
      "WHERE name = $1 AND module = $2",
      name, module, array_str.str(), tick);

    if (result.affected_rows() != 1) {
      throw std::runtime_error("Rule '" + name + "' in module '" + module + "' not found");
    }

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to add rule firing: " + std::string(e.what()));
  }
}

}  // namespace cx
