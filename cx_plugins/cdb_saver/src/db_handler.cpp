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

#include "cx_cdb_saver_plugin/db_handler.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "cx_cdb_saver_plugin/schema_sql.hpp"

namespace cx
{

DBHandler::DBHandler(DBHandlerConfig & config, rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: config_(config), parent_(parent)
{
  tick_ = 0;
  current_run_ = -1;

  if (!init_db(config)) {
    throw(std::runtime_error("Failed to initialize database"));
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
    w.exec(kSchemaSql);
    w.commit();
    db_connection.close();
    return true;
  } else {
    return false;
  }
}

void DBHandler::assert_fact(
  long long id, const std::string & deftemplate, const std::string & fact_json, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params(
      "SELECT assert_fact_upsert($1, $2, $3, $4::jsonb);", id, deftemplate, tick, fact_json);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert fact: " + std::string(e.what()));
  }
}

void DBHandler::retract_fact(long long id, long long tick)
{
  try {
    pqxx::work w(*connection_);

    pqxx::result result = w.exec_params(
      "UPDATE facts "
      "SET end_tick = $1 "
      "WHERE fact_id = $2 AND end_tick IS NULL",
      tick, id);

    if (result.affected_rows() != 1) {
      throw std::runtime_error(
        "fact with id " + std::to_string(id) + " not found or already retracted");
    }

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to retract fact: " + std::string(e.what()));
  }
}

void DBHandler::assert_defrule(
  const std::string & name, const std::string & module_name, const std::string & definition,
  const int salience, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params(
      "SELECT defrule_upsert($1, $2, $3, $4, $5);", name, module_name, tick, definition, salience);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert rule: " + std::string(e.what()));
  }
}

void DBHandler::retract_defrule(
  const std::string & name, const std::string & module_name, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT defrule_retract($1, $2, $3);", name, module_name, tick);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to retract rule: " + std::string(e.what()));
  }
}

void DBHandler::assert_deffunction(
  const std::string & name, const std::string & module_name, const std::string & definition,
  long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params(
      "SELECT deffunction_upsert($1, $2, $3, $4);", name, module_name, tick, definition);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert deffunction: " + std::string(e.what()));
  }
}

void DBHandler::retract_deffunction(
  const std::string & name, const std::string & module_name, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT deffunction_retract($1, $2, $3);", name, module_name, tick);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to retract deffunction: " + std::string(e.what()));
  }
}

void DBHandler::assert_deffacts(
  const std::string & name, const std::string & module_name, const std::string & definition,
  long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT deffacts_upsert($1, $2, $3, $4);", name, module_name, tick, definition);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert deffacts: " + std::string(e.what()));
  }
}

void DBHandler::retract_deffacts(
  const std::string & name, const std::string & module_name, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT deffacts_retract($1, $2, $3);", name, module_name, tick);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to retract deffacts: " + std::string(e.what()));
  }
}

void DBHandler::assert_defglobal(
  const std::string & name, const std::string & module_name, const std::string & value_json,
  long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params(
      "SELECT defglobal_upsert($1, $2, $3, $4::jsonb);", name, module_name, tick, value_json);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert defglobal: " + std::string(e.what()));
  }
}

void DBHandler::retract_defglobal(
  const std::string & name, const std::string & module_name, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT defglobal_retract($1, $2, $3);", name, module_name, tick);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to retract defglobal: " + std::string(e.what()));
  }
}

void DBHandler::assert_deftemplate(
  const std::string & name, const std::string & module_name, const std::string & definition,
  long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params(
      "SELECT deftemplate_upsert($1, $2, $3, $4);", name, module_name, tick, definition);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert deftemplate: " + std::string(e.what()));
  }
}

void DBHandler::retract_deftemplate(
  const std::string & name, const std::string & module_name, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT deftemplate_retract($1, $2, $3);", name, module_name, tick);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to retract deftemplate: " + std::string(e.what()));
  }
}

void DBHandler::assert_rule_fired(
  const std::string & name, const std::string & module_name, const std::vector<long long> & basis,
  long long tick)
{
  try {
    pqxx::work w(*connection_);

    pqxx::result result = w.exec_params(
      R"SQL(
            INSERT INTO rule_firing (rule_id, base, tick)
            SELECT rule_id, $3::bigint[], $4
            FROM defrules
            WHERE name = $1
              AND module = $2
              AND end_tick IS NULL
          )SQL",
      name, module_name, basis, tick);

    if (result.affected_rows() != 1) {
      throw std::runtime_error("Rule '" + name + "' in module '" + module_name + "' not found");
    }

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert rule firing: " + std::string(e.what()));
  }
}

void DBHandler::load_plugin(
  const std::string & plugin_name, const std::string & config_json, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT plugin_load($1, $2, $3::jsonb);", plugin_name, tick, config_json);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert plugin load: " + std::string(e.what()));
  }
}

void DBHandler::unload_plugin(const std::string & plugin_name, long long tick)
{
  try {
    pqxx::work w(*connection_);

    w.exec_params("SELECT plugin_unload($1, $2);", plugin_name, tick);

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert plugin unload: " + std::string(e.what()));
  }
}

long long DBHandler::start_run(int64_t start_time_ns, long long start_tick)
{
  try {
    pqxx::work w(*connection_);

    pqxx::row row = w.exec_params1(
      R"sql(
        INSERT INTO time_lookup (start_time, start_tick)
        VALUES (
          TIMESTAMPTZ 'epoch' + ($1::bigint / 1000000) * INTERVAL '1 millisecond',
          $2
        )
        RETURNING run_number;
      )sql",
      start_time_ns, start_tick);

    long long run_number = row[0].as<long long>();

    w.commit();
    return run_number;

  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to start run: " + std::string(e.what()));
  }
}

void DBHandler::end_run(long long run_number, int64_t end_time_ns, long long end_tick)
{
  try {
    pqxx::work w(*connection_);

    pqxx::result result = w.exec_params(
      R"sql(
        UPDATE time_lookup
        SET end_time = TIMESTAMPTZ 'epoch' + ($2::bigint / 1000000) * INTERVAL '1 millisecond',
            end_tick = $3
        WHERE run_number = $1
          AND end_tick IS NULL;
      )sql",
      run_number, end_time_ns, end_tick);

    if (result.affected_rows() != 1) {
      throw std::runtime_error(
        "run with number " + std::to_string(run_number) + " not found or already finished");
    }

    w.commit();

  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to finish run: " + std::string(e.what()));
  }
}

}  // namespace cx
