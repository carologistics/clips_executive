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

#include "cx_cdb_plugin/db_handler.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>
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

CREATE TYPE timed_text AS (
  tick  BIGINT,
  value TEXT
);

CREATE TABLE time_lookup (
  run_number BIGSERIAL PRIMARY KEY,
  start_time TIMESTAMPTZ NOT NULL DEFAULT CURRENT_TIMESTAMP,
  end_time   TIMESTAMPTZ,
  start_tick BIGINT      NOT NULL,
  end_tick   BIGINT
);

CREATE TABLE facts (
  fact_id     BIGINT PRIMARY KEY,
  deftemplate TEXT NOT NULL DEFAULT '',
  value       timed_fact[] NOT NULL DEFAULT '{}',
  start_tick  BIGINT NOT NULL,
  end_tick    BIGINT
);

CREATE TABLE defglobals (
  name        TEXT NOT NULL,
  module      TEXT NOT NULL,
  value       timed_fact[] NOT NULL DEFAULT '{}',
  start_tick  BIGINT NOT NULL,
  end_tick    BIGINT
);

CREATE TABLE defrules (
  rule_id     SERIAL PRIMARY KEY,
  name        TEXT NOT NULL,
  module      TEXT NOT NULL,
  value       timed_text[] NOT NULL DEFAULT '{}',
  salience    INTEGER NOT NULL,
  start_tick  BIGINT NOT NULL,
  end_tick    BIGINT
);

CREATE TABLE rule_firing (
  rule_id INT      NOT NULL REFERENCES defrules(rule_id) ON DELETE CASCADE,
  base    BIGINT[],
  tick    BIGINT   NOT NULL
);

CREATE TABLE deffunctions (
  name       TEXT NOT NULL,
  module     TEXT NOT NULL,
  value      timed_text[] NOT NULL DEFAULT '{}',
  start_tick BIGINT NOT NULL,
  end_tick   BIGINT
);

CREATE TABLE deftemplates (
  name       TEXT NOT NULL,
  module     TEXT NOT NULL,
  value      timed_text[] NOT NULL DEFAULT '{}',
  start_tick BIGINT NOT NULL,
  end_tick   BIGINT
);

CREATE TABLE deffacts (
  name       TEXT NOT NULL,
  module     TEXT NOT NULL,
  value      timed_text[] NOT NULL DEFAULT '{}',
  start_tick BIGINT NOT NULL,
  end_tick   BIGINT
);

CREATE UNIQUE INDEX defglobals_active_unique
ON defglobals (name, module)
WHERE end_tick IS NULL;

CREATE UNIQUE INDEX defrules_active_unique
ON defrules (name, module)
WHERE end_tick IS NULL;

CREATE UNIQUE INDEX deffunctions_active_unique
ON deffunctions (name, module)
WHERE end_tick IS NULL;

CREATE UNIQUE INDEX deftemplates_active_unique
ON deftemplates (name, module)
WHERE end_tick IS NULL;

CREATE UNIQUE INDEX deffacts_active_unique
ON deffacts (name, module)
WHERE end_tick IS NULL;

CREATE OR REPLACE FUNCTION assert_fact_upsert(
    p_fact_id BIGINT,
    p_deftemplate TEXT,
    p_tick BIGINT,
    p_value JSONB
) RETURNS void AS $$
BEGIN
    INSERT INTO facts (fact_id, deftemplate, value, start_tick, end_tick)
    VALUES (
        p_fact_id,
        p_deftemplate,
        ARRAY[ROW(p_tick, p_value)::timed_fact],
        p_tick,
        NULL
    )
    ON CONFLICT (fact_id) DO UPDATE
    SET value = facts.value || ARRAY[ROW(p_tick, p_value)::timed_fact],
        deftemplate = EXCLUDED.deftemplate;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION defglobal_upsert(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT,
    p_value JSONB
) RETURNS void AS $$
BEGIN
    INSERT INTO defglobals (name, module, value, start_tick, end_tick)
    VALUES (
        p_name,
        p_module,
        ARRAY[ROW(p_tick, p_value)::timed_fact],
        p_tick,
        NULL
    )
    ON CONFLICT (name, module) WHERE end_tick IS NULL DO UPDATE
    SET value = defglobals.value || ARRAY[ROW(p_tick, p_value)::timed_fact];
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION defrule_upsert(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT,
    p_definition TEXT,
    p_salience INTEGER
) RETURNS void AS $$
BEGIN
    INSERT INTO defrules (name, module, value, salience, start_tick, end_tick)
    VALUES (
        p_name,
        p_module,
        ARRAY[ROW(p_tick, p_definition)::timed_text],
        p_salience,
        p_tick,
        NULL
    )
    ON CONFLICT (name, module) WHERE end_tick IS NULL DO UPDATE
    SET value = defrules.value || ARRAY[ROW(p_tick, p_definition)::timed_text],
        salience = p_salience;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION deffunction_upsert(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT,
    p_definition TEXT
) RETURNS void AS $$
BEGIN
    INSERT INTO deffunctions (name, module, value, start_tick, end_tick)
    VALUES (
        p_name,
        p_module,
        ARRAY[ROW(p_tick, p_definition)::timed_text],
        p_tick,
        NULL
    )
    ON CONFLICT (name, module) WHERE end_tick IS NULL DO UPDATE
    SET value = deffunctions.value || ARRAY[ROW(p_tick, p_definition)::timed_text];
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION deftemplate_upsert(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT,
    p_definition TEXT
) RETURNS void AS $$
BEGIN
    INSERT INTO deftemplates (name, module, value, start_tick, end_tick)
    VALUES (
        p_name,
        p_module,
        ARRAY[ROW(p_tick, p_definition)::timed_text],
        p_tick,
        NULL
    )
    ON CONFLICT (name, module) WHERE end_tick IS NULL DO UPDATE
    SET value = deftemplates.value || ARRAY[ROW(p_tick, p_definition)::timed_text];
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION deffacts_upsert(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT,
    p_definition TEXT
) RETURNS void AS $$
BEGIN
    INSERT INTO deffacts (name, module, value, start_tick, end_tick)
    VALUES (
        p_name,
        p_module,
        ARRAY[ROW(p_tick, p_definition)::timed_text],
        p_tick,
        NULL
    )
    ON CONFLICT (name, module) WHERE end_tick IS NULL DO UPDATE
    SET value = deffacts.value || ARRAY[ROW(p_tick, p_definition)::timed_text];
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION defglobal_retract(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT
) RETURNS void AS $$
BEGIN
    UPDATE defglobals
    SET end_tick = p_tick
    WHERE name = p_name
      AND module = p_module
      AND end_tick IS NULL;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'defglobal %.% not found or already undefined', p_module, p_name;
    END IF;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION defrule_retract(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT
) RETURNS void AS $$
BEGIN
    UPDATE defrules
    SET end_tick = p_tick
    WHERE name = p_name
      AND module = p_module
      AND end_tick IS NULL;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'defrule %.% not found or already retracted', p_module, p_name;
    END IF;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION deffunction_retract(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT
) RETURNS void AS $$
BEGIN
    UPDATE deffunctions
    SET end_tick = p_tick
    WHERE name = p_name
      AND module = p_module
      AND end_tick IS NULL;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'deffunction %.% not found or already retracted', p_module, p_name;
    END IF;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION deftemplate_retract(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT
) RETURNS void AS $$
BEGIN
    UPDATE deftemplates
    SET end_tick = p_tick
    WHERE name = p_name
      AND module = p_module
      AND end_tick IS NULL;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'deftemplate %.% not found or already retracted', p_module, p_name;
    END IF;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION deffacts_retract(
    p_name TEXT,
    p_module TEXT,
    p_tick BIGINT
) RETURNS void AS $$
BEGIN
    UPDATE deffacts
    SET end_tick = p_tick
    WHERE name = p_name
      AND module = p_module
      AND end_tick IS NULL;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'deffacts %.% not found or already retracted', p_module, p_name;
    END IF;
END;
$$ LANGUAGE plpgsql;

-- TODO Disable in real application
CREATE OR REPLACE FUNCTION check_facts() RETURNS TRIGGER AS $$
DECLARE
    missing_id BIGINT;
BEGIN
    SELECT x
    INTO missing_id
    FROM unnest(NEW.base) AS x
    WHERE x IS NOT NULL
      AND NOT EXISTS (
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

CREATE OR REPLACE FUNCTION check_rule_active() RETURNS TRIGGER AS $$
BEGIN
    IF NOT EXISTS (
        SELECT 1
        FROM defrules r
        WHERE r.rule_id = NEW.rule_id
          AND r.end_tick IS NULL
    ) THEN
        RAISE EXCEPTION 'Rule with ID % does not exist or is inactive', NEW.rule_id;
    END IF;

    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER check_rule_active_trigger
BEFORE INSERT OR UPDATE ON rule_firing
FOR EACH ROW
EXECUTE FUNCTION check_rule_active();
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
  const std::string & name, const std::string & module,
  const std::vector<std::optional<long long>> & basis, long long tick)
{
  try {
    pqxx::work w(*connection_);

    std::ostringstream array_str;
    array_str << "{";
    for (size_t i = 0; i < basis.size(); ++i) {
      if (i > 0) {
        array_str << ",";
      }

      if (basis[i].has_value()) {
        array_str << *basis[i];
      } else {
        array_str << "NULL";
      }
    }
    array_str << "}";

    pqxx::result result = w.exec_params(
      "INSERT INTO rule_firing (rule_id, base, tick) "
      "SELECT rule_id, $3::bigint[], $4 "
      "FROM defrules "
      "WHERE name = $1 AND module = $2 AND end_tick IS NULL",
      name, module, array_str.str(), tick);

    if (result.affected_rows() != 1) {
      throw std::runtime_error("Rule '" + name + "' in module '" + module + "' not found");
    }

    w.commit();
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to assert rule firing: " + std::string(e.what()));
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
