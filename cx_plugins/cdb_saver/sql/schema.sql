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
  start_tick  BIGINT NOT NULL,
  end_tick    BIGINT
);

CREATE TABLE fact_values (
  fact_id BIGINT NOT NULL REFERENCES facts(fact_id) ON DELETE CASCADE,
  tick    BIGINT NOT NULL,
  value   JSONB NOT NULL,

  PRIMARY KEY (fact_id, tick)
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

CREATE TABLE plugins (
  name        TEXT NOT NULL PRIMARY KEY,
  start_tick  BIGINT NOT NULL,
  end_tick    BIGINT,
  config      JSONB NOT NULL DEFAULT '{}'::jsonb,

  CHECK (end_tick IS NULL OR end_tick >= start_tick)
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

CREATE UNIQUE INDEX plugins_active_unique
ON plugins (name)
WHERE end_tick IS NULL;

CREATE OR REPLACE FUNCTION assert_fact_upsert(
    p_fact_id BIGINT,
    p_deftemplate TEXT,
    p_tick BIGINT,
    p_value JSONB
) RETURNS void AS $$
BEGIN
    INSERT INTO facts (fact_id, deftemplate, start_tick, end_tick)
    VALUES (p_fact_id, p_deftemplate, p_tick, NULL)
    ON CONFLICT (fact_id) DO UPDATE
    SET deftemplate = EXCLUDED.deftemplate;

    INSERT INTO fact_values (fact_id, tick, value)
    VALUES (p_fact_id, p_tick, p_value)
    ON CONFLICT (fact_id, tick) DO UPDATE
    SET value = EXCLUDED.value;
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

CREATE OR REPLACE FUNCTION plugin_load(
    p_name TEXT,
    p_tick BIGINT,
    p_config JSONB DEFAULT '[]'::jsonb
) RETURNS void AS $$
BEGIN
    INSERT INTO plugins (name, start_tick, end_tick, config)
    VALUES (p_name, p_tick, NULL, p_config);

EXCEPTION WHEN unique_violation THEN
    RAISE EXCEPTION 'plugin % already loaded', p_name;
END;
$$ LANGUAGE plpgsql;

CREATE OR REPLACE FUNCTION plugin_unload(
    p_name TEXT,
    p_tick BIGINT
) RETURNS void AS $$
BEGIN
    UPDATE plugins
    SET end_tick = p_tick
    WHERE name = p_name
      AND end_tick IS NULL;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'plugin % not found or already unloaded', p_name;
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
