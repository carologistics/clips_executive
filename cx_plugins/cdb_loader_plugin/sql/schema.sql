CREATE OR REPLACE FUNCTION facts_cpp_at(
    p_restore_tick BIGINT,
    p_skip_external_addresses BOOLEAN DEFAULT FALSE
)
RETURNS TABLE (
    fact_id BIGINT,
    module TEXT,
    deftemplate TEXT,
    value_tick BIGINT,
    value JSONB,
    start_tick BIGINT,
    end_tick BIGINT
) AS $$
SELECT
    f.fact_id,
    f.module,
    f.deftemplate,
    latest.tick AS value_tick,
    latest.value AS value,
    f.start_tick,
    f.end_tick
FROM facts f
JOIN LATERAL (
    SELECT fv.tick, fv.value
    FROM fact_values fv
    WHERE fv.fact_id = f.fact_id
      AND fv.tick <= p_restore_tick
    ORDER BY fv.tick DESC
    LIMIT 1
) latest ON true
WHERE f.start_tick <= p_restore_tick
  AND (f.end_tick IS NULL OR f.end_tick > p_restore_tick)
  AND (
      NOT p_skip_external_addresses
      OR NOT jsonb_path_exists(
          latest.value,
          'lax $.** ? (@.type == "EXTERNAL_ADDRESS")'
      )
  );
$$ LANGUAGE sql STABLE;


CREATE OR REPLACE FUNCTION defglobals_cpp_at(
    p_restore_tick BIGINT,
    p_skip_external_addresses BOOLEAN DEFAULT FALSE
)
RETURNS TABLE (
    name TEXT,
    module TEXT,
    value_tick BIGINT,
    value JSONB,
    start_tick BIGINT,
    end_tick BIGINT
) AS $$
SELECT
    d.name,
    d.module,
    latest.tick AS value_tick,
    latest.value AS value,
    d.start_tick,
    d.end_tick
FROM defglobals d
JOIN LATERAL (
    SELECT h.tick, h.value
    FROM unnest(d.value) AS h(tick, value)
    WHERE h.tick <= p_restore_tick
    ORDER BY h.tick DESC
    LIMIT 1
) latest ON true
WHERE d.start_tick <= p_restore_tick
  AND (d.end_tick IS NULL OR d.end_tick > p_restore_tick)
  AND (
      NOT p_skip_external_addresses
      OR NOT jsonb_path_exists(
          latest.value,
          'lax $.** ? (@.type == "EXTERNAL_ADDRESS")'
      )
  );
$$ LANGUAGE sql STABLE;

CREATE OR REPLACE FUNCTION defrules_cpp_at(p_restore_tick BIGINT)
RETURNS TABLE (
    rule_id INTEGER,
    name TEXT,
    module TEXT,
    value_tick BIGINT,
    value TEXT,
    salience INTEGER,
    start_tick BIGINT,
    end_tick BIGINT
) AS $$
SELECT
    r.rule_id,
    r.name,
    r.module,
    latest.tick AS value_tick,
    latest.value AS value,
    r.salience,
    r.start_tick,
    r.end_tick
FROM defrules r
JOIN LATERAL (
    SELECT h.tick, h.value
    FROM unnest(r.value) AS h(tick, value)
    WHERE h.tick <= p_restore_tick
    ORDER BY h.tick DESC
    LIMIT 1
) latest ON true
WHERE r.start_tick <= p_restore_tick
  AND (r.end_tick IS NULL OR r.end_tick > p_restore_tick);
$$ LANGUAGE sql STABLE;


CREATE OR REPLACE FUNCTION deffunctions_cpp_at(p_restore_tick BIGINT)
RETURNS TABLE (
    name TEXT,
    module TEXT,
    value_tick BIGINT,
    value TEXT,
    start_tick BIGINT,
    end_tick BIGINT
) AS $$
SELECT
    d.name,
    d.module,
    latest.tick AS value_tick,
    latest.value AS value,
    d.start_tick,
    d.end_tick
FROM deffunctions d
JOIN LATERAL (
    SELECT h.tick, h.value
    FROM unnest(d.value) AS h(tick, value)
    WHERE h.tick <= p_restore_tick
    ORDER BY h.tick DESC
    LIMIT 1
) latest ON true
WHERE d.start_tick <= p_restore_tick
  AND (d.end_tick IS NULL OR d.end_tick > p_restore_tick);
$$ LANGUAGE sql STABLE;


CREATE OR REPLACE FUNCTION deftemplates_cpp_at(p_restore_tick BIGINT)
RETURNS TABLE (
    name TEXT,
    module TEXT,
    value_tick BIGINT,
    value TEXT,
    start_tick BIGINT,
    end_tick BIGINT
) AS $$
SELECT
    d.name,
    d.module,
    latest.tick AS value_tick,
    latest.value AS value,
    d.start_tick,
    d.end_tick
FROM deftemplates d
JOIN LATERAL (
    SELECT h.tick, h.value
    FROM unnest(d.value) AS h(tick, value)
    WHERE h.tick <= p_restore_tick
    ORDER BY h.tick DESC
    LIMIT 1
) latest ON true
WHERE d.start_tick <= p_restore_tick
  AND (d.end_tick IS NULL OR d.end_tick > p_restore_tick);
$$ LANGUAGE sql STABLE;


CREATE OR REPLACE FUNCTION deffacts_cpp_at(p_restore_tick BIGINT)
RETURNS TABLE (
    name TEXT,
    module TEXT,
    value_tick BIGINT,
    value TEXT,
    start_tick BIGINT,
    end_tick BIGINT
) AS $$
SELECT
    d.name,
    d.module,
    latest.tick AS value_tick,
    latest.value AS value,
    d.start_tick,
    d.end_tick
FROM deffacts d
JOIN LATERAL (
    SELECT h.tick, h.value
    FROM unnest(d.value) AS h(tick, value)
    WHERE h.tick <= p_restore_tick
    ORDER BY h.tick DESC
    LIMIT 1
) latest ON true
WHERE d.start_tick <= p_restore_tick
  AND (d.end_tick IS NULL OR d.end_tick > p_restore_tick);
$$ LANGUAGE sql STABLE;

CREATE OR REPLACE FUNCTION cdb_max_tick()
RETURNS BIGINT AS $$
DECLARE
    result BIGINT;
BEGIN
    SELECT MAX(tick)
    INTO result
    FROM (
        SELECT start_tick AS tick FROM time_lookup
        UNION ALL
        SELECT end_tick AS tick FROM time_lookup WHERE end_tick IS NOT NULL

        UNION ALL
        SELECT start_tick AS tick FROM facts
        UNION ALL
        SELECT end_tick AS tick FROM facts WHERE end_tick IS NOT NULL
        UNION ALL
        SELECT tick AS tick FROM fact_values

        UNION ALL
        SELECT start_tick AS tick FROM defglobals
        UNION ALL
        SELECT end_tick AS tick FROM defglobals WHERE end_tick IS NOT NULL
        UNION ALL
        SELECT h.tick AS tick
        FROM defglobals d
        CROSS JOIN LATERAL unnest(d.value) AS h(tick, value)

        UNION ALL
        SELECT start_tick AS tick FROM defrules
        UNION ALL
        SELECT end_tick AS tick FROM defrules WHERE end_tick IS NOT NULL
        UNION ALL
        SELECT h.tick AS tick
        FROM defrules r
        CROSS JOIN LATERAL unnest(r.value) AS h(tick, value)

        UNION ALL
        SELECT tick AS tick FROM rule_firing

        UNION ALL
        SELECT start_tick AS tick FROM deffunctions
        UNION ALL
        SELECT end_tick AS tick FROM deffunctions WHERE end_tick IS NOT NULL
        UNION ALL
        SELECT h.tick AS tick
        FROM deffunctions d
        CROSS JOIN LATERAL unnest(d.value) AS h(tick, value)

        UNION ALL
        SELECT start_tick AS tick FROM deftemplates
        UNION ALL
        SELECT end_tick AS tick FROM deftemplates WHERE end_tick IS NOT NULL
        UNION ALL
        SELECT h.tick AS tick
        FROM deftemplates d
        CROSS JOIN LATERAL unnest(d.value) AS h(tick, value)

        UNION ALL
        SELECT start_tick AS tick FROM deffacts
        UNION ALL
        SELECT end_tick AS tick FROM deffacts WHERE end_tick IS NOT NULL
        UNION ALL
        SELECT h.tick AS tick
        FROM deffacts d
        CROSS JOIN LATERAL unnest(d.value) AS h(tick, value)

        UNION ALL
        SELECT start_tick AS tick FROM plugins
        UNION ALL
        SELECT end_tick AS tick FROM plugins WHERE end_tick IS NOT NULL
    ) all_ticks;

    IF result IS NULL THEN
        RAISE EXCEPTION 'Could not resolve max tick: no ticks found in database';
    END IF;

    RETURN result;
END;
$$ LANGUAGE plpgsql;
