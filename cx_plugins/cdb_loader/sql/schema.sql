CREATE OR REPLACE VIEW facts_cpp AS
SELECT
    f.fact_id,
    f.module,
    f.deftemplate,
    COALESCE(
        jsonb_agg(
            jsonb_build_object(
                'tick', fv.tick,
                'value', fv.value
            )
            ORDER BY fv.tick
        ) FILTER (WHERE fv.fact_id IS NOT NULL),
        '[]'::jsonb
    ) AS value_history,
    f.start_tick,
    f.end_tick
FROM facts f
LEFT JOIN fact_values fv
  ON fv.fact_id = f.fact_id
GROUP BY
    f.fact_id,
    f.deftemplate,
    f.start_tick,
    f.end_tick;


CREATE OR REPLACE VIEW defglobals_cpp AS
SELECT
    d.name,
    d.module,
    COALESCE(
        (
            SELECT jsonb_agg(
                jsonb_build_object(
                    'tick', h.tick,
                    'value', h.value
                )
                ORDER BY h.tick
            )
            FROM unnest(d.value) AS h(tick, value)
        ),
        '[]'::jsonb
    ) AS value_history,
    d.start_tick,
    d.end_tick
FROM defglobals d;

CREATE OR REPLACE VIEW defrules_cpp AS
SELECT
    r.rule_id,
    r.name,
    r.module,
    COALESCE(
        (
            SELECT jsonb_agg(
                jsonb_build_object(
                    'tick', h.tick,
                    'value', h.value
                )
                ORDER BY h.tick
            )
            FROM unnest(r.value) AS h(tick, value)
        ),
        '[]'::jsonb
    ) AS value_history,
    r.salience,
    r.start_tick,
    r.end_tick
FROM defrules r;

CREATE OR REPLACE VIEW deffunctions_cpp AS
SELECT
    d.name,
    d.module,
    COALESCE(
        (
            SELECT jsonb_agg(
                jsonb_build_object(
                    'tick', h.tick,
                    'value', h.value
                )
                ORDER BY h.tick
            )
            FROM unnest(d.value) AS h(tick, value)
        ),
        '[]'::jsonb
    ) AS value_history,
    d.start_tick,
    d.end_tick
FROM deffunctions d;

CREATE OR REPLACE VIEW deftemplates_cpp AS
SELECT
    d.name,
    d.module,
    COALESCE(
        (
            SELECT jsonb_agg(
                jsonb_build_object(
                    'tick', h.tick,
                    'value', h.value
                )
                ORDER BY h.tick
            )
            FROM unnest(d.value) AS h(tick, value)
        ),
        '[]'::jsonb
    ) AS value_history,
    d.start_tick,
    d.end_tick
FROM deftemplates d;

CREATE OR REPLACE VIEW deffacts_cpp AS
SELECT
    d.name,
    d.module,
    COALESCE(
        (
            SELECT jsonb_agg(
                jsonb_build_object(
                    'tick', h.tick,
                    'value', h.value
                )
                ORDER BY h.tick
            )
            FROM unnest(d.value) AS h(tick, value)
        ),
        '[]'::jsonb
    ) AS value_history,
    d.start_tick,
    d.end_tick
FROM deffacts d;

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
