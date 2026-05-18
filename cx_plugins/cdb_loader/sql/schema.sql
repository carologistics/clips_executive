CREATE OR REPLACE VIEW facts_cpp AS
SELECT
    f.fact_id,
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
