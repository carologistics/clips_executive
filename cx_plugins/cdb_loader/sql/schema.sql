CREATE OR REPLACE VIEW facts_cpp AS
SELECT
    f.fact_id,
    f.deftemplate,
    COALESCE(
        (
            SELECT jsonb_agg(
                jsonb_build_object(
                    'tick', h.tick,
                    'value', h.value
                )
                ORDER BY h.tick
            )
            FROM unnest(f.value) AS h(tick, value)
        ),
        '[]'::jsonb
    ) AS value_history,
    f.start_tick,
    f.end_tick
FROM facts f;


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

CREATE OR REPLACE VIEW rule_firing_cpp AS
SELECT
    rf.rule_id,
    COALESCE(
        to_jsonb(rf.base),
        '[]'::jsonb
    ) AS base,
    rf.tick
FROM rule_firing rf;

CREATE OR REPLACE VIEW rule_firing_with_rule_cpp AS
SELECT
    rf.rule_id,
    r.module AS rule_module,
    r.name AS rule_name,
    r.salience AS rule_salience,
    COALESCE(
        to_jsonb(rf.base),
        '[]'::jsonb
    ) AS base,
    rf.tick
FROM rule_firing rf
JOIN defrules r
    ON r.rule_id = rf.rule_id;


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
