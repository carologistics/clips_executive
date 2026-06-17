# Copyright (c) 2025-2026 Carologistics
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure, stateless handlers for each unified-planning PlanKind.

Each handler accepts a plan result, metadata, and a Renamings instance, and
returns the corresponding ROS message type(s).  None of these functions have
side effects or hold state – they can be tested in isolation without any ROS
or planning infrastructure.
"""

from __future__ import annotations

from cx_pddl_interfaces.msg import HierarchicalPlanMethod, PlanAction, StnConstraint
from unified_planning.plans import TimeTriggeredPlan
from unified_planning.plans.plan import PlanKind
from unified_planning.plans.stn_plan import TimepointKind

from .renamings import Renamings

# Type alias for the union of all possible return values from dispatch.
PlanMessages = (
    list[PlanAction]
    | tuple[list[PlanAction], list[HierarchicalPlanMethod], int]
    | tuple[list[PlanAction], list[StnConstraint]]
)


# ---------------------------------------------------------------------------
# Sequential plan
# ---------------------------------------------------------------------------


def handle_sequential_plan(
    result,
    problem_name: str,
    goal_name: str,
    r: Renamings,
) -> list[PlanAction]:
    """Handle PlanKind.SEQUENTIAL_PLAN.

    unified-planning exposes this as an ordered list of ActionInstance objects
    with no timing information.
    """
    plan_actions: list[PlanAction] = []
    for order, act in enumerate(result.actions):
        action = PlanAction()
        action.name = r.resolve_name(act.action.name)
        action.args = [r.resolve_arg(p) for p in act.actual_parameters]
        action.order = order
        plan_actions.append(action)
    return plan_actions


# ---------------------------------------------------------------------------
# Temporal / time-triggered plan
# ---------------------------------------------------------------------------


def handle_temporal_plan(
    result,
    problem_name: str,
    goal_name: str,
    r: Renamings,
    delta_threshold: float = 0.1,
) -> list[PlanAction]:
    """Handle PlanKind.TIME_TRIGGERED_PLAN.

    Actions that start within *delta_threshold* seconds of each other are
    placed in the same equivalence class (can be executed in parallel).
    """
    plan_actions: list[PlanAction] = []

    for time, act, duration in result.timed_actions:
        pa = PlanAction()
        pa.name = r.resolve_name(act.action.name)
        pa.args = [r.resolve_arg(p) for p in act.actual_parameters]
        pa.start_time = float(time)
        pa.duration = float(duration)
        plan_actions.append(pa)

    return plan_actions


# ---------------------------------------------------------------------------
# Partial-order plan
# ---------------------------------------------------------------------------


def handle_partial_order_plan(
    result,
    problem_name: str,
    goal_name: str,
    r: Renamings,
) -> list[PlanAction]:
    """Handle PlanKind.PARTIAL_ORDER_PLAN.

    PartialOrderPlan exposes ordering constraints via get_adjacency_list, a
    property returning {ActionInstance: [successor, ...], ...}.  The adjacency
    list encodes successors (must-come-after), not predecessors.

    Action IDs are assigned by topological sort order so that the numbering is
    deterministic and every predecessor has a lower ID than its successor.
    """
    import networkx as nx

    adj: dict = result.get_adjacency_list

    # Reconstruct a DiGraph (unified-planning's internal graph is private).
    # Edge ai → succ means ai must execute before succ.
    graph = nx.DiGraph()
    for ai, successors in adj.items():
        graph.add_node(ai)
        for succ in successors:
            graph.add_edge(ai, succ)

    topo_order = list(nx.topological_sort(graph))
    id_map: dict[int, int] = {id(ai): idx for idx, ai in enumerate(topo_order)}

    plan_actions: list[PlanAction] = []
    for action_id, ai in enumerate(topo_order):
        action = PlanAction()
        action.name = r.resolve_name(ai.action.name)
        action.args = [r.resolve_arg(p) for p in ai.actual_parameters]
        action.action_id = action_id
        action.predecessors = [
            id_map[id(pred)] for pred in graph.predecessors(ai) if id(pred) in id_map
        ]
        plan_actions.append(action)
    for a in plan_actions:
        print(f"{a.name}({a.args}) with {a.predecessors}", flush=True)

    return plan_actions


# ---------------------------------------------------------------------------
# Hierarchical plan
# ---------------------------------------------------------------------------

# Constants for the flat_plan_kind field of PlanAction.
FLAT_SEQUENTIAL = 0
FLAT_TEMPORAL = 1


def handle_hierarchical_plan(
    result,
    problem_name: str,
    goal_name: str,
    r: Renamings,
) -> tuple[list[PlanAction], list[HierarchicalPlanMethod], int]:
    """Handle PlanKind.HIERARCHICAL_PLAN.

    HierarchicalPlan has two complementary views:

    * result.action_plan  – the flat executable plan (SequentialPlan or
                            TimeTriggeredPlan)
    * result.actions()    – list of (stable_id_str, ActionInstance) for every
                            primitive, in the same order as the flat plan
    * result.methods()    – list of (stable_id_str, MethodInstance) for every
                            abstract task / method node

    Returns:
        action_msgs    – primitives as PlanAction messages
        method_msgs    – abstract tasks as HierarchicalPlanMethod messages
        flat_plan_kind – FLAT_SEQUENTIAL (0) or FLAT_TEMPORAL (1)
    """
    flat = result.action_plan
    is_temporal = isinstance(flat, TimeTriggeredPlan)
    flat_plan_kind = FLAT_TEMPORAL if is_temporal else FLAT_SEQUENTIAL

    # Map ActionInstance identity → (flat_index, start_time, duration).
    # Keyed by id() because ActionInstance.__eq__ is identity-based.
    flat_info: dict[int, tuple[int, float, float]] = {}

    if is_temporal:
        for flat_idx, (time, act_inst, duration) in enumerate(flat.timed_actions):
            flat_info[id(act_inst)] = (flat_idx, float(time), float(duration))
    else:
        for flat_idx, act_inst in enumerate(flat.actions):
            flat_info[id(act_inst)] = (flat_idx, 0.0, 0.0)

    # Primitive actions
    action_msgs: list[PlanAction] = []
    for task_id, act_inst in result.actions():
        msg = PlanAction()
        msg.name = r.resolve_name(act_inst.action.name)
        msg.args = [r.resolve_arg(p) for p in act_inst.actual_parameters]
        msg.task_id = _parent_task_id(task_id)

        info = flat_info.get(id(act_inst))
        if info is None:
            # Defensive fallback: should not happen in a well-formed plan.
            msg.order = -1
        else:
            flat_idx, time, duration = info
            msg.order = flat_idx
            if is_temporal:
                msg.start_time = time
                msg.duration = duration

        action_msgs.append(msg)

    # Abstract task / method nodes
    method_msgs: list[HierarchicalPlanMethod] = []
    for task_id, method_inst in result.methods():
        msg = HierarchicalPlanMethod()
        msg.name = r.resolve_name(method_inst.method.name)
        msg.args = [r.resolve_arg(p) for p in method_inst.parameters]
        msg.task_id = task_id
        method_msgs.append(msg)

    return action_msgs, method_msgs, flat_plan_kind


def stn_to_partial_order(stn_plan):
    import math

    import networkx as nx
    from unified_planning.plans import PartialOrderPlan
    from unified_planning.plans.stn_plan import TimepointKind

    constraints = stn_plan.get_constraints()
    INF = math.inf

    # --- Step 1: Collect all timepoints as nodes ---
    all_nodes = list(constraints.keys())

    # --- Step 2+3: Build STN graph and run Floyd-Warshall via NetworkX ---
    stn_graph = nx.DiGraph()
    stn_graph.add_nodes_from(all_nodes)

    for left_node, constraint_list in constraints.items():
        for lower_bound, upper_bound, right_node in constraint_list:
            if right_node not in stn_graph:
                continue
            if upper_bound is not None and upper_bound != INF:
                stn_graph.add_edge(left_node, right_node, weight=float(upper_bound))
            if lower_bound is not None and lower_bound != -INF:
                stn_graph.add_edge(right_node, left_node, weight=float(-lower_bound))

    fw = nx.floyd_warshall(stn_graph, weight='weight')

    # --- Step 4: Collect action start/end timepoints ---
    action_instances = {}
    for node in all_nodes:
        if node.action_instance is not None:
            ai = node.action_instance
            if ai not in action_instances:
                action_instances[ai] = {'start': None, 'end': None}
            if node.kind == TimepointKind.START:
                action_instances[ai]['start'] = node
            elif node.kind == TimepointKind.END:
                action_instances[ai]['end'] = node

    # --- Step 5: Add ordering edge A->B if:
    #   (a) start_A must precede start_B: -fw[start_B][end_A] >= 0, OR
    #   (b) end_A must precede start_B: -fw[start_B][end_A] >= 0, OR
    #   (c) end_A must precede end_B but not vice versa:
    #       -fw[end_B][end_A] >= 0 and -fw[end_A][end_B] < 0
    action_list = list(action_instances.keys())
    graph = nx.DiGraph()
    graph.add_nodes_from(action_list)

    for i, ai in enumerate(action_list):
        s_ai = action_instances[ai]['start']
        e_ai = action_instances[ai]['end']
        if e_ai is None:
            continue

        for j, aj in enumerate(action_list):
            if i == j:
                continue
            e_aj = action_instances[aj]['end']
            s_aj = action_instances[aj]['start']
            if e_aj is None or s_aj is None:
                continue

            # Condition (a): end_A must precede start_B
            if -fw[s_aj][s_ai] >= 0:
                graph.add_edge(ai, aj)
                continue

            # Condition (b): end_A must precede start_B
            if -fw[s_aj][e_ai] >= 0:
                graph.add_edge(ai, aj)
                continue

            # Condition (c): end_A must precede end_B, but not vice versa
            if -fw[e_aj][e_ai] >= 0 and -fw[e_ai][e_aj] < 0:
                graph.add_edge(ai, aj)

    reduced = nx.transitive_reduction(graph)
    adj = {ai: list(reduced.successors(ai)) for ai in reduced.nodes}
    return PartialOrderPlan(adj)


# ---------------------------------------------------------------------------
# STN plan
# ---------------------------------------------------------------------------


def handle_stn_plan(
    result,
    problem_name: str,
    goal_name: str,
    r: Renamings,
) -> tuple[list[PlanAction], list[StnConstraint]]:
    """Handle PlanKind.STN_PLAN.

    unified-planning's STNPlan exposes constraints via get_constraints(), which
    returns::

        {STNPlanNode_A: [(lower_bound, upper_bound, STNPlanNode_B), ...]}

    with the semantics  ``lower <= Time(B) - Time(A) <= upper``.

    Nodes whose kind is GLOBAL_START or GLOBAL_END refer to the plan's own
    start/end timepoint rather than any action, and are represented with
    action_id -1 and -2 in StnConstraint.

    Actions plan_start and plan_end are explicitly added to the list.

    Trivial constraints (plan start before any other action, plan end after any action) are omitted in the constraint list).


    Returns:
        action_msgs     – one PlanAction per unique ActionInstance
        constraint_msgs – one StnConstraint per directed constraint edge
    """
    constraints_map = result.get_constraints()

    # ── Assign a stable integer ID to every unique ActionInstance ──────────
    # ActionInstance.__eq__ is identity-based, so we key by id().
    ai_to_id: dict[int, int] = {}
    ordered_ais: list = []  # preserves insertion order for message building

    def _register(node: STNPlanNode) -> int:
        """Return the integer action ID for *node*, registering it if new.
        Returns -1 for GLOBAL_START / GLOBAL_END nodes."""
        if node.kind == TimepointKind.GLOBAL_START:
            return -1
        if node.kind == TimepointKind.GLOBAL_END:
            return -2
        ai = node.action_instance
        key = id(ai)
        if key not in ai_to_id:
            ai_to_id[key] = len(ordered_ais)
            ordered_ais.append(ai)
        return ai_to_id[key]

    # Walk every node that appears on either side of a constraint to register
    # all action instances before we build the message lists.
    for node_a, entries in constraints_map.items():
        _register(node_a)
        for _lb, _ub, node_b in entries:
            _register(node_b)

    # ── Build PlanAction messages ───────────────────────────────────────
    action_msgs: list[PlanAction] = []
    for action_id, ai in enumerate(ordered_ais):
        msg = PlanAction()
        msg.action_id = action_id
        msg.name = r.resolve_name(ai.action.name)
        msg.args = [r.resolve_arg(p) for p in ai.actual_parameters]
        action_msgs.append(msg)
    start_msg = PlanAction()
    start_msg.name = 'plan_start'
    start_msg.action_id = -1
    action_msgs.append(start_msg)
    end_msg = PlanAction()
    end_msg.name = 'plan_end'
    end_msg.action_id = -2
    action_msgs.append(end_msg)

    # ── Build StnConstraint messages ───────────────────────────────────────
    # UP semantics: lower <= Time(A) - Time(B) <= upper
    # We expose this as: from=B, to=A with the same bounds, giving the
    # standard STN reading: from + lower <= to <= from + upper.
    #
    # GLOBAL_START / GLOBAL_END nodes represent the plan's own start/end
    # timepoint.  UP automatically inserts trivial anchor constraints of the
    # form  0 <= Time(action) - Time(GLOBAL_START) <= None  for every action
    # node to assert it happens after the plan starts.  These carry no
    # planning information and are dropped.
    #
    # All other constraints involving GLOBAL nodes are preserved because they
    # encode meaningful plan-level bounds, e.g.:
    #   end(plan) - start(action_3) <= 100   →  action_3 must start within
    #                                            100 s of the plan ending
    #   start(action_1) - start(plan) >= 5   →  action_1 may not start until
    #                                            at least 5 s into the plan

    def _is_trivial_anchor(node_a: STNPlanNode, lower, upper, node_b: STNPlanNode) -> bool:
        """Return True for the structural  0 <= Time(action) - Time(GLOBAL_START) <= inf
        constraints that UP inserts purely to anchor the STN."""
        return (
            node_b.kind == TimepointKind.GLOBAL_START
            and node_a.kind in (TimepointKind.START, TimepointKind.END)
            and lower is not None
            and float(lower) == 0.0
            and upper is None
        ) or (
            node_b.kind == TimepointKind.GLOBAL_END
            and node_a.kind in (TimepointKind.START, TimepointKind.END)
            and lower is not None
            and float(lower) == 0.0
            and upper is None
        )

    def _node_role(node: STNPlanNode) -> str:
        if node.kind in (TimepointKind.START, TimepointKind.END):
            return 'START' if node.kind == TimepointKind.START else 'END'
        # GLOBAL_START / GLOBAL_END
        return 'START' if node.kind == TimepointKind.GLOBAL_START else 'END'

    constraint_msgs: list[StnConstraint] = []
    for node_a, entries in constraints_map.items():
        for lower, upper, node_b in entries:
            if _is_trivial_anchor(node_a, lower, upper, node_b):
                continue

            msg = StnConstraint()
            # from = node_b, to = node_a  (see sign-convention note above)
            msg.from_action_id = _register(node_a)
            msg.from_role = _node_role(node_a)
            msg.to_action_id = _register(node_b)
            msg.to_role = _node_role(node_b)
            msg.is_lower_bounded = lower is not None
            msg.is_upper_bounded = upper is not None
            msg.lower_bound = float(lower) if lower is not None else 0.0
            msg.upper_bound = float(upper) if upper is not None else 0.0
            constraint_msgs.append(msg)

    return action_msgs, constraint_msgs


# ---------------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------------


def dispatch_plan_result(
    result,
    problem_name: str,
    goal_name: str,
    plan_kind: PlanKind,
    goal_problem,
    r: Renamings,
) -> PlanMessages:
    """Convert a raw plan result into the appropriate ROS message type(s).

    If the planner returned a different PlanKind than requested, an automatic
    conversion is attempted via unified-planning's convert_to().
    """
    import unified_planning as up

    if result is None:
        return []

    # Support relaxation of temporal plan to partial order plan
    if result.kind == PlanKind.TIME_TRIGGERED_PLAN and plan_kind == PlanKind.PARTIAL_ORDER_PLAN:
        stn_plan = result.convert_to(up.plans.plan.PlanKind.STN_PLAN, goal_problem)
        po_plan = stn_to_partial_order(stn_plan)
        return handle_partial_order_plan(po_plan, problem_name, goal_name, r)

    # Support relaxation of temporal plan to partial order plan
    if result.kind == PlanKind.STN_PLAN and plan_kind == PlanKind.PARTIAL_ORDER_PLAN:
        po_plan = stn_to_partial_order(result)
        return handle_partial_order_plan(po_plan, problem_name, goal_name, r)

    # Support relaxation of Hierarchical Plan to partial order plan
    if result.kind == PlanKind.HIERARCHICAL_PLAN and plan_kind == PlanKind.PARTIAL_ORDER_PLAN:
        time_triggered_plan = result.convert_to(
            up.plans.plan.PlanKind.TIME_TRIGGERED_PLAN, goal_problem
        )
        stn_plan = time_triggered_plan.convert_to(up.plans.plan.PlanKind.STN_PLAN, goal_problem)
        po_plan = stn_to_partial_order(stn_plan)
        return handle_partial_order_plan(po_plan, problem_name, goal_name, r)

    if result.kind != plan_kind:
        result = result.convert_to(plan_kind, goal_problem)

    match result.kind:
        case PlanKind.TIME_TRIGGERED_PLAN:
            return handle_temporal_plan(result, problem_name, goal_name, r)
        case PlanKind.SEQUENTIAL_PLAN:
            return handle_sequential_plan(result, problem_name, goal_name, r)
        case PlanKind.PARTIAL_ORDER_PLAN:
            return handle_partial_order_plan(result, problem_name, goal_name, r)
        case PlanKind.HIERARCHICAL_PLAN:
            return handle_hierarchical_plan(result, problem_name, goal_name, r)
        case PlanKind.STN_PLAN:
            return handle_stn_plan(result, problem_name, goal_name, r)
        case _:
            raise NotImplementedError(f"Unsupported plan kind: {result.kind}")


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _parent_task_id(task_id: str) -> str:
    """Strip the last ::segment to get the parent method's task-id."""
    idx = task_id.rfind('::')
    return task_id[:idx] if idx != -1 else ''
