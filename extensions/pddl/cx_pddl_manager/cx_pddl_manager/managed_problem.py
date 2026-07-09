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

from __future__ import annotations

import concurrent.futures as cf
from dataclasses import dataclass, field
import importlib
import logging
from pathlib import Path
from typing import Any

from unified_planning.engines.results import POSITIVE_OUTCOMES
from unified_planning.environment import Environment, get_environment
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.model import Problem
from unified_planning.model.htn import HierarchicalProblem
from unified_planning.plans.plan import PlanKind

from .plan_handlers import PlanMessages, dispatch_plan_result
from .renamings import Renamings

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# GoalSpec – pure data, no behaviour, no back-references
# ---------------------------------------------------------------------------


@dataclass
class GoalSpec:
    """
    Scoping information that restricts a base problem to a specific goal.

    Filters are applied as allowlists: when a filter list is empty it means
    "allow everything" (i.e. no restriction).
    """

    goals: list['up.model.fnode.FNode'] = field(default_factory=list)
    timed_goals: Dict['up.model.timing.TimeInterval', List['up.model.fnode.FNode']] = field(
        default_factory=dict
    )
    trajectory_constraints: list['up.model.fnode.FNode'] = field(default_factory=list)

    object_filters: list[str] = field(default_factory=list)
    fluent_filters: list[str] = field(default_factory=list)
    action_filters: list[str] = field(default_factory=list)

    quality_metrics: list['up.model.metrics.PlanQualityMetric'] = field(default_factory=list)

    # Convenience: treat an empty filter list as "no restriction"
    def _effective(self, lst: list) -> list | None:
        return lst if lst else None


# ---------------------------------------------------------------------------
# ManagedGoal – builder API that produces / mutates a GoalSpec
# ---------------------------------------------------------------------------


class ManagedGoal:
    """
    Thin builder that accumulates goal fluents and optional filters.

    Once the goal is fully configured, call ManagedProblem.plan() to trigger
    planning.

    Goal fluents are stored as pre-grounded FNode expressions.  Grounding
    requires access to the base Problem, so the expression is built by
    ManagedProblem and passed in here as an already-resolved FNode.
    """

    def __init__(self, name: str = 'base') -> None:
        self.name = name
        self.spec = GoalSpec()

    # ------------------------------------------------------------------
    # Goal fluents
    # ------------------------------------------------------------------

    def add_goal_expr(self, expr: Any) -> None:
        self.spec.goals.append(expr)

    def add_timed_goal_expr(self, expr: Any) -> None:
        self.spec.timed_goals.append(expr)

    def add_trajectory_constraint_expr(self, expr: Any) -> None:
        self.spec.trajectory_consraints.append(expr)

    def add_quality_metric(self, expr: Any) -> None:
        self.spec.quality_metrics.append(expr)

    def clear_goals(self) -> None:
        self.spec.goals.clear()

    def clear_timed_goals(self) -> None:
        self.spec.timed_goals.clear()

    def clear_trajectory_constraint_expr(self) -> None:
        self.spec.timed_goals.clear()

    def add_goals_to_problem(self, problem):
        problem._goals = self.spec.goals
        problem._timed_goals = self.spec.timed_goals
        problem._trajectory_constraints = self.spec.trajectory_constraints
        for metric in self.spec.quality_metrics:
            problem.add_quality_metric(spec)

    def get_goals(self) -> list[Any]:
        return list(self.spec.goals)

    # ------------------------------------------------------------------
    # Filters
    # ------------------------------------------------------------------

    def add_object_filter(self, obj: str) -> None:
        self.spec.object_filters.append(obj)

    def remove_object_filter(self, obj: str) -> None:
        self.spec.object_filters.remove(obj)

    def add_fluent_filter(self, fluent: str) -> None:
        self.spec.fluent_filters.append(fluent)

    def remove_fluent_filter(self, fluent: str) -> None:
        self.spec.fluent_filters.remove(fluent)

    def add_action_filter(self, action: str) -> None:
        self.spec.action_filters.append(action)

    def remove_action_filter(self, action: str) -> None:
        self.spec.action_filters.remove(action)


# ---------------------------------------------------------------------------
# Process-pool entry point (must be a module-level function to be picklable)
# ---------------------------------------------------------------------------


def _run_planner_process(
    dom: str,
    prob: str,
    planner_spec: str,
    plan_kind: PlanKind,
    problem_name: str,
    goal_name: str,
    renamings_map: dict[str, str],
) -> PlanMessages:
    """
    Run the planner in a subprocess and return plan messages.

    All arguments are plain Python scalars / dicts so ProcessPoolExecutor can
    pickle and dispatch this function without any class-level state.
    """
    env = get_environment()
    env.credits_stream = None

    problem = PDDLReader().parse_problem_string(dom, prob)

    module_name, class_name = planner_spec.split(':')
    importlib.import_module(module_name)  # side-effect: makes the module importable
    env.factory.add_engine('custom_planner', module_name, class_name)

    r = Renamings(renamings_map)

    with env.factory.OneshotPlanner(name='custom_planner') as planner:
        result = planner.solve(problem, timeout=60.0)
        if result.status in POSITIVE_OUTCOMES:
            return True, dispatch_plan_result(
                result.plan, problem_name, goal_name, plan_kind, problem, r
            )
        else:
            return False, []


# ---------------------------------------------------------------------------
# ManagedProblem
# ---------------------------------------------------------------------------


class ManagedProblem:
    """Owns the base planning problem, a pool of goals, and the process executor."""

    def __init__(
        self,
        problem: Problem,
        env: Environment,
        name: str = 'base',
        planner_spec: str = '',
        logger: logging.Logger | None = None,
        max_workers: int = 4,
    ) -> None:
        self.base_problem = problem.clone()
        self.name = name
        self.planner_spec = planner_spec
        self.logger = logger or log
        self.env = env
        self._fnode_mgr = env.expression_manager
        managed_goal = ManagedGoal(name='base')
        for expr in self.base_problem._goals:
            managed_goal.add_goal_expr(expr)
        self.base_problem.clear_goals()
        for expr in self.base_problem._timed_goals:
            managed_goal.add_timed_goal_expr(expr)
        self.base_problem.clear_timed_goals()
        for expr in self.base_problem._trajectory_constraints:
            managed_goal.add_trajectory_constraint_expr(expr)
        for expr in self.base_problem.quality_metrics:
            managed_goal.add_quality_metric(expr)
        self.base_problem.clear_quality_metrics()
        self._executor: cf.ProcessPoolExecutor = cf.ProcessPoolExecutor(max_workers=max_workers)
        self.goals: dict[str, ManagedGoal] = {'base': managed_goal}

    # ------------------------------------------------------------------
    # Goal management
    # ------------------------------------------------------------------

    def add_goal_from_string(self, goal: str = 'base', goal_description: str = '') -> ManagedGoal:
        """Create and register a new named goal, populate it given a goal descriotion string."""
        managed_goal = ManagedGoal(name=goal)
        writer = PDDLWriter(self.base_problem)
        domain_string = writer.get_domain()
        problem_string = writer.get_problem()
        problem_string = self._replace_goal_string(problem_string, goal_description)
        reader = PDDLReader()
        temp_problem = reader.parse_problem_string(domain_string, problem_string)
        for expr in temp_problem._goals:
            managed_goal.add_goal_expr(expr)
        for expr in temp_problem._timed_goals:
            managed_goal.add_timed_goal_expr(expr)
        for expr in temp_problem._trajectory_constraints:
            managed_goal.add_trajectory_constraint_expr(expr)
        for expr in temp_problem.quality_metrics:
            managed_goal.add_quality_metric(expr)
        self.goals[goal] = managed_goal
        return self.goals[goal]

    def _replace_goal_string(self, writer_output: str, new_goal: str) -> str:
        """Helper to replace a goal string from a problem with an empty goal description."""
        start = writer_output.index('(:goal')

        # Find the matching closing parenthesis of (:goal ...)
        depth = 0
        end = None

        for i in range(start, len(writer_output)):
            if writer_output[i] == '(':
                depth += 1
            elif writer_output[i] == ')':
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break

        if end is None:
            raise RuntimeError('Could not find end of goal')

        replacement = f"""{new_goal}"""

        return writer_output[:start] + replacement + writer_output[end:]

    def add_goal(self, goal: str = 'base') -> ManagedGoal:
        """Create and register a new named goal, returning it for configuration."""
        self.goals[goal] = ManagedGoal(name=goal)
        return self.goals[goal]

    def get_goal(self, goal: str = 'base') -> ManagedGoal:
        return self.goals[goal]

    def add_goal_fluent(self, name: str, args: list[str], value: Any, goal: str = 'base') -> None:
        """
        Ground a fluent and add it to the named goal.

        Grounding is done here because this class owns the base Problem.
        The resulting FNode is stored in the goal's GoalSpec.
        """
        grounded_args = [self.base_problem.object(a) for a in args]
        grounded_fluent = self._fnode_mgr.FluentExp(self.base_problem.fluent(name), grounded_args)
        expr = self._fnode_mgr.Equals(grounded_fluent, value) if value else grounded_fluent
        self.goals[goal].add_goal_expr(expr)

    # ------------------------------------------------------------------
    # Object / fluent / action mutation
    # ------------------------------------------------------------------
    def get_object_list(self) -> list:
        return list(self.base_problem.all_objects)

    def add_object(self, name: str, obj_type: str) -> None:
        self.base_problem.add_object(name, self.base_problem.user_type(obj_type))

    def remove_object(self, name: str) -> None:
        object_list = [o for o in self.get_object_list() if o.name != name]
        self.base_problem = self.filter_problem(object_filter=object_list)

    def get_fluent_list(self) -> list:
        return list(self.base_problem.fluents)

    def set_fluent(self, name: str, args: list[str], value: Any) -> None:
        grounded_args = [self.base_problem.object(a) for a in args]
        grounded_fluent = self._fnode_mgr.FluentExp(self.base_problem.fluent(name), grounded_args)
        self.base_problem.set_initial_value(grounded_fluent, value)

    def get_action_list(self) -> list:
        return list(self.base_problem.actions)

    def add_action(self, name: str, args: list) -> None:
        raise NotImplementedError('add_action is not implemented')

    def remove_action(self, name: str) -> None:
        actions = [a for a in self.base_problem.actions if a.name != name]
        self.base_problem = self.filter_problem(action_filter=actions)

    # ------------------------------------------------------------------
    # Planning entry point
    # ------------------------------------------------------------------

    def plan(
        self,
        goal_name: str = 'base',
        plan_kind: PlanKind = PlanKind.SEQUENTIAL_PLAN,
        output_dir: str | None = None,
    ) -> PlanMessages:
        """
        Scope the base problem to *goal_name*, run the planner, return messages.

        Args
        ----
            goal_name:  Key into self.goals.
            plan_kind:  Desired output plan representation.
            output_dir: If provided, write the PDDL domain/problem files here
                        (useful for debugging).

        Returns
        -------
            A list of plan-action messages whose concrete type depends on
            *plan_kind*.  For HIERARCHICAL_PLAN, a 3-tuple is returned instead;
            see plan_handlers.handle_hierarchical_plan for details.

        """
        managed_goal = self.goals[goal_name]
        spec = managed_goal.spec

        goal_problem = self.filter_problem(
            action_filter=spec._effective(spec.action_filters),
            object_filter=spec._effective(spec.object_filters),
            fluent_filter=spec._effective(spec.fluent_filters),
        )
        managed_goal.add_goals_to_problem(goal_problem)

        writer = PDDLWriter(goal_problem)
        dom = writer.get_domain()
        prob = writer.get_problem()
        renamings_map = {k: v.name for k, v in writer.nto_renamings.items()}

        self._maybe_write_pddl(writer, goal_name, output_dir)

        future = self._executor.submit(
            _run_planner_process,
            dom,
            prob,
            self.planner_spec,
            plan_kind,
            self.name,
            goal_name,
            renamings_map,
        )
        return future.result()

    # ------------------------------------------------------------------
    # Problem filtering
    # ------------------------------------------------------------------

    def filter_problem(
        self,
        action_filter: list | None = None,
        object_filter: list | None = None,
        fluent_filter: list | None = None,
    ) -> Problem:
        """
        Return a clone of the base problem restricted to the given allowlists.

        Each filter is an allowlist; passing None means "keep everything".
        HierarchicalProblem is always returned as a plain clone because the
        HTN task network is not amenable to post-hoc filtering.
        """
        if isinstance(self.base_problem, HierarchicalProblem):
            return self.base_problem.clone()

        target_problem = Problem(name=self.base_problem.name, environment=self.env)

        for obj in self.base_problem.all_objects:
            if object_filter is None or obj in object_filter:
                target_problem.add_object(obj)

        for fluent in self.base_problem.fluents:
            if fluent_filter is None or fluent.name in fluent_filter:
                default = 0 if (fluent.type.is_real_type() or fluent.type.is_int_type()) else False
                target_problem.add_fluent(fluent, default_initial_value=default)

        for f, val in self.base_problem.initial_values.items():
            args = [str(arg) for arg in f.args]
            if not object_filter or not any(
                arg not in {o.name for o in object_filter} for arg in args
            ):
                target_problem.set_initial_value(f, val)

        for action in self.base_problem.actions:
            if action_filter is None or action.name in action_filter:
                target_problem.add_action(action)
        return target_problem

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _maybe_write_pddl(
        self,
        writer: PDDLWriter,
        goal_name: str,
        output_dir: str | None,
    ) -> None:
        if not output_dir:
            return
        output_path = Path(output_dir)
        try:
            output_path.mkdir(parents=True, exist_ok=True)
            writer.write_domain(str(output_path / f'{self.name}_{goal_name}_domain.pddl'))
            writer.write_problem(str(output_path / f'{self.name}_{goal_name}_problem.pddl'))
        except Exception as exc:
            self.logger.error(f'Failed to write PDDL files: {exc}')
