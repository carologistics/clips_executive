
import concurrent.futures as cf
from pddl_msgs.srv import (
    AddFluent,
    AddFluents,
    RemoveFluents,
    AddObjects,
    RemoveObjects,
    SetFunctions,
    AddPddlInstance,
    CheckActionPrecondition,
    GetActionEffects,
    GetActionNames,
    GetFluents,
    GetFunctions,
    SetGoals,
    ClearGoals,
)
from pddl_msgs.msg import Fluent as FluentMsg, FluentEffect, FunctionEffect, Function, TimedPlanAction, Action as ActionMsg
from pddl_msgs.action import PlanTemporal
from std_msgs.msg import String

from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.shortcuts import (
    Problem,
    UserType,
    Object,
    Variable,
    UPState,
    BoolType,
    RealType,
    IntType,
)
from unified_planning.model.timing import Timepoint, TimepointKind
from unified_planning.plans import TimeTriggeredPlan, ActionInstance
from unified_planning.plans.plan import PlanKind
from unified_planning.plot import plot_time_triggered_plan
from unified_planning.engines.compilers import Grounder, GrounderHelper
from unified_planning.model import (
    Action,
    Fluent,
    FNode,
    ExpressionManager,
    UPState,
    Problem,
    MinimizeActionCosts,
    MinimizeExpressionOnFinalState,
    MaximizeExpressionOnFinalState,
    Oversubscription,
    Expression,
    Variable,
)
from unified_planning.model.effect import EffectKind
from unified_planning.model.timing import Timing, Timepoint, TimepointKind, TimeInterval
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.environment import get_environment

from unified_planning.model.walkers import StateEvaluator
from up_nextflap import NextFLAPImpl


def run_planner_process(env, dom, prob):
    env = get_environment()
    env.credits_stream = None
    reader = PDDLReader()
    problem = reader.parse_problem_string(dom,prob)
    env.factory.add_engine("nextflap", __name__, "NextFLAPImpl")
    with env.factory.OneshotPlanner(name="nextflap") as planner:
        result = planner.solve(problem, timeout=60.0)
        tPlan=None
        if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
            if result.plan.kind == PlanKind.TIME_TRIGGERED_PLAN:
                tPlan = result.plan.convert_to(
                    PlanKind.TIME_TRIGGERED_PLAN, result.plan
                )
                plan = result.plan
        else:
            return False

        return tPlan

class ManagedGoal():
    def __init__(self, problem, name="base"):
        self.problem = problem
        self.name = name
        self.goal_fluents = []
        self.object_filters = []
        self.fluent_filters = []
        self.action_filters = []

    def set_goal_fluent(self, name, args, value):
        grounded_args = []

        for arg in args:
            grounded_args.append(self.problem.base_problem.object(arg))

        grounded_fluent = self.problem.fnode_manager.FluentExp(
            self.problem.base_problem.fluent(name), grounded_args
        )
        if value:
            grounded_goal_expr = self.problem.fnode_manager.Equals(grounded_fluent, value)
            self.goal_fluents.append(grounded_goal_expr)
        else:
            self.goal_fluents.append(grounded_fluent)

    def remove_goal_fluent(self, name, args):
        raise NotImplementedError("remove_goal_fluent is not implemented")

    def get_goal_fluents(self):
        return self.goal_fluents

    def add_object_filter(self, obj):
        self.object_filters.append(obj)

    def add_fluent_filter(self, fluent):
        self.fluent_filters.append(fluent)

    def add_action_filter(self, action):
        self.action_filters.append(action)

    def remove_goal_fluent(self, fluent):
        self.goal_fluents.remove(fluent)

    def remove_object_filter(self, obj):
        self.object_filters.remove(obj)

    def remove_fluent_filter(self, fluent):
        self.fluent_filters.remove(fluent)

    def clear_goals(self):
        self.goal_fluents = []

    def plan_in_pool(self):
        action_filters = self.action_filters
        if len(self.action_filters) == 0:
            action_filters = None
        object_filters = self.object_filters
        if len(self.object_filters) == 0:
            object_filters = None
        fluent_filters = self.fluent_filters
        if len(self.fluent_filters) == 0:
            fluent_filters = None
        goal_problem = self.problem.filter_problem(action_filters, object_filters, fluent_filters)
        goal_problem.clear_goals()

        # add the goal fluents
        for fluent in self.goal_fluents:
            goal_problem.add_goal(fluent)

        writer = PDDLWriter(goal_problem)
        dom = writer.get_domain()
        prob = writer.get_problem()

        future = self.problem.executor.submit(run_planner_process, self.problem.env, dom, prob)
        result = future.result()

        plan_actions = []
        if result:
            delta_threshold = 0.1
            last_time = 0.0
            equiv_class_idx = 0
            # iterate over all actions in the plan to compute regions and generate response
            for time, act, duration in result.timed_actions:
                plan_action = TimedPlanAction()
                plan_action.pddl_instance = self.problem.name
                plan_action.goal_instance = self.name
                if f"{act.action.name}" in writer.nto_renamings.keys():
                    plan_action.name=f"{writer.get_item_named(f"{act.action.name}").name}"
                else:
                    plan_action.name = f"{act.action.name}"

                plan_action.args = []
                for arg in act.actual_parameters:
                    if f"{arg}" in writer.nto_renamings.keys():
                        plan_action.args.append(f"{writer.get_item_named(arg.__str__())}")
                    else:
                        plan_action.args.append(f"{arg}")
                plan_action.start_time = float(time)
                plan_action.duration = float(duration)
                if float(time) - last_time > delta_threshold:
                    equiv_class_idx += 1
                    last_time = float(time)
                plan_action.equiv_class = equiv_class_idx
                plan_actions.append(plan_action)

            return plan_actions
        return None

class ManagedProblem():
    def __init__(self, problem, env, name="base"):
        self.goals = {}
        self.base_problem = problem.clone()
        self.base_problem.clear_goals()
        self.name = name

        self.goals["base"] = ManagedGoal(self)
        self.executor = cf.ProcessPoolExecutor(max_workers=4)

        self.env = env
        self.fnode_manager = self.env.expression_manager
        self.env.factory.add_engine('nextflap', __name__, 'NextFLAPImpl')

    def filter_problem(self, action_filter=None, object_filter=None, fluent_filter=None):
        target_problem = Problem(self.base_problem.name, environment=self.env)

        # add the objects based on the object filters
        objects = self.get_object_list()
        for object in objects:
            if object_filter is None or object.name in object_filter:
                target_problem.add_object(object)

        # add the liftd fluents based on the fluent filters
        for fluent in self.base_problem.fluents:
            if fluent_filter is None or fluent.name in fluent_filter:
                target_problem.add_fluent(fluent)

        # set the initial values based on the fluent filters
        for f, val in self.base_problem.initial_values.items():
            args = [f"{arg}" for arg in f.args]

            if fluent_filter is None or (f.fluent().name in fluent_filter and len([obj for obj in args if obj not in object_filter]) == 0):
                target_problem.set_initial_value(f, val)

        # apply the actions based on the action filters
        for action in self.base_problem.actions:
            if action_filter is None or action.name in action_filter:
                target_problem.add_action(action)

        return target_problem

    def get_object_list(self):
        return self.base_problem.all_objects

    def add_object(self, name, type):
        self.base_problem.add_object(name, self.base_problem.user_type(type))

    def remove_object(self, name):
        object_list = self.get_object_list()
        for obj in object_list:
            if obj.name == name:
                object_list.remove(obj)
                break
        self.base_problem = self.filter_problem(self.get_action_list(), object_list, self.get_fluent_list())

    def get_fluent_list(self):
        return self.base_problem.fluents

    def set_fluent(self, name, args, value):
        grounded_args = []
        for arg in args:
            grounded_args.append(self.base_problem.object(arg))
        grounded_fluent = self.fnode_manager.FluentExp(
            self.base_problem.fluent(name), grounded_args
        )
        self.base_problem.set_initial_value(grounded_fluent, value)

    def get_action_list(self):
        return self.base_problem.actions

    def add_action(self, name, args):
        raise NotImplementedError("add_action is not implemented")

    def remove_action(self, name):
        actions = self.base_problem.actions
        for action in actions:
            if action.name == name:
                actions.remove(action)
                break
        self.base_problem = self.filter_problem(actions, self.get_object_list(), self.get_fluent_list())

    def add_goal(self, goal="base"):
        self.goals[goal] = ManagedGoal(self, goal)

    def get_goal(self, goal="base"):
        return self.goals[goal]

    def add_goal_fluent(self, name, args, value, goal):
        self.goals[goal].set_goal_fluent(name, args, value)

    def remove_goal_fluent(self, name, args, goal="base"):
        self.goals[goal].remove_goal_fluent(name, args)
