# -*- coding: utf-8 -*-
"""
Created on Sun May 28 19:07:59 2023

@author: Oscar Sapena Vercher
DSIC - UPV
"""

# Copyright 2021 AIPlan4EU project
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

import warnings
import unified_planning as up
from unified_planning.engines import Engine, Credits, PlanGenerationResultStatus, ValidationResult, ValidationResultStatus
from unified_planning.engines.mixins import OneshotPlannerMixin, PlanValidatorMixin
from unified_planning.model import ProblemKind
from typing import Callable, IO, Optional
from unified_planning.exceptions import UPProblemDefinitionError
from unified_planning.model.operators import OperatorKind
from unified_planning.model import EffectKind
from unified_planning.plans import ActionInstance, TimeTriggeredPlan, PartialOrderPlan
from fractions import Fraction
import numpy
import sys
import os

LIB_FOLDER = os.path.dirname(os.path.abspath(__file__))
sys.path.append(LIB_FOLDER)

import nextflap

credits = Credits('NextFLAP',
                  'Oscar Sapena Vercher - DSIC (UPV)',
                  'ossaver@upv.es',
                  'https://github.com/ossaver/NextFLAP',
                  'Free for Educational Use',
                  'NextFLAP offers the capability to generate a plan for classical, numerical and temporal problems.',
                  'NextFLAP offers the capability to generate a plan for classical, numerical and temporal problems.'
                )

class NextFLAPImpl(Engine, OneshotPlannerMixin, PlanValidatorMixin):
    """ Implementation of the NextFLAP Engine. """
    
    def __init__(self, **options):
        """ Engine initialization. No options are required. """
        Engine.__init__(self)
        OneshotPlannerMixin.__init__(self)
        PlanValidatorMixin.__init__(self)

    @property
    def name(self) -> str:
        return "NextFLAP"
    
    @staticmethod
    def supported_kind():
        supported_kind = ProblemKind()
        supported_kind.set_problem_class('ACTION_BASED')
        supported_kind.set_problem_type("SIMPLE_NUMERIC_PLANNING")
        supported_kind.set_problem_type("GENERAL_NUMERIC_PLANNING")        
        supported_kind.set_time('CONTINUOUS_TIME')
        supported_kind.set_time('INTERMEDIATE_CONDITIONS_AND_EFFECTS')
        supported_kind.set_time('TIMED_EFFECTS')
        supported_kind.set_time('TIMED_GOALS')
        supported_kind.set_time('DURATION_INEQUALITIES')
        supported_kind.set_expression_duration('STATIC_FLUENTS_IN_DURATIONS')
        supported_kind.set_expression_duration('FLUENTS_IN_DURATIONS')
        supported_kind.set_numbers('DISCRETE_NUMBERS')
        supported_kind.set_numbers('CONTINUOUS_NUMBERS')
        supported_kind.set_numbers('BOUNDED_TYPES')
        supported_kind.set_conditions_kind('NEGATIVE_CONDITIONS')
        supported_kind.set_conditions_kind('DISJUNCTIVE_CONDITIONS')
        supported_kind.set_conditions_kind('EQUALITIES')
        supported_kind.set_conditions_kind('EQUALITIES')
        supported_kind.set_conditions_kind('EXISTENTIAL_CONDITIONS')
        supported_kind.set_conditions_kind('UNIVERSAL_CONDITIONS')
        supported_kind.set_effects_kind('CONDITIONAL_EFFECTS')
        supported_kind.set_effects_kind('INCREASE_EFFECTS')
        supported_kind.set_effects_kind('DECREASE_EFFECTS')
        supported_kind.set_effects_kind('STATIC_FLUENTS_IN_BOOLEAN_ASSIGNMENTS')
        supported_kind.set_effects_kind('STATIC_FLUENTS_IN_NUMERIC_ASSIGNMENTS')
        supported_kind.set_effects_kind('FLUENTS_IN_BOOLEAN_ASSIGNMENTS')
        supported_kind.set_effects_kind('FLUENTS_IN_NUMERIC_ASSIGNMENTS')
        supported_kind.set_typing('FLAT_TYPING')
        supported_kind.set_typing('HIERARCHICAL_TYPING')
        supported_kind.set_fluents_type('NUMERIC_FLUENTS')
        supported_kind.set_fluents_type('OBJECT_FLUENTS')
        supported_kind.set_quality_metrics("PLAN_LENGTH")
        supported_kind.set_quality_metrics("MAKESPAN")
        return supported_kind
    
    @staticmethod
    def supports(problem_kind):
        return problem_kind <= NextFLAPImpl.supported_kind()
    
    @staticmethod
    def supports_plan(plan_kind):
        return plan_kind in [up.plans.PlanKind.PARTIAL_ORDER_PLAN]

    @staticmethod
    def get_credits(**kwargs) -> Optional[Credits]:
        return credits
    
    @staticmethod
    def _convert_fnode(node):
        """
        Translates a FNode into a representation suitable for NextFLAP.

        Parameters
        ----------
        node : FNode
            Node to be translated.

        Raises
        ------
        NotImplementedError
            If the feature represented in this node is not supported by NextFLAP.

        Returns
        -------
        exp : list
            List representation of the node suitable for NextFLAP.
        """
        exp = []
        t = node.node_type
        if t == OperatorKind.AND:
            exp.append('*and*')
        elif t == OperatorKind.OR:
            exp.append('*or*')
        elif t == OperatorKind.NOT:
            exp.append('*not*')
        elif t == OperatorKind.IMPLIES:
            exp.append('*imply*')
        elif t == OperatorKind.EXISTS:
            exp.append('*exists*')
            exp.append([['?' + v.name, v.type.name] for v in node.variables()])
        elif t == OperatorKind.FORALL:
            exp.append('*forall*')
            exp.append([['?' + v.name, v.type.name] for v in node.variables()])
        elif t == OperatorKind.FLUENT_EXP:
            exp.append('*fluent*')
            exp.append(node.fluent().name)
        elif t == OperatorKind.PARAM_EXP:
            exp.append('*param*')
            exp.append('?' + node.parameter().name)
        elif t == OperatorKind.VARIABLE_EXP:
            exp.append('*var*')
            exp.append('?' + node.variable().name)
        elif t == OperatorKind.OBJECT_EXP:
            exp.append('*obj*')
            exp.append(node.object().name)
        elif t == OperatorKind.TIMING_EXP:
            exp.append('*time*')
        elif t == OperatorKind.BOOL_CONSTANT:
            if node.is_true():
                exp.append('*true*')
            else:
                exp.append('*false*')
        elif t == OperatorKind.INT_CONSTANT:
            exp.append('*int*')
            exp.append(node.int_constant_value())
        elif t == OperatorKind.REAL_CONSTANT:
            exp.append('*real*')
            exp.append(node.real_constant_value().numerator /
                       node.real_constant_value().denominator)
        elif t == OperatorKind.PLUS:
            exp.append('*+*')
        elif t == OperatorKind.MINUS:
            exp.append('*-*')
        elif t == OperatorKind.TIMES:
            exp.append('***')
        elif t == OperatorKind.DIV:
            exp.append('*/*')
        elif t == OperatorKind.LE:
            exp.append('*<=*')
        elif t == OperatorKind.LT:
            exp.append('*<*')
        elif t == OperatorKind.EQUALS:
            exp.append('*=*')
        elif t == OperatorKind.IFF:
            exp.append('*and*')
            exp.append(['*imply*', NextFLAPImpl._convert_fnode(node.args[0]),
                        NextFLAPImpl._convert_fnode(node.args[1])])
            exp.append(['*imply*', NextFLAPImpl._convert_fnode(node.args[1]),
                        NextFLAPImpl._convert_fnode(node.args[0])])
        else:
            raise NotImplementedError
        if t != OperatorKind.IFF:
            for i in range(len(node.args)):
                exp.append(NextFLAPImpl._convert_fnode(node.args[i]))        
        return exp

    @staticmethod
    def _convert_effect(eff):
        """
        Translates an Effect into a representation suitable for NextFLAP.
    
        Parameters
        ----------
        eff : Effect
            Action effect.
    
        Raises
        ------
        NotImplementedError
            If the feature represented in this effect is not supported by NextFLAP.
    
        Returns
        -------
        list
            List representation of the effect suitable for NextFLAP.
        """
        if eff.kind == EffectKind.ASSIGN:
            op = '*=*'
        elif eff.kind == EffectKind.INCREASE:
            op = '*+=*'
        elif eff.kind == EffectKind.DECREASE:
            op = '*-=*'
        else:
            raise NotImplementedError
        if eff.is_conditional():
            return ['*when*', NextFLAPImpl._convert_fnode(eff.condition),
                    [op, NextFLAPImpl._convert_fnode(eff.fluent), NextFLAPImpl._convert_fnode(eff.value)]]
        else:
            return [op, NextFLAPImpl._convert_fnode(eff.fluent), NextFLAPImpl._convert_fnode(eff.value)]

    @staticmethod
    def _merge_conditional_effects(eff):
        """
        Merges a set of conditional effects into a single one if they share
        the same condition.
    
        Parameters
        ----------
        eff : list
            List of translated effects.
    
        Returns
        -------
        None.
        """
        i = 0
        while i < len(eff):
            if eff[i][0] == '*when*':
                j = i + 1
                while j < len(eff):
                    if eff[j][0] == '*when*' and eff[i][1] == eff[j][1]:
                        # Same condition
                        for k in range(2, len(eff[j])):
                            eff[i].append(eff[j][k])
                        del eff[j]
                    else:
                        j += 1
            i += 1

    @staticmethod
    def _add_action(action, durative):
        """
        Sends an action to NextFLAP, to be included in the planning task.

        Parameters
        ----------
        action : Action
            Action to be translated and sent to NextFLAP.
        durative : bool
            Indicates whether the action is durative (True) or not (False).

        Returns
        -------
        bool
            True if the action could be added successfully. False otherwise.
        """
        parameters = []
        for param in action.parameters:
            if param.type.is_real_type():
                typeName = '#real'
            elif param.type.is_int_type():
                typeName = '#int'
            else:
                typeName = param.type.name
            parameters.append(['?' + param.name, typeName])
        duration = []
        startCond = []
        overAllCond = []
        endCond = []
        startEff = []
        endEff = []
        if durative:
            if action.duration.lower == action.duration.upper:
                duration.append(NextFLAPImpl._convert_fnode(action.duration.lower))
            else: # Duration inequalities
                left = [action.duration.is_left_open()]
                left.append(NextFLAPImpl._convert_fnode(action.duration.lower))
                right = [action.duration.is_right_open()]
                right.append(NextFLAPImpl._convert_fnode(action.duration.upper))
                duration.append(left)
                duration.append(right)
            for key, value in action.conditions.items():
                if key.lower.is_from_start():
                    if key.upper.is_from_start():
                        for node in value:
                            startCond.append(NextFLAPImpl._convert_fnode(node))
                    else:
                        for node in value:
                            overAllCond.append(NextFLAPImpl._convert_fnode(node))
                else:
                    for node in value:
                        endCond.append(NextFLAPImpl._convert_fnode(node))
            for key, value in action.effects.items():
                if key.is_from_start():
                    for node in value:
                        startEff.append(NextFLAPImpl._convert_effect(node))
                else:
                    for node in value:
                        endEff.append(NextFLAPImpl._convert_effect(node))
        else:
            for cond in action.preconditions:
                startCond.append(NextFLAPImpl._convert_fnode(cond))
            NextFLAPImpl._group_preconditions(startCond)
            for eff in action.effects:
                startEff.append(NextFLAPImpl._convert_effect(eff))
        NextFLAPImpl._merge_conditional_effects(startEff)
        NextFLAPImpl._merge_conditional_effects(endEff)
        return nextflap.add_action(action.name, durative, parameters, duration,
                                   startCond, overAllCond, endCond,
                                   startEff, endEff)

    @staticmethod
    def _group_preconditions(conditions):
        """ Transforms a group of conditions in an AND condition """
        if len(conditions) > 1:
            cond = ['*and*']
            for term in conditions:
                cond.append(term)
            conditions.clear()
            conditions.extend(cond)
        NextFLAPImpl._remove_nested_ands(conditions)
        
    @staticmethod
    def _remove_nested_ands(conditions):
        """ Removes nested AND conditions """
        if len(conditions) > 0 and conditions[0] == '*and*':
            i = 1
            while i < len(conditions):
                term = conditions[i]
                if len(term) > 0 and term[0] == '*and*':
                    del conditions[i]
                    for subterm in term[1:]:
                        conditions.append(subterm)
                else:
                    i += 1
        
    @staticmethod
    def _translate(problem):
        """ Translates the problem to NextFLAP """
        try:
            # Introduce the definition of types in NextFLAP
            for t in problem.user_types:
                ancestors = [pt.name for pt in t.ancestors if t.name != pt.name]
                if not nextflap.add_type(t.name, ancestors):
                    raise UPProblemDefinitionError(nextflap.get_error())

            # Introduce the definition of objects in NextFLAP
            for obj in problem.all_objects:
                if not nextflap.add_object(obj.name, obj.type.name):
                    raise UPProblemDefinitionError(nextflap.get_error())
            
            # Introduce the definition predicates and functions in NextFLAP
            for fluent in problem.fluents:
                if fluent.type.is_bool_type():
                    fluentType = 'bool'
                else:
                    fluentType = 'number'
                params = [param.type.name for param in fluent.signature]
                if not nextflap.add_fluent(fluentType, fluent.name, params):
                    raise UPProblemDefinitionError(nextflap.get_error())
            
            # Introduce the set of durative actions in NextFLAP
            numDurativeActions = 0
            for action in problem.durative_actions:
                numDurativeActions += 1
                if not NextFLAPImpl._add_action(action, True):
                    raise UPProblemDefinitionError(nextflap.get_error())
            
            # Introduce the set of instantaneous actions in NextFLAP
            numInstantaneousActions = 0
            for action in problem.instantaneous_actions:
                numInstantaneousActions += 1
                if not NextFLAPImpl._add_action(action, False):
                    raise UPProblemDefinitionError(nextflap.get_error())
            
            # Introduce the problem initial state in NextFLAP
            initial_values = problem.explicit_initial_values
            for key, value in initial_values.items():
                if not nextflap.add_initial_value(NextFLAPImpl._convert_fnode(key), 
                                                  NextFLAPImpl._convert_fnode(value), 0.0):
                    raise UPProblemDefinitionError(nextflap.get_error())
            
            # Introduce the timed initial literals in NextFLAP
            for time, eff_list in problem.timed_effects.items():
                delay = float(time.delay)
                for eff in eff_list:
                    if not nextflap.add_initial_value(NextFLAPImpl._convert_fnode(eff.fluent),
                                                      NextFLAPImpl._convert_fnode(eff.value), delay):
                        raise UPProblemDefinitionError(nextflap.get_error())
            
            # Introduce the problem goals in NextFLAP
            goals = [NextFLAPImpl._convert_fnode(g) for g in problem.goals]
            NextFLAPImpl._group_preconditions(goals)
            if not nextflap.add_goal(goals):
                raise UPProblemDefinitionError(nextflap.get_error())
            
            return (True, numDurativeActions > 0)
        except UPProblemDefinitionError as e:
            warnings.warn(str(e))
            return (False, False)
    
    @staticmethod
    def _to_action(name, problem, durative):
        """
        Given its name, it generates an action instance. 

        Parameters
        ----------
        name : str
            Action name.
        problem : AbstractProblem
            Planning problem.
        durative : bool
            Indicates whether the action is durative (True) or not (False).

        Returns
        -------
        ActionInstance
            Action instance corresponding to the name.
        """
        action = name.split()
        if not durative:
            for a in problem.instantaneous_actions:
                if a.name == action[0]:
                    up_action = a
                    break
        else:
            for a in problem.durative_actions:
                if a.name == action[0]:
                    up_action = a
                    break
        params_tuple = []
        num_param = 0
        for param in action[1:]:
            if up_action.parameters[num_param].type.is_real_type():
                number = Fraction(round(float(param)*1000), 1000)
                fnode = problem.environment.expression_manager.Real(number)
            elif up_action.parameters[num_param].type.is_int_type():
                number = int(param)
                fnode = problem.environment.expression_manager.Int(number)
            else:
                obj = problem.object(param)
                fnode = problem.environment.expression_manager.ObjectExp(obj)
            params_tuple.append(fnode)
            num_param += 1
        return ActionInstance(up_action, tuple(params_tuple))
        
    @staticmethod
    def _to_durative_plan(plan_str, problem):
        """
        Translates a NextFLAP temporal plan into a TimeTriggeredPlan.

        Parameters
        ----------
        plan_str : str
            NextFLAP plan.
        problem : AbstractProblem
            Planning problem.

        Returns
        -------
        TimeTriggeredPlan
            Translated NextFLAP temporal plan.
        """
        lines = plan_str[1:-1].split('|')
        actions = []
        for line in lines:
            pos = line.index(':')
            start = Fraction(round(float(line[:pos])*1000), 1000)
            pos2 = line.index(')', pos)
            action = line[pos + 3:pos2]
            pos = line.index('[', pos2)
            pos2 = line.index(']', pos)
            durationValue = float(line[pos + 1:pos2])
            duration = Fraction(round(durationValue*1000), 1000)
            action_instance = NextFLAPImpl._to_action(action, problem, duration != None)
            actions.append((start, action_instance, duration))
        return TimeTriggeredPlan(actions, problem.environment)

    @staticmethod
    def _to_pop_plan(plan_str, problem):
        """
        Translates a NextFLAP non-temporal plan into a PartialOrderPlan.

        Parameters
        ----------
        plan_str : str
            NextFLAP plan.
        problem : AbstractProblem
            Planning problem.

        Returns
        -------
        PartialOrderPlan
            Translated NextFLAP non-temporal plan.
        """
        lines = plan_str[1:-1].split('|')
        actions = {}
        plan = dict()
        for line in lines:
            pos = line.find(':')
            if pos > 0: # id:action
                id = int(line[:pos])        
                name = line[pos + 1:]
                actions[id] = NextFLAPImpl._to_action(name, problem, False)
                plan[actions[id]] = list()
            else: # id1->id2
                pos = line.find("->")
                id_a1 = int(line[:pos])
                id_a2 = int(line[pos + 2:])
                a1 = actions[id_a1]
                a2 = actions[id_a2]
                adjacents = plan[a1]
                adjacents.append(a2)
        return PartialOrderPlan(plan, problem.environment)
    
    @staticmethod
    def _to_nextflap_plan(plan_str, problem, durativePlan):
        """
        Translates a NextFLAP plan into an UP Plan.


        Parameters
        ----------
        plan_str : TYPE
            NextFLAP plan.
        problem : TYPE
            Planning problem.
        durativePlan : bool
            Indicates whether the plan is temporal (True) or not (False).

        Returns
        -------
        Plan
            Translated NextFLAP plan.
        """
        if not plan_str.startswith('|'):
            return None    
        if (durativePlan):
            return NextFLAPImpl._to_durative_plan(plan_str, problem)
        else:
            return NextFLAPImpl._to_pop_plan(plan_str, problem)
    
    @staticmethod
    def _search(problem, durativePlan):
        """
        Searches for a plan.

        Parameters
        ----------
        problem : up.model.Problem
            Planning problem.

        Returns
        -------
        Plan (or None if no plan was found).
        """
        planStr = nextflap.solve(durativePlan)
        if not planStr.startswith('|'):
            return None    
        if (durativePlan):
            return NextFLAPImpl._to_durative_plan(planStr, problem)
        else:
            return NextFLAPImpl._to_pop_plan(planStr, problem)
        
    def _solve(self, problem: 'up.model.Problem',
              callback: Optional[Callable[['up.engines.PlanGenerationResult'], None]] = None,
              timeout: Optional[float] = None,
              output_stream: Optional[IO[str]] = None) -> 'up.engines.results.PlanGenerationResult':
        """ Solve a planning problem. """
        assert isinstance(problem, up.model.Problem)
        if output_stream is not None:
            warnings.warn('NextFLAP does not support output stream.', UserWarning)
        if timeout is not None:
            nextflap.start_task(timeout)
        else:
            nextflap.start_task(-1.0)
        ok, durativePlan = self._translate(problem)
        if ok:
            plan = self._search(problem, durativePlan)
        else:
            plan = None
        status = PlanGenerationResultStatus.UNSOLVABLE_INCOMPLETELY if plan is None else PlanGenerationResultStatus.SOLVED_SATISFICING
        res = up.engines.PlanGenerationResult(status, plan, self.name)
        return res

    def destroy(self):
        nextflap.end_task()
        
    def _validate(self, problem: 'up.model.AbstractProblem', plan: 'up.plans.Plan') -> 'up.engines.results.ValidationResult':
        assert isinstance(problem, up.model.Problem)
        matrix, node2index, index2node = self._compute_order_matrix(plan)
        valid = self._validate_plan(problem, plan, matrix, node2index, index2node)
        return ValidationResult(ValidationResultStatus.VALID if valid else ValidationResultStatus.INVALID,
                                self.name, [])

    @staticmethod
    def _compute_order_matrix(plan):
        """
        Computes a matrix that keeps the order between the actions of a given 
        partial-order plan.

        Parameters
        ----------
        plan : PartialOrderPlan

        Returns
        -------
        matrix : Numpy 2D bool matrix
            matrix[i][j] is True when action i must be executed before action j.
        node2index : dictionary action -> index.
        index2node : dictionary index -> action.
        """
        graph = plan.get_adjacency_list
        numNodes = len(graph)
        node2index = {}
        index2node = {}
        index = 0
        for node in graph:
            node2index[node] = index
            index2node[index] = node
            index += 1
        matrix = numpy.zeros((numNodes, numNodes), dtype=bool)
        for node in graph:
            i = node2index[node]
            adjacents = graph[node]
            for adj in adjacents:
                j = node2index[adj]
                matrix[i, j] = True
        for i in range(numNodes):
            for j in range(numNodes):
                if matrix[i, j]: # i -> j: all nodes after j go also after i
                    for k in range(numNodes):
                        if matrix[j, k]:
                            matrix[i, k] = True
        return matrix, node2index, index2node
    
    @staticmethod
    def _validate_plan(problem, plan, matrix, node2index, index2node):
        """
        Validates a partial order plan.

        Parameters
        ----------
        problem : Planning problem.
        plan : Partial order plan to check.
        matrix : Order matrix.
        node2index : dictionary action -> index.
        index2node : dictionary index -> action.

        Returns
        -------
        bool
            True if the plan is valid. False, otherwise.
        """
        try:
            state = {}
            initial_values = problem.explicit_initial_values
            for key, value in initial_values.items():
                NextFLAPImpl._add_to_state(NextFLAPImpl._convert_fnode(key),
                                           NextFLAPImpl._convert_fnode(value),
                                           state)
            executed = set()
            nextActions = NextFLAPImpl._get_next_plan_actions(matrix, executed)
            while len(nextActions) > 0:
                for i in nextActions:
                    executed.add(i)
                actions = [index2node[i] for i in nextActions]
                operators = {a:NextFLAPImpl._get_problem_action(a, problem) for a in actions}
                precs = {a:NextFLAPImpl._get_preconditions(a, operators) for a in actions}
                for a in actions:
                    if not NextFLAPImpl._preconditions_hold(precs[a], state, problem):
                        return False
                effs = {a:NextFLAPImpl._get_effects(a, operators, state, problem) for a in actions}
                if not NextFLAPImpl._check_simultaneity(actions, effs):
                    return False
                for a in actions:
                    NextFLAPImpl._update_state(effs[a], state)
                # Apply efects
                nextActions = NextFLAPImpl._get_next_plan_actions(matrix, executed)
            if not NextFLAPImpl._check_goal(state, problem):
                return False
            return True
        except:
            return False
    
    @staticmethod
    def _check_goal(state, problem):
        goal = [NextFLAPImpl._convert_fnode(g) for g in problem.goals]
        return NextFLAPImpl._preconditions_hold(goal, state, problem)
    
    @staticmethod
    def _add_to_state(key, value, state):
        """
        Adds a fluent to the state.

        Parameters
        ----------
        key : fluent name.
        value : fluent value.
        state : dictionary fluent_name -> value

        Returns
        -------
        None.
        """
        fluent = key[1]
        for i in range(2, len(key)):
            fluent += ' ' + key[i][1]
        if value[0] != '*false*': 
            if value[0] == '*true*':
                state[fluent] = True
            else:
                state[fluent] = value[1]
    
    @staticmethod
    def _get_next_plan_actions(matrix, executed):
        """
        Returns the next set of actions to execute.

        Parameters
        ----------
        matrix : order matrix.
        executed : set of already executed actions.

        Returns
        -------
        nextActions : list of actions to execute next.
        """
        nextActions = []
        numNodes = matrix.shape[0]
        for i in range(numNodes):
            if i not in executed:
                all_prev_executed = True
                for j in range(numNodes):
                    if matrix[j, i] and j not in executed:
                        all_prev_executed = False
                        break
                if all_prev_executed:
                    nextActions.append(i)
        return nextActions
    
    @staticmethod
    def _get_problem_action(a, problem):
        """
        Searches for a given action in the problem description.

        Parameters
        ----------
        a : action name.
        problem : planning problem.

        Returns
        -------
        pa : problem action.
        """
        for pa in problem.instantaneous_actions:
            if pa.name == a.action.name:
                return pa

    @staticmethod
    def _get_parameters(a, operators):
        """
        Links the operator parameters with the grounded action parameters.

        Parameters
        ----------
        a : action.
        operators : dictionary plan_action_name -> ungrounded_problem_action
            
        Returns
        -------
        dict
            Dictionary parameter_name -> object_name.
        """
        op = operators[a]
        groundedParams = [NextFLAPImpl._convert_fnode(p)[1] for p in a.actual_parameters]
        opParams = [p.name for p in op.parameters]
        return {opParams[i]:groundedParams[i] for i in range(len(opParams))}
        
    @staticmethod
    def _get_preconditions(a, operators):
        """
        Returns the grounded preconditions of a given action.

        Parameters
        ----------
        a : action.
        operators : dictionary plan_action_name -> ungrounded_problem_action.

        Returns
        -------
        precs : list
            List of grounded preconditions.
        """
        precs = []
        params = NextFLAPImpl._get_parameters(a, operators)
        op = operators[a]
        for cond in op.preconditions:
            precs.append(NextFLAPImpl._convert_fnode(cond))
        NextFLAPImpl._ground_parameters(precs, params)
        return precs
    
    @staticmethod
    def _ground_parameters(precs, params):
        """
        Replaces the action parameters by the corresponding objects in the
        action preconditions.

        Parameters
        ----------
        precs : list of preconditions.
        params : Dictionary parameter_name -> object_name.

        Returns
        -------
        None.
        """
        if not isinstance(precs, list) or len(precs) < 1:
            return
        if isinstance(precs[0], str) and precs[0] == '*param*':
            precs[0] = '*obj*'
            precs[1] = params[precs[1]]
        else:
            for i in range(len(precs)):
                item = precs[i]
                if isinstance(item, list):
                    NextFLAPImpl._ground_parameters(item, params)

            
    @staticmethod
    def _preconditions_hold(precs, state, problem):
        """
        Check if the preconditions hold in the given state.

        Parameters
        ----------
        precs : list of preconditions.
        state : dictionary fluent_name -> value.
        problem : planning problem.

        Returns
        -------
        bool
            True if the preconditions hold. False, otherwise
        """
        for prec in precs:
            if not NextFLAPImpl._precondition_hold(prec, state, problem):
                return False
        return True
    
    @staticmethod
    def _precondition_hold(prec, state, problem, neg = False):
        """
        Checks if a precondition holds in the given state.

        Parameters
        ----------
        prec : precondition.
        state : dictionary fluent_name -> value.
        problem : planning problem.
        neg : bool, optional
            Negated precondition. The default is False.

        Returns
        -------
        bool
            True if the precondition holds. False, otherwise.
        """
        t = prec[0] 
        if t == '*and*':
            for cond in prec[1:]:
                if not NextFLAPImpl._precondition_hold(cond, state, problem):
                    return False
        elif t == '*fluent*':
            fluent = NextFLAPImpl._get_fluent(prec)
            if neg:
                return fluent not in state
            else:
                return fluent in state
        elif t in ['*<=*', '*<*', '*=*']:
            return NextFLAPImpl._numeric_condition_hold(t, neg, prec[1], prec[2], state)
        elif t == '*or*':
            for cond in prec[1:]:
                if NextFLAPImpl._precondition_hold(cond, state, problem):
                    return True
            return False
        elif t == '*not*':
            return NextFLAPImpl._precondition_hold(prec[1], state, problem, not neg)
        elif t == '*imply*':
            value = (not NextFLAPImpl._precondition_hold(prec[1], state, problem) or 
                     NextFLAPImpl._precondition_hold(prec[2], state, problem))
            if neg:
                return not value
            else:
                return value
        elif t == '*exists*':
            value = NextFLAPImpl._evaluate_exists(prec[1], prec[2], state, problem)
            if neg:
                return not value
            else:
                return value
        elif t == '*forall*':
            value = NextFLAPImpl._evaluate_forall(prec[1], prec[2], state, problem)
            if neg:
                return not value
            else:
                return value
        else:
            print('Unk. type:', t, '->', prec)
            return False
        return True
    
    @staticmethod
    def _get_fluent(fluent):
        """
        Gets the string representation (fluent name) of a given fluent.

        Parameters
        ----------
        fluent : list of items.

        Returns
        -------
        f : str
            Fluent name.
        """
        f = fluent[1]
        for i in range(2, len(fluent)):
            arg = fluent[i]
            f += ' ' + arg[1]
        return f
    
    @staticmethod
    def _numeric_condition_hold(comp, neg, left, right, state):
        """
        Checks if a numeric condition holds in the state.

        Parameters
        ----------
        comp : comparator.
        neg : negated condition.
        left : left term of the comparison.
        right : right term of the comparison.
        state : dictionary fluent_name -> value.

        Returns
        -------
        bool
            True if the condition holds. False, otherwise.
        """
        if comp == '*=*' and left[0] == '*obj*' and right[0] == '*obj*': # equality
            return left[1] != right[1] if neg else left[1] == right[1]

        value1 = NextFLAPImpl._evaluate_numeric_expression(left, state)
        value2 = NextFLAPImpl._evaluate_numeric_expression(right, state)
        if comp == '*<=*':
            if neg:
                return value1 > value2
            else:
                return value1 <= value2
        elif comp == '*<*':
            if neg:
                return value1 >= value2
            else:
                return value1 < value2
        elif comp == '*=*':
            if neg:
                return value1 != value2
            else:
                return value1 == value2
        else:
            return False
        
    @staticmethod
    def _evaluate_numeric_expression(exp, state):
        """
        Returns the value of a numeric expression.

        Parameters
        ----------
        exp : numeric expression.
        state : dictionary fluent_name -> value.

        Returns
        -------
        Value of the expression.
        """
        t = exp[0]
        if t == '*fluent*':
            fluent = NextFLAPImpl._get_fluent(exp)
            if fluent in state:
                return state[fluent]
            else:
                return 0
        elif t == '*int*' or t == '*real*':
            return exp[1]
        elif t in ['*+*', '*-*', '***', '*/*']:
            res = NextFLAPImpl._evaluate_numeric_expression(exp[1], state)
            for i in range(2, len(exp)):
                value = NextFLAPImpl._evaluate_numeric_expression(exp[i], state)
                if t == '*+*':
                    res += value
                elif t == '*-*':
                    res -= value
                elif t == '***':
                    res *= value
                else:
                    res /= value;
            return res
        else:
            return False
        
    @staticmethod
    def _ground_condition_arguments(args, problem):
        """
        Gets all possible objects for the given variables (used in exists
        and forall expressions)

        Parameters
        ----------
        args : list of pairs [var_name, var_type].
        problem : planning problem.

        Returns
        -------
        List of pairs (var_name, list_of_objects).
        """
        arguments = {vname:[] for vname, _ in args}
        for vname, vtype in args:
            objList = arguments[vname]
            for obj in problem.all_objects:
                if obj.type.name == vtype:
                    objList.append(obj.name)
        return list(arguments.items())
    
    @staticmethod
    def _increase_combination_index(combinationIndex, arguments):
        """
        Moves to the next combination of object assignment for the
        variables (used in exists and forall expressions)

        Parameters
        ----------
        combinationIndex : list of integer indexes
            For each variable, the index indicates its current value (object).
        arguments : list of pairs (var_name, list_of_objects).

        Returns
        -------
        bool
            False if this is the last possible combination. True, otherwise
        """
        i = len(combinationIndex) - 1
        combinationIndex[i] += 1
        while combinationIndex[i] >= len(arguments[i][1]):
            combinationIndex[i] = 0
            i -= 1
            if i < 0:
                return False
            combinationIndex[i] += 1
        return True
    
    @staticmethod
    def _get_combination_values(arguments, combinationIndex):
        """
        Gets the assigned values to the variables (used in exists and forall 
        expressions)

        Parameters
        ----------
        arguments : list of pairs (var_name, list_of_objects).
        combinationIndex : list of integer indexes
            For each variable, the index indicates its current value (object).

        Returns
        -------
        Dictionary var_name -> object_name.
        """
        values = {}
        for i in range(len(combinationIndex)):
            values[arguments[i][0]] = arguments[i][1][combinationIndex[i]]
        return values
    
    @staticmethod
    def _make_combination_copy(cond, values, copy):
        """
        Makes a copy of a given condition, changing the variables (of exists
        and forall expressions) by the corresponding objects. 

        Parameters
        ----------
        cond : condition.
        values : dictionary var_name -> object_name.
        copy : list that will store the copy of the condition.

        Returns
        -------
        None.
        """
        if len(cond) == 0:
            return
        if isinstance(cond[0], str) and cond[0] == '*var*':
            copy.append('*obj*')
            copy.append(values[cond[1]])
        else:
            for item in cond:
                if isinstance(item, list):
                    itemCopy = []
                    NextFLAPImpl._make_combination_copy(item, values, itemCopy)
                    copy.append(itemCopy)
                else:
                    copy.append(item)
    
    @staticmethod
    def _make_combination(cond, arguments, combinationIndex):
        """
        Returns a copy of a given condition, changing the variables (of exists
        and forall expressions) by the corresponding objects. 

        Parameters
        ----------
        cond : condition.
        arguments : list of pairs (var_name, list_of_objects).
        combinationIndex : list of integer indexes
            For each variable, the index indicates its current value (object).

        Returns
        -------
        Copy of the condition.
        """
        values = NextFLAPImpl._get_combination_values(arguments, combinationIndex)
        copy = []
        NextFLAPImpl._make_combination_copy(cond, values, copy)
        return copy, NextFLAPImpl()._increase_combination_index(combinationIndex, arguments)
        
    @staticmethod
    def _evaluate_exists(args, cond, state, problem):
        """
        Checks if an exists expression holds in the given state.

        Parameters
        ----------
        args : variables defined in the exists expression.
        cond : condition to check.
        state : dictionary fluent_name -> value.
        problem : planning problem.

        Returns
        -------
        bool
            True if the condition holds. False, otherwise.
        """
        arguments = NextFLAPImpl()._ground_condition_arguments(args, problem)
        combinationIndex = [0] * len(arguments)
        while True:
            groundedCond, go_on = NextFLAPImpl()._make_combination(cond, arguments, combinationIndex)
            if NextFLAPImpl._precondition_hold(groundedCond, state, problem):
                return True
            if not go_on:
                break
        return False
    
    @staticmethod
    def _evaluate_forall(args, cond, state, problem):
        """
        Checks if a forall expression holds in the given state.

        Parameters
        ----------
        args : variables defined in the forall expression.
        cond : condition to check.
        state : dictionary fluent_name -> value.
        problem : planning problem.

        Returns
        -------
        bool
            True if the condition holds. False, otherwise.
        """
        arguments = NextFLAPImpl._ground_condition_arguments(args, problem)
        combinationIndex = [0] * len(arguments)
        while True:
            groundedCond, go_on = NextFLAPImpl._make_combination(cond, arguments, combinationIndex)
            if not NextFLAPImpl._precondition_hold(groundedCond, state, problem):
                return False
            if not go_on:
                break
        return True
    
    @staticmethod
    def _get_effects(a, operators, state, problem):
        """
        Gets the action effects.

        Parameters
        ----------
        a : action.
        operators : dictionary plan_action_name -> ungrounded_problem_action.
        state : dictionary fluent_name -> value.
        problem : planning problem.

        Returns
        -------
        List of action effects.
        """
        effs = []
        params = NextFLAPImpl._get_parameters(a, operators)
        op = operators[a]
        for e in op.effects:
            effs.append(NextFLAPImpl._convert_effect(e))
        groundedEffects = []
        for e in effs:
            if e[0] == '*when*':
                NextFLAPImpl._ground_parameters(e[1], params)
                if NextFLAPImpl._precondition_hold(e[1], state, problem):
                    for i in range(2, len(e)):
                        NextFLAPImpl._ground_parameters(e[i], params)
                        groundedEffects.append(e[i])
            else:
                NextFLAPImpl._ground_parameters(e, params)
                groundedEffects.append(e)
        return groundedEffects
    
    @staticmethod
    def _check_simultaneity(actions, effects):
        """
        Checks that parallel actions do not have contradictory effects.

        Parameters
        ----------
        actions : list of actions to check.
        effects : dictionary action -> action_effects.

        Returns
        -------
        bool
            True if no contradictory effects were found. False, otherwise
        """
        if len(actions) < 2:
            return True
        for a1 in actions:
            for a2 in actions:
                if a1 != a2:
                    if NextFLAPImpl._check_contradictory_effects(effects[a1], effects[a2]):
                        return False
        return True
    
    @staticmethod
    def _check_contradictory_effects(effs1, effs2):
        """
        Checks if the set of effects of two actions are contradictory.

        Parameters
        ----------
        effs1 : effects of the first action.
        effs2 : effects of the second action.

        Returns
        -------
        bool
            True if there are contradictory effects. False, otherwise.
        """
        for e1 in effs1:
            for e2 in effs2:
                if NextFLAPImpl._contradictory_effects(e1, e2):
                    return True
        return False
        
    @staticmethod
    def _is_propositional_effect(e):
        """
        Check if the effect is propositional.

        Parameters
        ----------
        e : effect.

        Returns
        -------
        bool
            True if the effect is propositional. False, otherwise.
        """
        return e[0] == '*=*' and (e[2][0] == '*true*' or e[2][0] == '*false*')
    
    @staticmethod
    def _contradictory_effects(e1, e2):
        """
        Checks if the effects of two actions are contradictory.

        Parameters
        ----------
        e1 : effect of the first action.
        e2 : effect of the second action.

        Returns
        -------
        bool
            True if they are contradictory effects. False, otherwise.
        """
        if NextFLAPImpl._is_propositional_effect(e1):
            if NextFLAPImpl._is_propositional_effect(e2):
                if e1[2] != e2[2]:
                    f1 = NextFLAPImpl._get_fluent(e1[1])
                    f2 = NextFLAPImpl._get_fluent(e2[1])
                    return f1 == f2
        elif not NextFLAPImpl._is_propositional_effect(e2):
            # Only += and -= are compatible when updating the same fluent
            f1 = NextFLAPImpl._get_fluent(e1[1])
            f2 = NextFLAPImpl._get_fluent(e2[1])
            if f1 == f2:
                if e1[0] not in ['*+=*', '*-=*'] or e2[0] not in ['*+=*', '*-=*']:
                    return True
        return False
    
    @staticmethod
    def _update_state(effs, state):
        """
        Updates the state with the given effects.

        Parameters
        ----------
        effs : list of effects.
        state : dictionary fluent_name -> value.

        Returns
        -------
        None.
        """
        for e in effs:
            if NextFLAPImpl._is_propositional_effect(e):
                NextFLAPImpl._update_prop_state(e[1], e[2], state)
            else:
                NextFLAPImpl._update_num_state(e[0], e[1], e[2], state)
    
    @staticmethod
    def _update_prop_state(key, value, state):
        """
        Updates the state with the given propositional effect.

        Parameters
        ----------
        key : fluent.
        value : value.
        state : dictionary fluent_name -> value.

        Returns
        -------
        None.
        """
        f = NextFLAPImpl._get_fluent(key)
        if value[0] == '*true*':
            state[f] = True
        elif f in state:
            del state[f]

    @staticmethod
    def _update_num_state(op, key, value, state):
        """
        Updates the state with the given numeric effect.

        Parameters
        ----------
        key : fluent.
        value : value.
        state : dictionary fluent_name -> value.

        Returns
        -------
        None.
        """
        f = NextFLAPImpl._get_fluent(key)
        v = NextFLAPImpl._evaluate_numeric_expression(value, state)
        if op == '*=*':
            state[f] = v
        elif op == '*+=*':
            state[f] += v
        elif op == '*-=*':
            state[f] -= v
        elif op == '**=*':
            state[f] *= v
        else:
            state[f] /= v
