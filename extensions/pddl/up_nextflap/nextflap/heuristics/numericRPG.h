#ifndef NUMERIC_RPG_H
#define NUMERIC_RPG_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Computes a numeric relaxed plan based on fluent      */
/* intervals for heuristic plan evaluation.             */
/********************************************************/

#include "../planner/intervalCalculations.h"
#include "../planner/state.h"
#include "../sas/sasTask.h"
#include "../utils/priorityQueue.h"
#include <unordered_set>
#include <vector>

// Numeric effect of an action
class NumericRPGEffect {
public:
  bool numeric; // Variable is propositional or numeric
  TVariable
      var;      // Variable or numeric variable, depending on the previous field
  TValue value; // Value reached for the propositional variable
  float minValue; // Minimum numeric value reached, if it is a numeric variable
  float maxValue; // Maximum numeric value reached, if it is a numeric variable
  SASAction *a;   // Productor action

  NumericRPGEffect(TVariable v, TValue val, SASAction *a) {
    numeric = false;
    var = v;
    value = val;
    this->a = a;
  }
  NumericRPGEffect(TVariable v, float min, float max, SASAction *a) {
    numeric = true;
    var = v;
    minValue = min;
    maxValue = max;
    this->a = a;
  }
};

// Numeric condition
class NumericRPGCondition : public PriorityQueueItem {
public:
  char type; // 'V': sas variable, '-': numeric var (minimum value required),
             // '+': numeric var (maximum value required)
  TVariable var;
  TValue value;
  int level;
  SASAction *producer; // Only for numeric conditions

  NumericRPGCondition(SASCondition *c, int l) {
    type = 'V';
    var = c->var;
    value = c->value;
    level = l;
  }
  NumericRPGCondition(TVariable v, bool maxRequired, int l, SASAction *p) {
    type = maxRequired ? '+' : '-';
    var = v;
    level = l;
    producer = p;
  }
  inline int compare(PriorityQueueItem *other) {
    return ((NumericRPGCondition *)other)->level - level;
  }
  virtual ~NumericRPGCondition() {}
};

// Actions that produce a given value interval for a variable
class NumericRPGproducers {
public:
  SASAction *minProducer;
  float minValue;
  SASAction *maxProducer;
  float maxValue;

  NumericRPGproducers() { minProducer = maxProducer = nullptr; }
};

// Numeric relaxed planning graph
class NumericRPG : public FluentIntervalData {
private:
  SASTask *task;
  std::vector<SASAction *> remainingGoals;
  std::vector<std::vector<NumericRPGproducers>>
      numVarProducers; // For each numeric variable, the actions that updated
                       // its value interval
  std::vector<TInterval> numVarValue; // Last value interval for each variable
  std::vector<std::vector<int>> actionLevel; // Levels where the action appears
  std::vector<std::vector<int>>
      literalLevel; // Level of each pair (variable, value)
  std::vector<NumericRPGEffect> nextLevel;
  std::vector<TVarValue> reachedValues;
  std::unordered_set<TVariable> reachedNumValues;
  std::vector<int> goalLevel;
  PriorityQueue openConditions;
  std::vector<SASAction *> achievedNumericActions;
  std::unordered_set<TVarValue> numericSubgoals;
  int limit;

  void initialize();
  void createFirstFluentLevel(TState *fs, std::vector<SASAction *> *tilActions);
  void updateNumericValueInterval(int var, float minValue, float maxValue);
  void createFirstActionLevel();
  bool isApplicable(SASAction *a, int level);
  void programActionEffects(SASAction *a, int level);
  void programNumericEffect(TVariable v, int level, TFloatValue min,
                            TFloatValue max, SASAction *a);
  void expand();
  bool updateNumericValues(int level);
  void checkAction(SASAction *a, int level);
  bool checkGoal(SASAction *a, int level);
  void addSubgoals(SASAction *a, int level, NumericRPGCondition *cp);
  void addSubgoal(SASCondition *c);
  void addSubgoal(SASAction *a, SASNumericCondition *c, int level,
                  std::vector<NumericRPGCondition *> *numCond);
  SASAction *searchBestAction(TVariable v, TValue value, int level,
                              int *actionLevel);
  int findLevel(int actionIndex, int maxLevel);
  int findMinNumVarLevel(TVariable v, int maxLevel);
  int findMaxNumVarLevel(TVariable v, int maxLevel);
  void addMinValueSubgoal(SASAction *a, SASNumericExpression *e, int level,
                          std::vector<NumericRPGCondition *> *numCond);
  void addMaxValueSubgoal(SASAction *a, SASNumericExpression *e, int level,
                          std::vector<NumericRPGCondition *> *numCond);
  void addNumericSubgoal(TVariable v, int level, bool max,
                         std::vector<NumericRPGCondition *> *numCond);
  bool *calculateCondEffHold(SASAction *a, int level, IntervalCalculations &ic);
  bool checkCondEffectHold(SASConditionalEffect &e, int level,
                           IntervalCalculations &ic);

public:
  NumericRPG(TState *fs, std::vector<SASAction *> *tilActions, SASTask *task,
             int limit);
  int evaluate();
  int evaluateInitialPlan(/*bool* usefulActions*/);
  TFloatValue getMinValue(TVariable v, int numState);
  TFloatValue getMaxValue(TVariable v, int numState);
};

#endif // !NUMERIC_RPG_H
