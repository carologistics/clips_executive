#ifndef SAS_TASK_H
#define SAS_TASK_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Finite-domain task representation.                   */
/********************************************************/

#include "../utils/utils.h"
#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#define FICTITIOUS_FUNCTION 999999U

class SASValue {
public:
  unsigned int index;
  unsigned int fncIndex;
  std::string name;
  std::string toString();
};

class SASVariable {
public:
  unsigned int index;
  std::string name;
  std::vector<unsigned int> possibleValues;
  std::vector<unsigned int> value;
  std::vector<float> time;

  void addPossibleValue(unsigned int value);
  void addInitialValue(unsigned int sasValue, bool isTrue, float timePoint);
  std::string toString();
  std::string toString(std::vector<SASValue> &values);
  std::string toStringInitialState(std::vector<SASValue> &values);
  unsigned int getOppositeValue(unsigned int v);
  TVarValue getInitialStateValue();
  int getPossibleValueIndex(unsigned int value);
};

class NumericVariable {
public:
  unsigned int index;
  std::string name;
  std::vector<float> value;
  std::vector<float> time;
  void addInitialValue(float value, float time);
  std::string toString();
  std::string toStringInitialState();
  float getInitialStateValue();
};

class SASControlVar;

class SASNumericExpression {
public:
  char type; // 'N' = GE_NUMBER, 'V' = GE_VAR, '+' = GE_SUM, '-' = GE_SUB, '/' =
             // GE_DIV, '*' = GE_MUL,
             //                  'D' = GE_DURATION, '#' = GE_SHARP_T, 'C' =
             //                  GE_CONTROL_VAR
  float value; // if type == 'N'
  uint16_t var;
  std::vector<SASNumericExpression>
      terms; // if type == '+' | '-' | '*' | '/' | '#'
  std::string toString(std::vector<NumericVariable> *numVariables,
                       std::vector<SASControlVar> *controlVars);
  bool equals(SASNumericExpression *e);
  void copyFrom(SASNumericExpression *e);
  bool findControlVar(int cv);
  bool findFluent(TVariable v);
  void getVariables(std::vector<TVariable> *vars);
};

class SASDurationCondition {
public:
  char time; // 'S' = AT_START, 'E' = AT_END, 'A' = OVER_ALL, 'N' = NONE
  char comp; // '=' = CMP_EQ, '<' = CMP_LESS, 'L' = CMP_LESS_EQ, '>' =
             // CMP_GREATER, 'G' = CMP_GREATER_EQ, 'N' = CMP_NEQ
  SASNumericExpression exp;
  std::string toString(std::vector<NumericVariable> *numVariables,
                       std::vector<SASControlVar> *controlVars);
};

class SASDuration {
public:
  std::vector<SASDurationCondition> conditions;
  float minDuration;
  float maxDuration;
  bool constantDuration;
  bool durationNeededInEffects;
  std::vector<int> controlVarsNeededInDuration;

  std::string toString(std::vector<NumericVariable> *numVariables,
                       std::vector<SASControlVar> *controlVars);
};

class SASCondition {
public:
  unsigned int var;
  unsigned int value;
  bool isModified;
  SASCondition(unsigned int var, unsigned int value);
  bool equals(SASCondition *c) { return var == c->var && value == c->value; }
};

class SASNumericCondition {
private:
  void swapCondition();
  void swapTerms();
  void reshapeTerms(int cv);
  void reshapeFluentTerms(TVariable v);

public:
  char comp; // '=' = CMP_EQ, '<' = CMP_LESS, 'L' = CMP_LESS_EQ, '>' =
             // CMP_GREATER, 'G' = CMP_GREATER_EQ, 'N' = CMP_NEQ, '-' = dummy
             // comparator
  std::vector<SASNumericExpression> terms;
  std::string toString(std::vector<NumericVariable> *numVariables,
                       std::vector<SASControlVar> *controlVars);
  bool equals(SASNumericCondition *c);
  void copyFrom(SASNumericCondition *c);
  void reshape(int cv);
  void reshapeFluent(TVariable v);
  void getVariables(std::vector<TVariable> *vars);
};

class SASNumericEffect {
public:
  char op; // '=' = AS_ASSIGN, '+' = AS_INCREASE, '-' = AS_DECREASE, '*' =
           // AS_SCALE_UP, '/' = AS_SCALE_DOWN
  unsigned int var;
  SASNumericExpression exp;
  std::string toString(std::vector<NumericVariable> *numVariables,
                       std::vector<SASControlVar> *controlVars);
};

class SASGoalDescription {
public:
  char time; // 'S' = AT_START, 'E' = AT_END, 'A' = OVER_ALL, 'N' = NONE
  char type; // 'V' = (= var value), '&' = AND, '|' = OR, '!' = NOT, COMPARISON
             // (see comp in SASNumericCondition)
  unsigned int var, value;               // (= var value)
  std::vector<SASGoalDescription> terms; // AND, OR, NOT
  std::vector<SASNumericExpression> exp; // Comparison
};

class SASPreference {
public:
  unsigned int index;
  SASGoalDescription preference;
};

class TInterval {
public:
  TFloatValue minValue; // Minimum value
  TFloatValue maxValue; // Maximum value
  TInterval() {}
  TInterval(TFloatValue min, TFloatValue max) {
    minValue = min;
    maxValue = max;
  }
};

class SASControlVarCondition {
public:
  SASNumericCondition condition;
  bool inActionPrec;
  int numCvs;
};

class SASControlVar {
public:
  int index;
  char type; // 'I' = integer, 'N' = number
  std::string name;

  std::vector<SASControlVarCondition>
      conditions; // Conditions with this control var.

  /*
  std::vector<SASNumericCondition> simpleConditions;	// Conditions with only
  this control var. std::vector<SASNumericCondition> complexConditions;	//
  Conditions involving other control vars.
  */

  void copyCondition(SASNumericCondition *c, int cv, int numCvs, bool inPrecs);
  std::string toString(std::vector<NumericVariable> *numVariables,
                       std::vector<SASControlVar> *controlVars);
};

class SASConditionalEffect {
public:
  std::vector<SASCondition> startCond;
  std::vector<SASCondition> endCond;
  std::vector<SASNumericCondition> startNumCond;
  std::vector<SASNumericCondition> endNumCond;
  std::vector<SASCondition> startEff;
  std::vector<SASCondition> endEff;
  std::vector<SASNumericEffect> startNumEff;
  std::vector<SASNumericEffect> endNumEff;
};

class SASAction {
private:
  void postProcessDuration(SASDurationCondition &dc);
  void searchForControlVarsInDuration(SASNumericExpression *e);
  void updateMinDuration(float value);
  void updateMaxDuration(float value);
  float evaluateMinDuration(SASNumericExpression *e);
  float evaluateMaxDuration(SASNumericExpression *e);
  bool checkDurationInEffect(SASNumericExpression *e);
  void removeSameCondition(SASCondition *c,
                           std::vector<SASCondition> &condList);
  void removeSameNumericCondition(SASNumericCondition *c,
                                  std::vector<SASNumericCondition> &condList);
  void postprocessControlVariables();
  void postprocessNumericVariables();
  char analyzeNumericCondition(SASNumericCondition *c, std::vector<int> *cvs);
  void containsControlVar(SASNumericExpression *e, std::vector<int> *cvs);
  bool containsFluents(SASNumericExpression *e);
  void containsFluents(SASNumericExpression *e, std::vector<TVariable> *vars);
  void copyCondition(
      SASNumericCondition *c, TVariable v,
      std::unordered_map<TVariable, std::vector<SASNumericCondition>> &cons);

public:
  unsigned int index;
  std::string name;
  std::vector<SASControlVar> controlVars;
  std::unordered_map<TVariable, std::vector<SASNumericCondition>>
      startNumConstrains;
  std::unordered_map<TVariable, std::vector<SASNumericCondition>>
      endNumConstrains;
  SASDuration duration;
  std::vector<SASCondition> startCond;
  std::vector<SASCondition> endCond;
  std::vector<SASCondition> overCond;
  std::vector<SASNumericCondition> startNumCond;
  std::vector<SASNumericCondition> overNumCond;
  std::vector<SASNumericCondition> endNumCond;
  std::vector<SASCondition> startEff;
  std::vector<SASCondition> endEff;
  std::vector<SASNumericEffect> startNumEff;
  std::vector<SASNumericEffect> endNumEff;
  std::vector<SASPreference> preferences;
  std::vector<SASConditionalEffect> conditionalEff;
  bool isGoal;
  bool isTIL;
  bool instantaneous;

  SASAction(bool instantaneous, bool isTIL, bool isGoal) {
    this->instantaneous = instantaneous;
    this->isTIL = isTIL;
    this->isGoal = isGoal;
  }
  void postProcess();
};

class SASConstraint {
public:
  char type; // '&' = RT_AND, 'P' = RT_PREFERENCE, 'G' = RT_GOAL_PREFERENCE, 'E'
             // = RT_AT_END, 'A' = RT_ALWAYS = 4, 'S' = RT_SOMETIME, 'W' =
             // RT_WITHIN 'O' = RT_AT_MOST_ONCE, 'F' = RT_SOMETIME_AFTER, 'B' =
             // RT_SOMETIME_BEFORE, 'T' = RT_ALWAYS_WITHIN, 'D' =
             // RT_HOLD_DURING, 'H' = RT_HOLD_AFTER
  std::vector<SASConstraint> terms; // '&', 'P'
  unsigned int preferenceIndex;     // 'P', 'G'
  std::vector<SASGoalDescription>
      goal; // 'G' | 'E' | 'A' | 'S' | 'W' | 'O' | 'F' | 'B' | 'T' | 'D' | 'H'
  std::vector<float> time; // 'W' | 'T' | 'D' | 'H'
};

class SASMetric {
public:
  char type; // '+' = MT_PLUS, '-' = MT_MINUS, '*' = MT_PROD, '/' = MT_DIV, 'N'
             // = MT_NUMBER, 'T' = MT_TOTAL_TIME, 'V' = MT_IS_VIOLATED, 'F' =
             // MT_FLUENT
  float value;                  // 'N'
  unsigned int index;           // 'V' | 'F'
  std::vector<SASMetric> terms; // '+' | '-' | '*' | '/'
};

class GoalDeadline {
public:
  float time;
  std::vector<TVarValue> goals;
};

class SASConditionalProducer {
public:
  SASAction *a;
  unsigned int numEff;
  SASConditionalProducer(SASAction *a, unsigned int numEff) {
    this->a = a;
    this->numEff = numEff;
  }
};

class SASTask {
private:
  std::unordered_map<TMutex, bool> mutex;
  std::unordered_map<TVarValue, std::vector<TVarValue> *> mutexWithVarValue;
  std::unordered_map<TMutex, bool> permanentMutex;
  std::unordered_map<TMutex, bool> permanentMutexActions;
  std::unordered_map<std::string, unsigned int> valuesByName;
  std::vector<TVarValue> goalList;
  bool *staticNumFunctions;
  std::vector<GoalDeadline> goalDeadlines;

  inline static TMutex getMutexCode(TVariable var1, TValue value1,
                                    TVariable var2, TValue value2) {
    TMutex code = (var1 << 16) + value1;
    code = (code << 16) + var2;
    code = (code << 16) + value2;
    return code;
  }
  // void computeActionCost(SASAction* a, bool* variablesOnMetric);
  bool checkVariablesUsedInMetric(SASMetric *m, bool *variablesOnMetric);
  bool checkVariableExpression(SASNumericExpression *e,
                               bool *variablesOnMetric);
  float computeFixedExpression(SASNumericExpression *e);
  float evaluateMetric(SASMetric *m, float *numState, float makespan);
  bool checkActionMutex(SASAction *a1, SASAction *a2);
  bool checkActionOrdering(SASAction *a1, SASAction *a2);
  void computeMutexWithVarValues();
  void checkReachability(TVarValue vv,
                         std::unordered_map<TVarValue, bool> *goals);
  void checkEffectReached(SASCondition *c,
                          std::unordered_map<TVarValue, bool> *goals,
                          std::unordered_map<TVarValue, bool> *visitedVarValue,
                          std::vector<TVarValue> *state);
  void addGoalToList(SASCondition *c);

public:
  static const unsigned int OBJECT_TRUE = 0;
  static const unsigned int OBJECT_FALSE = 1;
  static const unsigned int OBJECT_UNDEFINED = 2;
  std::vector<SASVariable> variables;
  std::vector<SASValue> values;
  std::vector<NumericVariable> numVariables;
  std::vector<SASAction> actions;
  std::vector<std::string> preferenceNames;
  std::vector<SASAction> goals;
  std::vector<SASConstraint> constraints;
  char metricType; // '>' = Maximize, '<' = Minimize , 'X' = no metric specified
  SASMetric metric;
  bool metricDependsOnDuration; // True if the metric depends on the plan
                                // duration
  std::vector<SASAction *> **requirers;
  std::vector<SASAction *> *numRequirers;
  std::vector<SASAction *> *numGoalRequirers;
  std::vector<SASAction *> **producers;
  std::vector<SASConditionalProducer> **condProducers;
  std::vector<SASAction *> actionsWithoutConditions;
  TValue *initialState; // Values of the SAS variables in the initial state
  float
      *numInitialState; // Values of the numeric variables in the initial state
  bool variableCosts;   // True if there are actions with a cost that depends on
                        // the state
  int numGoalsInPlateau;
  char domainType;
  bool tilActions;
  std::vector<TVariable>
      *numVarReqAtStart; // For each action, numeric variables that are required
                         // in the start point of the action
  std::vector<TVariable>
      *numVarReqAtEnd; // For each action, numeric variables that are required
                       // in the end point of the action
  std::vector<TVariable> *numVarReqGoal; // For each goal, numeric variables
                                         // that are required in the goal

  SASTask();
  ~SASTask();
  void addMutex(unsigned int var1, unsigned int value1, unsigned int var2,
                unsigned int value2);
  bool isMutex(unsigned int var1, unsigned int value1, unsigned int var2,
               unsigned int value2);
  bool isPermanentMutex(unsigned int var1, unsigned int value1,
                        unsigned int var2, unsigned int value2);
  bool isPermanentMutex(SASAction *a1, SASAction *a2);
  SASVariable *createNewVariable();
  SASVariable *createNewVariable(std::string name);
  inline static TVarValue getVariableValueCode(unsigned int var,
                                               unsigned int value) {
    return (var << 16) + value;
  }
  inline static TVariable getVariableIndex(TVarValue code) {
    return code >> 16;
  }
  inline static TValue getValueIndex(TVarValue code) { return code & 0xFFFF; }
  unsigned int createNewValue(std::string name, unsigned int fncIndex);
  unsigned int findOrCreateNewValue(std::string name, unsigned int fncIndex);
  unsigned int getValueByName(const std::string &name);
  NumericVariable *createNewNumericVariable(std::string name);
  SASAction *createNewAction(std::string name, bool instantaneous, bool isTIL,
                             bool isGoal);
  SASAction *createNewGoal();
  void computeInitialState();
  void computeRequirers();
  void computeProducers();
  void computeNumericVariablesInActions();
  void computeNumericVariablesInActions(SASAction &a);
  void computeNumericVariablesInGoals(SASAction &a);
  void computeNumericVariablesInActions(SASNumericCondition *c,
                                        std::vector<TVariable> *vars);
  void computeNumericVariablesInActions(SASNumericExpression *e,
                                        std::vector<TVariable> *vars);
  void computePermanentMutex();
  void postProcessActions();
  void addToRequirers(TVariable v, TValue val, SASAction *a);
  void addToProducers(TVariable v, TValue val, SASAction *a);
  void addToCondProducers(TVariable v, TValue val, SASAction *a,
                          unsigned int eff);
  // void computeInitialActionsCost(bool keepStaticData);
  // float computeActionCost(SASAction* a, float* numState, float makespan);
  float evaluateNumericExpression(SASNumericExpression *e, float *s,
                                  float duration);
  void updateNumericState(float *s, SASNumericEffect *e, float duration);
  float getActionDuration(SASAction *a, float *s);
  bool holdsNumericCondition(SASNumericCondition &cond, float *s,
                             float duration);
  inline float evaluateMetric(float *numState, float makespan) {
    return evaluateMetric(&metric, numState, makespan);
  }
  inline bool hasPermanentMutexAction() {
    return !permanentMutexActions.empty();
  }
  std::vector<TVarValue> *getListOfGoals();
  std::string toString();
  inline static std::string toStringTime(char time) {
    switch (time) {
    case 'S':
      return "at start";
    case 'E':
      return "at end";
    case 'A':
      return "over all";
    default:
      return "";
    }
  }
  inline static std::string toStringComparator(char cmp) {
    switch (cmp) {
    case '=':
      return "=";
    case '<':
      return "<";
    case 'L':
      return "<=";
    case '>':
      return ">";
    case 'G':
      return ">=";
    default:
      return "!=";
    }
  }
  inline static std::string toStringAssignment(char op) {
    switch (op) {
    case '=':
      return "assign";
    case '+':
      return "increase";
    case '-':
      return "decrease";
    case '*':
      return "scale-up";
    default:
      return "scale-down";
    }
  }
  std::string toStringCondition(SASCondition &c);
  std::string toStringPreference(SASPreference *pref,
                                 std::vector<SASControlVar> *controlVars);
  std::string toStringGoalDescription(SASGoalDescription *g,
                                      std::vector<SASControlVar> *controlVars);
  std::string toStringConstraint(SASConstraint *c,
                                 std::vector<SASControlVar> *controlVars);
  std::string toStringMetric(SASMetric *m);
  std::string toStringAction(SASAction &a);
  void addGoalDeadline(float time, TVarValue goal);
  inline bool areGoalDeadlines() { return !goalDeadlines.empty(); }
  inline std::vector<GoalDeadline> *getGoalDeadlines() {
    return &goalDeadlines;
  };
};

#endif
