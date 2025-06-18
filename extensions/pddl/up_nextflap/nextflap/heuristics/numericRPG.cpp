#include "numericRPG.h"
using namespace std;

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Computes a numeric relaxed plan based on fluent      */
/* intervals for heuristic plan evaluation.             */
/********************************************************/

// #define NUMRPG_DEBUG

// Constructor
NumericRPG::NumericRPG(TState *fs, std::vector<SASAction *> *tilActions,
                       SASTask *task, int limit) {
  this->task = task;
  this->limit = limit > 100 ? 100 : limit;
  initialize();
  createFirstFluentLevel(fs, tilActions);
  createFirstActionLevel();
  expand();
}

// Graph initialization
void NumericRPG::initialize() {
  int numVars = task->variables.size();
  int numValues = task->values.size();
  int numNumVars = task->numVariables.size();
  literalLevel.resize(numVars);
  for (int i = 0; i < numVars; i++) {
    literalLevel[i].resize(numValues, MAX_INT32);
  }
  actionLevel.resize(task->actions.size());
  numVarProducers.resize(numNumVars);
  numVarValue.resize(numNumVars);
  for (SASAction &a : task->goals)
    remainingGoals.push_back(&a);
  goalLevel.resize(task->goals.size(), MAX_INT32);
}

// Build the first fluent level of the graph
void NumericRPG::createFirstFluentLevel(TState *fs,
                                        std::vector<SASAction *> *tilActions) {
#ifdef NUMRPG_DEBUG
  cout << "L0" << endl;
#endif
  // Propositional values
  for (unsigned int i = 0; i < fs->numSASVars; i++) {
    TValue v = fs->state[i];
    literalLevel[i][v] = 0;
#ifdef NUMRPG_DEBUG
    cout << task->variables[i].name << "=" << task->values[v].name << endl;
#endif
  }
  // Numeric values
  for (unsigned int i = 0; i < fs->numNumVars; i++) {
    numVarValue[i].minValue = fs->minState[i];
    numVarValue[i].maxValue = fs->maxState[i];
#ifdef NUMRPG_DEBUG
    cout << "* " << task->numVariables[i].name << "=[" << fs->minState[i] << ","
         << fs->maxState[i] << "]" << endl;
#endif
  }
  // TIL
  if (tilActions != nullptr) {
    for (SASAction *a : *tilActions) {
      std::vector<TNumVarChange> v;
      IntervalCalculations ic(a, 0, this, task);
      ic.applyEndEffects(&v, nullptr);
      for (SASCondition &c : a->endEff) {
        literalLevel[c.var][c.value] = 0;
      }
      for (TNumVarChange &c : v) {
        updateNumericValueInterval(c.v, c.min, c.max);
      }
    }
  }
}

// Updates the numeric interval of a variable
void NumericRPG::updateNumericValueInterval(int var, float minValue,
                                            float maxValue) {
  if (minValue < numVarValue[var].minValue) {
    numVarValue[var].minValue = minValue;
  }
  if (maxValue > numVarValue[var].maxValue) {
    numVarValue[var].maxValue = maxValue;
  }
}

// Build the first action level of the graph
void NumericRPG::createFirstActionLevel() {
#ifdef NUMRPG_DEBUG
  cout << "A0" << endl;
#endif
  int i = 0;
  while (i < remainingGoals.size()) {
    SASAction *a = remainingGoals[i];
    if (isApplicable(a, 0)) {
      IntervalCalculations ic(a, 0, this, task);
      bool *hold = calculateCondEffHold(a, 0, ic);
      if (ic.supportedNumericStartConditions(hold)) {
#ifdef NUMRPG_DEBUG
        cout << a->name << endl;
#endif
        programActionEffects(a, 1);
        goalLevel[a->index] = 0;
        remainingGoals.erase(remainingGoals.begin() + i);
      } else
        i++;
      if (hold != nullptr)
        delete[] hold;
    } else
      i++;
  }
  for (SASAction &a : task->actions) {
    if (isApplicable(&a, 0)) {
      programActionEffects(&a, 1);
    }
  }
}

// Check if an action can be applied in the given level of the graph
bool NumericRPG::isApplicable(SASAction *a, int level) {
  for (SASCondition &c : a->startCond) {
    if (literalLevel[c.var][c.value] > level)
      return false;
  }
  for (SASCondition &c : a->overCond) {
    if (literalLevel[c.var][c.value] > level)
      return false;
  }
  for (SASCondition &c : a->endCond) {
    if (literalLevel[c.var][c.value] > level)
      return false;
  }
  return true;
}

bool *NumericRPG::calculateCondEffHold(SASAction *a, int level,
                                       IntervalCalculations &ic) {
  int size = a->conditionalEff.size();
  if (size == 0)
    return nullptr;
  bool *hold = new bool[size];
  for (int i = 0; i < size; i++) {
    hold[i] = checkCondEffectHold(a->conditionalEff[i], level, ic);
  }
  return hold;
}

bool NumericRPG::checkCondEffectHold(SASConditionalEffect &e, int level,
                                     IntervalCalculations &ic) {
  for (SASCondition &c : e.startCond) {
    if (literalLevel[c.var][c.value] > level)
      return false;
  }
  for (SASCondition &c : e.endCond) {
    if (literalLevel[c.var][c.value] > level)
      return false;
  }
  for (SASNumericCondition &c : e.startNumCond) {
    if (!ic.supportedCondition(&c))
      return false;
  }
  for (SASNumericCondition &c : e.endNumCond) {
    if (!ic.supportedCondition(&c))
      return false;
  }
  return true;
}

// Programs the action effects (to be added later in the graph)
void NumericRPG::programActionEffects(SASAction *a, int level) {
  std::vector<TNumVarChange> svc, evc;
  IntervalCalculations ic(a, level, this, task);
  if (!ic.supportedNumericStartConditions(nullptr))
    return;
  bool *holdCondPrec = calculateCondEffHold(a, level, ic);
  ic.applyStartEffects(&svc, holdCondPrec);
  ic.applyEndEffects(&evc, holdCondPrec);
  if (!ic.supportedNumericEndConditions(nullptr)) {
    if (holdCondPrec != nullptr)
      delete[] holdCondPrec;
    return;
  }
  bool newEffects = false;
  for (SASCondition &c : a->startEff) {
    if (literalLevel[c.var][c.value] > level) {
      literalLevel[c.var][c.value] = level;
      nextLevel.emplace_back(c.var, c.value, a);
      newEffects = true;
#ifdef NUMRPG_DEBUG
      cout << "\tEffect: " << task->variables[c.var].name << "="
           << task->values[c.value].name << endl;
#endif
    }
  }
  for (SASCondition &c : a->endEff) {
    if (literalLevel[c.var][c.value] > level) {
      literalLevel[c.var][c.value] = level;
      nextLevel.emplace_back(c.var, c.value, a);
      newEffects = true;
#ifdef NUMRPG_DEBUG
      cout << "\tEffect: " << task->variables[c.var].name << "="
           << task->values[c.value].name << endl;
#endif
    }
  }
  for (TNumVarChange &e : svc) {
    programNumericEffect(e.v, level, e.min, e.max, a);
    if (nextLevel.size() > 0 && nextLevel.back().a == a)
      newEffects = true;
  }
  for (TNumVarChange &e : evc) {
    programNumericEffect(e.v, level, e.min, e.max, a);
    if (nextLevel.size() > 0 && nextLevel.back().a == a)
      newEffects = true;
  }
  if (!a->conditionalEff.empty()) {
    for (unsigned int i = 0; i < a->conditionalEff.size(); i++) {
      if (holdCondPrec[i]) {
        SASConditionalEffect &e = a->conditionalEff[i];
        for (SASCondition &c : e.startEff) {
          if (literalLevel[c.var][c.value] > level) {
            literalLevel[c.var][c.value] = level;
            nextLevel.emplace_back(c.var, c.value, a);
            newEffects = true;
#ifdef NUMRPG_DEBUG
            cout << "\tEffect: " << task->variables[c.var].name << "="
                 << task->values[c.value].name << endl;
#endif
          }
        }
        for (SASCondition &c : e.endEff) {
          if (literalLevel[c.var][c.value] > level) {
            literalLevel[c.var][c.value] = level;
            nextLevel.emplace_back(c.var, c.value, a);
            newEffects = true;
#ifdef NUMRPG_DEBUG
            cout << "\tEffect: " << task->variables[c.var].name << "="
                 << task->values[c.value].name << endl;
#endif
          }
        }
      }
    }
    delete[] holdCondPrec;
  }
  if (newEffects) { // Action generates new values
    if (actionLevel[a->index].empty() &&
        (a->endNumEff.size() > 0 || a->startNumEff.size() > 0))
      achievedNumericActions.push_back(a);
    actionLevel[a->index].push_back(level -
                                    1); // Action added to the current level
#ifdef NUMRPG_DEBUG
    // cout << "\tAction added to level" << endl;
    cout << a->name << endl;
#endif
  } else if (actionLevel[a->index].empty() && a->endNumEff.empty() &&
             a->startNumEff.empty()) {
    // Action does not produces new values, but appears the first time and has
    // no numeric effects -> add to the RPG not to check it again
    actionLevel[a->index].push_back(level -
                                    1); // Action added to the current level
#ifdef NUMRPG_DEBUG
    // cout << "\tAction added to level" << endl;
    cout << a->name << endl;
#endif
  }
}

// Programs a numeric effect
void NumericRPG::programNumericEffect(TVariable v, int level, TFloatValue min,
                                      TFloatValue max, SASAction *a) {
  TFloatValue currentMin = numVarValue[v].minValue,
              currentMax = numVarValue[v].maxValue;
  if (min < currentMin || max > currentMax) {
    min = std::min(min, currentMin);
    max = std::max(max, currentMax);
    nextLevel.emplace_back(v, min, max, a);
#ifdef NUMRPG_DEBUG
    cout << "\tEffect: " << task->numVariables[v].name << "=[" << min << ", "
         << max << "]" << endl;
#endif
  }
}

// Graph expansion
void NumericRPG::expand() {
  int currentLevel = 0;
  std::unordered_set<int> checkedActions;
  while (remainingGoals.size() > 0 && nextLevel.size() > 0) {
    currentLevel++;
#ifdef NUMRPG_DEBUG
    cout << "L" << currentLevel << endl;
#endif
    if (!updateNumericValues(currentLevel)) // Update numeric values
      break;                                // Unreachable goals
    int i = 0;
    while (i < remainingGoals.size()) {
      if (checkGoal(remainingGoals[i], currentLevel)) { // Goal achieved
        remainingGoals.erase(remainingGoals.begin() + i);
      } else
        i++;
    }
    if (remainingGoals.empty())
      break;
#ifdef NUMRPG_DEBUG
    cout << "A" << currentLevel << endl;
#endif

    for (SASAction *a : achievedNumericActions) {
      programActionEffects(a, currentLevel + 1);
      checkedActions.insert(a->index);
    }

    for (TVarValue vv :
         reachedValues) { // Add actions that require this proposition
      for (SASAction *a : task->requirers[task->getVariableIndex(vv)]
                                         [task->getValueIndex(vv)]) {
        if (checkedActions.find(a->index) == checkedActions.end()) {
          checkAction(a, currentLevel);
          checkedActions.insert(a->index);
        }
      }
    }
    for (const TVariable &v :
         reachedNumValues) { // Add actions that need this numeric value
      for (SASAction *a : task->numRequirers[v]) {
        if (checkedActions.find(a->index) == checkedActions.end()) {
          checkAction(a, currentLevel);
          checkedActions.insert(a->index);
        }
      }
    }
    checkedActions.clear();
  }
#ifdef NUMRPG_DEBUG
  cout << "Remaining goals: " << remainingGoals.size() << endl;
#endif
}

// Updates the intervals of the numeric variables every level
bool NumericRPG::updateNumericValues(int level) {
  bool onlyNumericVariables = true;
  reachedValues.clear();
  reachedNumValues.clear();
  for (NumericRPGEffect &c : nextLevel) {
    if (c.numeric) {
      bool changeMin = c.minValue < numVarValue[c.var].minValue;
      bool changeMax = c.maxValue > numVarValue[c.var].maxValue;
      if (changeMin || changeMax) {
        numVarProducers[c.var].resize(level);
        NumericRPGproducers &prod = numVarProducers[c.var][level - 1];
        reachedNumValues.insert(c.var);
        if (changeMin) {
          numVarValue[c.var].minValue = c.minValue;
          prod.minProducer = c.a;
          prod.minValue = c.minValue;
        }
        if (changeMax) {
          numVarValue[c.var].maxValue = c.maxValue;
          prod.maxProducer = c.a;
          prod.maxValue = c.maxValue;
        }
      }
    } else {
      onlyNumericVariables = false;
      reachedValues.push_back(task->getVariableValueCode(c.var, c.value));
#ifdef NUMRPG_DEBUG
      cout << task->variables[c.var].name << "=" << task->values[c.value].name
           << endl;
#endif
    }
  }
#ifdef NUMRPG_DEBUG
  for (int i = 0; i < numVarValue.size(); i++)
    cout << task->numVariables[i].name << "=[" << numVarValue[i].minValue << ","
         << numVarValue[i].maxValue << "]" << endl;
#endif
  nextLevel.clear();
  if (onlyNumericVariables) { // Only numeric changes -> check if there are
                              // still unreached actions that need them
    if (--limit <= 0)
      return false;
    for (TVariable v : reachedNumValues) {
      for (SASAction *a : task->numRequirers[v]) {
        if (actionLevel[a->index].empty())
          return true;
      }
      for (SASAction *g : task->numGoalRequirers[v]) {
        if (goalLevel[g->index] == MAX_INT32)
          return true;
      }
    }
    return false; // Stop expansion -> all the actions that require these
                  // variables are already in the RPG
  } else
    return true;
}

// Check if an action can be inserted in the graph
void NumericRPG::checkAction(SASAction *a, int level) {
  if (actionLevel[a->index].size() > 0 && a->endNumEff.empty() &&
      a->startNumEff.empty())
    return; // Action already in the RPG without numeric effects
  if (!isApplicable(a, level))
    return; // Action not applicable
#ifdef NUMRPG_DEBUG
    // cout << "Checking " << a->name << endl;
#endif
  programActionEffects(a, level + 1);
}

// Checks if a goal is reached
bool NumericRPG::checkGoal(SASAction *a, int level) {
  if (!isApplicable(a, level))
    return false; // Action not applicable
  IntervalCalculations ic(a, level, this, task);
  if (!ic.supportedNumericStartConditions(nullptr))
    return false;
#ifdef NUMRPG_DEBUG
  cout << "Goal " << a->index << " achieved" << endl;
#endif
  goalLevel[a->index] = level;
  return true;
}

// Heuristic evaluation: length of the relaxed plan
int NumericRPG::evaluate() {
  if (remainingGoals.size() > 0)
    return MAX_UINT16;
  int h = 0, level;
  for (SASAction &g : task->goals) {
    addSubgoals(&g, goalLevel[g.index], nullptr);
  }
  SASAction *a;
  while (openConditions.size() > 0) {
    NumericRPGCondition *c = (NumericRPGCondition *)openConditions.poll();
    if (c->type == 'V') {
      a = searchBestAction(c->var, c->value, c->level, &level);
    } else {
      level = c->level;
      a = c->producer;
    }
    if (a != nullptr) {
      h++;
      addSubgoals(a, level, c->type != 'V' ? c : nullptr);
    }
    delete c;
  }
#ifdef NUMRPG_DEBUG
  cout << "H = " << h << endl;
#endif
  return h;
}

// Evaluation of the initial plan
int NumericRPG::evaluateInitialPlan(/*bool* usefulActions*/) {
  if (remainingGoals.size() > 0)
    return MAX_UINT16;
  int h = 0, level;
  for (SASAction &g : task->goals) {
    addSubgoals(&g, goalLevel[g.index], nullptr);
  }
  SASAction *a;
  while (openConditions.size() > 0) {
    NumericRPGCondition *c = (NumericRPGCondition *)openConditions.poll();
#ifdef NUMRPG_DEBUG
    cout << "Condition: " << task->variables[c->var].name << "="
         << task->values[c->value].name << endl;
#endif
    if (c->type == 'V') {
      a = searchBestAction(c->var, c->value, c->level, &level);
    } else {
      level = c->level;
      a = c->producer;
    }
    if (a != nullptr) {
      h++;
      // usefulActions[a->index] = true;
      addSubgoals(a, level, c->type != 'V' ? c : nullptr);
    }
    delete c;
  }
#ifdef NUMRPG_DEBUG
  cout << "H = " << h << endl;
#endif
  return h;
}

// Add the conditions of an action as new subgoals for the relaxed plan
void NumericRPG::addSubgoals(SASAction *a, int level, NumericRPGCondition *cp) {
#ifdef NUMRPG_DEBUG
  cout << "Adding subgoals of action " << a->name << endl;
#endif
  for (SASCondition &c : a->startCond)
    addSubgoal(&c);
  for (SASCondition &c : a->overCond)
    addSubgoal(&c);
  for (SASCondition &c : a->endCond)
    addSubgoal(&c);
  std::vector<NumericRPGCondition *> numCond;
  for (SASNumericCondition &c : a->startNumCond)
    addSubgoal(a, &c, level, &numCond);
  for (SASNumericCondition &c : a->overNumCond)
    addSubgoal(a, &c, level, &numCond);
  for (SASNumericCondition &c : a->endNumCond)
    addSubgoal(a, &c, level, &numCond);
  bool needToAddNumVarCond = cp != nullptr;
  for (NumericRPGCondition *c : numCond) {
    openConditions.add(c);
    if (needToAddNumVarCond &&
        (c->level == level - 1 || (c->type != 'V' && c->var == cp->var)))
      needToAddNumVarCond = false;
#ifdef NUMRPG_DEBUG
    cout << "* Level " << (c->level + 1) << ": "
         << task->numVariables[c->var].name << " (" << c->type << ")" << endl;
#endif
  }
  if (needToAddNumVarCond) {
    numCond.clear();
    int varLevel = cp->type == '+' ? findMaxNumVarLevel(cp->var, level)
                                   : findMinNumVarLevel(cp->var, level);
    if (varLevel >= 0) {
      addNumericSubgoal(cp->var, varLevel, cp->type == '+', &numCond);
      for (NumericRPGCondition *c : numCond) {
        openConditions.add(c);
#ifdef NUMRPG_DEBUG
        cout << "* Level " << (c->level + 1) << ": "
             << task->numVariables[c->var].name << " (" << c->type << ")"
             << endl;
#endif
      }
    }
  }
}

// Add the given condition of an action as new subgoal for the relaxed plan
void NumericRPG::addSubgoal(SASCondition *c) {
  int level = literalLevel[c->var][c->value];
  if (level > 0) {                      // Not solved yet
    literalLevel[c->var][c->value] = 0; // Not to repeat it again
    openConditions.add(new NumericRPGCondition(c, level));
#ifdef NUMRPG_DEBUG
    cout << "* Level " << level << ": " << task->variables[c->var].name << "="
         << task->values[c->value].name << endl;
#endif
  }
}

// Add the given numeric condition of an action as new subgoal for the relaxed
// plan
void NumericRPG::addSubgoal(SASAction *a, SASNumericCondition *c, int level,
                            std::vector<NumericRPGCondition *> *numCond) {
  switch (c->comp) {
  case '-':
    break;
  case '>': // >
  case 'G': // x >= y
    addMaxValueSubgoal(a, &(c->terms.at(0)), level, numCond);
    addMinValueSubgoal(a, &(c->terms.at(1)), level, numCond);
    break;
  case '<':
  case 'L':
    addMinValueSubgoal(a, &(c->terms.at(0)), level, numCond);
    addMaxValueSubgoal(a, &(c->terms.at(1)), level, numCond);
    break;
  default:
    addMinValueSubgoal(a, &(c->terms.at(0)), level, numCond);
    addMaxValueSubgoal(a, &(c->terms.at(0)), level, numCond);
    addMinValueSubgoal(a, &(c->terms.at(1)), level, numCond);
    addMaxValueSubgoal(a, &(c->terms.at(1)), level, numCond);
  }
}

// Add the given (maximize) numeric condition of an action as new subgoal for
// the relaxed plan
void NumericRPG::addMaxValueSubgoal(
    SASAction *a, SASNumericExpression *e, int level,
    std::vector<NumericRPGCondition *> *numCond) {
  if (e->type == 'V') {
    int varLevel = findMaxNumVarLevel(e->var, level);
    if (varLevel >= 0) {
      addNumericSubgoal(e->var, varLevel, true, numCond);
    }
  } else {
    for (SASNumericExpression &t : e->terms) {
      addMaxValueSubgoal(a, &t, level, numCond);
    }
  }
}

// Add the given (minimize) numeric condition of an action as new subgoal for
// the relaxed plan
void NumericRPG::addMinValueSubgoal(
    SASAction *a, SASNumericExpression *e, int level,
    std::vector<NumericRPGCondition *> *numCond) {
  if (e->type == 'V') {
    int varLevel = findMinNumVarLevel(e->var, level);
    if (varLevel > 0) {
      addNumericSubgoal(e->var, varLevel, false, numCond);
    }
  } else {
    for (SASNumericExpression &t : e->terms) {
      addMaxValueSubgoal(a, &t, level, numCond);
    }
  }
}

// Add the given condition of an action as new subgoal for the relaxed plan
void NumericRPG::addNumericSubgoal(
    TVariable v, int level, bool max,
    std::vector<NumericRPGCondition *> *numCond) {
  TVarValue vv = task->getVariableValueCode(v, level);
  if (numericSubgoals.find(vv) != numericSubgoals.end())
    return;
  numericSubgoals.insert(vv);
  NumericRPGproducers &prod = numVarProducers[v][level];
  SASAction *a = max ? prod.maxProducer : prod.minProducer;
  numCond->push_back(new NumericRPGCondition(v, max, level, a));
}

// Searches the best action to support the condition
SASAction *NumericRPG::searchBestAction(TVariable v, TValue value, int level,
                                        int *actionLevel) {
  SASAction *best = nullptr;
  for (SASAction *a : task->producers[v][value]) {
    int prodActionLevel = findLevel(a->index, level);
    if (prodActionLevel != -1) {
      if (prodActionLevel == 0) {
        *actionLevel = 0;
        return a;
      } else if (best == nullptr || prodActionLevel < *actionLevel) {
        best = a;
        *actionLevel = prodActionLevel;
      }
    }
  }
  for (SASConditionalProducer &cp : task->condProducers[v][value]) {
    int prodActionLevel = findLevel(cp.a->index, level);
    if (prodActionLevel != -1) {
      if (prodActionLevel == 0) {
        *actionLevel = 0;
        return cp.a;
      } else if (best == nullptr || prodActionLevel < *actionLevel) {
        best = cp.a;
        *actionLevel = prodActionLevel;
      }
    }
  }
  return best;
}

// Check the last level (before maxLevel) where v changes its lower value
int NumericRPG::findMinNumVarLevel(TVariable v, int maxLevel) {
  std::vector<NumericRPGproducers> &prod = numVarProducers[v];
  int level = maxLevel;
  if (prod.size() < level)
    level = (int)prod.size();
  for (int i = level - 1; i >= 0; i--) {
    if (prod[i].minProducer != nullptr)
      return i;
  }
  return -1;
}

// Check the last level (before maxLevel) where v changes its higher value
int NumericRPG::findMaxNumVarLevel(TVariable v, int maxLevel) {
  std::vector<NumericRPGproducers> &prod = numVarProducers[v];
  int level = (int)prod.size();
  if (level > maxLevel)
    level = maxLevel;
  for (int i = level - 1; i >= 0; i--) {
    if (prod[i].maxProducer != nullptr)
      return i;
  }
  return -1;
}

// Check the last level (before maxLevel) where v changes its value
int NumericRPG::findLevel(int actionIndex, int maxLevel) {
  std::vector<int> &levels = actionLevel[actionIndex];
  for (int i = (int)levels.size() - 1; i >= 0; i--) {
    if (levels[i] < maxLevel)
      return levels[i];
  }
  return -1;
}

// Returns the lower value of a variable
TFloatValue NumericRPG::getMinValue(TVariable v, int numState) {
  return numVarValue[v].minValue;
}

// Returns the higher value of a variable
TFloatValue NumericRPG::getMaxValue(TVariable v, int numState) {
  return numVarValue[v].maxValue;
}
