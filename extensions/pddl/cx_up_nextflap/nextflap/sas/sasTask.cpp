/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Finite-domain task representation.                   */
/********************************************************/

#include "sasTask.h"
#include <limits>
#include <time.h>
using namespace std;

/********************************************************/
/* CLASS: SASValue                                   */
/********************************************************/

string SASValue::toString() { return to_string(index) + ":" + name; }

/********************************************************/
/* CLASS: SASVariable                                   */
/********************************************************/

void SASVariable::addPossibleValue(unsigned int value) {
  possibleValues.push_back(value);
}

void SASVariable::addInitialValue(unsigned int sasValue, bool isTrue,
                                  float timePoint) {
  unsigned int index = numeric_limits<unsigned int>::max();
  for (unsigned int i = 0; i < possibleValues.size(); i++)
    if (possibleValues[i] == sasValue) {
      index = i;
      break;
    }
  if (index == numeric_limits<unsigned int>::max()) {
    throwError("Invalid intial value " + std::to_string(sasValue) +
               " for variable " + name);
  }
  if (isTrue) {
    value.push_back(sasValue);
    time.push_back(timePoint);
  } else if (possibleValues.size() == 2) {
    index = index == 0 ? 1 : 0;
    value.push_back(possibleValues[index]);
    time.push_back(timePoint);
  } else {
    throwError("Unable to translate negated initial value for variable " +
               name);
  }
}

string SASVariable::toString() { return to_string(index) + ":" + name; }

string SASVariable::toString(vector<SASValue> &values) {
  string s = toString();
  for (unsigned int i = 0; i < possibleValues.size(); i++)
    s += "\n* " + values[possibleValues[i]].name;
  return s;
}

string SASVariable::toStringInitialState(vector<SASValue> &values) {
  string s = "";
  for (unsigned int i = 0; i < value.size(); i++) {
    if (i > 0)
      s += " ";
    s += "(at " + to_string(time[i]) + " (= " + name + " " +
         values[value[i]].name + "))";
  }
  if (value.size() == 0)
    s = "Uninitialized: " + name;
  return s;
}

unsigned int SASVariable::getOppositeValue(unsigned int v) {
  if (possibleValues.size() != 2) {
    if (possibleValues.size() == 3) {
      int undef = getPossibleValueIndex(SASTask::OBJECT_UNDEFINED);
      if (undef != -1 && v != SASTask::OBJECT_UNDEFINED) {
        unsigned int pv1 = possibleValues[0];
        if (pv1 == SASTask::OBJECT_UNDEFINED)
          pv1 = possibleValues[1];
        unsigned int pv2 = possibleValues[2];
        if (pv2 == SASTask::OBJECT_UNDEFINED)
          pv2 = possibleValues[1];
        if (pv1 == v)
          return pv2;
        if (pv2 == v)
          return pv1;
      }
    }
    throwError("Unable to translate negated initial value for variable " +
               name);
  }
  if (possibleValues[0] == v)
    return possibleValues[1];
  if (possibleValues[1] == v)
    return possibleValues[0];
  throwError("Invalid value " + std::to_string(v) + " for variable " + name);
  return 0;
}

TVarValue SASVariable::getInitialStateValue() {
  for (unsigned int i = 0; i < time.size(); i++)
    if (time[i] == 0)
      return value[i];
  return MAX_INT32;
}

int SASVariable::getPossibleValueIndex(unsigned int value) {
  for (unsigned int i = 0; i < possibleValues.size(); i++) {
    if (possibleValues[i] == value)
      return (int)i;
  }
  return -1;
}

/********************************************************/
/* CLASS: NumericVariable                               */
/********************************************************/

void NumericVariable::addInitialValue(float value, float time) {
  for (unsigned int i = 0; i < this->value.size(); i++) {
    if (time == this->time[i]) {
      if (this->value[i] == value)
        break;
      throwError("Contradictory value " + std::to_string(value) + " in time " +
                 std::to_string(time) + " for variable " + name);
    }
  }
  this->value.push_back(value);
  this->time.push_back(time);
}

string NumericVariable::toString() { return to_string(index) + ":" + name; }

string NumericVariable::toStringInitialState() {
  string s = "";
  for (unsigned int i = 0; i < value.size(); i++) {
    if (i > 0)
      s += " ";
    s += "(at " + to_string(time[i]) + " (= " + name + " " +
         to_string(value[i]) + "))";
  }
  if (value.size() == 0)
    s = "Uninitialized: " + name;
  return s;
}

float NumericVariable::getInitialStateValue() {
  for (unsigned int i = 0; i < time.size(); i++)
    if (time[i] == 0)
      return value[i];
  return 0.0f;
}

/********************************************************/
/* CLASS: SASNumericExpression                          */
/********************************************************/

string SASNumericExpression::toString(vector<NumericVariable> *numVariables,
                                      vector<SASControlVar> *controlVars) {
  string s;
  switch (type) {
  case 'N':
    s = to_string(value);
    break;
  case 'V':
    s = (*numVariables)[var].name;
    break;
  case '+':
  case '-':
  case '/':
  case '*':
    s = "(";
    s += type;
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(numVariables, controlVars);
    s += ")";
    break;
  case '#':
    if (terms.size() == 0)
      s = "#t";
    else
      s = "(* #t " + terms[0].toString(numVariables, controlVars) + ")";
    break;
  case 'C':
    s = (*controlVars)[var].name;
    break;
  default:
    s = "?duration";
  }
  return s;
}

bool SASNumericExpression::equals(SASNumericExpression *e) {
  if (type != e->type)
    return false;
  switch (type) {
  case 'N':
    return value == e->value;
  case 'C':
  case 'V':
    return var == e->var;
  case '+':
  case '-':
  case '/':
  case '*':
  case '#':
    for (int i = 0; i < terms.size(); i++) {
      if (!terms[i].equals(&(e->terms[i])))
        return false;
    }
  }
  return true;
}

void SASNumericExpression::copyFrom(SASNumericExpression *e) {
  type = e->type;
  value = e->value;
  var = e->var;
  terms.clear();
  for (SASNumericExpression &t : e->terms) {
    terms.emplace_back();
    terms.back().copyFrom(&t);
  }
}

bool SASNumericExpression::findControlVar(int cv) {
  if (type == 'C')
    return (int)var == cv;
  for (SASNumericExpression &t : terms)
    if (t.findControlVar(cv))
      return true;
  return false;
}

bool SASNumericExpression::findFluent(TVariable v) {
  if (type == 'V')
    return var == v;
  for (SASNumericExpression &t : terms)
    if (t.findFluent(v))
      return true;
  return false;
}

void SASNumericExpression::getVariables(std::vector<TVariable> *vars) {
  if (type == 'V') {
    bool found = false;
    for (int i = 0; i < vars->size(); i++)
      if (vars->at(i) == var) {
        found = true;
        break;
      }
    if (!found)
      vars->push_back(var);
  } else {
    for (SASNumericExpression &t : terms)
      t.getVariables(vars);
  }
}

/********************************************************/
/* CLASS: SASDuration                                   */
/********************************************************/

string SASDuration::toString(vector<NumericVariable> *numVariables,
                             vector<SASControlVar> *controlVars) {
  string dur = "";
  for (SASDurationCondition &dc : conditions)
    dur += dc.toString(numVariables, controlVars) + "\n";
  return dur;
}

std::string
SASDurationCondition::toString(std::vector<NumericVariable> *numVariables,
                               std::vector<SASControlVar> *controlVars) {
  return SASTask::toStringTime(time) + "(" + SASTask::toStringComparator(comp) +
         " ?duration " + exp.toString(numVariables, controlVars) + ")";
}

/********************************************************/
/* CLASS: SASCondition                                  */
/********************************************************/

SASCondition::SASCondition(unsigned int var, unsigned int value) {
  this->var = var;
  this->value = value;
  this->isModified = false;
}

/********************************************************/
/* CLASS: SASNumericCondition                           */
/********************************************************/

string SASNumericCondition::toString(vector<NumericVariable> *numVariables,
                                     vector<SASControlVar> *controlVars) {
  string s = "(" + SASTask::toStringComparator(comp);
  for (unsigned int i = 0; i < terms.size(); i++)
    s += " " + terms[i].toString(numVariables, controlVars);
  return s + ")";
}

bool SASNumericCondition::equals(SASNumericCondition *c) {
  if (comp != c->comp || terms.size() != c->terms.size())
    return false;
  for (int i = 0; i < terms.size(); i++) {
    if (!terms[i].equals(&(c->terms[i])))
      return false;
  }
  return true;
}

void SASNumericCondition::copyFrom(SASNumericCondition *c) {
  comp = c->comp;
  terms.clear();
  for (SASNumericExpression &t : c->terms) {
    terms.emplace_back();
    terms.back().copyFrom(&t);
  }
}

void SASNumericCondition::reshape(int cv) {
  if (terms[1].findControlVar(cv))
    swapTerms();
  reshapeTerms(cv);
}

void SASNumericCondition::reshapeFluent(TVariable v) {
  if (terms[1].findFluent(v))
    swapTerms();
  reshapeFluentTerms(v);
}

void SASNumericCondition::getVariables(std::vector<TVariable> *vars) {
  vars->clear();
  for (SASNumericExpression &t : terms)
    t.getVariables(vars);
}

void SASNumericCondition::swapCondition() {
  if (comp == '<')
    comp = '>';
  else if (comp == 'L')
    comp = 'G';
  else if (comp == '>')
    comp = '<';
  else if (comp == 'G')
    comp = 'L';
}

void SASNumericCondition::swapTerms() {
  terms.emplace_back();
  terms.back().copyFrom(&terms[0]);
  terms.erase(terms.begin());
  swapCondition();
}

void SASNumericCondition::reshapeTerms(int cv) {
  SASNumericExpression *left = &terms[0], *right = &terms[1];
  char type = left->type;
  while (type == '+' || type == '-' || type == '*' || type == '/') {
    bool cvOnLeft = left->terms[0].findControlVar(cv);
    SASNumericExpression copyOfRight, copyOfLeft;
    copyOfRight.copyFrom(right);
    char rightOp;
    if (cvOnLeft) {
      // cv + exp comp right -> cv comp right - exp
      // cv - exp comp right -> cv comp right + exp
      // cv * exp comp right -> cv comp right / exp
      // cv / exp comp right -> cv comp right * exp
      if (type == '+')
        rightOp = '-';
      else if (type == '-')
        rightOp = '+';
      else if (type == '*')
        rightOp = '/';
      else
        rightOp = '*';
      right->type = rightOp;
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&(left->terms[1]));
      copyOfLeft.copyFrom(&(left->terms[0]));
      left->copyFrom(&copyOfLeft);
    } else if (type == '+' || type == '*') {
      // exp + cv comp right -> cv comp right - exp
      // exp * cv comp right -> cv comp right / exp
      rightOp = type == '+' ? '-' : '/';
      right->type = rightOp;
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&(left->terms[0]));
      copyOfLeft.copyFrom(&(left->terms[1]));
      left->copyFrom(&copyOfLeft);
    } else if (type == '-') {
      // exp - cv comp right -> cv swap_comp exp - right
      swapCondition();
      copyOfLeft.copyFrom(&(left->terms[0]));
      right->type = '-';
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfLeft);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      copyOfLeft.copyFrom(&(left->terms[1]));
      left->copyFrom(&copyOfLeft);
    } else {
      // exp / cv comp right -> exp comp right * cv -> swap
      right->type = '*';
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&(left->terms[1]));
      copyOfLeft.copyFrom(&(left->terms[0]));
      left->copyFrom(&copyOfLeft);
      swapTerms();
    }
    type = left->type;
  }
}

void SASNumericCondition::reshapeFluentTerms(TVariable v) {
  SASNumericExpression *left = &terms[0], *right = &terms[1];
  char type = left->type;
  while (type == '+' || type == '-' || type == '*' || type == '/') {
    bool vOnLeft = left->terms[0].findFluent(v);
    SASNumericExpression copyOfRight, copyOfLeft;
    copyOfRight.copyFrom(right);
    char rightOp;
    if (vOnLeft) {
      // v + exp comp right -> c comp right - exp
      // v - exp comp right -> c comp right + exp
      // v * exp comp right -> c comp right / exp
      // v / exp comp right -> c comp right * exp
      if (type == '+')
        rightOp = '-';
      else if (type == '-')
        rightOp = '+';
      else if (type == '*')
        rightOp = '/';
      else
        rightOp = '*';
      right->type = rightOp;
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&(left->terms[1]));
      copyOfLeft.copyFrom(&(left->terms[0]));
      left->copyFrom(&copyOfLeft);
    } else if (type == '+' || type == '*') {
      // exp + v comp right -> v comp right - exp
      // exp * v comp right -> v comp right / exp
      rightOp = type == '+' ? '-' : '/';
      right->type = rightOp;
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&(left->terms[0]));
      copyOfLeft.copyFrom(&(left->terms[1]));
      left->copyFrom(&copyOfLeft);
    } else if (type == '-') {
      // exp - v comp right -> v swap_comp exp - right
      swapCondition();
      copyOfLeft.copyFrom(&(left->terms[0]));
      right->type = '-';
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfLeft);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      copyOfLeft.copyFrom(&(left->terms[1]));
      left->copyFrom(&copyOfLeft);
    } else {
      // exp / c comp right -> exp comp right * c -> swap
      right->type = '*';
      right->terms.clear();
      right->terms.emplace_back();
      right->terms.back().copyFrom(&copyOfRight);
      right->terms.emplace_back();
      right->terms.back().copyFrom(&(left->terms[1]));
      copyOfLeft.copyFrom(&(left->terms[0]));
      left->copyFrom(&copyOfLeft);
      swapTerms();
    }
    type = left->type;
  }
}

/********************************************************/
/* CLASS: SASNumericEffect                              */
/********************************************************/

string SASNumericEffect::toString(vector<NumericVariable> *numVariables,
                                  vector<SASControlVar> *controlVars) {
  return "(" + SASTask::toStringAssignment(op) + " " +
         (*numVariables)[var].toString() + " " +
         exp.toString(numVariables, controlVars) + ")";
}

/********************************************************/
/* CLASS: SASTask                                       */
/********************************************************/

// Constructor
SASTask::SASTask() {
  createNewValue("<true>", FICTITIOUS_FUNCTION);
  createNewValue("<false>", FICTITIOUS_FUNCTION);
  createNewValue("<undefined>", FICTITIOUS_FUNCTION);
  requirers = nullptr;
  producers = nullptr;
  condProducers = nullptr;
  staticNumFunctions = nullptr;
  numGoalsInPlateau = 1;
  numRequirers = nullptr;
  numGoalRequirers = nullptr;
  initialState = nullptr;
  numInitialState = nullptr;
  numVarReqAtStart = nullptr;
  numVarReqAtEnd = nullptr;
  numVarReqGoal = nullptr;
}

SASTask::~SASTask() {
  if (requirers != nullptr) {
    for (int i = 0; i < variables.size(); i++) {
      delete[] requirers[i];
    }
    delete[] requirers;
  }
  if (producers != nullptr) {
    for (int i = 0; i < variables.size(); i++) {
      delete[] producers[i];
    }
    delete[] producers;
  }
  if (condProducers != nullptr) {
    for (int i = 0; i < variables.size(); i++) {
      delete[] condProducers[i];
    }
    delete[] condProducers;
  }
  if (initialState != nullptr)
    delete[] initialState;
  if (numInitialState != nullptr)
    delete[] numInitialState;
  if (staticNumFunctions != nullptr)
    delete[] staticNumFunctions;
  if (numRequirers != nullptr)
    delete[] numRequirers;
  if (numGoalRequirers != nullptr)
    delete[] numGoalRequirers;
  if (numVarReqAtStart != nullptr)
    delete[] numVarReqAtStart;
  if (numVarReqAtEnd != nullptr)
    delete[] numVarReqAtEnd;
  if (numVarReqGoal != nullptr)
    delete[] numVarReqGoal;
}

// Adds a mutex relationship between (var1, value1) and (var2, value2)
void SASTask::addMutex(unsigned int var1, unsigned int value1,
                       unsigned int var2, unsigned int value2) {
  mutex[getMutexCode(var1, value1, var2, value2)] = true;
  mutex[getMutexCode(var2, value2, var1, value1)] = true;
  // cout << "Mutex added: " << variables[var1].name << "=" <<
  // values[value1].name << " and " << 	variables[var2].name << "=" <<
  // values[value2].name << endl;
}

// Checks if (var1, value1) and (var2, value2) are mutex
bool SASTask::isMutex(unsigned int var1, unsigned int value1, unsigned int var2,
                      unsigned int value2) {
  return mutex.find(getMutexCode(var1, value1, var2, value2)) != mutex.end();
}

bool SASTask::isPermanentMutex(unsigned int var1, unsigned int value1,
                               unsigned int var2, unsigned int value2) {
  return permanentMutex.find(getMutexCode(var1, value1, var2, value2)) !=
         permanentMutex.end();
}

bool SASTask::isPermanentMutex(SASAction *a1, SASAction *a2) {
  uint64_t n = a1->index;
  n = (n << 32) + a2->index;
  return permanentMutexActions.find(n) != permanentMutexActions.end();
}

// Adds a new variable
SASVariable *SASTask::createNewVariable() {
  variables.emplace_back();
  SASVariable *v = &(variables.back());
  v->index = variables.size() - 1;
  v->name = "var" + to_string(v->index);
  return v;
}

// Adds a new variable
SASVariable *SASTask::createNewVariable(string name) {
  variables.emplace_back();
  SASVariable *v = &(variables.back());
  v->index = variables.size() - 1;
  v->name = name;
  return v;
}

// Adds a new value
unsigned int SASTask::createNewValue(string name, unsigned int fncIndex) {
  // if (fncIndex == FICTITIOUS_FUNCTION) cout << "NEW VALUE: " << name << endl;
  unordered_map<string, unsigned int>::const_iterator it =
      valuesByName.find(name);
  if (it != valuesByName.end())
    return it->second;
  values.emplace_back();
  SASValue *v = &(values.back());
  v->index = values.size() - 1;
  v->fncIndex = fncIndex;
  v->name = name;
  valuesByName[v->name] = v->index;
  return v->index;
}

// Adds a new value if it does not exist yet
unsigned int SASTask::findOrCreateNewValue(std::string name,
                                           unsigned int fncIndex) {
  std::unordered_map<string, unsigned int>::const_iterator value =
      valuesByName.find(name);
  if (value == valuesByName.end())
    return createNewValue(name, fncIndex);
  else
    return value->second;
}

// Return the index of a value through its name
unsigned int SASTask::getValueByName(const string &name) {
  return valuesByName[name];
}

// Adds a new numeric variable
NumericVariable *SASTask::createNewNumericVariable(string name) {
  numVariables.emplace_back();
  NumericVariable *v = &(numVariables.back());
  v->index = numVariables.size() - 1;
  v->name = name;
  return v;
}

// Adds a new action
SASAction *SASTask::createNewAction(string name, bool instantaneous, bool isTIL,
                                    bool isGoal) {
  actions.emplace_back(instantaneous, isTIL, isGoal);
  SASAction *a = &(actions.back());
  a->index = actions.size() - 1;
  a->name = name;
  a->isGoal = false;
  return a;
}

// Adds a new goal
SASAction *SASTask::createNewGoal() {
  goals.emplace_back(true, false, true);
  SASAction *a = &(goals.back());
  a->index = goals.size() - 1;
  a->name = "<goal>";
  a->isGoal = true;
  return a;
}

// Computes the initial state
void SASTask::computeInitialState() {
  initialState = new TValue[variables.size()];
  for (unsigned int i = 0; i < variables.size(); i++) {
    initialState[i] = variables[i].getInitialStateValue();
  }
  numInitialState = new float[numVariables.size()];
  for (unsigned int i = 0; i < numVariables.size(); i++) {
    numInitialState[i] = numVariables[i].getInitialStateValue();
  }
}

// Computes the list of actions that requires a (= var value)
void SASTask::computeRequirers() {
  requirers = new vector<SASAction *> *[variables.size()];
  for (unsigned int i = 0; i < variables.size(); i++) {
    requirers[i] = new vector<SASAction *>[values.size()];
  }
  unsigned int numActions = actions.size();
  for (unsigned int i = 0; i < numActions; i++) {
    SASAction *a = &(actions[i]);
    for (unsigned int j = 0; j < a->startCond.size(); j++)
      addToRequirers(a->startCond[j].var, a->startCond[j].value, a);
    for (unsigned int j = 0; j < a->overCond.size(); j++)
      addToRequirers(a->overCond[j].var, a->overCond[j].value, a);
    for (unsigned int j = 0; j < a->endCond.size(); j++)
      addToRequirers(a->endCond[j].var, a->endCond[j].value, a);
    if (a->startCond.empty() && a->overCond.empty() && a->endCond.empty()) {
      actionsWithoutConditions.push_back(a);
    }
    for (SASConditionalEffect &e : a->conditionalEff) {
      for (unsigned int j = 0; j < e.startCond.size(); j++)
        addToRequirers(e.startCond[j].var, e.startCond[j].value, a);
      for (unsigned int j = 0; j < e.endCond.size(); j++)
        addToRequirers(e.endCond[j].var, e.endCond[j].value, a);
    }
  }
}

// Adds a requirer, checking for no duplicates
void SASTask::addToRequirers(TVariable v, TValue val, SASAction *a) {
  std::vector<SASAction *> &req = requirers[v][val];
  for (unsigned int i = 0; i < req.size(); i++) {
    if (req[i] == a)
      return;
  }
  req.push_back(a);
}

// Computes the list of actions that produces (= var value)
void SASTask::computeProducers() {
  producers = new vector<SASAction *> *[variables.size()];
  condProducers = new vector<SASConditionalProducer> *[variables.size()];
  for (unsigned int i = 0; i < variables.size(); i++) {
    producers[i] = new vector<SASAction *>[values.size()];
    condProducers[i] = new vector<SASConditionalProducer>[values.size()];
  }
  unsigned int numActions = actions.size();
  for (unsigned int i = 0; i < numActions; i++) {
    SASAction *a = &(actions[i]);
    for (unsigned int j = 0; j < a->startEff.size(); j++)
      addToProducers(a->startEff[j].var, a->startEff[j].value, a);
    for (unsigned int j = 0; j < a->endEff.size(); j++)
      addToProducers(a->endEff[j].var, a->endEff[j].value, a);
    for (unsigned int k = 0; k < a->conditionalEff.size(); k++) {
      SASConditionalEffect &e = a->conditionalEff[k];
      for (unsigned int j = 0; j < e.startEff.size(); j++)
        addToCondProducers(e.startEff[j].var, e.startEff[j].value, a, k);
      for (unsigned int j = 0; j < e.endEff.size(); j++)
        addToCondProducers(e.endEff[j].var, e.endEff[j].value, a, k);
    }
  }
}

void SASTask::computeNumericVariablesInActions() {
  this->numVarReqAtStart = new std::vector<TVariable>[actions.size()];
  this->numVarReqAtEnd = new std::vector<TVariable>[actions.size()];
  this->numRequirers = new std::vector<SASAction *>[numVariables.size()];
  this->numVarReqGoal = new std::vector<TVariable>[goals.size()];
  this->numGoalRequirers = new std::vector<SASAction *>[numVariables.size()];
  for (SASAction &a : actions) {
    computeNumericVariablesInActions(a);
  }
  for (SASAction &a : goals) {
    computeNumericVariablesInGoals(a);
  }
}

void SASTask::computeNumericVariablesInActions(SASAction &a) {
  int i = a.index;
  for (SASNumericCondition &c : a.startNumCond) {
    computeNumericVariablesInActions(&c, &(numVarReqAtStart[i]));
  }
  for (SASNumericCondition &c : a.overNumCond) {
    computeNumericVariablesInActions(&c, &(numVarReqAtStart[i]));
    computeNumericVariablesInActions(&c, &(numVarReqAtEnd[i]));
  }
  for (SASNumericCondition &c : a.endNumCond) {
    computeNumericVariablesInActions(&c, &(numVarReqAtEnd[i]));
  }
  for (TVariable v : numVarReqAtStart[i]) {
    numRequirers[v].push_back(&a);
    // cout << "Action " << a.name << " requires at start " <<
    // numVariables[v].name << endl;
  }
  for (TVariable v : numVarReqAtEnd[i]) {
    bool found = false;
    for (int j = 0; j < numVarReqAtStart[i].size(); j++)
      if (numVarReqAtStart[i].at(j) == v) {
        found = true;
        break;
      }
    if (!found)
      numRequirers[v].push_back(&a);
    // cout << "Action " << a.name << " requires at end " <<
    // numVariables[v].name << endl;
  }
}

void SASTask::computeNumericVariablesInGoals(SASAction &a) {
  int i = a.index;
  for (SASNumericCondition &c : a.startNumCond) {
    computeNumericVariablesInActions(&c, &(numVarReqGoal[i]));
  }
  for (TVariable v : numVarReqGoal[i]) {
    numGoalRequirers[v].push_back(&a);
    // cout << "Action " << a.name << " requires at start " <<
    // numVariables[v].name << endl;
  }
}

void SASTask::computeNumericVariablesInActions(SASNumericCondition *c,
                                               std::vector<TVariable> *vars) {
  for (SASNumericExpression &e : c->terms) {
    computeNumericVariablesInActions(&e, vars);
  }
}

void SASTask::computeNumericVariablesInActions(SASNumericExpression *e,
                                               std::vector<TVariable> *vars) {
  if (e->type == 'V') {
    TVariable v = e->var;
    for (TVariable w : *vars)
      if (w == v)
        return;
    vars->push_back(v);
  } else if (e->type == '+' || e->type == '-' || e->type == '*' ||
             e->type == '/') {
    for (SASNumericExpression &ne : e->terms) {
      computeNumericVariablesInActions(&ne, vars);
    }
  }
}

void SASTask::computeMutexWithVarValues() {
  uint32_t vv1, vv2;
  std::unordered_map<uint64_t, bool>::const_iterator got = mutex.begin();
  std::unordered_map<uint32_t, std::vector<uint32_t> *>::const_iterator it;
  for (; got != mutex.end(); ++got) {
    uint64_t n = got->first;
    vv2 = n & 0xFFFFFFFF;
    vv1 = (uint32_t)(n >> 32);
    it = mutexWithVarValue.find(vv1);
    if (it == mutexWithVarValue.end()) {
      std::vector<uint32_t> *item = new std::vector<uint32_t>();
      item->push_back(vv2);
      mutexWithVarValue[vv1] = item;
    } else {
      it->second->push_back(vv2);
    }
  }
}

void SASTask::checkEffectReached(
    SASCondition *c, std::unordered_map<TVarValue, bool> *goals,
    std::unordered_map<TVarValue, bool> *visitedVarValue,
    std::vector<TVarValue> *state) {
  TVarValue code = getVariableValueCode(c->var, c->value);
  goals->erase(code);
  if (visitedVarValue->find(code) == visitedVarValue->end()) {
    (*visitedVarValue)[code] = true;
    state->push_back(code);
  }
}

void SASTask::checkReachability(TVarValue vv,
                                std::unordered_map<TVarValue, bool> *goals) {
  unsigned int numActions = actions.size();
  bool *visited = new bool[numActions];
  for (unsigned int i = 0; i < numActions; i++)
    visited[i] = false;
  std::vector<TVarValue> state;
  std::unordered_map<TVarValue, bool> visitedVarValue;
  state.push_back(vv);
  visitedVarValue[vv] = true;
  unsigned int start = 0;
  while (start < state.size() && !goals->empty()) {
    TVariable v = getVariableIndex(state[start]);
    TValue value = getValueIndex(state[start]);
    start++;
    std::vector<SASAction *> &req = requirers[v][value];
    // cout << variables[v].name << "=" << values[value].name << endl;
    for (unsigned int i = 0; i < req.size(); i++) {
      SASAction *a = req[i];
      if (!visited[a->index]) {
        visited[a->index] = true;
        // cout << " - " << a->name << endl;
        for (unsigned int j = 0; j < a->startEff.size(); j++)
          checkEffectReached(&(a->startEff[j]), goals, &visitedVarValue,
                             &state);
        for (unsigned int j = 0; j < a->endEff.size(); j++)
          checkEffectReached(&(a->endEff[j]), goals, &visitedVarValue, &state);
      }
    }
  }
  delete[] visited;
}

void SASTask::computePermanentMutex() {
  // clock_t tini = clock();
  computeMutexWithVarValues();
  std::unordered_map<uint32_t, std::vector<uint32_t> *>::const_iterator it;
  std::unordered_map<uint32_t, bool>::const_iterator ug;
  for (it = mutexWithVarValue.begin(); it != mutexWithVarValue.end(); ++it) {
    // cout << it->second->size() << endl;
    std::unordered_map<uint32_t, bool> goals;
    for (unsigned int i = 0; i < it->second->size(); i++) {
      goals[it->second->at(i)] = true;
    }
    checkReachability(it->first, &goals);
    for (ug = goals.begin(); ug != goals.end(); ++ug) {
      TMutex code = (((TMutex)it->first) << 32) + ug->first;
      permanentMutex[code] = true;
    }
  }
  if (permanentMutex.size() > 0) {
    unsigned int numActions = actions.size();
    for (unsigned int i = 0; i < numActions - 1; i++) {
      SASAction *a1 = &(actions[i]);
      for (unsigned int j = i + 1; j < numActions; j++) {
        if (checkActionMutex(a1, &(actions[j]))) {
          // cout << a1->name << " <- mutex -> " << actions[j].name << endl;
          uint64_t n = a1->index;
          n = (n << 32) + actions[j].index;
          permanentMutexActions[n] = true;
          n = actions[j].index;
          n = (n << 32) + a1->index;
          permanentMutexActions[n] = true;
        }
      }
    }
  }
  // cout << (float) (((int) (1000 * (clock() - tini)/(float)
  // CLOCKS_PER_SEC))/1000.0) << " sec." << endl;
}

void SASTask::postProcessActions() {
  int numDecEff = 0;
  for (SASAction &a : actions) {
    a.postProcess();

    int dec = 0;
    for (SASNumericEffect &e : a.startNumEff) {
      if (e.op == '-')
        dec++;
    }
    if (dec > numDecEff)
      numDecEff = dec;
  }
  for (SASAction &a : goals)
    a.postProcess();
}

bool SASTask::checkActionMutex(SASAction *a1, SASAction *a2) {
  return checkActionOrdering(a1, a2) && checkActionOrdering(a2, a1);
}

bool SASTask::checkActionOrdering(SASAction *a1, SASAction *a2) {
  for (unsigned int i = 0; i < a1->startEff.size(); i++) {
    TVariable v1 = a1->startEff[i].var;
    TValue value1 = a1->startEff[i].value;
    for (unsigned int j = 0; j < a2->startCond.size(); j++) {
      TVariable v2 = a2->startCond[j].var;
      TValue value2 = a2->startCond[j].value;
      if (isPermanentMutex(v1, value1, v2, value2))
        return true;
    }
    for (unsigned int j = 0; j < a2->overCond.size(); j++) {
      TVariable v2 = a2->overCond[j].var;
      TValue value2 = a2->overCond[j].value;
      if (isPermanentMutex(v1, value1, v2, value2))
        return true;
    }
    for (unsigned int j = 0; j < a2->endCond.size(); j++) {
      TVariable v2 = a2->endCond[j].var;
      TValue value2 = a2->endCond[j].value;
      if (isPermanentMutex(v1, value1, v2, value2))
        return true;
    }
  }
  for (unsigned int i = 0; i < a1->endEff.size(); i++) {
    TVariable v1 = a1->endEff[i].var;
    TValue value1 = a1->endEff[i].value;
    for (unsigned int j = 0; j < a2->startCond.size(); j++) {
      TVariable v2 = a2->startCond[j].var;
      TValue value2 = a2->startCond[j].value;
      if (isPermanentMutex(v1, value1, v2, value2))
        return true;
    }
    for (unsigned int j = 0; j < a2->overCond.size(); j++) {
      TVariable v2 = a2->overCond[j].var;
      TValue value2 = a2->overCond[j].value;
      if (isPermanentMutex(v1, value1, v2, value2))
        return true;
    }
    for (unsigned int j = 0; j < a2->endCond.size(); j++) {
      TVariable v2 = a2->endCond[j].var;
      TValue value2 = a2->endCond[j].value;
      if (isPermanentMutex(v1, value1, v2, value2))
        return true;
    }
  }
  return false;
}

void SASTask::addToCondProducers(TVariable v, TValue val, SASAction *a,
                                 unsigned int eff) {
  std::vector<SASConditionalProducer> &prod = condProducers[v][val];
  for (unsigned int i = 0; i < prod.size(); i++) {
    if (prod[i].a == a && prod[i].numEff == eff)
      return;
  }
  prod.emplace_back(a, eff);
}

// Adds a producer, checking for no duplicates
void SASTask::addToProducers(TVariable v, TValue val, SASAction *a) {
  std::vector<SASAction *> &prod = producers[v][val];
  for (unsigned int i = 0; i < prod.size(); i++) {
    if (prod[i] == a)
      return;
  }
  prod.push_back(a);
}

/* Computes the cost of the actions according to the metric (if possible)
// The cost of an action cannot be computed in the following cases:
// * The duration of the action depends on the state (numeric value of one or
more variables) and the metric depends on the plan duration.
// * A numeric variable (e.g. fuel-used) is modified through the action, and
this change of value depends on the value of another numeric
//   variable in the state (e.g. temperature), and the numeric variable
(fuel-used) is used in the metric. void SASTask::computeInitialActionsCost(bool
keepStaticData) { if (keepStaticData) {	// Static functions have not been
deleted, so it is necessary to check which ones are static staticNumFunctions =
new bool[numVariables.size()]; for (unsigned int i = 0; i < numVariables.size();
i++) { staticNumFunctions[i] = true;
                }
                for (unsigned int i = 0; i < actions.size(); i++) {
                        SASAction &a = actions[i];
                        for (unsigned int j = 0; i < a.startNumEff.size(); i++)
{ staticNumFunctions[a.startNumEff[j].var] = false;
                        }
                        for (unsigned int j = 0; i < a.endNumEff.size(); i++) {
                                staticNumFunctions[a.endNumEff[j].var] = false;
                        }
                }
        }
        else {
                staticNumFunctions = nullptr;
        }
        bool *variablesOnMetric = new bool[numVariables.size()];
        for (unsigned int i = 0; i < numVariables.size(); i++) {
                variablesOnMetric[i] = false;
        }
        variableCosts = false;
        metricDependsOnDuration = checkVariablesUsedInMetric(&metric,
variablesOnMetric); for (unsigned int i = 0; i < actions.size(); i++) {
                computeActionCost(&actions[i], variablesOnMetric);
                if (!actions[i].fixedCost) variableCosts = true;
        }
        for (unsigned int i = 0; i < goals.size(); i++) {
                goals[i].setGoalCost();
        }
        delete[] variablesOnMetric;
}
*/

// Check the numeric variables used in the metric function
// Return true if the metric depends on the plan duration
bool SASTask::checkVariablesUsedInMetric(SASMetric *m,
                                         bool *variablesOnMetric) {
  bool metricDependsOnDuration = false;
  switch (m->type) {
  case '+':
  case '-':
  case '*':
  case '/':
    for (unsigned int i = 0; i < m->terms.size(); i++) {
      if (checkVariablesUsedInMetric(&(m->terms[i]), variablesOnMetric)) {
        metricDependsOnDuration = true;
      }
    }
    break;
  case 'T':
    metricDependsOnDuration = true;
    break;
  case 'F':
    variablesOnMetric[m->index] = true;
    break;
  default:
    break;
  }
  return metricDependsOnDuration;
}

/* Computes the cost of the action according to the metric (if possible)
// The cost of an action cannot be computed in the following cases:
// * The duration of the action depends on the state (numeric value of one or
more variables) and the metric depends on the plan duration.
// * A numeric variable (e.g. fuel-used) is modified through the action, and
this change of value depends on the value of another numeric
//   variable in the state (e.g. temperature), and the numeric variable
(fuel-used) is used in the metric. void SASTask::computeActionCost(SASAction* a,
bool* variablesOnMetric) { a->fixedDuration = true; for (unsigned int i = 0; i <
a->duration.conditions.size(); i++) { if
(checkVariableExpression(&(a->duration.conditions[i].exp), nullptr)) {
                        a->fixedDuration = false;
                        break;
                }
        }
        if (a->fixedDuration) {
                for (unsigned int i = 0; i < a->duration.conditions.size(); i++)
{
                        a->fixedDurationValue.push_back(computeFixedExpression(&(a->duration.conditions[i].exp)));
                        //cout << "Duration of " << a->name << " is " <<
a->fixedDurationValue[i] << endl;
                }
        }
        a->fixedCost = a->fixedDuration || !metricDependsOnDuration;
        if (a->fixedCost) {
                for (unsigned int i = 0; i < a->startNumEff.size(); i++) {
                        if (checkVariableExpression(&(a->startNumEff[i].exp),
variablesOnMetric)) { a->fixedCost = false; break;
                        }
                }
                if (a->fixedCost) {
                        for (unsigned int i = 0; i < a->endNumEff.size(); i++) {
                                if
(checkVariableExpression(&(a->endNumEff[i].exp), variablesOnMetric)) {
                                        a->fixedCost = false;
                                        break;
                                }
                        }
                        if (a->fixedCost) {
                                a->fixedCostValue = computeActionCost(a,
numInitialState, 0);
                                //cout << " * Fixed cost is " <<
a->fixedCostValue << endl;
                        }
                }
        }
}
*/

/* Computes the cost of aplying an action in a given state
float SASTask::computeActionCost(SASAction* a, float* numState, float makespan)
{ float startMetricValue = evaluateMetric(&metric, numState, makespan),
                endMetricValue,
                dur = a->fixedDurationValue[0];
        if (a->startNumEff.empty() && a->endNumEff.empty()) {
                endMetricValue = evaluateMetric(&metric, numState, makespan +
dur);
        }
        else {
                unsigned int numV = numVariables.size();
                float* newState = new float[numV];
                for (unsigned int i = 0; i < numV; i++) {
                        newState[i] = numState[i];
                }
                for (unsigned int i = 0; i < a->startNumEff.size(); i++) {
                        updateNumericState(newState, &(a->startNumEff[i]), dur);
                }
                for (unsigned int i = 0; i < a->endNumEff.size(); i++) {
                        updateNumericState(newState, &(a->endNumEff[i]), dur);
                }
                endMetricValue = evaluateMetric(&metric, newState, makespan +
dur); delete[] newState;
        }
        return endMetricValue - startMetricValue;
}
*/

// Updates the numeric state through the given action effect
void SASTask::updateNumericState(float *s, SASNumericEffect *e,
                                 float duration) {
  float v = evaluateNumericExpression(&(e->exp), s, duration);
  switch (e->op) {
  case '=':
    s[e->var] = v;
    break;
  case '+':
    s[e->var] += v;
    break;
  case '-':
    s[e->var] -= v;
    break;
  case '*':
    s[e->var] *= v;
    break;
  case '/':
    s[e->var] /= v;
    break;
  default:;
  }
}

// Computes the duration of an action (used in TemporalRPG only)
float SASTask::getActionDuration(SASAction *a, float *s) {
  if (a->duration.constantDuration) {
    return a->duration.minDuration;
  }
  SASDuration &d = a->duration;
  SASDurationCondition *duration = &(d.conditions[0]);
  return evaluateNumericExpression(&(duration->exp), s, 0);
}

// Checks if a numeric condition holds in the given numeric state
bool SASTask::holdsNumericCondition(SASNumericCondition &cond, float *s,
                                    float duration) {
  float v1 = evaluateNumericExpression(&(cond.terms[0]), s, duration);
  float v2 = evaluateNumericExpression(&(cond.terms[1]), s, duration);
  // cout << "Condition: " << v1 << " " << cond.comp << " " << v2 << endl;
  switch (cond.comp) {
  case '=':
    return v1 == v2;
  case '<':
    return v1 < v2;
  case 'L':
    return v1 <= v2;
  case '>':
    return v1 > v2;
  case 'G':
    return v1 >= v2;
  case 'N':
    return v1 != v2;
  }
  return false;
}

// Evaluates a numeric expression in a given state and with the given action
// duration
float SASTask::evaluateNumericExpression(SASNumericExpression *e, float *s,
                                         float duration) {
  if (e->type == 'N')
    return e->value; // NUMBER
  if (e->type == 'V')
    return s[e->var]; // VAR
  if (e->type == 'D')
    return duration;
  if (e->type == 'C')
    return 1;
  if (e->type == '#') {
    throwError("#t in duration not supported yet");
  }
  float res = evaluateNumericExpression(&(e->terms[0]), s, duration);
  for (unsigned int i = 1; i < e->terms.size(); i++) {
    switch (e->type) {
    case '+':
      res += evaluateNumericExpression(&(e->terms[i]), s, duration);
      break; // SUM
    case '-':
      res -= evaluateNumericExpression(&(e->terms[i]), s, duration);
      break; // SUB
    case '*':
      res *= evaluateNumericExpression(&(e->terms[i]), s, duration);
      break; // MUL
    case '/':
      res /= evaluateNumericExpression(&(e->terms[i]), s, duration);
      break; // DIV
    }
  }
  return res;
}

// Calculates the metric cost in the given state and with the given plan
// duration
float SASTask::evaluateMetric(SASMetric *m, float *numState, float makespan) {
  switch (m->type) {
  case 'N':
    return m->value;
  case 'T':
    return makespan;
  case 'F':
    return numState[m->index];
  case 'V':
    return 0;
  case '+':
  case '-':
  case '*':
  case '/': {
    float v = evaluateMetric(&(m->terms[0]), numState, makespan);
    if (m->terms.size() == 1) {
      return m->type == '-' ? -v : v;
    } else {
      for (unsigned int i = 1; i < m->terms.size(); i++) {
        float t = evaluateMetric(&(m->terms[i]), numState, makespan);
        switch (m->type) {
        case '+':
          v += t;
          break;
        case '-':
          v -= t;
          break;
        case '*':
          v *= t;
          break;
        case '/':
          v /= t;
          break;
        default:;
        }
      }
      return v;
    }
  } break;
  default:
    return 0;
    // cout << "Metric type not supported yet. Method SASTask::evaluateMetric."
    // << endl; exit(1);
  }
}

// Computes the value of a non-variable expression (that doesn't depend on the
// value of numeric variables)
float SASTask::computeFixedExpression(SASNumericExpression *e) {
  float res = 0;
  switch (e->type) {
  case 'N':
    res = e->value;
    break;
  case '+':
  case '-':
  case '/':
  case '*':
    res = computeFixedExpression(&(e->terms[0]));
    if (e->terms.size() == 1) {
      if (e->type == '-')
        res = -res;
    } else {
      for (unsigned int i = 1; i < e->terms.size(); i++) {
        float t = computeFixedExpression(&(e->terms[i]));
        switch (e->type) {
        case '+':
          res += t;
          break;
        case '-':
          res -= t;
          break;
        case '/':
          res /= t;
          break;
        case '*':
          res *= t;
          break;
        default:;
        }
      }
    }
    break;
  case 'V':
    res = numVariables[e->var].getInitialStateValue();
    break;
  default:;
  }
  return res;
}

// Checks if the given expression depends on the state
bool SASTask::checkVariableExpression(SASNumericExpression *e,
                                      bool *variablesOnMetric) {
  switch (e->type) {
  case 'V':
    if (staticNumFunctions != nullptr && staticNumFunctions[e->var]) {
      return false; // Variable is static -> does not depend on the state
    }
    if (variablesOnMetric == nullptr) { // Not necessary to check if the
                                        // variable is used in the metric
      return true;
    } else {
      return variablesOnMetric[e->var]; // Only if the variable is used in the
                                        // metric
    }
    break;
  case '+':
  case '-':
  case '/':
  case '*':
    for (unsigned int i = 0; i < e->terms.size(); i++) {
      if (checkVariableExpression(&(e->terms[i]), variablesOnMetric)) {
        return true;
      }
    }
    return false;
  case 'C':
    return true;
  default:
    return false;
  }
}

// Returns a string representation of this task
string SASTask::toString() {
  vector<SASControlVar> controlVars;
  string s = "OBJECTS:\n";
  for (unsigned int i = 0; i < values.size(); i++)
    s += values[i].toString() + "\n";
  s += "VARIABLES:\n";
  for (unsigned int i = 0; i < variables.size(); i++)
    s += variables[i].toString(values) + "\n";
  for (unsigned int i = 0; i < numVariables.size(); i++)
    s += numVariables[i].toString() + "\n";
  s += "INITIAL STATE:\n";
  for (unsigned int i = 0; i < variables.size(); i++)
    s += variables[i].toStringInitialState(values) + "\n";
  for (unsigned int i = 0; i < numVariables.size(); i++)
    s += numVariables[i].toStringInitialState() + "\n";
  for (unsigned int i = 0; i < actions.size(); i++)
    s += toStringAction(actions[i]);
  for (unsigned int i = 0; i < goals.size(); i++)
    s += toStringAction(goals[i]);
  s += "CONSTRAINTS:\n";
  for (unsigned int i = 0; i < constraints.size(); i++)
    s += toStringConstraint(&constraints[i], &controlVars) + "\n";
  if (metricType != 'X') {
    s += "METRIC:\n";
    if (metricType == '<')
      s += "MINIMIZE ";
    else
      s += "MAXIMIZE ";
    s += toStringMetric(&metric);
  }
  return s;
}

// Returns a string representation of this action
string SASTask::toStringAction(SASAction &a) {
  string s = "ACTION " + a.name + "\n";
  for (SASControlVar &cv : a.controlVars) {
    s += " :control " + cv.toString(&numVariables, &a.controlVars) + "\n";
  }
  for (auto &v : a.startNumConstrains) {
    s += " :start-const on " + numVariables[v.first].name + "\n";
    for (SASNumericCondition &c : v.second)
      s += "    " + c.toString(&numVariables, &a.controlVars) + "\n";
  }
  s += " :duration " + a.duration.toString(&numVariables, &a.controlVars);
  for (unsigned int i = 0; i < a.startNumCond.size(); i++)
    s += " :con (at start " +
         a.startNumCond[i].toString(&numVariables, &a.controlVars) + ")\n";
  for (unsigned int i = 0; i < a.startCond.size(); i++)
    s += " :con (at start " + toStringCondition(a.startCond[i]) + ")\n";
  for (unsigned int i = 0; i < a.overCond.size(); i++)
    s += " :con (over all " + toStringCondition(a.overCond[i]) + ")\n";
  for (unsigned int i = 0; i < a.overNumCond.size(); i++)
    s += " :con (over all " +
         a.overNumCond[i].toString(&numVariables, &a.controlVars) + ")\n";
  for (unsigned int i = 0; i < a.endCond.size(); i++)
    s += " :con (at end " + toStringCondition(a.endCond[i]) + ")\n";
  for (unsigned int i = 0; i < a.endNumCond.size(); i++)
    s += " :con (at end " +
         a.endNumCond[i].toString(&numVariables, &a.controlVars) + ")\n";
  for (unsigned int i = 0; i < a.preferences.size(); i++)
    s += " :con (" + toStringPreference(&(a.preferences[i]), &a.controlVars) +
         ")\n";
  for (unsigned int i = 0; i < a.startEff.size(); i++)
    s += " :eff (at start " + toStringCondition(a.startEff[i]) + ")\n";
  for (unsigned int i = 0; i < a.startNumEff.size(); i++)
    s += " :eff (at start " +
         a.startNumEff[i].toString(&numVariables, &a.controlVars) + ")\n";
  for (unsigned int i = 0; i < a.endEff.size(); i++)
    s += " :eff (at end " + toStringCondition(a.endEff[i]) + ")\n";
  for (unsigned int i = 0; i < a.endNumEff.size(); i++)
    s += " :eff (at end " +
         a.endNumEff[i].toString(&numVariables, &a.controlVars) + ")\n";

  for (unsigned int i = 0; i < a.conditionalEff.size(); i++) {
    s += " :cond.eff\n";
    SASConditionalEffect &e = a.conditionalEff[i];
    for (SASCondition &c : e.startCond)
      s += "\t:con (at-start " + toStringCondition(c) + ")\n";
    for (SASCondition &c : e.endCond)
      s += "\t:con (at-end " + toStringCondition(c) + ")\n";
    for (SASNumericCondition &c : e.startNumCond)
      s += "\t:con (at-start " + c.toString(&numVariables, &a.controlVars) +
           ")\n";
    for (SASNumericCondition &c : e.endNumCond)
      s +=
          "\t:con (at-end " + c.toString(&numVariables, &a.controlVars) + ")\n";
    for (SASCondition &c : e.startEff)
      s += "\t:eff (at-start " + toStringCondition(c) + ")\n";
    for (SASCondition &c : e.endEff)
      s += "\t:eff (at-end " + toStringCondition(c) + ")\n";
    for (SASNumericEffect &c : e.startNumEff)
      s += "\t:eff (at-start " + c.toString(&numVariables, &a.controlVars) +
           ")\n";
    for (SASNumericEffect &c : e.endNumEff)
      s +=
          "\t:eff (at-end " + c.toString(&numVariables, &a.controlVars) + ")\n";
  }

  return s;
}

// Returns a string representation of this condition
string SASTask::toStringCondition(SASCondition &c) {
  return "(= " + variables[c.var].name + " " + values[c.value].name + ")";
}

// Returns a string representation of this preference
string SASTask::toStringPreference(SASPreference *pref,
                                   vector<SASControlVar> *controlVars) {
  return "preference " + preferenceNames[pref->index] + " " +
         toStringGoalDescription(&(pref->preference), controlVars);
}

// Returns a string representation of this goal description
string SASTask::toStringGoalDescription(SASGoalDescription *g,
                                        vector<SASControlVar> *controlVars) {
  string s = "(" + toStringTime(g->time);
  if (g->time != 'N')
    s += " (";
  switch (g->type) {
  case 'V':
    s += "= " + variables[g->var].name + " " + values[g->value].name;
    break;
  case '&':
  case '|':
  case '!':
    if (g->type == '&')
      s += "and";
    else if (g->type == '|')
      s += "or";
    else
      s += "not";
    for (unsigned int i = 0; i < g->terms.size(); i++)
      s += " " + toStringGoalDescription(&(g->terms[i]), controlVars);
    break;
  default:
    s += toStringComparator(g->type);
    for (unsigned int i = 0; i < g->exp.size(); i++) {
      s += " " + g->exp[i].toString(&numVariables, controlVars);
    }
  }
  if (g->time != 'N')
    s += ")";
  return s + ")";
}

// Returns a string representation of this constraint
string SASTask::toStringConstraint(SASConstraint *c,
                                   std::vector<SASControlVar> *controlVars) {
  string s = "(";
  switch (c->type) {
  case '&':
    s += "and";
    for (unsigned int i = 0; i < c->terms.size(); i++)
      s += " " + toStringConstraint(&(c->terms[i]), controlVars);
    break;
  case 'P':
    s += "preference " + preferenceNames[c->preferenceIndex] + " " +
         toStringConstraint(&(c->terms[0]), controlVars);
    break;
  case 'G':
    s += "preference " + preferenceNames[c->preferenceIndex] + " " +
         toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  case 'E':
    s += "at end " + toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  case 'A':
    s += "always " + toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  case 'S':
    s += "sometime " + toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  case 'W':
    s += "within " + to_string(c->time[0]) + " " +
         toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  case 'O':
    s += "at-most-once " + toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  case 'F':
    s += "sometime-after " +
         toStringGoalDescription(&(c->goal[0]), controlVars) + " " +
         toStringGoalDescription(&(c->goal[1]), controlVars);
    break;
  case 'B':
    s += "sometime-before " +
         toStringGoalDescription(&(c->goal[0]), controlVars) + " " +
         toStringGoalDescription(&(c->goal[1]), controlVars);
    break;
  case 'T':
    s += "always-within " + to_string(c->time[0]) + " " +
         toStringGoalDescription(&(c->goal[0]), controlVars) + " " +
         toStringGoalDescription(&(c->goal[1]), controlVars);
    break;
  case 'D':
    s += "hold-during " + to_string(c->time[0]) + " " + to_string(c->time[1]) +
         " " + toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  case 'H':
    s += "hold-after " + to_string(c->time[0]) + " " +
         toStringGoalDescription(&(c->goal[0]), controlVars);
    break;
  }
  return s + ")";
}

// Returns a string representation of the metric
string SASTask::toStringMetric(SASMetric *m) {
  string s = "";
  switch (m->type) {
  case '+':
  case '-':
  case '*':
  case '/':
    s += "(";
    s += m->type;
    for (unsigned int i = 0; i < m->terms.size(); i++)
      s += " " + toStringMetric(&(m->terms[i]));
    s += ")";
    break;
  case 'N':
    s += to_string(m->value);
    break;
  case 'T':
    s += "total-time";
    break;
  case 'V':
    s += "(is-violated " + preferenceNames[m->index] + ")";
    break;
  case 'F':
    s += numVariables[m->index].name;
    break;
  }
  return s;
}

std::vector<TVarValue> *SASTask::getListOfGoals() {
  if (goalList.empty()) {
    for (unsigned int i = 0; i < goals.size(); i++) {
      SASAction &g = goals[i];
      for (unsigned int j = 0; j < g.startCond.size(); j++)
        addGoalToList(&(g.startCond[j]));
      for (unsigned int j = 0; j < g.overCond.size(); j++)
        addGoalToList(&(g.overCond[j]));
      for (unsigned int j = 0; j < g.endCond.size(); j++)
        addGoalToList(&(g.endCond[j]));
    }
  }
  return &goalList;
}

void SASTask::addGoalToList(SASCondition *c) {
  TVarValue vv = getVariableValueCode(c->var, c->value);
  for (unsigned int i = 0; i < goalList.size(); i++) {
    if (goalList[i] == vv)
      return;
  }
  goalList.push_back(vv);
}

void SASTask::addGoalDeadline(float time, TVarValue goal) {
  for (unsigned int i = 0; i < goalDeadlines.size(); i++) {
    if (goalDeadlines[i].time == time) {
      goalDeadlines[i].goals.push_back(goal);
    }
  }
  goalDeadlines.emplace_back();
  goalDeadlines.back().time = time;
  goalDeadlines.back().goals.push_back(goal);
}

/************************************/
/* SASAction                        */
/************************************/

void SASAction::postProcessDuration(SASDurationCondition &dc) {
  if (dc.exp.type != 'N') { // Not a number -> depends on the state
    duration.constantDuration = false;
    searchForControlVarsInDuration(&dc.exp);
    switch (dc.comp) {
    case '=':
      duration.minDuration = evaluateMinDuration(&dc.exp);
      duration.maxDuration = evaluateMaxDuration(&dc.exp);
      break;
    case 'L':
      updateMaxDuration(evaluateMaxDuration(&dc.exp) + EPSILON);
      break;
    case '<':
      updateMaxDuration(evaluateMaxDuration(&dc.exp));
      break;
    case 'G':
      updateMinDuration(evaluateMinDuration(&dc.exp) - EPSILON);
      break;
    case '>':
      updateMinDuration(evaluateMinDuration(&dc.exp));
      break;
    }
  } else {
    float value = dc.exp.value;
    switch (dc.comp) {
    case '=':
      duration.minDuration = value;
      duration.maxDuration = value;
      break;
    case 'L':
      value += EPSILON;
    case '<':
      updateMaxDuration(value);
      duration.constantDuration = false;
      break;
    case 'G':
      value -= EPSILON;
    case '>':
      updateMinDuration(value);
      duration.constantDuration = false;
      break;
    }
  }
}

void SASAction::searchForControlVarsInDuration(SASNumericExpression *e) {
  if (e->type == 'C') {
    bool found = false;
    for (int i = 0; i < duration.controlVarsNeededInDuration.size(); i++)
      if (duration.controlVarsNeededInDuration.at(i) == e->var) {
        found = true;
        break;
      }
    if (!found)
      duration.controlVarsNeededInDuration.push_back(e->var);
  } else {
    for (SASNumericExpression &t : e->terms) {
      searchForControlVarsInDuration(&t);
    }
  }
}

void SASAction::updateMinDuration(float value) {
  if (duration.minDuration < value)
    duration.minDuration = value;
}

void SASAction::updateMaxDuration(float value) {
  if (duration.maxDuration > value)
    duration.maxDuration = value;
}

float SASAction::evaluateMinDuration(SASNumericExpression *e) {
  if (e->type == 'N')
    return e->value;
  if (e->type == 'V')
    return -FLOAT_INFINITY; // Minimum value of a numeric variable
  if (e->type == 'C')
    return -FLOAT_INFINITY;
  float v1 = evaluateMinDuration(&(e->terms[0])), v2;
  switch (e->type) {
  case '+': // GE_SUM
    v2 = evaluateMinDuration(&(e->terms[1]));
    return v1 + v2;
  case '-': // GE_SUB
    v2 = evaluateMaxDuration(&(e->terms[1]));
    return v1 - v2;
  case '/': // GE_DIV
    v2 = evaluateMaxDuration(&(e->terms[1]));
    return v2 == FLOAT_INFINITY ? 0 : v1 / v2;
  case '*': // GE_MUL
    v2 = evaluateMinDuration(&(e->terms[1]));
    return v1 * v2;
  }
  return -FLOAT_INFINITY;
}

float SASAction::evaluateMaxDuration(SASNumericExpression *e) {
  if (e->type == 'N')
    return e->value;
  if (e->type == 'V')
    return FLOAT_INFINITY; // Maximum value of a numeric variable
  if (e->type == 'C')
    return FLOAT_INFINITY;
  float v1 = evaluateMaxDuration(&(e->terms[0])), v2;
  switch (e->type) {
  case '+': // GE_SUM
    v2 = evaluateMaxDuration(&(e->terms[1]));
    return v1 + v2;
  case '-': // GE_SUB
    v2 = evaluateMinDuration(&(e->terms[1]));
    return v1 - v2;
  case '/': // GE_DIV
    v2 = evaluateMinDuration(&(e->terms[1]));
    return v2 == -FLOAT_INFINITY ? 0 : v1 / v2;
  case '*': // GE_MUL
    v2 = evaluateMaxDuration(&(e->terms[1]));
    return v1 * v2;
  }
  return FLOAT_INFINITY;
}

bool SASAction::checkDurationInEffect(SASNumericExpression *e) {
  if (e->type == 'D') {
    duration.durationNeededInEffects = true;
    return true;
  }
  for (SASNumericExpression &t : e->terms) {
    if (checkDurationInEffect(&t))
      return true;
  }
  return false;
}

void SASAction::removeSameCondition(SASCondition *c,
                                    std::vector<SASCondition> &condList) {
  int i = ((int)condList.size()) - 1;
  while (i >= 0) {
    if (condList.at(i).equals(c)) {
      condList.erase(condList.begin() + i);
    }
    i--;
  }
}

void SASAction::removeSameNumericCondition(
    SASNumericCondition *c, std::vector<SASNumericCondition> &condList) {
  int i = ((int)condList.size()) - 1;
  while (i >= 0) {
    if (condList.at(i).equals(c)) {
      condList.erase(condList.begin() + i);
    }
    i--;
  }
}

void SASAction::postprocessControlVariables() {
  for (int i = (int)startNumCond.size() - 1; i >= 0; i--) {
    std::vector<int> cvs;
    char res = analyzeNumericCondition(
        &startNumCond[i], &cvs); // 'w': without control variables
                                 // 'c': with control variable and without
                                 // fluents -> move cond. to control. var
                                 //      changing to the form: cv comp expr
                                 // 'b': both control var. and fluents -> keep
                                 // but also copy to control. var
                                 //      changing to the form: cv comp expr
    if (res != 'w') {
      bool del = res == 'c';
      for (int cv : cvs)
        controlVars[cv].copyCondition(&startNumCond[i], cv, (int)cvs.size(),
                                      !del);
      if (del)
        startNumCond.erase(startNumCond.begin() + i);
    }
  }
  for (int i = (int)overNumCond.size() - 1; i >= 0; i--) {
    std::vector<int> cvs;
    char res = analyzeNumericCondition(&overNumCond[i], &cvs);
    if (res != 'w') {
      bool del = res == 'c';
      for (int cv : cvs)
        controlVars[cv].copyCondition(&overNumCond[i], cv, (int)cvs.size(),
                                      !del);
      if (del)
        overNumCond.erase(overNumCond.begin() + i);
    }
  }
  for (int i = (int)endNumCond.size() - 1; i >= 0; i--) {
    std::vector<int> cvs;
    char res = analyzeNumericCondition(&endNumCond[i], &cvs);
    if (res != 'w') {
      bool del = res == 'c';
      for (int cv : cvs)
        controlVars[cv].copyCondition(&endNumCond[i], cv, (int)cvs.size(),
                                      !del);
      if (del)
        endNumCond.erase(endNumCond.begin() + i);
    }
  }
}

void SASAction::postprocessNumericVariables() {
  for (SASNumericCondition &c : startNumCond) {
    std::vector<TVariable> fluents;
    containsFluents(&c.terms[0], &fluents);
    if (c.terms.size() > 1)
      containsFluents(&c.terms[1], &fluents);
    for (TVariable v : fluents)
      copyCondition(&c, v, startNumConstrains);
  }
  for (SASNumericCondition &c : overNumCond) {
    std::vector<TVariable> fluents;
    containsFluents(&c.terms[0], &fluents);
    if (c.terms.size() > 1)
      containsFluents(&c.terms[1], &fluents);
    for (TVariable v : fluents) {
      copyCondition(&c, v, startNumConstrains);
      copyCondition(&c, v, endNumConstrains);
    }
  }
  for (SASNumericCondition &c : endNumCond) {
    std::vector<TVariable> fluents;
    containsFluents(&c.terms[0], &fluents);
    if (c.terms.size() > 1)
      containsFluents(&c.terms[1], &fluents);
    for (TVariable v : fluents)
      copyCondition(&c, v, endNumConstrains);
  }
}

char SASAction::analyzeNumericCondition(SASNumericCondition *c,
                                        std::vector<int> *cvs) {
  for (SASNumericExpression &e : c->terms) {
    containsControlVar(&e, cvs);
  }
  if (cvs->empty())
    return 'w';
  for (SASNumericExpression &e : c->terms) {
    if (containsFluents(&e))
      return 'b';
  }
  return 'c';
}

void SASAction::containsControlVar(SASNumericExpression *e,
                                   std::vector<int> *cvs) {
  if (e->type == 'C') {
    bool found = false;
    for (int i = 0; i < cvs->size(); i++)
      if (cvs->at(i) == e->var) {
        found = true;
        break;
      }
    if (!found)
      cvs->push_back(e->var);
  } else {
    for (SASNumericExpression &se : e->terms)
      containsControlVar(&se, cvs);
  }
}

bool SASAction::containsFluents(SASNumericExpression *e) {
  if (e->type == 'V') {
    return true;
  }
  for (SASNumericExpression &se : e->terms)
    if (containsFluents(&se))
      return true;
  return false;
}

void SASAction::containsFluents(SASNumericExpression *e,
                                std::vector<TVariable> *vars) {
  if (e->type == 'V') {
    bool found = false;
    for (int i = 0; i < vars->size(); i++)
      if (vars->at(i) == e->var) {
        found = true;
        break;
      }
    if (!found)
      vars->push_back(e->var);
  }
  for (SASNumericExpression &se : e->terms)
    containsFluents(&se, vars);
}

void SASAction::copyCondition(
    SASNumericCondition *c, TVariable v,
    std::unordered_map<TVariable, std::vector<SASNumericCondition>> &cons) {
  if (cons.find(v) == cons.end()) {
    std::vector<SASNumericCondition> newVector;
    cons[v] = newVector;
  }
  std::vector<SASNumericCondition> *conditions;
  conditions = &cons[v];
  conditions->emplace_back();
  SASNumericCondition *copy = &conditions->back();
  copy->copyFrom(c);
  if (c->comp != '-')
    copy->reshapeFluent(v);
}

void SASAction::postProcess() {
  for (SASCondition &c : overCond) {
    removeSameCondition(&c, startCond);
    removeSameCondition(&c, endCond);
  }
  for (SASNumericCondition &c : overNumCond) {
    removeSameNumericCondition(&c, startNumCond);
    removeSameNumericCondition(&c, endNumCond);
  }
  duration.constantDuration = true;
  duration.minDuration = EPSILON;
  if (isGoal)
    duration.maxDuration = EPSILON;
  else
    duration.maxDuration = FLOAT_INFINITY;
  for (SASDurationCondition &dc : duration.conditions) {
    postProcessDuration(dc);
  }
  duration.durationNeededInEffects = false;
  for (SASNumericEffect &e : startNumEff) {
    if (checkDurationInEffect(&e.exp))
      break;
  }
  for (SASNumericEffect &e : endNumEff) {
    if (checkDurationInEffect(&e.exp))
      break;
  }
  postprocessControlVariables();
  postprocessNumericVariables();
}

/************************************/
/* SASControlVar                    */
/************************************/

void SASControlVar::copyCondition(SASNumericCondition *c, int cv, int numCvs,
                                  bool inPrecs) {
  SASNumericCondition *copy;
  conditions.emplace_back();
  SASControlVarCondition *newCondition = &conditions.back();
  copy = &newCondition->condition;
  newCondition->inActionPrec = inPrecs;
  newCondition->numCvs = numCvs;
  copy->copyFrom(c);
  copy->reshape(cv);
}

std::string SASControlVar::toString(std::vector<NumericVariable> *numVariables,
                                    std::vector<SASControlVar> *controlVars) {
  std::string s = name;
  for (SASControlVarCondition &c : conditions) {
    s += "\n    " + c.condition.toString(numVariables, controlVars);
  }
  return s;
}
