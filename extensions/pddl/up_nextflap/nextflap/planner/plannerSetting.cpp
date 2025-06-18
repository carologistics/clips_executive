#include "plannerSetting.h"

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Initializes and launches the planner.                */
/********************************************************/

using namespace std;

PlannerSetting::PlannerSetting(SASTask *sTask) {
  initialTime = clock();
  this->task = sTask;
  this->generateTrace = generateTrace;
  createInitialPlan();
  forceAtEndConditions = checkForceAtEndConditions();
  filterRepeatedStates = checkRepeatedStates();
  // if (!forceAtEndConditions) cout << "End conditions can be left unsupported"
  // << endl; if (!filterRepeatedStates) cout << "Repeated states will not be
  // pruned" << endl;
  initialState = new TState(this->task);
  task->tilActions = !tilActions.empty();
  this->planner = nullptr;
}

// Creates the initial empty plan that only contains the initial and the TIL
// fictitious actions
void PlannerSetting::createInitialPlan() {
  SASAction *initialAction = createInitialAction();
  initialPlan = new Plan(initialAction, nullptr, 0, nullptr);
  initialPlan->setDuration(EPSILON, EPSILON);
  initialPlan->setTime(-EPSILON, 0, true);
  initialPlan->addFluentIntervals();
  initialPlan = createTILactions(initialPlan);
}

// Creates and returns the initial fictitious action
SASAction *PlannerSetting::createInitialAction() {
  vector<unsigned int> varList;
  for (unsigned int i = 0; i < task->variables.size();
       i++) { // Non-numeric effects
    SASVariable &var = task->variables[i];
    for (unsigned int j = 0; j < var.value.size(); j++) {
      if (var.time[j] == 0) { // Initial state effect
        varList.push_back(i);
        break;
      }
    }
  }
  for (unsigned int i = 0; i < task->numVariables.size();
       i++) { // Numeric effects
    NumericVariable &var = task->numVariables[i];
    for (unsigned int j = 0; j < var.value.size(); j++) {
      if (var.time[j] == 0) { // Initial state effect
        varList.push_back(i + (unsigned int)task->variables.size());
        break;
      }
    }
  }
  return createFictitiousAction(EPSILON, varList, 0, "#initial", false, false);
}

// Create a fictitious action with the given duration and with the effects with
// the modified variables in the given time point
SASAction *PlannerSetting::createFictitiousAction(float actionDuration,
                                                  vector<unsigned int> &varList,
                                                  float timePoint, string name,
                                                  bool isTIL, bool isGoal) {
  SASAction *a = new SASAction(true, isTIL, isGoal);
  a->index = MAX_UNSIGNED_INT;
  a->name = name;
  SASDurationCondition duration;
  duration.time = 'N';
  duration.comp = '=';
  duration.exp.type = 'N'; // Number (epsilon duration)
  duration.exp.value = actionDuration;
  a->duration.conditions.push_back(duration);
  for (unsigned int i = 0; i < varList.size(); i++) {
    unsigned int varIndex = varList[i];
    if (varIndex < task->variables.size()) { //	Non-numeric effect
      SASVariable &var = task->variables[varIndex];
      for (unsigned int j = 0; j < var.value.size(); j++) {
        if (var.time[j] == timePoint) {
          a->endEff.emplace_back(varIndex, var.value[j]);
          break;
        }
      }
    } else { // Numeric effect
      varIndex -= (unsigned int)task->variables.size();
      NumericVariable &var = task->numVariables[varIndex];
      for (unsigned int j = 0; j < var.value.size(); j++) {
        if (var.time[j] == timePoint) {
          SASNumericEffect eff;
          eff.op = '=';
          eff.var = varIndex;
          eff.exp.type = 'N'; // Number
          eff.exp.value = var.value[j];
          a->endNumEff.push_back(eff);
          break;
        }
      }
    }
  }
  return a;
}

// Adds the fictitious TIL actions to the initial plan. Returns the resulting
// plan
Plan *PlannerSetting::createTILactions(Plan *parentPlan) {
  Plan *result = parentPlan;
  unordered_map<float, vector<unsigned int>>
      til; // Time point -> variables modified at that time
  for (unsigned int i = 0; i < task->variables.size();
       i++) { // Non-numeric effects
    SASVariable &var = task->variables[i];
    for (unsigned int j = 0; j < var.value.size(); j++) {
      if (var.time[j] > 0) { // Time-initial literal
        if (til.find(var.time[j]) == til.end()) {
          vector<unsigned int> v;
          v.push_back(i);
          til[var.time[j]] = v;
        } else
          til[var.time[j]].push_back(i);
      }
    }
  }
  for (unsigned int i = 0; i < task->numVariables.size();
       i++) { // Numeric effects
    NumericVariable &var = task->numVariables[i];
    for (unsigned int j = 0; j < var.value.size(); j++) {
      if (var.time[j] > 0) { // Numeric time-initial literal
        if (til.find(var.time[j]) == til.end()) {
          vector<unsigned int> v;
          v.push_back(i + (unsigned int)task->variables.size());
          til[var.time[j]] = v;
        } else
          til[var.time[j]].push_back(i + (unsigned int)task->variables.size());
      }
    }
  }
  for (auto it = til.begin(); it != til.end(); ++it) {
    float timePoint = it->first;
    SASAction *a =
        createFictitiousAction(timePoint, it->second, timePoint,
                               "#til" + to_string(timePoint), true, false);
    tilActions.push_back(a);
    result = new Plan(a, result, 0, nullptr);
    result->setDuration(timePoint, timePoint);
    result->setTime(0, timePoint, true);
    result->addFluentIntervals();
  }
  return result;
}

bool PlannerSetting::
    checkForceAtEndConditions() { // Check if it's required to leave at-end
                                  // conditions not supported for some actions
  vector<vector<unsigned short>> varValues;
  varValues.resize(task->variables.size());
  for (unsigned int i = 0; i < task->variables.size(); i++) {
    SASVariable &var = task->variables[i];
    for (unsigned int j = 0; j < var.value.size(); j++) {
      varValues[i].push_back((unsigned short)var.value[j]);
    }
  }
  RPG rpg(varValues, task, true, &tilActions);
  for (unsigned int i = 0; i < task->goals.size(); i++) {
    if (rpg.isExecutable(&(task->goals[i])))
      return true;
  }
  return false;
}

bool PlannerSetting::checkRepeatedStates() {
  TVariable v;
  TValue value;
  for (SASAction &a : task->actions) {
    for (unsigned int i = 0; i < a.startEff.size(); i++) {
      v = a.startEff[i].var;
      value = a.startEff[i].value;
      for (unsigned int j = 0; j < a.endEff.size(); j++) {
        if (a.endEff[j].var == v && a.endEff[j].value != value) {
          // If (v = value) is required by any action, the domain is concurrent
          vector<SASAction *> &req = task->requirers[v][value];
          for (unsigned int k = 0; k < req.size(); k++) {
            if (req[k] != &a) {
              return false;
            }
          }
          break;
        }
      }
    }
  }
  return true;
}

Plan *PlannerSetting::plan(float bestMakespan, ParsedTask *parsedTask) {
  if (planner == nullptr) {
    planner = new Planner(task, initialPlan, initialState, forceAtEndConditions,
                          filterRepeatedStates, generateTrace, &tilActions,
                          parsedTask);
  } else {
    planner->clearSolution();
  }
  return planner->plan(bestMakespan);
}
