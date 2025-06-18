#include "planner.h"
#include "printPlan.h"
#include "z3Checker.h"

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Planning search.
 */
/********************************************************/

using namespace std;

// Checks if the search space is empty
bool Planner::emptySearchSpace() { return selector->size() == 0; }

// Constructor
Planner::Planner(SASTask *task, Plan *initialPlan, TState *initialState,
                 bool forceAtEndConditions, bool filterRepeatedStates,
                 bool generateTrace, std::vector<SASAction *> *tilActions,
                 ParsedTask *parsedTask) {
  this->bestH = MAX_INT32;
  this->parsedTask = parsedTask;
  this->task = task;
  this->initialPlan = initialPlan;
  this->initialState = initialState;
  this->forceAtEndConditions = forceAtEndConditions;
  this->filterRepeatedStates = filterRepeatedStates;
  // cout << ";Filter repeated states: " << filterRepeatedStates << endl;
  this->parentPlanner = parentPlanner;
  this->expandedNodes = 0;
  this->generateTrace = generateTrace;
  this->tilActions = tilActions;
  successors = new Successors(initialState, task, forceAtEndConditions,
                              filterRepeatedStates, tilActions);
  this->initialH = FLOAT_INFINITY;
  this->solution = nullptr;
  selector = new SearchQueue();
  successors->evaluator.calculateFrontierState(this->initialPlan);
  /*
  selector->addQueue(SEARCH_NRPG);
  if (successors->evaluator.informativeLandmarks()) {	// Landmarks available
          selector->addQueue(SEARCH_LAND);
  }*/
  selector->add(this->initialPlan);
  successors->evaluator.evaluateInitialPlan(initialPlan);
}

// Starts the search
Plan *Planner::plan(float bestMakespan) {
  this->bestMakespan = bestMakespan;
  while (solution == nullptr && this->selector->size() > 0) {
    if (parsedTask->timeout > 0 &&
        parsedTask->ellapsedTime() > parsedTask->timeout)
      break;
    searchStep();
  }
  return solution;
}

// Clears the last solution found
void Planner::clearSolution() {
  solution = nullptr;
  successors->solution = nullptr;
}

// Checks if a plan is valid
bool Planner::checkPlan(Plan *p) {
  Z3Checker checker;
  p->z3Checked = true;
  bool valid = checker.checkPlan(p, false);
  return valid;
}

// Marks a plan as invalid
void Planner::markAsInvalid(Plan *p) {
  markChildrenAsInvalid(p);
  Plan *parent = p->parentPlan;
  if (parent != nullptr && !parent->isRoot() && !parent->z3Checked) {
    if (!checkPlan(parent)) {
      markAsInvalid(parent);
    }
  }
}

// Marks a children plan as invalid
void Planner::markChildrenAsInvalid(Plan *p) {
  if (p->childPlans != nullptr) {
    for (Plan *child : *(p->childPlans)) {
      child->invalid = true;
      markChildrenAsInvalid(child);
    }
  }
}

// Makes one search step
void Planner::searchStep() {
  Plan *base = selector->poll();
#if _DEBUG
  cout << "Base plan: " << base->id << ", " << base->action->name
       << "(G = " << base->g << ", H=" << base->h << ")" << endl;
#endif
  if (!base->invalid && !successors->repeatedState(base)) {
    if (base->action->startNumCond.size() > 0 ||
        base->action->overNumCond.size() > 0 ||
        base->action->endNumCond.size() > 0) {
      if (base->h <= 1) {
        if (!checkPlan(base)) { // Validity checking
          return;
        }
      }
    }
    if (base->h < bestH) {
      if (debugFile != nullptr)
        *debugFile << ";H: " << base->h << " (" << base->hLand << ")" << endl;
      bestH = base->h;
    }
    expandBasePlan(base);
    addSuccessors(base);
  }
}

// Expands the current base plan
void Planner::expandBasePlan(Plan *base) {
  if (base->expanded()) {
    sucPlans.clear();
    return;
  }
  successors->computeSuccessors(base, &sucPlans, bestMakespan);
  ++expandedNodes;
  if (successors->solution != nullptr) {
    // PrintPlan::print(successors->solution);
    if (checkPlan(successors->solution)) {
      solution = successors->solution;
    } else {
      markAsInvalid(successors->solution);
      successors->solution = nullptr;
    }
  }
}

// Adds the successor plans to the selector
void Planner::addSuccessors(Plan *base) {
  base->addChildren(sucPlans);
  for (Plan *p : sucPlans) {
    // cout << "* Successor: " << p->id << ", " << p->action->name << "(G = " <<
    // p->g << ", H=" << p->h << ")" << endl;
    selector->add(p);
  }
}
