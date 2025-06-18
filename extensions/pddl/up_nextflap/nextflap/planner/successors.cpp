/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Calculation of succesors of a given plan             */
/********************************************************/

#include "successors.h"
#include "intervalCalculations.h"
#include "printPlan.h"
using namespace std;

// #define DEBUG_SUCC_ON

/********************************************************/
/* CLASS: Successors                                    */
/********************************************************/

// Computes the order relationships among time points
void Successors::computeOrderMatrix() {
  if (currentIteration ==
      MAX_UNSIGNED_INT) { // Reset matrix (maximum number of iterations reached)
    currentIteration = 1;
    for (unsigned int i = 0; i < matrix.size(); i++)
      for (unsigned int j = 0; j < matrix[i].size(); j++)
        matrix[i][j] = 0;
  }
  newStep = planComponents.size(); // Steps start by 0
  TTimePoint lastPoint = stepToEndPoint(newStep);
  if (lastPoint >= matrix.size())
    resizeMatrix();
  for (unsigned int i = 0; i < planComponents.size(); i++) {
    if (planComponents.get(i)->action != nullptr) {
      matrix[stepToStartPoint(i)][stepToEndPoint(i)] =
          currentIteration; // Start point of the step always before the end
                            // point of the step
    }
  }
  matrix[lastPoint - 1][lastPoint] =
      currentIteration; // The same for the new step to be added
  for (unsigned int i = 0; i < planComponents.size(); i++) {
    Plan &p = *(planComponents.get(i));
    for (unsigned int j = 0; j < p.orderings.size(); j++) { // Plan orderings
      matrix[firstPoint(p.orderings[j])][secondPoint(p.orderings[j])] =
          currentIteration;
    }
    if (i > 0) {
      matrix[1][i << 1] = currentIteration; // Orderings with the initial step
      matrix[1][(i << 1) + 1] = currentIteration;
    }
  }
}

// Makes the order matrix larger
void Successors::resizeMatrix() {
  unsigned int newSize = (unsigned int)matrix.size() + MATRIX_INCREASE;
  matrix.resize(newSize);
  for (unsigned int i = 0; i < newSize; i++)
    matrix[i].resize(newSize, 0);
}

// Fill the planEffects matrix with the effects produced by the base plan
void Successors::computeBasePlanEffects(std::vector<TTimePoint> &linearOrder) {
  planEffects.setCurrentIteration(currentIteration, &planComponents);
  PlanPoint *planPoint;
  std::vector<SASCondition> *eff;
  for (int i = 1; i < linearOrder.size(); i++) {
    TTimePoint timePoint = linearOrder[i];
    TStep step = timePointToStep(timePoint);
    Plan *plan = planComponents.get(step);
    bool atStart = (timePoint & 1) == 0;
    if (atStart) {
      planPoint = &(plan->startPoint);
      eff = &(plan->action->startEff);
    } else {
      planPoint = &(plan->endPoint);
      eff = &(plan->action->endEff);
    }
    for (SASCondition &cond : *eff) {
      planEffects.addEffect(cond, timePoint);
    }
    if (plan->holdCondEff != nullptr) {
      for (int numCondEff : *plan->holdCondEff) {
        SASConditionalEffect &ce = plan->action->conditionalEff[numCondEff];
        eff = atStart ? &ce.startEff : &ce.endEff;
        for (SASCondition &cond : *eff) {
          planEffects.addEffect(cond, timePoint);
        }
      }
    }
    if (planPoint->numVarValues != nullptr) {
      for (TFluentInterval &fi : *(planPoint->numVarValues)) {
        planEffects.addNumEffect(fi, timePoint);
      }
    }
  }
}

void Successors::fullSuccessorsCalculation() {
  for (unsigned int i = 0; i < task->goals.size(); i++) {
    fullActionCheck(&(task->goals[i]), MAX_UINT16, 0, 0, 0);
  }
  for (unsigned int i = 0; i < task->actions.size(); i++) {
    fullActionCheck(&(task->actions[i]), MAX_UINT16, 0, 0, 0);
  }
}

// Checks if the given action can generate a successor plan
void Successors::fullActionCheck(SASAction *a, TVariable var, TValue value,
                                 TTimePoint effectTime,
                                 TTimePoint startTimeNewAction) {
  /*
  if (basePlan->id == 25 && a->name.compare("move-painting pos-2-2 pos-3-2 g1 n1
  n0") == 0) { cout << "aqui" << endl; PrintPlan::rawPrint(basePlan, task);
  }
  */
  if (supportedConditions(
          a)) { // Check if the (non-numeric) action preconditions can be
                // supported by the steps in the current base plan
    int numSupportState = supportedNumericConditions(a);
    if (numSupportState != -2) { // Supported
      // if (a->isGoal)
      //	cout << "aqui" << endl;
      // cout << "Action " << a->name << " supported" << endl;
      PlanBuilder pb(a, newStep, &matrix, numSupportState, &planEffects, task);
      unsigned int n = 0;
      if (var != MAX_UINT16) {
        n = addActionSupport(&pb, var, value, effectTime, startTimeNewAction);
      }
      if (numSupportState != -1)
        setNumericCausalLinks(&pb, numSupportState);
      else
        fullActionSupportCheck(&pb);
      for (unsigned int k = 0; k < n; k++)
        pb.removeLastLink();
    }
  }
}

void Successors::setNumericCausalLinks(PlanBuilder *pb, int numSupportState) {
  std::vector<TTimePoint> initialSupportingTimePoints;
  while (numSupportState >= 0 && this->solution == nullptr) {
    std::vector<TTimePoint> supportingTimePoints;
    computeSupportingTimePoints(pb->action, numSupportState,
                                &supportingTimePoints);
    if (supportingTimePoints != initialSupportingTimePoints) {
      addNumericSupport(pb, 0, &supportingTimePoints);
      initialSupportingTimePoints = supportingTimePoints;
    }
    numSupportState--;
  }
}

bool Successors::setNumericCausalLinks(PlanBuilder *pb, int numSupportState,
                                       SASConditionalEffect &e, int *numLinks) {
  std::vector<TVariable> vars;
  for (SASNumericCondition &c : e.startNumCond) {
    c.getVariables(&vars);
    for (TVariable v : vars) {
      int ns = numSupportState;
      while (planEffects.numStates[ns].values[v] == nullptr)
        ns--;
      TTimePoint tp = planEffects.numStates[ns].timepoint;
      if (!pb->addNumLink(v, tp, stepToStartPoint(newStep)))
        return false;
      (*numLinks)++;
    }
  }
  for (SASNumericCondition &c : e.endNumCond) {
    c.getVariables(&vars);
    for (TVariable v : vars) {
      int ns = numSupportState;
      while (planEffects.numStates[ns].values[v] == nullptr)
        ns--;
      TTimePoint tp = planEffects.numStates[ns].timepoint;
      if (!pb->addNumLink(v, tp, stepToEndPoint(newStep)))
        return false;
      (*numLinks)++;
    }
  }
  return true;
}

void Successors::computeSupportingTimePoints(
    SASAction *action, int numSupportState,
    std::vector<TTimePoint> *supportingTimePoints) {
  if (action->isGoal) {
    for (TVariable v : task->numVarReqGoal[action->index]) {
      int ns = numSupportState;
      while (planEffects.numStates[ns].values[v] == nullptr)
        ns--;
      supportingTimePoints->push_back(planEffects.numStates[ns].timepoint);
    }
  } else {
    for (TVariable v : task->numVarReqAtStart[action->index]) {
      int ns = numSupportState;
      while (planEffects.numStates[ns].values[v] == nullptr)
        ns--;
      supportingTimePoints->push_back(planEffects.numStates[ns].timepoint);
    }

    for (TVariable v : task->numVarReqAtEnd[action->index]) {
      int ns = numSupportState;
      while (planEffects.numStates[ns].values[v] == nullptr)
        ns--;
      supportingTimePoints->push_back(planEffects.numStates[ns].timepoint);
    }
  }
}

void Successors::addNumericSupport(
    PlanBuilder *pb, int numCond,
    std::vector<TTimePoint> *supportingTimePoints) {
  std::vector<TVariable> &atStart =
      pb->action->isGoal ? task->numVarReqGoal[pb->action->index]
                         : task->numVarReqAtStart[pb->action->index];
  if (numCond < atStart.size()) {
    TTimePoint tp = supportingTimePoints->at(numCond);
    TVariable v = atStart[numCond];
    if (pb->addNumLink(v, tp, stepToStartPoint(newStep))) {
      addNumericSupport(pb, numCond + 1, supportingTimePoints);
      pb->removeLastLink();
    }
  } else {
    if (pb->action->isGoal) {
      fullActionSupportCheck(pb); // Continue with non-numeric conditions
    } else {
      std::vector<TVariable> &atEnd = task->numVarReqAtEnd[pb->action->index];
      int n = numCond - (int)atStart.size();
      if (n < atEnd.size()) {
        TTimePoint tp = supportingTimePoints->at(numCond);
        TVariable v = atEnd[n];
        if (pb->addNumLink(v, tp, stepToEndPoint(newStep))) {
          addNumericSupport(pb, numCond + 1, supportingTimePoints);
          pb->removeLastLink();
        }
      } else {
        fullActionSupportCheck(pb); // Continue with non-numeric conditions
      }
    }
  }
}

int Successors::supportedNumericConditions(SASAction *a) {
  if (planEffects.numStates.empty())
    return -1; // Supported since there are no numeric fluents in the problem
  if (a->startNumCond.empty() && a->overNumCond.empty() &&
      a->endNumCond.empty())
    return -1; // Supported since action has no numeric conditions
  for (int i = (int)planEffects.numStates.size() - 1; i >= 0; i--) {
    IntervalCalculations ic(a, i, &planEffects, task);
    if (ic.supportedNumericStartConditions(nullptr))
      return i;
  }
  return -2; // Index of the numeric state that supports the numeric conditions.
             // -2 means unsupported conditions
}

int Successors::supportedNumericConditions(SASConditionalEffect *e,
                                           SASAction *a) {
  if (planEffects.numStates.empty())
    return -1; // Supported since there are no numeric fluents in the problem
  if (e->startNumCond.empty() && e->endNumCond.empty())
    return -1; // Supported since action has no numeric conditions
  for (int i = (int)planEffects.numStates.size() - 1; i >= 0; i--) {
    IntervalCalculations ic(a, i, &planEffects, task);
    if (ic.supportedNumericConditions(e))
      return i;
  }
  return -2; // Index of the numeric state that supports the numeric conditions.
             // -2 means unsupported conditions
}

bool Successors::supportedConditions(const SASAction *a) {
  for (unsigned int i = 0; i < a->startCond.size(); i++)
    if (!supportedCondition(a->startCond[i]))
      return false;
  for (unsigned int i = 0; i < a->overCond.size(); i++) {
    if (!supportedCondition(a->overCond[i]))
      return false;
  }
  // if (forceAtEndConditions) {
  for (unsigned int i = 0; i < a->endCond.size(); i++)
    if (!supportedCondition(a->endCond[i]))
      return false;
  //}
  return true;
}

Successors::Successors(TState *state, SASTask *task, bool forceAtEndConditions,
                       bool filterRepeatedStates,
                       std::vector<SASAction *> *tilActions)
    : planEffects(task) {
  this->task = task;
  this->initialState = state;
  this->filterRepeatedStates = filterRepeatedStates;
  numVariables = (unsigned int)task->variables.size();
  numActions = (unsigned int)task->actions.size();
  idPlan = 0;
  solution = nullptr;
  evaluator.initialize(state, task, tilActions, forceAtEndConditions);
  successors = nullptr;
  basePlan = nullptr;
  newStep = 0;
  for (unsigned int i = 0; i < numActions; i++) {
    checkedAction.push_back(0);
  }
  currentIteration = 0;
  matrix.resize(INITAL_MATRIX_SIZE);
  for (unsigned int i = 0; i < INITAL_MATRIX_SIZE; i++)
    matrix[i].resize(INITAL_MATRIX_SIZE, 0);
}

void Successors::checkConditionalEffects(PlanBuilder *pb, int numEff) {
  if (numEff >= pb->action->conditionalEff.size()) {
    generateSuccessor(pb);
  } else {
    SASConditionalEffect &e = pb->action->conditionalEff[numEff];
    int numLinks = pb->causalLinks.size();
    pb->condEffHold[numEff] = true;
    for (SASCondition &c : e.startCond)
      if (!holdConditionalCondition(c, pb, stepToStartPoint(newStep))) {
        pb->condEffHold[numEff] = false;
        break;
      }
    if (pb->condEffHold[numEff]) {
      for (SASCondition &c : e.endCond)
        if (!holdConditionalCondition(c, pb, stepToEndPoint(newStep))) {
          pb->condEffHold[numEff] = false;
          break;
        }
      if (pb->condEffHold[numEff]) {
        int numSupportState = supportedNumericConditions(&e, pb->action);
        if (numSupportState != -2) { // Supported
          // Conditions hold
          pb->condEffHold[numEff] =
              numSupportState == -1 ||
              setNumericCausalLinks(pb, numSupportState, e, &numLinks);
        } else
          pb->condEffHold[numEff] = false;
      }
    }
    if (pb->condEffHold[numEff]) { // Check threats
      checkConditionalThreats(numLinks, numEff, pb);
    }
    while (pb->causalLinks.size() > numLinks)
      pb->removeLastLink();
    if (!pb->condEffHold[numEff]) {
      checkConditionalEffects(pb, numEff + 1);
    }
  }
}

void Successors::checkConditionalThreats(int numLinks, int numEff,
                                         PlanBuilder *pb) {
  std::vector<Threat> threats;
  for (unsigned int i = numLinks; i < pb->causalLinks.size(); i++) {
    PlanBuilderCausalLink &cl = pb->causalLinks[i];
    TTimePoint p1 = cl.firstPoint(), p2 = cl.secondPoint();
    TVariable var = cl.getVar();
    TValue v = cl.getValue();
    if (v == MAX_UINT16) { // Numeric causal link
      for (NumVarChange &nvc : planEffects.numStates) {
        if (nvc.values[var] !=
            nullptr) { // Value of the num. vble. is changed at this timepoint
          TTimePoint pc = nvc.timepoint;
          if (!existOrder(pc, p1) && !existOrder(p2, pc) && pc != p1 &&
              pc != p2) {
            // cout << "Num. var. " << task->numVariables[var].name << " threat
            // by " << pc << endl;
            threats.emplace_back(p1, p2, pc, var, true);
          }
        }
      }
    } else { // SAS causal link
      // cout << "CL: " << task->variables[var].name << "=" <<
      // task->values[v].name << "  " << p1 << " --> " << p2 << endl;
      VarChange &vc = planEffects.varChanges[var];
      if (vc.iteration == currentIteration) {
        for (unsigned int j = 0; j < vc.timePoints.size(); j++) {
          if (vc.values[j] != v) {
            TTimePoint pc = vc.timePoints[j];
            // cout << "Dif. value in time point " << pc << endl;
            if (!existOrder(pc, p1) && !existOrder(p2, pc) && pc != p1 &&
                pc != p2) {
              // cout << "Threat by " << pc << endl;
              threats.emplace_back(p1, p2, pc, var, false);
            }
          }
        }
      }
    }
  }
  solveConditionalThreats(pb, &threats, numEff);
}

void Successors::solveConditionalThreats(PlanBuilder *pb,
                                         std::vector<Threat> *threats,
                                         int numEff) {
  // cout << threats->size() << " threats remaining" << endl;
  if (threats->size() == 0) {
    checkConditionalEffects(pb, numEff + 1);
  } else {
    Threat t = threats->back();
    threats->pop_back();
#ifdef DEBUG_SUCC_ON
    cout << t.p1 << " --> " << t.p2 << " (threatened by " << t.tp << ")"
         << endl;
#endif
    if (!existOrder(t.tp, t.p1) &&
        !existOrder(t.p2, t.tp)) { // Threat already exists
      bool promotion, demotion;
      if (mutexPoints(t.tp, t.p2, t.var, pb)) {
#ifdef DEBUG_SUCC_ON
        cout << "Unsolvable threat" << endl;
#endif
        promotion = demotion = false;
      } else {
        promotion = t.p1 > 1 && !existOrder(t.p1, t.tp);
        demotion = !existOrder(t.tp, t.p2);
      }
      if (promotion && demotion) { // Both choices are possible
#ifdef DEBUG_SUCC_ON
        cout << "Promotion and demotion valid" << endl;
#endif

        if (pb->addOrdering(t.p2, t.tp)) {
          solveConditionalThreats(pb, threats, numEff);
          pb->removeLastOrdering();
          if (pb->addOrdering(t.tp, t.p1)) {
            solveConditionalThreats(pb, threats, numEff);
            pb->removeLastOrdering();
          }
        } else if (pb->addOrdering(t.tp, t.p1)) {
          solveConditionalThreats(pb, threats, numEff);
          pb->removeLastOrdering();
        } else
          pb->condEffHold[numEff] = false;
      } else if (demotion) { // Only demotion is possible: p2 -> tp
#ifdef DEBUG_SUCC_ON
        cout << "Demotion valid" << endl;
        cout << "Order " << t.p2 << " -> " << t.tp << " added" << endl;
#endif
        if (pb->addOrdering(t.p2, t.tp)) {
          solveConditionalThreats(pb, threats, numEff);
          pb->removeLastOrdering();
        } else
          pb->condEffHold[numEff] = false;
      } else if (promotion) { // Only promotion is possible: tp -> p1
#ifdef DEBUG_SUCC_ON
        cout << "Promotion valid" << endl;
        cout << "Order " << t.tp << " -> " << t.p1 << " added" << endl;
#endif
        if (pb->addOrdering(t.tp, t.p1)) {
          solveConditionalThreats(pb, threats, numEff);
          pb->removeLastOrdering();
        } else
          pb->condEffHold[numEff] = false;
      } else {
        pb->condEffHold[numEff] = false;
#ifdef DEBUG_SUCC_ON
        // Unsolvable threat
        cout << "Unsolvable threat" << endl;
#endif
      }
    } else {
#ifdef DEBUG_SUCC_ON
      cout << "Not a threat now" << endl;
#endif
      solveConditionalThreats(pb, threats, numEff);
    }
  }
}

bool Successors::holdConditionalCondition(SASCondition &c, PlanBuilder *pb,
                                          TTimePoint condPoint) {
  if (currentIteration == planEffects.planEffects[c.var][c.value].iteration) {
    vector<TTimePoint> *supports =
        &(planEffects.planEffects[c.var][c.value].timePoints);
    for (unsigned int i = 0; i < supports->size(); i++) {
      TTimePoint p = (*supports)[i];
      // cout << "+ CL: " << p << " ---> " << condPoint << " (" <<
      // task->variables[c->var].name << "," << task->values[c->value].name <<
      // ")" << endl;
      if (pb->addLink(&c, p, condPoint)) { // Causal link added: p --- (c->var =
                                           // c->value) ----> condPoint
        return true;
      }
    }
  }
  return false;
}

void Successors::checkCondEffConditions(int numEff, int numCond,
                                        PlanBuilder *pb) {
  if (numEff >= pb->action->conditionalEff.size()) {
    generateSuccessor(pb);
  } else {
    SASConditionalEffect &e = pb->action->conditionalEff[numEff];
    if (numCond < e.startCond.size()) {
      checkCondEffCondition(numEff, numCond, &e.startCond[numCond],
                            stepToStartPoint(newStep), pb);
    } else if (numCond < e.startCond.size() + e.endCond.size()) {
      int index = numCond - e.startCond.size();
      checkCondEffCondition(numEff, numCond, &e.endCond[index],
                            stepToEndPoint(newStep), pb);
    } else { // Check the numeric conditions
      int numSupportState = supportedNumericConditions(&e, pb->action);
      if (numSupportState != -2) { // Supported
        // Conditions hold
        int numLinks = 0;
        if (numSupportState == -1 ||
            setNumericCausalLinks(pb, numSupportState, e, &numLinks)) {
          pb->condEffHold[numEff] = true;
          checkCondEffConditions(numEff + 1, 0, pb);
          while (numLinks > 0) {
            pb->removeLastLink();
            numLinks--;
          }
        }
      }
    }
    if (numCond == 0 && !pb->condEffHold[numEff]) { // Conditions do not hold ->
                                                    // next conditional effect
      checkCondEffConditions(numEff + 1, 0, pb);
    }
  }
}

void Successors::checkCondEffCondition(int numEff, int numCond, SASCondition *c,
                                       TTimePoint condPoint, PlanBuilder *pb) {
  if (currentIteration == planEffects.planEffects[c->var][c->value].iteration) {
    vector<TTimePoint> *supports =
        &(planEffects.planEffects[c->var][c->value].timePoints);
    for (unsigned int i = 0; i < supports->size(); i++) {
      TTimePoint p = (*supports)[i];
      // cout << "+ CL: " << p << " ---> " << condPoint << " (" <<
      // task->variables[c->var].name << "," << task->values[c->value].name <<
      // ")" << endl;
      if (pb->addLink(c, p, condPoint)) { // Causal link added: p --- (c->var =
                                          // c->value) ----> condPoint
        checkCondEffConditions(numEff, numCond + 1, pb);
        pb->removeLastLink();
      }
    }
  }
}

// Generates a successor plan from the plan builder data
void Successors::generateSuccessor(PlanBuilder *pb) {
  pb->addOrdering(pb->lastTimePoint - 1,
                  pb->lastTimePoint); // Ordering from the begining to the end
                                      // of the new step
  Plan *p = pb->generatePlan(basePlan, ++idPlan);
  if (p != nullptr) {
    addSuccessor(p);
  } else {
    // cout << "* Successor invalid: " << idPlan << ", " << pb->action->name <<
    // endl;
  }
  pb->removeLastOrdering();
}

// Destructor
Successors::~Successors() {}

void Successors::addSuccessor(Plan *p) {
  if (PrintPlan::getMakespan(p) > bestMakespan) {
    delete p;
  } else {
    evaluator.calculateFrontierState(p);
    evaluator.evaluate(p);

    // cout << "* Successor: " << idPlan << ", " << p->action->name << "(G=" <<
    // p->g << ", H=" << p->h << ")" << endl; cout << "Plan " << p->id << " ("
    // << p->action->name << ") generated" << endl;
    if (p->isSolution()) {
      // cout << "SOLUTION PLAN" << endl;
      solution = p;
    } else {
      successors->push_back(p);
    }
  }
}

// Fills vector suc with the possible successor plans of the given base plan
void Successors::computeSuccessors(Plan *base, std::vector<Plan *> *suc,
                                   float bestMakespan) {
  this->basePlan = base;
  this->bestMakespan = bestMakespan;
  currentIteration++;
  planComponents.calculate(base);
  computeOrderMatrix();
  linearizer.linearize(planComponents);
  computeBasePlanEffects(linearizer.linearOrder);
  successors = suc;
  suc->clear();
  for (SASAction &a : task->goals) {
    fullActionCheck(&a, MAX_UINT16, 0, 0, 0);
  }
  if (base->isRoot() ||
      base->action->endEff.empty()) { // Full calculation of successors
    fullSuccessorsCalculation();
  } else { // Calculation of successores based on the parent plan
    computeSuccessorsSupportedByLastActions();
    computeSuccessorsThroughBrotherPlans();
  }
}

bool Successors::repeatedState(Plan *p) {
  if (!filterRepeatedStates)
    return false;
  uint64_t code = p->fs->getCode();
  std::unordered_map<uint64_t, std::vector<Plan *>>::const_iterator got =
      memo.find(code);
  if (got != memo.end()) {
    for (Plan *op : got->second) {
      if (p->fs->compareTo(op->fs)) {
        return true; // Repeated state
      }
    }
    memo[code].push_back(p);
  } else {
    std::vector<Plan *> v;
    v.push_back(p);
    memo[code] = v;
  }
  return false;
}

// Computes the succesors obtained by adding new actions which are supported by
// the last action added in the base plan
void Successors::computeSuccessorsSupportedByLastActions() {
  if (basePlan->repeatedState)
    return;
  SASAction *a = basePlan->action;
  TTimePoint startTimeNewAction = stepToStartPoint(newStep);
  TTimePoint startTimeLastAction = startTimeNewAction - 2;
  for (SASCondition &c : a->startEff) {
    vector<SASAction *> &req = task->requirers[c.var][c.value];
    for (SASAction *ra : req) {
      if (!visitedAction(ra)) {
        setVisitedAction(ra);
        // cout << "Action " << ra->name << " supported by at-start" << endl;
        fullActionCheck(ra, c.var, c.value, startTimeLastAction,
                        startTimeNewAction);
      }
    }
  }
  for (SASCondition &c : a->endEff) {
    vector<SASAction *> &req = task->requirers[c.var][c.value];
    for (SASAction *ra : req) {
      if (!visitedAction(ra)) {
        setVisitedAction(ra);
        // cout << "Action " << ra->name << " supported by at-end" << endl;
        fullActionCheck(ra, c.var, c.value, startTimeLastAction + 1,
                        startTimeNewAction);
      }
    }
  }
  for (SASNumericEffect &c : a->startNumEff) {
    vector<SASAction *> &req = task->numRequirers[c.var];
    for (SASAction *ra : req) {
      if (!visitedAction(ra)) {
        setVisitedAction(ra);
        fullActionCheck(ra, MAX_UINT16, 0, startTimeLastAction,
                        startTimeNewAction);
      }
    }
  }
  for (SASNumericEffect &c : a->endNumEff) {
    vector<SASAction *> &req = task->numRequirers[c.var];
    for (SASAction *ra : req) {
      if (!visitedAction(ra)) {
        setVisitedAction(ra);
        fullActionCheck(ra, MAX_UINT16, 0, startTimeLastAction + 1,
                        startTimeNewAction);
      }
    }
  }
}

// Adds a causal link to support one precondition. Return the number of links
// added (two in the case of over-all conditions)
unsigned int Successors::addActionSupport(PlanBuilder *pb, TVariable var,
                                          TValue value, TTimePoint effectTime,
                                          TTimePoint startTimeNewAction) {
  SASAction *a = pb->action;
  for (unsigned int i = 0; i < a->startCond.size(); i++) {
    if (a->startCond[i].var == var && a->startCond[i].value == value) {
      pb->setPrecondition = i;
      // cout << "+ CLS: " << effectTime << " ---> " << startTimeNewAction << "
      // (" << task->variables[var].name << "," << task->values[value].name <<
      // ")" << endl;
      if (pb->addLink(&(a->startCond[i]), effectTime, startTimeNewAction))
        return 1;
      return 0;
    }
  }
  for (unsigned int i = 0; i < a->overCond.size(); i++) {
    if (a->overCond[i].var == var && a->overCond[i].value == value) {
      pb->setPrecondition = i + (unsigned int)a->startCond.size();
      // cout << "+ CLO: " << effectTime << " ---> " << startTimeNewAction << "
      // (" << task->variables[var].name << "," << task->values[value].name <<
      // ")" << endl; cout << "+ CLO: " << effectTime << " ---> " <<
      // (startTimeNewAction+1) << " (" << task->variables[var].name << "," <<
      // task->values[value].name << ")" << endl;
      if (pb->addLink(&(a->overCond[i]), effectTime, startTimeNewAction)) {
        if (pb->addLink(&(a->overCond[i]), effectTime, startTimeNewAction + 1))
          return 2;
        pb->removeLastLink();
        return 0;
      }
      return 0;
    }
  }
  for (unsigned int i = 0; i < a->endCond.size(); i++) {
    if (a->endCond[i].var == var && a->endCond[i].value == value) {
      pb->setPrecondition = i + (unsigned int)a->startCond.size() +
                            (unsigned int)a->overCond.size();
      // cout << "+ CLE: " << effectTime << " ---> " << (startTimeNewAction+1)
      // << " (" << task->variables[var].name << "," << task->values[value].name
      // << ")" << endl;
      if (pb->addLink(&(a->endCond[i]), effectTime, startTimeNewAction + 1))
        return 1;
      return 0;
    }
  }
  return 0;
}

void Successors::computeSuccessorsThroughBrotherPlans() {
  Plan *parentPlan = basePlan->parentPlan;
  vector<Plan *> *brotherPlans = parentPlan->childPlans;
  for (unsigned int i = 0; i < brotherPlans->size(); i++) {
    Plan *brotherPlan = (*brotherPlans)[i];
    if (brotherPlan != basePlan && !brotherPlan->expanded() &&
        !visitedAction(brotherPlan->action)) {
      setVisitedAction(brotherPlan->action);
      fullActionCheck(brotherPlan->action, MAX_UINT16, 0, 0, 0);
    }
  }
}

// Checks if it is possible to support the next precondition of the action
void Successors::fullActionSupportCheck(PlanBuilder *pb) {
  if (pb->currentPrecondition == pb->setPrecondition) {
    pb->currentPrecondition++;
    fullActionSupportCheck(pb);
    pb->currentPrecondition--;
  } else if (pb->currentPrecondition <
             pb->action->startCond.size()) { // At-start condition
    fullConditionSupportCheck(pb,
                              &(pb->action->startCond[pb->currentPrecondition]),
                              stepToStartPoint(newStep), false, false);
  } else if (pb->currentPrecondition <
             pb->action->startCond.size() +
                 pb->action->overCond.size()) { // Over-all condition
    fullConditionSupportCheck(
        pb,
        &(pb->action->overCond[pb->currentPrecondition -
                               pb->action->startCond.size()]),
        stepToStartPoint(newStep), true, false);
  } else if (pb->currentPrecondition <
             pb->action->startCond.size() + pb->action->overCond.size() +
                 pb->action->endCond.size()) { // At-end condition
    fullConditionSupportCheck(
        pb,
        &(pb->action
              ->endCond[pb->currentPrecondition - pb->action->startCond.size() -
                        pb->action->overCond.size()]),
        stepToEndPoint(newStep), false, /*!forceAtEndConditions*/ false);
  } else { // Al condition supported -> check threats
    checkThreats(pb);
  }
}

// Supports a non-numeric action condition, solving the threats that appear (if
// any)
void Successors::fullConditionSupportCheck(PlanBuilder *pb, SASCondition *c,
                                           TTimePoint condPoint, bool overAll,
                                           bool canLeaveOpen) {
  // cout << "Checking condition " << task->variables[c->var].name << "," <<
  // task->values[c->value].name << " for action " << pb->action->name << endl;
  bool supportFound = false;
  if (currentIteration == planEffects.planEffects[c->var][c->value].iteration) {
    vector<TTimePoint> *supports =
        &(planEffects.planEffects[c->var][c->value].timePoints);
    for (unsigned int i = 0; i < supports->size(); i++) {
      TTimePoint p = (*supports)[i];
      // cout << "+ CL: " << p << " ---> " << condPoint << " (" <<
      // task->variables[c->var].name << "," << task->values[c->value].name <<
      // ")" << endl;
      if (pb->addLink(c, p, condPoint)) { // Causal link added: p --- (c->var =
                                          // c->value) ----> condPoint
        if (overAll)
          pb->addLink(c, p, condPoint + 1);
        pb->currentPrecondition++;
        fullActionSupportCheck(pb);
        pb->currentPrecondition--;
        pb->removeLastLink();
        if (overAll)
          pb->removeLastLink();
        supportFound = true;
      }
    }
  }
  if (!supportFound && canLeaveOpen) {
    int precNumber = pb->currentPrecondition -
                     (int)pb->action->startCond.size() -
                     (int)pb->action->overCond.size();
    pb->openCond.push_back(precNumber);
    pb->currentPrecondition++;
    // cout << "Leaving precondition open: " << precNumber << endl;
    fullActionSupportCheck(pb);
    pb->currentPrecondition--;
  }
}

// Threats between the causal links in the base plan and the effects of the new
// action
void Successors::checkThreatsBetweenCausalLinksInBasePlanAndNewActionEffects(
    PlanBuilder *pb, std::vector<Threat> *threats) {
  TTimePoint p2 = 0;
  for (TStep i = 0; i < planComponents.size(); i++) {
    Plan *planComp = planComponents.get(i);
    for (TCausalLink &cl : planComp->startPoint.causalLinks) {
      checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(pb, threats, cl,
                                                                p2);
    }
    for (TNumericCausalLink &cl : planComp->startPoint.numCausalLinks) {
      checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(pb, threats, cl,
                                                                p2);
    }
    p2++;
    for (TCausalLink &cl : planComp->endPoint.causalLinks) {
      checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(pb, threats, cl,
                                                                p2);
    }
    for (TNumericCausalLink &cl : planComp->endPoint.numCausalLinks) {
      checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(pb, threats, cl,
                                                                p2);
    }
    p2++;
  }
}

void Successors::checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(
    PlanBuilder *pb, std::vector<Threat> *threats, TCausalLink &cl,
    TTimePoint p2) {
  TTimePoint p1 = cl.timePoint, pc = pb->lastTimePoint - 1;
  if (!existOrder(pc, p1) && !existOrder(p2, pc)) {
    TVariable var = task->getVariableIndex(cl.varVal);
    TValue v = task->getValueIndex(cl.varVal);
    // cout << " - Threat : " << p1 << " -- " << task->variables[var].name <<
    // "," << task->values[v].name << " --> " << p2 << endl;
    for (SASCondition &eff : pb->action->startEff) {
      if (eff.var == var && eff.value != v) {
        threats->emplace_back(p1, p2, pc, var, false);
        // cout << "   Threat found" << endl;
        break;
      }
    }
    pc++;
    for (SASCondition &eff : pb->action->endEff) {
      if (eff.var == var && eff.value != v) {
        threats->emplace_back(p1, p2, pc, var, false);
        // cout << "   Threat found" << endl;
        break;
      }
    }
    pc--;
  }
}

void Successors::checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(
    PlanBuilder *pb, std::vector<Threat> *threats, TNumericCausalLink &cl,
    TTimePoint p2) {
  TTimePoint p1 = cl.timePoint, pc = pb->lastTimePoint - 1;
  if (!existOrder(pc, p1) && !existOrder(p2, pc)) {
    TVariable var = cl.var;
    // cout << " - Threat : " << p1 << " -- " << task->numVariables[var].name <<
    // " --> " << p2 << endl;
    for (SASNumericEffect &eff : pb->action->startNumEff) {
      if (eff.var == var) {
        threats->emplace_back(p1, p2, pc, var, true);
        // cout << "   Threat found" << endl;
        break;
      }
    }
    pc++;
    for (SASNumericEffect &eff : pb->action->endNumEff) {
      if (eff.var == var) {
        threats->emplace_back(p1, p2, pc, var, false);
        // cout << "   Threat found" << endl;
        break;
      }
    }
    pc--;
  }
}

// Threats between the new causal links and the actions in the base plan
void Successors::checkThreatsBetweenNewCausalLinksAndActionsInBasePlan(
    PlanBuilder *pb, std::vector<Threat> *threats) {
  for (PlanBuilderCausalLink &cl : pb->causalLinks) {
    TTimePoint p1 = cl.firstPoint(), p2 = cl.secondPoint();
    TVariable var = cl.getVar();
    TValue v = cl.getValue();
    if (v == MAX_UINT16) { // Numeric causal link
      for (NumVarChange &nvc : planEffects.numStates) {
        if (nvc.values[var] !=
            nullptr) { // Value of the num. vble. is changed at this timepoint
          TTimePoint pc = nvc.timepoint;
          if (!existOrder(pc, p1) && !existOrder(p2, pc) && pc != p1 &&
              pc != p2) {
            // cout << "Num. var. " << task->numVariables[var].name << " threat
            // by " << pc << endl;
            threats->emplace_back(p1, p2, pc, var, true);
          }
        }
      }
    } else { // SAS causal link
      // cout << "CL: " << task->variables[var].name << "=" <<
      // task->values[v].name << "  " << p1 << " --> " << p2 << endl;
      VarChange &vc = planEffects.varChanges[var];
      if (vc.iteration == currentIteration) {
        for (unsigned int j = 0; j < vc.timePoints.size(); j++) {
          if (vc.values[j] != v) {
            TTimePoint pc = vc.timePoints[j];
            // cout << "Dif. value in time point " << pc << endl;
            if (!existOrder(pc, p1) && !existOrder(p2, pc) && pc != p1 &&
                pc != p2) {
              // cout << "Threat by " << pc << endl;
              threats->emplace_back(p1, p2, pc, var, false);
            }
          }
        }
      }
    }
  }
}

// Check the threats between the causal links of the base plan and the new
// action
void Successors::checkThreats(PlanBuilder *pb) {
  vector<Threat> threats;
  checkThreatsBetweenCausalLinksInBasePlanAndNewActionEffects(pb, &threats);
  checkThreatsBetweenNewCausalLinksAndActionsInBasePlan(pb, &threats);
  solveThreats(pb, &threats);
}

// Checks if in both time-steps the same fluent (var=value) is required, and in
// both time-steps that variable is modified.
bool Successors::mutexPoints(TTimePoint p1, TTimePoint p2, TVariable var,
                             PlanBuilder *pb) {
  TStep s1 = p1 >> 1, s2 = p2 >> 1;
  SASAction *a1 =
      s1 == planComponents.size() ? pb->action : planComponents.get(s1)->action;
  SASAction *a2 =
      s2 == planComponents.size() ? pb->action : planComponents.get(s2)->action;
  if (a1->instantaneous || a2->instantaneous) {
    SASCondition *c1 = getRequiredValue(a1, var);
    if (c1 == nullptr || !c1->isModified)
      return false;
    SASCondition *c2 = getRequiredValue(a2, var);
    return c2 != nullptr && c2->isModified && c2->value == c1->value;
  } else {
    SASCondition *c1 = getRequiredValue(p1, a1, var);
    if (c1 == nullptr || !c1->isModified)
      return false;
    SASCondition *c2 = getRequiredValue(p2, a2, var);
    return c2 != nullptr && c2->isModified && c2->value == c1->value;
  }
}

SASCondition *Successors::getRequiredValue(SASAction *a, TVariable var) {
  for (SASCondition &c : a->startCond)
    if (c.var == var)
      return &c;
  return nullptr;
}

SASCondition *Successors::getRequiredValue(TTimePoint p, SASAction *a,
                                           TVariable var) {
  std::vector<SASCondition> *cond =
      (p & 1) == 0 ? &(a->startCond) : &(a->endCond);
  for (unsigned int i = 0; i < cond->size(); i++)
    if ((*cond)[i].var == var)
      return &((*cond)[i]);
  cond = &(a->overCond); // Check over-all conditions then
  for (unsigned int i = 0; i < cond->size(); i++)
    if ((*cond)[i].var == var)
      return &((*cond)[i]);
  return nullptr;
}

// Solves the threats in the plan
void Successors::solveThreats(PlanBuilder *pb, std::vector<Threat> *threats) {
  // cout << threats->size() << " threats remaining" << endl;
  if (threats->size() == 0) {
    checkContradictoryEffects(pb);
  } else {
    Threat t = threats->back();
    threats->pop_back();
#ifdef DEBUG_SUCC_ON
    cout << t.p1 << " --> " << t.p2 << " (threatened by " << t.tp << ")"
         << endl;
#endif
    if (!existOrder(t.tp, t.p1) &&
        !existOrder(t.p2, t.tp)) { // Threat already exists
      bool promotion, demotion;
      if (mutexPoints(t.tp, t.p2, t.var, pb)) {
#ifdef DEBUG_SUCC_ON
        cout << "Unsolvable threat" << endl;
#endif
        promotion = demotion = false;
      } else {
        promotion = t.p1 > 1 && !existOrder(t.p1, t.tp);
        demotion = !existOrder(t.tp, t.p2);
      }
      if (promotion && demotion) { // Both choices are possible
#ifdef DEBUG_SUCC_ON
        cout << "Promotion and demotion valid" << endl;
#endif
        if (pb->addOrdering(t.p2, t.tp)) {
          solveThreats(pb, threats);
          pb->removeLastOrdering();
        }
        if (pb->addOrdering(t.tp, t.p1)) {
          solveThreats(pb, threats);
          pb->removeLastOrdering();
        }
      } else if (demotion) { // Only demotion is possible: p2 -> tp
#ifdef DEBUG_SUCC_ON
        cout << "Demotion valid" << endl;
        cout << "Order " << t.p2 << " -> " << t.tp << " added" << endl;
#endif
        if (pb->addOrdering(t.p2, t.tp)) {
          solveThreats(pb, threats);
          pb->removeLastOrdering();
        }
      } else if (promotion) { // Only promotion is possible: tp -> p1
#ifdef DEBUG_SUCC_ON
        cout << "Promotion valid" << endl;
        cout << "Order " << t.tp << " -> " << t.p1 << " added" << endl;
#endif
        if (pb->addOrdering(t.tp, t.p1)) {
          solveThreats(pb, threats);
          pb->removeLastOrdering();
        }
      }
#ifdef DEBUG_SUCC_ON
      else { // Unsolvable threat
        cout << "Unsolvable threat" << endl;
      }
#endif
    } else {
#ifdef DEBUG_SUCC_ON
      cout << "Not a threat now" << endl;
#endif
      solveThreats(pb, threats);
    }
  }
}

void Successors::checkContradictoryEffects(PlanBuilder *pb) {
  if (pb->currentEffect < pb->action->startEff.size()) { // At-start effect
    checkContradictoryEffects(pb, &(pb->action->startEff[pb->currentEffect]),
                              stepToStartPoint(newStep));
  } else if (pb->currentEffect <
             pb->action->endEff.size() +
                 pb->action->startEff.size()) { // End effect
    checkContradictoryEffects(
        pb,
        &(pb->action->endEff[pb->currentEffect - pb->action->startEff.size()]),
        stepToEndPoint(newStep));
  } else {
    checkConditionalEffects(pb, 0);
  }
}

void Successors::checkContradictoryEffects(PlanBuilder *pb, SASCondition *c,
                                           TTimePoint effPoint) {
  VarChange &vc = planEffects.varChanges[c->var];
  if (vc.iteration == currentIteration) {
    for (unsigned int j = 0; j < vc.timePoints.size(); j++) {
      if (vc.values[j] != c->value) {
        TTimePoint p = vc.timePoints[j];
        if (p > 1 && !existOrder(p, effPoint) && !existOrder(effPoint, p)) {
          if (pb->addOrdering(p, effPoint)) {
            checkContradictoryEffects(pb, c, effPoint);
            pb->removeLastOrdering();
          }
          if (pb->addOrdering(effPoint, p)) {
            checkContradictoryEffects(pb, c, effPoint);
            pb->removeLastOrdering();
          }
          return;
          // cout << task->variables[c->var].name << ": " <<
          // task->values[vc.values[j]].name << " <---> " <<
          // task->values[c->value].name << endl;
        }
      }
    }
  }
  pb->currentEffect++;
  checkContradictoryEffects(pb);
  pb->currentEffect--;
}
