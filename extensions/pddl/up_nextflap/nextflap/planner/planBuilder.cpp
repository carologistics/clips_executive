#include "planBuilder.h"
#include "z3Checker.h"

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Classes for building a new plan (search node).       */
/********************************************************/

using namespace std;

/********************************************************/
/* CLASS: PlanBuilderCausalLink                         */
/********************************************************/

PlanBuilderCausalLink::PlanBuilderCausalLink() {}

PlanBuilderCausalLink::PlanBuilderCausalLink(TVariable var, TValue v,
                                             TTimePoint p1, TTimePoint p2) {
  varValue = (((TVarValue)var) << 16) + v;
  ordering = (((TOrdering)p2) << 16) + p1;
}

PlanBuilderCausalLink::PlanBuilderCausalLink(TVarValue vv, TTimePoint p1,
                                             TTimePoint p2) {
  varValue = vv;
  ordering = (((TOrdering)p2) << 16) + p1;
}

/********************************************************/
/* CLASS: PlanBuilder                                   */
/********************************************************/

PlanBuilder::PlanBuilder(SASAction *a, TStep lastStep,
                         std::vector<std::vector<unsigned int>> *matrix,
                         int numSupportState, PlanEffects *planEffects,
                         SASTask *task) {
  this->task = task;
  action = a;
  this->matrix = matrix;
  this->planEffects = planEffects;
  this->iteration = planEffects->iteration;
  currentPrecondition = currentEffect = 0;
  setPrecondition = MAX_UNSIGNED_INT;
  lastTimePoint = stepToEndPoint(lastStep);
  this->numSupportState = numSupportState;
  if (a->conditionalEff.empty())
    this->condEffHold = nullptr;
  else {
    int size = a->conditionalEff.size();
    this->condEffHold = new bool[size];
    // for (int i = 0; i < size; i++) this->condEffHold[i] = false;
  }
}

PlanBuilder::~PlanBuilder() {
  if (condEffHold != nullptr)
    delete[] condEffHold;
}

// Deletes the las inserted causal link
void PlanBuilder::removeLastLink() {
  causalLinks.pop_back();
  removeLastOrdering();
}

// Deletes the last inserted ordering
void PlanBuilder::removeLastOrdering() {
  unsigned int newOrderings = numOrderingsAdded.back();
  numOrderingsAdded.pop_back();
  for (; newOrderings > 0; newOrderings--) {
    TOrdering o = orderings.back();
    orderings.pop_back();
    clearOrder(firstPoint(o), secondPoint(o));
    // cout << "  - Removing ord: " << Successors::firstPoint(o) << " -> " <<
    // Successors::secondPoint(o) << endl;
  }
}

// Add a new causal link to the plan
bool PlanBuilder::addLink(SASCondition *c, TTimePoint p1, TTimePoint p2) {
  if (addOrdering(p1, p2)) {
    causalLinks.emplace_back(c->var, c->value, p1, p2);
    return true;
  }
  return false;
}

// Add a new numeric causal link to the plan
bool PlanBuilder::addNumLink(TVariable v, TTimePoint p1, TTimePoint p2) {
  if (addOrdering(p1, p2)) {
    causalLinks.emplace_back(task->getVariableValueCode(v, MAX_UINT16), p1, p2);
    return true;
  }
  return false;
}

// Checks if TILs (timed initial literals) are properly scheduled
bool PlanBuilder::invalidTILorder(TTimePoint p1, TTimePoint p2) {
  TStep s1 = timePointToStep(p1), s2 = timePointToStep(p2);
  if (s1 < planEffects->planComponents->size() &&
      s2 < planEffects->planComponents->size()) {
    Plan *plan1 = planEffects->planComponents->get(s1);
    if (plan1->fixedInit) {
      Plan *plan2 = planEffects->planComponents->get(s2);
      if (plan2->fixedInit) {
        TFloatValue time1 = (p1 & 1) == 0 ? plan1->startPoint.updatedTime
                                          : plan1->endPoint.updatedTime;
        TFloatValue time2 = (p2 & 1) == 0 ? plan2->startPoint.updatedTime
                                          : plan2->endPoint.updatedTime;
        return time2 < time1;
      }
    }
  }
  return false;
}

// Computes a linear order of the plan timepoints
void PlanBuilder::topologicalOrder(std::vector<TTimePoint> *linearOrder) {
  unsigned int size = lastTimePoint + 1;
  linearOrder->resize(size, 0);
  std::vector<bool> visited(size, false);
  topologicalOrder(1, linearOrder, lastTimePoint, &visited);
}

// Computes a linear order of the plan timepoints
unsigned int PlanBuilder::topologicalOrder(TTimePoint orig,
                                           std::vector<TTimePoint> *linearOrder,
                                           unsigned int pos,
                                           std::vector<bool> *visited) {
  (*visited)[orig] = true;
  for (unsigned int i = 2; i <= linearOrder->size(); i++) {
    if (existOrder(orig, i)) {
      if (!((*visited)[i])) {
        // cout << orig << " -> " << i << endl;
        pos = topologicalOrder(i, linearOrder, pos, visited);
      }
    }
  }
  (*linearOrder)[pos--] = orig;
  return pos;
}

// Check if there are steps in the plan that need to be delayed
bool PlanBuilder::delaySteps(Plan *p, std::vector<TTimePoint> &pointToDelay,
                             std::vector<TFloatValue> &newTime,
                             std::vector<TTimePoint> &linearOrder) {
  // Following the linear order, try to delay the points that do not meet the
  // constraints. If one action cannot be delayed of it delays a previous point
  // (according to the linear order) we call Z3 to verify if there is a valid
  // schedule
  std::unordered_map<TTimePoint, int> indexInLinearOrder;
  for (int i = 0; i < linearOrder.size(); i++)
    indexInLinearOrder[linearOrder[i]] = i;
  PriorityQueue pq((unsigned int)linearOrder.size());
  for (int i = 0; i < pointToDelay.size(); i++) {
    TTimePoint tp = pointToDelay[i];
    if ((tp & 1) == 1) { // End of the action -> delay its begining
      Plan *step = planEffects->planComponents->get(timePointToStep(tp));
      TFloatValue startTime = step->startPoint.updatedTime + newTime[i] -
                              step->endPoint.updatedTime;
      pq.add(new PBTimepointToDelay(tp - 1, startTime,
                                    indexInLinearOrder[tp - 1]));
    } else {
      pq.add(new PBTimepointToDelay(pointToDelay[i], newTime[i],
                                    indexInLinearOrder[pointToDelay[i]]));
    }
  }
  std::vector<TFloatValue> resNewTimes(linearOrder.size(), FLOAT_INFINITY);
  bool ok = true;
  TStep currentStep = timePointToStep(lastTimePoint);
  while (pq.size() > 0 && ok) {
    PBTimepointToDelay *pqItem = (PBTimepointToDelay *)pq.poll();
    TTimePoint tp = pqItem->tp;
    TFloatValue pqNewTime = pqItem->newTime;
    int pqOrder = pqItem->linearOrderIndex;
    // cout << "Timepoint: " << tp << ", Order: " << pqOrder << ", Time: " <<
    // pqNewTime << endl;
    delete pqItem;
    if (resNewTimes[tp] != FLOAT_INFINITY &&
        resNewTimes[tp] < pqNewTime) { // New delay -> avoid inf. loops
      ok = false;                      // Let Z3 to solve
      break;
    }
    if (resNewTimes[tp] == FLOAT_INFINITY) { // Update the time if it is needed
      TStep indexStepToDelay = timePointToStep(tp);
      Plan *stepToDelay =
          indexStepToDelay < currentStep
              ? planEffects->planComponents->get(indexStepToDelay)
              : p;
      bool update = (tp & 1) == 0
                        ? stepToDelay->startPoint.updatedTime < pqNewTime
                        : stepToDelay->endPoint.updatedTime < pqNewTime;
      if (update) { // Update needed
        if (stepToDelay->fixedInit) {
          ok = false; // Action cannot be delayed
          break;
        }
        if ((tp & 1) == 0) { // Update also the end of the action
          TFloatValue inc = pqNewTime - stepToDelay->startPoint.updatedTime;
          resNewTimes[tp + 1] = stepToDelay->endPoint.updatedTime + inc;
          // Check the following steps that could also need to be delayed
          for (int nextIndex = indexInLinearOrder[tp + 1] + 1;
               nextIndex < linearOrder.size(); nextIndex++) {
            TTimePoint nextTimepoint = linearOrder[nextIndex];
            if (existOrder(tp + 1, nextTimepoint)) {
              // cout << "* Timepoint " << nextTimepoint << " added by " << (tp
              // + 1) << endl;
              pq.add(new PBTimepointToDelay(
                  nextTimepoint, resNewTimes[tp + 1] + EPSILON, nextIndex));
            }
          }
        }
        resNewTimes[tp] = pqNewTime;
        // Check the following steps that could also need to be delayed
        for (int nextIndex = pqOrder + 1; nextIndex < linearOrder.size();
             nextIndex++) {
          TTimePoint nextTimepoint = linearOrder[nextIndex];
          if (existOrder(tp, nextTimepoint)) {
            // cout << "* Timepoint " << nextTimepoint << " added by " << tp <<
            // endl;
            pq.add(new PBTimepointToDelay(
                nextTimepoint, resNewTimes[tp] + EPSILON, nextIndex));
          }
        }
      }
    }
  }
  while (pq.size() > 0)
    delete (PBTimepointToDelay *)pq.poll();
  if (ok) {
    for (int tp = 0; tp < resNewTimes.size(); tp++) {
      if (resNewTimes[tp] != FLOAT_INFINITY) {
        p->addPlanUpdate(tp, resNewTimes[tp]);
      }
    }
  }
  return ok;
}

// Adds an ordering to the plan
bool PlanBuilder::addOrdering(TTimePoint p1, TTimePoint p2) {
  if (p1 == p2 || existOrder(p2, p1))
    return false;
  if (existOrder(p1, p2))
    numOrderingsAdded.push_back(0); // Ordering already exists
  else if (invalidTILorder(p1, p2))
    return false;
  else {
    unsigned int newOrderings = 0;
    TTimePoint prevP1, nextP2;
    prevPoints.clear();
    nextPoints.clear();
    prevPoints.push_back(p1);
    nextPoints.push_back(p2);
    for (TTimePoint t = 1; t <= lastTimePoint; t++) {
      if (existOrder(t, p1))
        prevPoints.push_back(t);
      if (existOrder(p2, t))
        nextPoints.push_back(t);
    }
    for (unsigned int i = 0; i < prevPoints.size(); i++) {
      prevP1 = prevPoints[i];
      // cout << "* PrevPoint = " << prevP1 << " (from " << lastTimePoint << ")"
      // << endl;
      for (unsigned int j = 0; j < nextPoints.size(); j++) {
        nextP2 = nextPoints[j];
        // cout << "* NextPoint = " << nextP2 << endl;
        if (prevP1 != nextP2 && !existOrder(prevP1, nextP2)) {
          newOrderings++;
          setOrder(prevP1, nextP2);
          orderings.push_back(getOrdering(prevP1, nextP2));
          // cout << "+ Ord: " << prevP1 << " ---> " << nextP2 << endl;
        } // else cout << "   * Ord " << prevP1 << " -> " << nextP2 << " already
          // exists" << endl;
      }
    }
    numOrderingsAdded.push_back(newOrderings);
  }
  return true;
}

// Creates the new plan
Plan *PlanBuilder::generatePlan(Plan *basePlan, TPlanId idPlan) {
  Plan *p = new Plan(this->action, basePlan, idPlan, this->condEffHold);
  IntervalCalculations ic(this->action, this->numSupportState, planEffects,
                          task);
  ic.applyStartEffects(p, this->condEffHold);
  ic.applyEndEffects(p, this->condEffHold);
  if (!ic.supportedNumericEndConditions(this->condEffHold)) {
    delete p;
    return nullptr;
  }
  ic.copyControlVars(p);
  ic.copyDuration(p);
  setActionStartTime(p);
  // std::vector<TTimePoint> linearOrder;
  // topologicalOrder(&linearOrder);
  for (PlanBuilderCausalLink &pbcl : causalLinks) {
    if (pbcl.getValue() == MAX_UINT16)
      addNumericCausalLinkToPlan(p, pbcl.firstPoint(), pbcl.secondPoint(),
                                 pbcl.getVar());
    else
      addCausalLinkToPlan(p, pbcl.firstPoint(), pbcl.secondPoint(),
                          pbcl.varValue);
  }
  for (TOrdering o : orderings) {
    // cout << "Adding ordering: " << firstPoint(o) << "->" << secondPoint(o) <<
    // endl;
    p->orderings.push_back(o);
  }
  return p;
}

// Adds a new causal link to the plan
void PlanBuilder::addCausalLinkToPlan(Plan *p, TTimePoint p1, TTimePoint p2,
                                      TVarValue varValue) {
  PlanPoint &pPoint = ((p2 & 1) == 0) ? p->startPoint : p->endPoint;
  pPoint.addCausalLink(p1, varValue);
}

// Adds a new numeric causal link to the plan
void PlanBuilder::addNumericCausalLinkToPlan(Plan *p, TTimePoint p1,
                                             TTimePoint p2, TVariable var) {
  PlanPoint &pPoint = ((p2 & 1) == 0) ? p->startPoint : p->endPoint;
  pPoint.addNumericCausalLink(p1, var);
}

// Sets the start time of the new action (as soon as possible)
void PlanBuilder::setActionStartTime(Plan *p) {
  TTimePoint startNewStep = lastTimePoint - 1;
  p->startPoint.setInitialTime(EPSILON);
  for (TOrdering o : orderings) {
    TTimePoint endPoint = secondPoint(o);
    if (endPoint == startNewStep) { // []------>[   New step   ]
      TTimePoint startPoint = firstPoint(o);
      Plan *prevStep =
          planEffects->planComponents->get(timePointToStep(startPoint));
      if ((startPoint & 1) == 0) { // [---prevStep---]----->[   NewStep   ]
        TFloatValue time = prevStep->startPoint.updatedTime + EPSILON;
        if (p->startPoint.updatedTime < time) {
          p->startPoint.setInitialTime(time);
        }
      } else { // [   prevStep   ]----->[   NewStep   ]
        TFloatValue time = prevStep->endPoint.updatedTime + EPSILON;
        if (p->startPoint.updatedTime < time) {
          p->startPoint.setInitialTime(time);
        }
      }
    } else if (endPoint == lastTimePoint) { // []-------[---NewStep--->]
      TTimePoint startPoint = firstPoint(o);
      Plan *prevStep =
          planEffects->planComponents->get(timePointToStep(startPoint));
      if ((startPoint & 1) == 0) { // [---prevStep---]-----[---NewStep--->]
        TFloatValue time = prevStep->startPoint.updatedTime + EPSILON;
        if (p->startPoint.updatedTime + p->actionDuration.minValue < time) {
          p->startPoint.setInitialTime(time - p->actionDuration.minValue);
        }
      } else { // [   prevStep   ]-----[---NewStep--->]
        TFloatValue time = prevStep->endPoint.updatedTime + EPSILON;
        if (p->startPoint.updatedTime + p->actionDuration.minValue < time) {
          p->startPoint.setInitialTime(time - p->actionDuration.minValue);
        }
      }
    }
  }
  p->endPoint.setInitialTime(p->startPoint.updatedTime);
  if (p->actionDuration.maxValue < FLOAT_INFINITY)
    p->endPoint.setInitialTime(
        p->endPoint.updatedTime +
        (p->actionDuration.minValue + p->actionDuration.maxValue) / 2.0f);
  else
    p->endPoint.setInitialTime(p->endPoint.updatedTime +
                               p->actionDuration.minValue);
}

// Checks if the steps ordered after the new one must be delayed
bool PlanBuilder::checkFollowingSteps(Plan *p,
                                      std::vector<TTimePoint> &linearOrder) {
  std::vector<TTimePoint> pointToDelay;
  std::vector<TFloatValue> newTime;
  TTimePoint startNewStep = lastTimePoint - 1;
  for (TOrdering o : orderings) {
    TTimePoint startPoint = firstPoint(o);
    if (startPoint == startNewStep) { // [---NewStep---]----->[]
      TTimePoint endPoint = secondPoint(o);
      Plan *nextStep =
          planEffects->planComponents->get(timePointToStep(endPoint));
      if ((endPoint & 1) == 0) { // [---NewStep---]----->[         ]
        TFloatValue nextTime = nextStep->startPoint.updatedTime;
        if (p->startPoint.updatedTime + EPSILON >
            nextTime) { // The begining of the next step must be delayed
          pointToDelay.push_back(endPoint);
          newTime.push_back(p->startPoint.updatedTime + EPSILON);
        }
      } else { // [---NewStep---]------[-------->]
        TFloatValue nextTime = nextStep->endPoint.updatedTime;
        if (p->startPoint.updatedTime + EPSILON >
            nextTime) { // The end of the next step must be delayed
          pointToDelay.push_back(endPoint);
          newTime.push_back(p->startPoint.updatedTime + EPSILON);
        }
      }
    } else if (startPoint == lastTimePoint) { // [   NewStep   ]----->[]
      TTimePoint endPoint = secondPoint(o);
      Plan *nextStep =
          planEffects->planComponents->get(timePointToStep(endPoint));
      if ((endPoint & 1) == 0) { // [   NewStep   ]----->[         ]
        TFloatValue nextTime = nextStep->startPoint.updatedTime;
        if (p->startPoint.updatedTime + p->actionDuration.minValue + EPSILON >
            nextTime) { // The begining of the next step must be delayed
          pointToDelay.push_back(endPoint);
          newTime.push_back(p->startPoint.updatedTime +
                            p->actionDuration.minValue + EPSILON);
        }
      } else { // [   NewStep   ]------[-------->]
        TFloatValue nextTime = nextStep->endPoint.updatedTime;
        if (p->startPoint.updatedTime + p->actionDuration.minValue + EPSILON >
            nextTime) { // The end of the next step must be delayed
          pointToDelay.push_back(endPoint);
          newTime.push_back(p->startPoint.updatedTime +
                            p->actionDuration.minValue + EPSILON);
        }
      }
    }
  }
  if (pointToDelay.empty())
    return true;
  else
    return delaySteps(p, pointToDelay, newTime, linearOrder);
}
