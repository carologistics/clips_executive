/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* POP plan
 */
/********************************************************/

#include "plan.h"
using namespace std;

/********************************************************/
/* CLASS: Plan                                          */
/********************************************************/

Plan::Plan(SASAction *action, Plan *parentPlan, TPlanId idPlan,
           bool *holdCondEff) {
  this->parentPlan = parentPlan;
  this->action = action;
  this->childPlans = nullptr;
  this->id = idPlan;
  this->cvarValues = nullptr;
  this->planUpdates = nullptr;
  this->fixedInit = false;
  if (parentPlan != nullptr)
    this->g = parentPlan->g + 1;
  else
    this->g = 0;
  this->h = (int)MAX_UINT16;
  this->hLand = 0;
  this->repeatedState = false;
  this->fs = nullptr;
  z3Checked = false;
  invalid = false;
  this->holdCondEff = nullptr;
  if (holdCondEff != nullptr) {
    for (unsigned int i = 0; i < action->conditionalEff.size(); i++) {
      if (holdCondEff[i]) {
        addConditionalEffect(i);
      }
    }
  }
}

Plan::~Plan() {
  if (fs != nullptr)
    delete fs;
  if (holdCondEff != nullptr)
    delete holdCondEff;
}

void Plan::addFluentIntervals(PlanPoint &pp,
                              std::vector<SASNumericEffect> &eff) {
  if (eff.size() > 0) {
    pp.numVarValues = new std::vector<TFluentInterval>();
    for (int i = 0; i < eff.size(); i++) {
      SASNumericEffect *ne = &(eff[i]);
      pp.numVarValues->emplace_back(ne->var, ne->exp.value, ne->exp.value);
    }
  }
}

void Plan::addConditionalEffect(unsigned int numEff) {
  if (holdCondEff == nullptr) {
    holdCondEff = new vector<int>();
  }
  holdCondEff->push_back(numEff);
}

void Plan::setDuration(TFloatValue min, TFloatValue max) {
  this->actionDuration.minValue = min;
  this->actionDuration.maxValue = max;
}

void Plan::setTime(TTime init, TTime end, bool fixed) {
  this->startPoint.setInitialTime(init);
  this->fixedInit = fixed;
  this->endPoint.setInitialTime(end);
}

// Compares this plan with the given one. Returns a negative number if this is
// better, 0 if both are equally good or a positive number if p is better
int Plan::compare(Plan *p) {
  if (SIGNIFICATIVE_LANDMARKS) {
    int v1 = g + h + 2 * hLand, v2 = p->g + p->h + 2 * p->hLand;
    if (v1 < v2)
      return -1;
    if (v1 > v2)
      return 1;
    return 0;
  } else {
    int v1 = g + 2 * h, v2 = p->g + 2 * p->h;
    if (v1 < v2)
      return -1;
    if (v1 > v2)
      return 1;
    return 0;
  }
}

bool Plan::isRoot() { return parentPlan == nullptr || this->action->isTIL; }

void Plan::addFluentIntervals() {
  addFluentIntervals(this->startPoint, this->action->startNumEff);
  addFluentIntervals(this->endPoint, this->action->endNumEff);
}

void Plan::addChildren(std::vector<Plan *> &suc) {
  childPlans = new std::vector<Plan *>(suc);
}

void Plan::addPlanUpdate(TTimePoint tp, TFloatValue time) {
  if (planUpdates == nullptr)
    planUpdates = new std::vector<TPlanUpdate>();
  planUpdates->emplace_back(tp, time);
}

int Plan::getCheckDistance() {
  if (z3Checked || parentPlan == nullptr)
    return 0;
  return 1 + parentPlan->getCheckDistance();
}

/*
int Plan::getH(int queue)
{
        if (queue == SEARCH_NRPG) return h;
        return hLand;
}
*/

void PlanPoint::addCausalLink(TTimePoint timePoint, TVarValue varVal) {
  for (TCausalLink &cl : causalLinks)
    if (cl.varVal == varVal)
      return; // Repeated causal link
  causalLinks.emplace_back(timePoint, varVal);
}

void PlanPoint::addNumericCausalLink(TTimePoint timePoint, TVarValue var) {
  for (TNumericCausalLink &cl : numCausalLinks)
    if (cl.var == var)
      return; // Repeated causal link
  numCausalLinks.emplace_back(timePoint, var);
}

void PlanPoint::addNumericValue(TVariable v, TFloatValue min, TFloatValue max) {
  if (numVarValues == nullptr)
    numVarValues = new std::vector<TFluentInterval>();
  numVarValues->emplace_back(v, min, max);
}
