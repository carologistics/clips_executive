#ifndef PLAN_H
#define PLAN_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* POP plan
 */
/********************************************************/

#include "../sas/sasTask.h"
#include "../utils/utils.h"
#include "state.h"

/*
#define SEARCH_NRPG 0
#define SEARCH_LAND 1
*/

class TFluentInterval {
public:
  TVariable numVar; // Numeric variable
  TInterval interval;
  TFluentInterval() {}
  TFluentInterval(TVariable v, TFloatValue min, TFloatValue max)
      : interval(min, max) {
    numVar = v;
  }
};

class TPlanUpdate { // Changes in previous steps of the plan
public:
  TTimePoint timePoint;
  TFloatValue newTime;
  TPlanUpdate(TTimePoint tp, TFloatValue t) {
    timePoint = tp;
    newTime = t;
  }
};

class TCausalLink {
public:
  TTimePoint timePoint; // supporting timepoint
  TVarValue varVal;     // (variable = value) supported
  TCausalLink(TTimePoint t, TVarValue v) {
    timePoint = t;
    varVal = v;
  }
};

class TNumericCausalLink {
public:
  TTimePoint timePoint; // supporting timepoint
  TVariable var;        // numeric variable supported
  TNumericCausalLink(TTimePoint t, TVariable v) {
    timePoint = t;
    var = v;
  }
};

class PlanPoint {
private:
  TTime time; // Scheduled time for the begining/end of the action

public:
  TTime updatedTime; // Final scheduled time, after applying updates in child
                     // plans
  std::vector<TFluentInterval>
      *numVarValues; // Numeric var. values. Vector is nullptr if the action
                     // does not modify numeric vbles.
  std::vector<TCausalLink>
      causalLinks; // New causal links. TStep is the support action. These CLs
                   // are sorted like the action prec. at that time point,
                   // followed by over all precs.
  std::vector<TNumericCausalLink> numCausalLinks;

  PlanPoint() { numVarValues = nullptr; }
  void addCausalLink(TTimePoint timePoint, TVarValue varVal);
  void addNumericCausalLink(TTimePoint timePoint, TVarValue var);
  void addNumericValue(TVariable v, TFloatValue min, TFloatValue max);
  inline void copyInitialTime() { updatedTime = time; }
  inline void setInitialTime(TFloatValue t) { time = updatedTime = t; }
};

class Plan {
private:
  void addFluentIntervals(PlanPoint &pp, std::vector<SASNumericEffect> &eff);
  void addConditionalEffect(unsigned int numEff);

public:
  TPlanId id;
  Plan *parentPlan; // Pointer to its parent plan
  std::vector<Plan *>
      *childPlans;   // Vector of child plans. This vector is nullptr if
                     // the plan has not been expanded yet
  SASAction *action; // New action added
  bool
      fixedInit; // True if the initial time is fixed (action cannot be delayed)
  TInterval actionDuration;           // Action duration
  std::vector<TInterval> *cvarValues; // Control var. values. Vector is nullptr
                                      // if the action has no control vars.
  std::vector<TPlanUpdate>
      *planUpdates; // Changes in previous steps of the plan
  std::vector<TOrdering>
      orderings; // New orderings (first time point [lower 16 bits] -> second
                 // time point [higher 16 bits])
  PlanPoint startPoint; // At-start plan data
  PlanPoint endPoint;   // At-end plan data
  bool
      repeatedState; // True if the plan leads to a repeated state in the search
  int g;             // Plan length
  int h;             // Heuristic value
  int hLand;
  TState *fs;     // Frontier state
  bool z3Checked; // Plan checked by z3 solver?
  bool invalid;   // Invalid plan (after z3 checking)
  // int numUsefulActions;					// Number of
  // useful actions included in the plan
  std::vector<int> *holdCondEff;

  Plan(SASAction *action, Plan *parentPlan, TPlanId idPlan, bool *holdCondEff);
  ~Plan();
  void setDuration(TFloatValue min, TFloatValue max);
  void setTime(TTime init, TTime end, bool fixed);
  int compare(Plan *p);
  bool isRoot();
  inline bool expanded() { return childPlans != nullptr; }
  void addFluentIntervals();
  inline bool isSolution() { return action != nullptr && action->isGoal; }
  void addChildren(std::vector<Plan *> &suc);
  void addPlanUpdate(TTimePoint tp, TFloatValue time);
  int getCheckDistance();
  // int getH(int queue);
};

#endif
