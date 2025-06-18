#ifndef PLANNER_H
#define PLANNER_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Planning search.
 */
/********************************************************/

#include "../parser/parsedTask.h"
#include "../sas/sasTask.h"
#include "plan.h"
#include "selector.h"
#include "state.h"
#include "successors.h"

class Planner {
private:
  SASTask *task;
  ParsedTask *parsedTask;
  Plan *initialPlan;
  TState *initialState;
  bool forceAtEndConditions;
  bool filterRepeatedStates;
  bool generateTrace;
  Planner *parentPlanner;
  unsigned int expandedNodes;
  Successors *successors;
  std::vector<SASAction *> *tilActions;
  float initialH;
  Plan *solution;
  std::vector<Plan *> sucPlans;
  SearchQueue *selector;
  // clock_t startTime;
  float bestMakespan;
  int bestNumSteps;
  int bestH;

  bool emptySearchSpace();
  void searchStep();
  void expandBasePlan(Plan *base);
  void addSuccessors(Plan *base);
  bool checkPlan(Plan *p);
  void markAsInvalid(Plan *p);
  void markChildrenAsInvalid(Plan *p);

public:
  Planner(SASTask *task, Plan *initialPlan, TState *initialState,
          bool forceAtEndConditions, bool filterRepeatedStates,
          bool generateTrace, std::vector<SASAction *> *tilActions,
          ParsedTask *parsedTask);
  Plan *plan(float bestMakespan);
  void clearSolution();
};

#endif
