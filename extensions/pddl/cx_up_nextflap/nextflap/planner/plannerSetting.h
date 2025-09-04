#ifndef PLANNER_SETTING_H
#define PLANNER_SETTING_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Initializes and launches the planner.                */
/********************************************************/

#include "../heuristics/rpg.h"
#include "../sas/sasTask.h"
#include "plan.h"
#include "planner.h"
#include "state.h"
#include <time.h>

class PlannerSetting {
private:
  SASTask *task;
  clock_t initialTime;
  bool generateTrace;
  Plan *initialPlan;
  std::vector<SASAction *> tilActions;
  bool forceAtEndConditions;
  bool filterRepeatedStates;
  TState *initialState;
  Planner *planner;

  void createInitialPlan();
  SASAction *createInitialAction();
  SASAction *createFictitiousAction(float actionDuration,
                                    std::vector<unsigned int> &varList,
                                    float timePoint, std::string name,
                                    bool isTIL, bool isGoal);
  Plan *createTILactions(Plan *parentPlan);
  bool checkForceAtEndConditions(); // Check if it's required to leave at-end
                                    // conditions not supported for some actions
  bool checkRepeatedStates();

public:
  PlannerSetting(SASTask *sTask);
  Plan *plan(float bestMakespan, ParsedTask *parsedTask);
};

#endif
