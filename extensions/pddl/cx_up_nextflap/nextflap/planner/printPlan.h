#ifndef PRINT_PLAN_H
#define PRINT_PLAN_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Class to plan printing on screen.                    */
/********************************************************/

#include "plan.h"
#include "z3Checker.h"

class PrintPlan {
private:
  static std::string printDurative(Plan *p, TControVarValues *cvarValues);
  static std::string printPOP(Plan *p, TControVarValues *cvarValues);

public:
  static std::string actionName(SASAction *a);
  static std::string print(Plan *p, TControVarValues *cvarValues = nullptr,
                           bool durativePlan = true);
  static float getMakespan(Plan *p);
  static void rawPrint(Plan *p, SASTask *task);
};

#endif // !PRINT_PLAN_H
