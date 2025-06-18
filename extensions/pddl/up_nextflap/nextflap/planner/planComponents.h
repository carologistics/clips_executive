#ifndef PLAN_COMPONENTS_H
#define PLAN_COMPONENTS_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Calculation of the list of steps of a plan (each     */
/* step is called plan component.)                      */
/********************************************************/

#include "plan.h"

class PlanComponents {
private:
  TStep numSteps;
  std::vector<Plan *>
      basePlanComponents; // The base plan is made up by incremental components,
                          // which are stored in this vector

public:
  void calculate(Plan *base);
  inline TStep size() { return numSteps; }
  inline Plan *get(TStep index) { return basePlanComponents[index]; }
  void removeLast() {
    basePlanComponents.pop_back();
    numSteps--;
  }
};

#endif