#ifndef PLAN_EFFECTS_H
#define PLAN_EFFECTS_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Class to keep the information about the achieved     */
/* values by the variables at each timepoint.           */
/********************************************************/

#include "../sas/sasTask.h"
#include "../utils/utils.h"
#include "intervalCalculations.h"
#include "plan.h"
#include "planComponents.h"
#include <vector>

class PlanEffect {
public:
  std::vector<TTimePoint>
      timePoints; // Points of time where this effect is produced
                  // Dividing the time point by 2 we get the number of the step
                  // in the plan (each step has two time points: start and end)
  unsigned int iteration; // The information is valid only if the iteration
                          // matches with the current one

  PlanEffect() { iteration = 0; }
  void add(TTimePoint time, unsigned int iteration);
};

class VarChange {
public:
  std::vector<TValue> values; // Value that the variable takes
  std::vector<TTimePoint>
      timePoints;         // Points of time where this effect is produced
  unsigned int iteration; // The information is valid only if the iteration
                          // matches with the current one

  VarChange() { iteration = 0; }
  void add(TValue v, TTimePoint time, unsigned int iteration);
};

class NumVarChange {
public:
  TTimePoint timepoint;
  std::vector<TInterval *>
      values; // Interval of values for each numeric variable
};

class PlanEffects : public FluentIntervalData {
private:
  SASTask *task;

  TFloatValue getNumVarMinValue(TVariable var, int stateIndex);
  TFloatValue getMinActionDuration(TTimePoint timepoint);
  TFloatValue getMinControlVarValue(TTimePoint timepoint, TVariable var);
  TFloatValue getNumVarMaxValue(TVariable var, int stateIndex);
  TFloatValue getMaxActionDuration(TTimePoint timepoint);
  TFloatValue getMaxControlVarValue(TTimePoint timepoint, TVariable var);

public:
  PlanEffect **planEffects; // Plan effects: (var, value) -> PlanEffect
  VarChange *varChanges;    // Variable changes: var -> VarChange
  std::vector<NumVarChange>
      numStates; // Sequence of numeric states in each timepoint
  unsigned int iteration;
  PlanComponents *planComponents;

  PlanEffects(SASTask *task);
  ~PlanEffects();
  void setCurrentIteration(unsigned int currentIteration,
                           PlanComponents *planComponents);
  void addEffect(SASCondition &eff, TTimePoint timePoint);
  void addNumEffect(TFluentInterval &eff, TTimePoint timePoint);
  TFloatValue getMinValue(TVariable v, int numState);
  TFloatValue getMaxValue(TVariable v, int numState);
};

#endif // !PLAN_EFFECTS_H
