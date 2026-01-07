#ifndef INTERVAL_CALC_H
#define INTERVAL_CALC_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Class for calculations with numeric intervals.       */
/********************************************************/

#include "../sas/sasTask.h"
#include "plan.h"

// Base class for numeric interval
class FluentIntervalData {
public:
  virtual TFloatValue getMinValue(TVariable v, int numState) = 0;
  virtual TFloatValue getMaxValue(TVariable v, int numState) = 0;
};

// Value change of a numeric variable
class TNumVarChange {
public:
  TVariable v;
  TFloatValue min, max;

  TNumVarChange(TVariable v, TFloatValue min, TFloatValue max) {
    this->v = v;
    this->min = min;
    this->max = max;
  }
};

// Class for calculations with numeric intervals
class IntervalCalculations {
private:
  SASAction *a;
  std::vector<TInterval> fluentValues;
  TInterval duration;
  std::vector<TInterval> cvarValues;

  void evaluateExpression(SASNumericExpression *e, TFloatValue *min,
                          TFloatValue *max);
  void constrainInterval(char comp, SASNumericExpression *e,
                         TInterval *cvarInt);
  void calculateControlVarIntervals();
  void calculateDuration();
  void applyEffect(SASNumericEffect *e);
  void constrainAtStartFluent(TVariable v);
  void constrainAtEndFluent(TVariable v);

public:
  IntervalCalculations(SASAction *a, int numState,
                       FluentIntervalData *fluentData, SASTask *task);
  bool supportedNumericStartConditions(bool *holdCondEff);
  bool supportedNumericEndConditions(bool *holdCondEff);
  bool supportedNumericConditions(SASConditionalEffect *e);
  void constrainAtStartFluents();
  void applyStartEffects(Plan *p, bool *holdCondEff);
  void applyStartEffects(std::vector<TNumVarChange> *v, bool *holdCondEff);
  void applyEndEffects(Plan *p, bool *holdCondEff);
  void applyEndEffects(std::vector<TNumVarChange> *v, bool *holdCondEff);
  void copyControlVars(Plan *p);
  void copyDuration(Plan *p);
  bool supportedCondition(SASNumericCondition *c);
};

#endif