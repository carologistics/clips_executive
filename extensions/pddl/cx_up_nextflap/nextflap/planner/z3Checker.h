#ifndef Z3_CHECKER_H
#define Z3_CHECKER_H

#include "plan.h"
#include "planComponents.h"
#include "z3++.h"
#include <unordered_map>
#include <vector>
using namespace z3;

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Plan validity checking through Z3 solver.            */
/********************************************************/

typedef std::unordered_map<TStep, std::vector<float>> TControVarValues;

class Z3StepVariables {
public:
  TStep s;
  Plan *p;
  std::vector<expr> times;       // duration, startTime, endTime
  std::vector<expr> controlVars; // Control vars
  std::vector<expr> fluents;     // Numeric variables values
  std::unordered_map<TVariable, int>
      startFluentIndex; // At start numeric vble. -> index in fluents vector
  std::unordered_map<TVariable, int>
      endFluentIndex; // At end numeric vble. -> index in fluents vector
  Z3StepVariables(TStep step, Plan *plan) {
    s = step;
    p = plan;
  }
};

class Z3Checker {
private:
  bool optimizeMakespan;
  PlanComponents planComponents;
  std::vector<Z3StepVariables> stepVars;
  context *cont;
  solver *checker;
  optimize *optimizer;

  void defineVariables(Plan *p, TStep s);
  void defineConstraints(Plan *p, TStep s);
  expr &getDurationVar(TStep s);
  expr &getPointVar(TTimePoint tp);
  expr &getControlVar(int var, TStep s);
  expr &getProductorVar(TVariable var, TTimePoint tp);
  expr &getFluent(TVariable var, TTimePoint tp);
  expr getNumericExpression(SASNumericExpression &e, TTimePoint tp);
  inline int intVal(TFloatValue n) { return (int)(n * 1000); };
  void add(expr const &e);
  void defineNumericContraint(SASNumericCondition &prec, TTimePoint tp);
  void defineDurationConstraint(SASDurationCondition &d, TStep s);
  void defineNumericEffect(SASNumericEffect &e, TTimePoint tp);
  // void showModel(model m);
  void updatePlan(Plan *p, model m, TControVarValues *cvarValues);
  void updateFluentValues(std::vector<TFluentInterval> *numValues,
                          TTimePoint tp, model &m);

public:
  bool checkPlan(Plan *p, bool optimizeMakespan,
                 TControVarValues *cvarValues = nullptr);
};

#endif
