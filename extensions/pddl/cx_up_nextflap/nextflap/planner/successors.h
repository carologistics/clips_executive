#ifndef SUCCESSORS_H
#define SUCCESSORS_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Calculation of succesors of a given plan             */
/********************************************************/

#include "../heuristics/evaluator.h"
#include "../sas/sasTask.h"
#include "linearizer.h"
#include "plan.h"
#include "planBuilder.h"
#include "planComponents.h"
#include "planEffects.h"
#include "state.h"

#define INITAL_MATRIX_SIZE 400
#define MATRIX_INCREASE 200

class Threat {
public:
  TTimePoint p1;
  TTimePoint p2;
  TTimePoint tp;
  TVariable var;
  bool numeric;

  Threat(TTimePoint c1, TTimePoint c2, TTimePoint p, TVariable v,
         bool numeric) {
    p1 = c1;
    p2 = c2;
    tp = p;
    var = v;
    this->numeric = numeric;
  }
};

class Successors {
private:
  SASTask *task;
  TState *initialState;
  bool filterRepeatedStates;
  unsigned int numVariables;       // Number of variables
  unsigned int numActions;         // Number of grounded actions
  PlanEffects planEffects;         // Plan effects
  TPlanId idPlan;                  // Plan counter
  std::vector<Plan *> *successors; // Vector to return the sucessor plans
  Plan *basePlan;                  // Base plan
  TStep newStep;                   // New step to add as successor
  std::vector<unsigned int> checkedAction;
  unsigned int currentIteration;
  PlanComponents planComponents; // The base plan is made up by incremental
                                 // components, which are stored in this vector
  std::vector<std::vector<unsigned int>>
      matrix; // Orders between time points in the current plan
  Linearizer linearizer;
  float bestMakespan;

  void computeOrderMatrix();
  void resizeMatrix();
  void computeBasePlanEffects(std::vector<TTimePoint> &linearOrder);
  void fullSuccessorsCalculation();
  void fullActionCheck(SASAction *a, TVariable var, TValue value,
                       TTimePoint effectTime, TTimePoint startTimeNewAction);
  bool supportedConditions(const SASAction *a);
  int supportedNumericConditions(SASAction *a);
  int supportedNumericConditions(SASConditionalEffect *e, SASAction *a);
  inline bool supportedCondition(const SASCondition &c) {
    return planEffects.planEffects[c.var][c.value].iteration ==
           currentIteration;
  }
  void fullActionSupportCheck(PlanBuilder *pb);
  void fullConditionSupportCheck(PlanBuilder *pb, SASCondition *c,
                                 TTimePoint condPoint, bool overAll,
                                 bool canLeaveOpen);
  void setNumericCausalLinks(PlanBuilder *pb, int numSupportState);
  void
  computeSupportingTimePoints(SASAction *action, int numSupportState,
                              std::vector<TTimePoint> *supportingTimePoints);
  void addNumericSupport(PlanBuilder *pb, int numCond,
                         std::vector<TTimePoint> *supportingTimePoints);
  void checkThreats(PlanBuilder *pb);
  void checkThreatsBetweenCausalLinksInBasePlanAndNewActionEffects(
      PlanBuilder *pb, std::vector<Threat> *threats);
  void checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(
      PlanBuilder *pb, std::vector<Threat> *threats, TCausalLink &cl,
      TTimePoint p2);
  void checkThreatBetweenCausalLinkInBasePlanAndNewActionEffects(
      PlanBuilder *pb, std::vector<Threat> *threats, TNumericCausalLink &cl,
      TTimePoint p2);
  inline bool existOrder(TTimePoint t1, TTimePoint t2) {
    return matrix[t1][t2] == currentIteration;
  }
  void checkThreatsBetweenNewCausalLinksAndActionsInBasePlan(
      PlanBuilder *pb, std::vector<Threat> *threats);
  void solveThreats(PlanBuilder *pb, std::vector<Threat> *threats);
  void checkContradictoryEffects(PlanBuilder *pb);
  void checkContradictoryEffects(PlanBuilder *pb, SASCondition *c,
                                 TTimePoint effPoint);
  void generateSuccessor(PlanBuilder *pb);
  bool mutexPoints(TTimePoint p1, TTimePoint p2, TVariable var,
                   PlanBuilder *pb);
  SASCondition *getRequiredValue(TTimePoint p, SASAction *a, TVariable var);
  SASCondition *getRequiredValue(SASAction *a, TVariable var);
  void addSuccessor(Plan *p);
  void computeSuccessorsSupportedByLastActions();
  inline bool visitedAction(SASAction *a) {
    return checkedAction[a->index] == currentIteration;
  }
  inline void setVisitedAction(SASAction *a) {
    checkedAction[a->index] = currentIteration;
  }
  unsigned int addActionSupport(PlanBuilder *pb, TVariable var, TValue value,
                                TTimePoint effectTime,
                                TTimePoint startTimeNewAction);
  void computeSuccessorsThroughBrotherPlans();
  void checkConditionalEffects(PlanBuilder *pb, int numEff);
  void checkCondEffConditions(int numEff, int numCond, PlanBuilder *pb);
  void checkCondEffCondition(int numEff, int numCond, SASCondition *c,
                             TTimePoint condPoint, PlanBuilder *pb);
  bool setNumericCausalLinks(PlanBuilder *pb, int numSupportState,
                             SASConditionalEffect &e, int *numLinks);
  bool holdConditionalCondition(SASCondition &c, PlanBuilder *pb,
                                TTimePoint condPoint);
  void checkConditionalThreats(int numLinks, int numEff, PlanBuilder *pb);
  void solveConditionalThreats(PlanBuilder *pb, std::vector<Threat> *threats,
                               int numEff);

public:
  Evaluator evaluator;
  std::unordered_map<uint64_t, std::vector<Plan *>> memo;
  Plan *solution;

  Successors(TState *state, SASTask *task, bool forceAtEndConditions,
             bool filterRepeatedStates, std::vector<SASAction *> *tilActions);
  ~Successors();
  void computeSuccessors(Plan *base, std::vector<Plan *> *suc,
                         float bestMakespan);
  bool repeatedState(Plan *p);
};

#endif // !SUCCESSORS_H
