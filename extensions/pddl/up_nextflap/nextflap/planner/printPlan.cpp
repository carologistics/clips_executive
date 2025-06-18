#include "printPlan.h"
#include "linearizer.h"
#include "planComponents.h"
#include <iomanip>
#include <string>

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Class to plan printing on screen.                    */
/********************************************************/

using namespace std;

std::string PrintPlan::printDurative(Plan *p, TControVarValues *cvarValues) {
  std::string res = "|";
  PlanComponents planComponents;
  planComponents.calculate(p);
  Linearizer linearizer;
  linearizer.linearize(planComponents);
  float makespan = 0;
  for (TTimePoint tp : linearizer.linearOrder) {
    Plan *pc = planComponents.get(timePointToStep(tp));
    if ((tp & 1) == 0 && !pc->isRoot() && !pc->action->isGoal) {
      float duration = round3d(pc->endPoint.updatedTime) -
                       round3d(pc->startPoint.updatedTime);
      res += std::to_string(round3d(pc->startPoint.updatedTime - 0.001)) +
             ": (" + actionName(pc->action);
      if (cvarValues != nullptr && !pc->action->controlVars.empty()) {
        std::vector<float> values = cvarValues->at(timePointToStep(tp));
        for (SASControlVar &cv : pc->action->controlVars) {
          for (int i = 0; i < values.size(); i++) {
            if (pc->action->controlVars[i].name.compare(cv.name) == 0) {
              res += " " + std::to_string(values[i]);
              break;
            }
          }
        }
      }
      res += ") [" + std::to_string(round3d(duration)) + "]|";
      if (pc->endPoint.updatedTime > makespan)
        makespan = pc->endPoint.updatedTime;
    }
  }
  // cout << ";Makespan: " << fixed << setprecision(3) << makespan << endl;
  return res;
}

std::string PrintPlan::printPOP(Plan *p, TControVarValues *cvarValues) {
  std::string res = "|";
  PlanComponents planComponents;
  planComponents.calculate(p);
  int ncomp = planComponents.size();
  for (int i = 0; i < ncomp; i++) {
    Plan *pc = planComponents.get(i);
    if (!pc->isRoot() && !pc->action->isGoal) {
      res += std::to_string(i) + ":" + actionName(pc->action) + "|";
    }
  }
  for (int i = 0; i < ncomp; i++) {
    Plan *pc = planComponents.get(i);
    if (!pc->isRoot() && !pc->action->isGoal) {
      bool *stepsBefore = new bool[ncomp];
      for (int j = 0; j < ncomp; j++)
        stepsBefore[j] = false;
      // Search for orderings j -> i
      for (int j = 0; j < ncomp; j++) {
        Plan *opc = planComponents.get(j);
        for (TOrdering o : opc->orderings) {
          TStep start = timePointToStep(firstPoint(o)),
                end = timePointToStep(secondPoint(o));
          if (end == i)
            stepsBefore[start] = true;
        }
      }
      // Causal links
      for (TCausalLink cl : pc->startPoint.causalLinks) {
        TStep start = timePointToStep(cl.timePoint);
        stepsBefore[start] = true;
      }
      for (TNumericCausalLink cl : pc->startPoint.numCausalLinks) {
        TStep start = timePointToStep(cl.timePoint);
        stepsBefore[start] = true;
      }
      for (TCausalLink cl : pc->endPoint.causalLinks) {
        TStep start = timePointToStep(cl.timePoint);
        stepsBefore[start] = true;
      }
      for (TNumericCausalLink cl : pc->endPoint.numCausalLinks) {
        TStep start = timePointToStep(cl.timePoint);
        stepsBefore[start] = true;
      }
      // Print orders
      for (int j = 0; j < ncomp; j++) {
        if (stepsBefore[j]) {
          Plan *opc = planComponents.get(j);
          if (!opc->isRoot() && !opc->action->isGoal) {
            res += std::to_string(j) + "->" + std::to_string(i) + "|";
          }
        }
      }
      delete[] stepsBefore;
    }
  }
  return res;
}

std::string PrintPlan::actionName(SASAction *a) {
  std::size_t colon = a->name.find(':');
  if (colon == std::string::npos)
    return a->name;
  std::size_t gap = a->name.find(' ', colon);
  if (gap == std::string::npos)
    return a->name.substr(0, colon);
  return a->name.substr(0, colon) + a->name.substr(gap);
}

std::string PrintPlan::print(Plan *p, TControVarValues *cvarValues,
                             bool durativePlan) {
  if (durativePlan)
    return PrintPlan::printDurative(p, cvarValues);
  else
    return PrintPlan::printPOP(p, cvarValues);
}

float PrintPlan::getMakespan(Plan *p) {
  PlanComponents planComponents;
  planComponents.calculate(p);
  Linearizer linearizer;
  linearizer.linearize(planComponents);
  float makespan = 0;
  for (TTimePoint tp : linearizer.linearOrder) {
    Plan *pc = planComponents.get(timePointToStep(tp));
    if ((tp & 1) == 0 && !pc->isRoot() && !pc->action->isGoal &&
        pc->endPoint.updatedTime > makespan)
      makespan = pc->endPoint.updatedTime;
  }
  return makespan;
}

void PrintPlan::rawPrint(Plan *p, SASTask *task) {
#ifdef _DEBUG
  std::vector<Plan *> planComponents;
  Plan *current = p;
  while (current != nullptr) {
    planComponents.insert(planComponents.begin(), current);
    current = current->parentPlan;
  }
  for (int step = 0; step < planComponents.size(); step++) {
    p = planComponents[step];
    cout << "Plan component " << step << endl;
    cout << "  (" << stepToStartPoint(step) << ") " << p->action->name << " ("
         << stepToEndPoint(step) << ")" << endl;
    for (TOrdering o : p->orderings)
      cout << "  " << firstPoint(o) << " -> " << secondPoint(o) << endl;
    for (TCausalLink cl : p->startPoint.causalLinks)
      cout << "  " << cl.timePoint << " --> start, "
           << task->variables[task->getVariableIndex(cl.varVal)].name << "="
           << task->values[task->getValueIndex(cl.varVal)].name << endl;
    for (TNumericCausalLink cl : p->startPoint.numCausalLinks)
      cout << "  " << cl.timePoint << " --> start, "
           << task->numVariables[cl.var].name << endl;
    for (TCausalLink cl : p->endPoint.causalLinks)
      cout << "  " << cl.timePoint << " --> end, "
           << task->variables[task->getVariableIndex(cl.varVal)].name << "="
           << task->values[task->getValueIndex(cl.varVal)].name << endl;
    for (TNumericCausalLink cl : p->endPoint.numCausalLinks)
      cout << "  " << cl.timePoint << " --> end, "
           << task->numVariables[cl.var].name << endl;
  }
#endif // _DEBUG
}
