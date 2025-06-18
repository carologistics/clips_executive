#include "z3Checker.h"

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Plan validity checking through Z3 solver.            */
/********************************************************/

bool Z3Checker::checkPlan(Plan *p, bool optimizeMakespan,
                          TControVarValues *cvarValues) {
  z3::set_param("parallel.enable", true);
  z3::set_param("pp.decimal", true);
  // z3::set_param("pp.decimal-precision", 3);
  // std::cout << (optimizeMakespan ? "o" : ".");
  this->optimizeMakespan = optimizeMakespan;
  bool valid = false;
  planComponents.calculate(p);
  // for (int i = 0; i < planComponents.size(); i++)
  //     std::cout << i << ": " << planComponents.get(i)->action->name <<
  //     std::endl;
  try {
    context c;
    this->cont = &c;
    for (TStep s = 0; s < planComponents.size(); s++) {
      defineVariables(planComponents.get(s), s);
    }
    if (optimizeMakespan)
      optimizer = new optimize(c);
    else
      checker = new solver(c);
    for (TStep s = 0; s < planComponents.size(); s++) {
      defineConstraints(planComponents.get(s), s);
    }
    TStep lastStep = planComponents.size() - 1;
    if (optimizeMakespan) {
      optimizer->minimize(getPointVar(stepToEndPoint(lastStep)));
      valid = optimizer->check() == sat;
      // showModel(optimizer->get_model());
      if (valid)
        updatePlan(p, optimizer->get_model(), cvarValues);
    } else {
      valid = checker->check() == sat;
      // if (p->action->isGoal) std::cout << checker->assertions() << std::endl;
      if (valid)
        updatePlan(p, checker->get_model(), cvarValues);
    }
    if (optimizeMakespan)
      delete optimizer;
    else
      delete checker;
    stepVars.clear();
    Z3_finalize_memory();
  } catch (std::exception &ex) {
    throwError("unexpected error: " + std::string(ex.what()));
  }
  // std::cout << (valid ? "v" : "x") << std::endl;
  return valid;
}

void Z3Checker::defineVariables(Plan *p, TStep s) {
  char varName[10];
  this->stepVars.emplace_back(s, p);
  Z3StepVariables &vars = this->stepVars.back();
  sprintf(varName, "d%d", s);
  vars.times.push_back(cont->real_const(varName)); // Duration

  sprintf(varName, "t%d", stepToStartPoint(s));
  vars.times.push_back(cont->int_const(varName)); // Start time
  sprintf(varName, "t%d", stepToEndPoint(s));
  vars.times.push_back(cont->int_const(varName)); // End time
  if (p->cvarValues != nullptr) {
    for (int cv = 0; cv < p->cvarValues->size(); cv++) {
      sprintf(varName, "c%ds%d", cv, s); // Control var
      vars.controlVars.push_back(cont->real_const(varName));
    }
  }
  if (p->startPoint.numVarValues != nullptr) {
    for (TFluentInterval &i : *(p->startPoint.numVarValues)) {
      sprintf(varName, "f%dt%d", i.numVar, stepToStartPoint(s)); // Fluent
      vars.startFluentIndex[i.numVar] = (int)vars.fluents.size();
      vars.fluents.push_back(cont->real_const(varName));
    }
  }
  if (p->endPoint.numVarValues != nullptr) {
    for (TFluentInterval &i : *(p->endPoint.numVarValues)) {
      sprintf(varName, "f%dt%d", i.numVar, stepToEndPoint(s)); // Fluent
      vars.endFluentIndex[i.numVar] = (int)vars.fluents.size();
      vars.fluents.push_back(cont->real_const(varName));
    }
  }
}

void Z3Checker::defineConstraints(Plan *p, TStep s) {
  SASAction *a = p->action;
  TTimePoint start = stepToStartPoint(s), end = stepToEndPoint(s);

  for (SASNumericCondition &c :
       a->startNumCond) { // Fluents and control vars. conditions
    defineNumericContraint(c, start);
  }

  for (SASNumericCondition &c : a->overNumCond) {
    defineNumericContraint(c, start);
    defineNumericContraint(c, end);
  }
  for (SASNumericCondition &c : a->endNumCond) {
    defineNumericContraint(c, end);
  }

  for (SASControlVar &cv : a->controlVars) {
    for (SASControlVarCondition &c : cv.conditions) {
      if (!c.inActionPrec) {
        defineNumericContraint(c.condition, start);
      }
    }
  }

  if (p->holdCondEff != nullptr) {
    for (unsigned int numEff : *p->holdCondEff) {
      SASConditionalEffect &e = a->conditionalEff[numEff];
      for (SASNumericCondition &c : e.startNumCond) {
        defineNumericContraint(c, start);
      }
      for (SASNumericCondition &c : e.endNumCond) {
        defineNumericContraint(c, end);
      }
      for (SASNumericEffect &c : e.startNumEff) {
        defineNumericEffect(c, start);
      }
      for (SASNumericEffect &c : e.endNumEff) {
        defineNumericEffect(c, end);
      }
    }
  }

  for (SASNumericEffect &e : a->startNumEff) {
    defineNumericEffect(e, start);
  }

  for (SASNumericEffect &e : a->endNumEff) {
    defineNumericEffect(e, end);
  }

  if (a->isTIL)
    add(getPointVar(start) == 0);
  else if (s == 0)
    add(getPointVar(0) == -1);

  for (SASDurationCondition &d : a->duration.conditions) { // Action duration
    defineDurationConstraint(d, s);
    expr intDur(cont->real_val(10000, 1) * getDurationVar(s));
    add(10 * getPointVar(end) < 10 * getPointVar(start) + intDur + 5);
    add(10 * getPointVar(end) > 10 * getPointVar(start) + intDur - 5);
  }

  for (TOrdering o : p->orderings) { // Orderings
    TTimePoint tp1 = firstPoint(o), tp2 = secondPoint(o);
    if (tp1 + 1 != tp2 || (tp1 & 1) == 1)
      add(getPointVar(tp1) < getPointVar(tp2));
  }
}

expr &Z3Checker::getDurationVar(TStep s) { return stepVars[s].times[0]; }

expr &Z3Checker::getPointVar(TTimePoint tp) {
  return stepVars[timePointToStep(tp)]
      .times[1 + (tp & 1)]; // 1 -> start, 2 -> end
}

expr &Z3Checker::getControlVar(int var, TStep s) {
  return stepVars[s].controlVars[var];
}

expr &Z3Checker::getProductorVar(TVariable var, TTimePoint tp) {
  TStep s = timePointToStep(tp);
  Plan *p = planComponents.get(s);
  if ((tp & 1) == 1) { // End point
    for (TNumericCausalLink &cl : p->endPoint.numCausalLinks) {
      if (cl.var == var) {
        return getFluent(cl.var, cl.timePoint);
      }
    }
  }
  for (TNumericCausalLink &cl : p->startPoint.numCausalLinks) {
    if (cl.var == var) {
      return getFluent(cl.var, cl.timePoint);
    }
  }
  for (TNumericCausalLink &cl : p->endPoint.numCausalLinks) {
    if (cl.var == var) {
      return getFluent(cl.var, cl.timePoint);
    }
  }
  throwError("Error: numeric causal link not defined for fluent " +
             std::to_string(var) + " in timepoint " + std::to_string(tp) +
             "(action " + p->action->name + ")");
  return getFluent(0, 0); // For avoiding warnings
}

expr &Z3Checker::getFluent(TVariable var, TTimePoint tp) {
  Z3StepVariables &vars = stepVars[timePointToStep(tp)];
  int index =
      (tp & 1) == 0 ? vars.startFluentIndex[var] : vars.endFluentIndex[var];
  return vars.fluents[index];
}

expr Z3Checker::getNumericExpression(SASNumericExpression &e, TTimePoint tp) {
  switch (e.type) {
  case 'N': // GE_NUMBER
    return cont->real_val(intVal(e.value), 1000);
  case 'V': // GE_VAR
    return getProductorVar(e.var, tp);
  case '+':
    return getNumericExpression(e.terms[0], tp) +
           getNumericExpression(e.terms[1], tp);
  case '-':
    return getNumericExpression(e.terms[0], tp) -
           getNumericExpression(e.terms[1], tp);
  case '/':
    return getNumericExpression(e.terms[0], tp) /
           getNumericExpression(e.terms[1], tp);
  case '*':
    return getNumericExpression(e.terms[0], tp) *
           getNumericExpression(e.terms[1], tp);
  case 'D': // GE_DURATION
    return getDurationVar(timePointToStep(tp));
  case 'C': // GE_CONTROL_VAR
    return getControlVar(e.var, timePointToStep(tp));
  }
  throwError("Error: wrong numeric expression");
  return cont->int_val(0); // To avoid warnings
}

void Z3Checker::add(expr const &e) {
  // std::cout << "Constraint: " << e.to_string() << std::endl;
  if (optimizeMakespan)
    optimizer->add(e);
  else
    checker->add(e);
}

void Z3Checker::defineNumericContraint(SASNumericCondition &prec,
                                       TTimePoint tp) {
  switch (prec.comp) {
  case '=':
    add(getNumericExpression(prec.terms[0], tp) ==
        getNumericExpression(prec.terms[1], tp));
    break;
  case '<':
    add(getNumericExpression(prec.terms[0], tp) <
        getNumericExpression(prec.terms[1], tp));
    break;
  case 'L':
    add(getNumericExpression(prec.terms[0], tp) <=
        getNumericExpression(prec.terms[1], tp));
    break;
  case '>':
    add(getNumericExpression(prec.terms[0], tp) >
        getNumericExpression(prec.terms[1], tp));
    break;
  case 'G':
    add(getNumericExpression(prec.terms[0], tp) >=
        getNumericExpression(prec.terms[1], tp));
    break;
  case 'N':
    add(getNumericExpression(prec.terms[0], tp) !=
        getNumericExpression(prec.terms[1], tp));
    break;
  }
}

void Z3Checker::defineDurationConstraint(SASDurationCondition &d, TStep s) {
  TTimePoint tp = d.time != 'E' ? stepToStartPoint(s) : stepToEndPoint(s);
  switch (d.comp) {
  case '=':
    add(getDurationVar(s) == getNumericExpression(d.exp, tp));
    break;
  case '<':
    add(getDurationVar(s) < getNumericExpression(d.exp, tp));
    break;
  case 'L':
    add(getDurationVar(s) <= getNumericExpression(d.exp, tp));
    break;
  case '>':
    add(getDurationVar(s) > getNumericExpression(d.exp, tp));
    break;
  case 'G':
    add(getDurationVar(s) >= getNumericExpression(d.exp, tp));
    break;
  case 'N':
    add(getDurationVar(s) != getNumericExpression(d.exp, tp));
    break;
  }
}

void Z3Checker::defineNumericEffect(SASNumericEffect &e, TTimePoint tp) {
  switch (e.op) {
  case '=':
    add(getFluent(e.var, tp) == getNumericExpression(e.exp, tp));
    break;
  case '+':
    add(getFluent(e.var, tp) ==
        getProductorVar(e.var, tp) + getNumericExpression(e.exp, tp));
    break;
  case '-':
    add(getFluent(e.var, tp) ==
        getProductorVar(e.var, tp) - getNumericExpression(e.exp, tp));
    break;
  case '*':
    add(getFluent(e.var, tp) ==
        getProductorVar(e.var, tp) * getNumericExpression(e.exp, tp));
    break;
  case '/':
    add(getFluent(e.var, tp) ==
        getProductorVar(e.var, tp) / getNumericExpression(e.exp, tp));
    break;
  }
}

/*
void Z3Checker::showModel(model m)
{
    for (unsigned i = 0; i < m.size(); i++) {
        func_decl v = m[i];
        assert(v.arity() == 0);
        std::cout << v.name() << " = " << m.get_const_interp(v) << "\n";
    }
}
*/

void Z3Checker::updatePlan(Plan *p, model m, TControVarValues *cvarValues) {
  TTimePoint tp = 0;
  for (TStep s = 0; s < planComponents.size(); s++) {
    TTimePoint startPoint = stepToStartPoint(s), endPoint = startPoint + 1;
    // std::cout << m.eval(getPointVar(tp)).as_int64() << std::endl;
    // std::cout << m.eval(getPointVar(tp + 1)).as_int64() << std::endl;
    TFloatValue startTime =
        round3d(m.eval(getPointVar(tp++)).as_int64() / 1000.0);
    TFloatValue endTime =
        round3d(m.eval(getPointVar(tp++)).as_int64() / 1000.0f);
    // std::cout << "Time point " << stepToStartPoint(s) << ": " << startTime <<
    // std::endl; std::cout << "Time point " << stepToEndPoint(s) << ": " <<
    // endTime << std::endl;
    Plan *pc = planComponents.get(s);
    if (cvarValues != nullptr && pc->cvarValues != nullptr) {
      std::vector<float> valuesList;
      for (int cv = 0; cv < pc->action->controlVars.size(); cv++)
        valuesList.push_back(m.eval(getControlVar(cv, s)).as_double());
      (*cvarValues)[s] = valuesList;
    }
    if (p == pc) {
      p->setTime(startTime, endTime, p->fixedInit);
    } else {
      if (abs(startTime - pc->startPoint.updatedTime) > EPSILON / 2) {
        // std::cout << "Time point " << startPoint << ": " << startTime <<
        // std::endl;
        p->addPlanUpdate(startPoint, startTime);
      }
      if (abs(endTime - pc->endPoint.updatedTime) > EPSILON / 2) {
        // std::cout << "Time point " << endPoint << ": " << endTime <<
        // std::endl;
        p->addPlanUpdate(endPoint, endTime);
      }
    }
    // updateFluentValues(pc->startPoint.numVarValues, startPoint, m);
    // updateFluentValues(pc->startPoint.numVarValues, endPoint, m);
  }
  // showModel(checker->get_model());
}

void Z3Checker::updateFluentValues(std::vector<TFluentInterval> *numValues,
                                   TTimePoint tp, model &m) {
  if (numValues == nullptr)
    return;
  for (int i = 0; i < numValues->size(); i++) {
    TFluentInterval *f = &(numValues->at(i));
    TFloatValue value =
        (TFloatValue)m.eval(getFluent(f->numVar, tp)).as_double();
    f->interval.minValue = f->interval.maxValue = value;
  }
}
