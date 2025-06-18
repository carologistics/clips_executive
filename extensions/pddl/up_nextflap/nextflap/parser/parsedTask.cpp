/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Stores the data parsed from the domain and problem   */
/* files.                                               */
/********************************************************/

#include "parsedTask.h"
#include "../utils/utils.h"

using namespace std;

/********************************************************/
/* CLASS: Type (PDDL type)                              */
/********************************************************/

Type::Type(unsigned int index, string name) {
  this->index = index;
  this->name = name;
}

string Type::toString() { return name + "(" + to_string(index) + ")"; }

/********************************************************/
/* CLASS: Variable (?name - type list)                  */
/********************************************************/

Variable::Variable(string name, const vector<unsigned int> &types) {
  this->name = name;
  for (unsigned int i = 0; i < types.size(); i++)
    this->types.push_back(types[i]);
}

string Variable::toString(const vector<Type> &taskTypes) {
  string res = name + " - ";
  if (types.size() == 1)
    res += taskTypes[types[0]].name;
  else {
    res += "(either";
    for (unsigned int i = 0; i < types.size(); i++)
      res += " " + taskTypes[types[i]].name;
    res += ")";
  }
  return res;
}

/********************************************************/
/* CLASS: Object (PDDL object or constant)              */
/********************************************************/

Object::Object(unsigned int index, string name, bool isConstant) {
  this->index = index;
  this->name = name;
  this->isConstant = isConstant;
}

string Object::toString() {
  string res = name + "-";
  if (types.size() == 1)
    res += to_string(types[0]);
  else {
    res += "(either";
    for (unsigned int i = 0; i < types.size(); i++)
      res += " " + to_string(types[i]);
    res += ")";
  }
  return res;
}

/********************************************************/
/* CLASS: Function (PDDL function or predicate)         */
/********************************************************/

Function::Function() {}

Function::Function(string name, const vector<Variable> &parameters) {
  this->name = name;
  for (unsigned int i = 0; i < parameters.size(); i++)
    this->parameters.push_back(parameters[i]);
}

void Function::setValueTypes(const vector<unsigned int> &valueTypes) {
  for (unsigned int i = 0; i < valueTypes.size(); i++)
    this->valueTypes.push_back(valueTypes[i]);
}

string Function::toString(const vector<Type> &taskTypes) {
  string res = "(" + name;
  for (unsigned int i = 0; i < parameters.size(); i++)
    res += " " + parameters[i].toString(taskTypes);
  return res + ")";
}

/********************************************************/
/* CLASS: Term (variable or constant)                   */
/********************************************************/

Term::Term() {}

Term::Term(TermType type, unsigned int index) {
  this->type = type;
  this->index = index;
}

string Term::toString(const vector<Variable> &parameters,
                      const vector<Variable> &controlVars,
                      const vector<Object> &objects) {
  if (type == TERM_PARAMETER)
    return index < parameters.size()
               ? parameters[index].name
               : "Index error " + std::to_string(index) + " from " +
                     std::to_string(parameters.size());
  else if (type == TERM_CONSTANT)
    return objects[index].name;
  else
    return controlVars[index].name;
}

/********************************************************/
/* CLASS: Literal (atomic formula(term))                */
/********************************************************/

string Literal::toString(const vector<Variable> &parameters,
                         const vector<Variable> &controlVars,
                         const vector<Function> &functions,
                         const vector<Object> &objects) {
  string s = "(" + functions[fncIndex].name;
  for (unsigned int i = 0; i < params.size(); i++) {
    s += " " + params[i].toString(parameters, controlVars, objects);
  }
  return s + ")";
}

/********************************************************/
/* CLASS: NumericExpression (numeric expression)        */
/********************************************************/

NumericExpression::NumericExpression() {}

NumericExpression::NumericExpression(float value) {
  type = NumericExpressionType::NET_NUMBER;
  this->value = value;
}

NumericExpression::NumericExpression(unsigned int fncIndex,
                                     const vector<Term> &fncParams) {
  type = NumericExpressionType::NET_FUNCTION;
  function.fncIndex = fncIndex;
  for (unsigned int i = 0; i < fncParams.size(); i++)
    function.params.push_back(fncParams[i]);
}

NumericExpression::NumericExpression(Symbol s,
                                     const vector<NumericExpression> &operands,
                                     SyntaxAnalyzer *syn) {
  switch (s) {
  case Symbol::MINUS:
    if (operands.size() == 1)
      type = NET_NEGATION;
    else if (operands.size() == 2)
      type = NET_SUB;
    else
      syn->notifyError("Invalid number of operands in subtraction");
    break;
  case Symbol::PLUS:
    if (operands.size() >= 2)
      type = NET_SUM;
    else
      syn->notifyError("Invalid number of operands in addition");
    break;
  case Symbol::PROD:
    if (operands.size() >= 2)
      type = NET_MUL;
    else
      syn->notifyError("Invalid number of operands in product");
    break;
  case Symbol::DIV:
    if (operands.size() == 2)
      type = NET_DIV;
    else
      syn->notifyError("Invalid number of operands in division");
    break;
  default:
    syn->notifyError("Invalid expression type");
  }
  for (unsigned int i = 0; i < operands.size(); i++)
    this->operands.push_back(operands[i]);
}

string NumericExpression::toString(const vector<Variable> &parameters,
                                   const vector<Variable> &controlVars,
                                   const vector<Function> &functions,
                                   const vector<Object> &objects) {
  if (type == NumericExpressionType::NET_NUMBER)
    return to_string(value);
  if (type == NumericExpressionType::NET_FUNCTION)
    return function.toString(parameters, controlVars, functions, objects);
  if (type == NumericExpressionType::NET_TERM)
    return term.toString(parameters, controlVars, objects);
  string s = "(";
  switch (type) {
  case NET_NEGATION:
  case NET_SUB:
    s += "-";
    break;
  case NET_SUM:
    s += "+";
    break;
  case NET_MUL:
    s += "*";
    break;
  case NET_DIV:
    s += "/";
    break;
  default:
    s += "?";
  }
  for (unsigned int i = 0; i < operands.size(); i++)
    s +=
        " " + operands[i].toString(parameters, controlVars, functions, objects);
  return s + ")";
}

/********************************************************/
/* CLASS: Duration (duration constraint)                */
/********************************************************/

Duration::Duration(Symbol s, const NumericExpression &exp) {
  time = TimeSpecifier::NONE;
  this->exp = exp;
  if (s == Symbol::EQUAL)
    comp = Comparator::CMP_EQ;
  else if (s == Symbol::LESS_EQ)
    comp = Comparator::CMP_LESS_EQ;
  else if (s == Symbol::LESS)
    comp = Comparator::CMP_LESS;
  else if (s == Symbol::GREATER)
    comp = Comparator::CMP_GREATER;
  else
    comp = Comparator::CMP_GREATER_EQ;
}

string Duration::toString(const vector<Variable> &parameters,
                          const vector<Variable> &controlVars,
                          const vector<Function> &functions,
                          const vector<Object> &objects) {
  string s = "(";
  if (time == AT_START)
    s = "at start (";
  else if (time == AT_END)
    s = "at end (";
  if (comp == CMP_EQ)
    s += "=";
  else if (comp == CMP_LESS_EQ)
    s += "<=";
  else if (comp == CMP_LESS)
    s += "<";
  else if (comp == CMP_GREATER_EQ)
    s += ">=";
  else if (comp == CMP_GREATER)
    s += ">";
  s +=
      " ?duration " + exp.toString(parameters, controlVars, functions, objects);
  if (time == AT_START || time == AT_END)
    s += ")";
  return s + ")";
}

/********************************************************/
/* CLASS: GoalDescription (GD)                          */
/********************************************************/

void GoalDescription::setLiteral(Literal literal) {
  type = GoalDescriptionType::GD_LITERAL;
  this->literal = literal;
}

string GoalDescription::toString(const vector<Variable> &opParameters,
                                 const vector<Variable> &controlVars,
                                 const vector<Function> &functions,
                                 const vector<Object> &objects,
                                 const vector<Type> &taskTypes) {
  string s;
  switch (time) {
  case AT_START:
    s = "AT START ";
    break;
  case AT_END:
    s = "AT END ";
    break;
  case OVER_ALL:
    s = "OVER ALL ";
    break;
  default:
    s = "";
  }
  switch (type) {
  case GD_LITERAL:
    s += literal.toString(opParameters, controlVars, functions, objects);
    break;
  case GD_AND:
    s += "(AND";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(opParameters, controlVars, functions,
                                   objects, taskTypes);
    s += ")";
    break;
  case GD_NOT:
    s += "(NOT " +
         terms[0].toString(opParameters, controlVars, functions, objects,
                           taskTypes) +
         ")";
    break;
  case GD_OR:
    s += "(OR";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " (" +
           terms[i].toString(opParameters, controlVars, functions, objects,
                             taskTypes) +
           ")";
    s += ")";
    break;
  case GD_IMPLY:
    s += "(IMPLY " +
         terms[0].toString(opParameters, controlVars, functions, objects,
                           taskTypes) +
         " " +
         terms[1].toString(opParameters, controlVars, functions, objects,
                           taskTypes) +
         ")";
    break;
  case GD_EXISTS:
  case GD_FORALL: {
    s += type == GD_EXISTS ? "(EXISTS (" : "(FORALL (";
    vector<Variable> mergedParameters;
    for (unsigned int i = 0; i < opParameters.size(); i++)
      mergedParameters.push_back(opParameters[i]);
    for (unsigned int i = 0; i < parameters.size(); i++) {
      if (i > 0)
        s += " ";
      s += parameters[i].toString(taskTypes);
      mergedParameters.push_back(parameters[i]);
    }
    s += ") " +
         terms[0].toString(mergedParameters, controlVars, functions, objects,
                           taskTypes) +
         ")";
  } break;
  case GD_F_CMP:
    switch (comparator) {
    case CMP_EQ:
      s += "(= (";
      break;
    case CMP_LESS:
      s += "(< (";
      break;
    case CMP_LESS_EQ:
      s += "(<= (";
      break;
    case CMP_GREATER:
      s += "(> (";
      break;
    case CMP_GREATER_EQ:
      s += "(>= (";
      break;
    case CMP_NEQ:
      s += "(!= (";
      break;
    }
    s += exp[0].toString(opParameters, controlVars, functions, objects) +
         ") (" +
         exp[1].toString(opParameters, controlVars, functions, objects) + "))";
    break;
  case GD_EQUALITY:
    s += "(= " + eqTerms[0].toString(opParameters, controlVars, objects) + " " +
         eqTerms[1].toString(opParameters, controlVars, objects) + ")";
    break;
  case GD_INEQUALITY:
    s += "(!= " + eqTerms[0].toString(opParameters, controlVars, objects) +
         " " + eqTerms[1].toString(opParameters, controlVars, objects) + ")";
    break;
  case GD_NEG_LITERAL:
    s += "~" + literal.toString(opParameters, controlVars, functions, objects);
    break;
  }
  return s;
}

/********************************************************/
/* CLASS: DurativeCondition (<da-GD>)                   */
/********************************************************/

string DurativeCondition::toString(const vector<Variable> &opParameters,
                                   const vector<Variable> &controlVars,
                                   const vector<Function> &functions,
                                   const vector<Object> &objects,
                                   const vector<Type> &taskTypes) {
  string s = "(";
  if (type == CT_AND) {
    s += "AND";
    for (unsigned int i = 0; i < conditions.size(); i++)
      s += " " + conditions[i].toString(opParameters, controlVars, functions,
                                        objects, taskTypes);
  } else if (type == CT_GOAL) {
    s +=
        goal.toString(opParameters, controlVars, functions, objects, taskTypes);
  } else if (type == CT_FORALL) {
    s += "FORALL (";
    vector<Variable> mergedParameters;
    for (unsigned int i = 0; i < opParameters.size(); i++)
      mergedParameters.push_back(opParameters[i]);
    for (unsigned int i = 0; i < parameters.size(); i++) {
      if (i > 0)
        s += " ";
      s += parameters[i].toString(taskTypes);
      mergedParameters.push_back(parameters[i]);
    }
    s += ") " + conditions[0].toString(mergedParameters, controlVars, functions,
                                       objects, taskTypes);
  } else { // CT_PREFERENCE
    s += "PREFERENCE " + preferenceName + "(" +
         goal.toString(opParameters, controlVars, functions, objects,
                       taskTypes) +
         ")";
  }
  return s + ")";
}

/********************************************************/
/* CLASS: EffectExpression (<f-exp-da>)                 */
/********************************************************/

string EffectExpression::toString(const vector<Variable> &opParameters,
                                  const vector<Variable> &controlVars,
                                  const vector<Function> &functions,
                                  const vector<Object> &objects) {
  string s;
  switch (type) {
  case EE_NUMBER:
    s = to_string(value);
    break;
  case EE_DURATION:
    s = "?duration";
    break;
  case EE_TERM:
    s = term.toString(opParameters, controlVars, objects);
    break;
  case EE_SHARP_T:
    s = "#t";
    break;
  case EE_OPERATION:
    switch (operation) {
    case OT_SUM:
      s = "+ ";
      break;
    case OT_SUB:
      s = "- ";
      break;
    case OT_DIV:
      s = "/ ";
      break;
    case OT_MUL:
      s = "* ";
      break;
    }
    for (unsigned int i = 0; i < operands.size(); i++)
      s += " " +
           operands[i].toString(opParameters, controlVars, functions, objects);
    break;
  case EE_FLUENT:
    s = fluent.toString(opParameters, controlVars, functions, objects);
    break;
  default:
    s = "undefined";
  }
  return s;
}

/********************************************************/
/* CLASS: FluentAssignment (<p-effect>)                 */
/********************************************************/

string FluentAssignment::toString(const vector<Variable> &opParameters,
                                  const vector<Variable> &controlVars,
                                  const vector<Function> &functions,
                                  const vector<Object> &objects) {
  string s;
  switch (type) {
  case AS_ASSIGN:
    s = "ASSIGN ";
    break;
  case AS_INCREASE:
    s = "INCREASE ";
    break;
  case AS_DECREASE:
    s = "DECREASE ";
    break;
  case AS_SCALE_UP:
    s = "SCALE-UP ";
    break;
  default:
    s = "SCALE-DOWN ";
  }
  return s + fluent.toString(opParameters, controlVars, functions, objects) +
         " " + exp.toString(opParameters, controlVars, functions, objects);
}

/********************************************************/
/* CLASS: TimedEffect (<timed-effect>)                  */
/********************************************************/

string TimedEffect::toString(const vector<Variable> &opParameters,
                             const vector<Variable> &controlVars,
                             const vector<Function> &functions,
                             const vector<Object> &objects) {
  string s;
  if (time == AT_START)
    s = "AT START ";
  else if (time == AT_END)
    s = "AT END ";
  else
    s = "";
  switch (type) {
  case TE_AND:
    s += "AND";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " +
           terms[i].toString(opParameters, controlVars, functions, objects);
    break;
  case TE_NOT:
    s += "(NOT " +
         terms[0].toString(opParameters, controlVars, functions, objects) + ")";
    break;
  case TE_LITERAL:
    s += literal.toString(opParameters, controlVars, functions, objects);
    break;
  case TE_ASSIGNMENT:
    s += assignment.toString(opParameters, controlVars, functions, objects);
    break;
  default:;
  }
  return s;
}

/********************************************************/
/* CLASS: ContinuousEffect (<f-exp-t>)                  */
/********************************************************/

string ContinuousEffect::toString(const vector<Variable> &opParameters,
                                  const vector<Variable> &controlVars,
                                  const vector<Function> &functions,
                                  const vector<Object> &objects) {
  if (product) {
    return "(* #t " +
           numExp.toString(opParameters, controlVars, functions, objects) + ")";
  } else {
    return "#t";
  }
}

/************************************************************************/
/* CLASS: AssignmentContinuousEffect (<assign-op-t> <f-head> <f-exp-t>) */
/************************************************************************/

string AssignmentContinuousEffect::toString(
    const vector<Variable> &opParameters, const vector<Variable> &controlVars,
    const vector<Function> &functions, const vector<Object> &objects) {
  string s = type == AS_INCREASE ? "INCREASE " : "DECREASE ";
  return s + fluent.toString(opParameters, controlVars, functions, objects) +
         " " + contEff.toString(opParameters, controlVars, functions, objects);
}

/********************************************************/
/* CLASS: DurativeEffect (<da-effect>)                  */
/********************************************************/

string DurativeEffect::toString(const vector<Variable> &opParameters,
                                const vector<Variable> &controlVars,
                                const vector<Function> &functions,
                                const vector<Object> &objects,
                                const vector<Type> &taskTypes) {
  string s = "(";
  switch (type) {
  case DET_AND:
    s += "AND";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(opParameters, controlVars, functions,
                                   objects, taskTypes);
    break;
  case DET_TIMED_EFFECT:
    s += timedEffect.toString(opParameters, controlVars, functions, objects);
    break;
  case DET_FORALL: {
    s += "FORALL (";
    vector<Variable> mergedParameters;
    for (unsigned int i = 0; i < opParameters.size(); i++)
      mergedParameters.push_back(opParameters[i]);
    for (unsigned int i = 0; i < parameters.size(); i++) {
      if (i > 0)
        s += " ";
      s += parameters[i].toString(taskTypes);
      mergedParameters.push_back(parameters[i]);
    }
    s += ") " + terms[0].toString(mergedParameters, controlVars, functions,
                                  objects, taskTypes);
  } break;
  case DET_WHEN:
    s += "WHEN " +
         condition.toString(opParameters, controlVars, functions, objects,
                            taskTypes) +
         " (" +
         timedEffect.toString(opParameters, controlVars, functions, objects) +
         ")";
    break;
  case DET_ASSIGNMENT:
    s += assignment.toString(opParameters, controlVars, functions, objects);
    break;
  }
  return s + ")";
}

/********************************************************/
/* CLASS: DurativeAction (PDDL durative action)         */
/********************************************************/

string DurativeAction::toString(const vector<Function> &functions,
                                const vector<Object> &objects,
                                const vector<Type> &taskTypes) {
  string s = "DURATIVE-ACTION " + name + "\n* PARAMETERS (";
  for (unsigned int i = 0; i < parameters.size(); i++) {
    if (i > 0)
      s += " ";
    s += parameters[i].toString(taskTypes);
  }
  s += ")\n* DURATION (";
  for (unsigned int i = 0; i < duration.size(); i++) {
    if (i > 0)
      s += " ";
    s += duration[i].toString(parameters, controlVars, functions, objects);
  }
  return s + ")\n* CONDITION " +
         condition.toString(parameters, controlVars, functions, objects,
                            taskTypes) +
         "\n* EFFECT " +
         effect.toString(parameters, controlVars, functions, objects,
                         taskTypes);
}

/********************************************************/
/* CLASS: Precondition (<pre-GD>)                       */
/********************************************************/

string Precondition::toString(const vector<Variable> &opParameters,
                              const vector<Variable> &controlVars,
                              const vector<Function> &functions,
                              const vector<Object> &objects,
                              const vector<Type> &taskTypes) {
  string s;
  switch (type) {
  case PT_LITERAL:
    s = literal.toString(opParameters, controlVars, functions, objects);
    break;
  case PT_AND:
    s = "(AND";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(opParameters, controlVars, functions,
                                   objects, taskTypes);
    s += ")";
    break;
  case PT_NOT:
    s = "(NOT " +
        terms[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
        ")";
    break;
  case PT_OR:
    s = "(OR";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(opParameters, controlVars, functions,
                                   objects, taskTypes);
    s += ")";
    break;
  case PT_IMPLY:
    s = "(IMPLY " +
        terms[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
        terms[1].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
        ")";
    break;
  case PT_EXISTS:
  case PT_FORALL: {
    s = type == PT_EXISTS ? "(EXISTS (" : "(FORALL (";
    vector<Variable> mergedParameters;
    for (unsigned int i = 0; i < opParameters.size(); i++)
      mergedParameters.push_back(opParameters[i]);
    for (unsigned int i = 0; i < parameters.size(); i++) {
      if (i > 0)
        s += " ";
      s += parameters[i].toString(taskTypes);
      mergedParameters.push_back(parameters[i]);
    }
    s += ") " +
         terms[0].toString(mergedParameters, controlVars, functions, objects,
                           taskTypes) +
         ")";
  } break;
  case PT_F_CMP:
  case PT_EQUALITY:
  case PT_PREFERENCE:
  case PT_GOAL:
    if (type == PT_PREFERENCE)
      s = "(PREFERENCE " + preferenceName + " ";
    else
      s = "";
    s +=
        goal.toString(opParameters, controlVars, functions, objects, taskTypes);
    if (type == PT_PREFERENCE)
      s += ")";
    break;
  case PT_NEG_LITERAL:
    s = "~" + literal.toString(opParameters, controlVars, functions, objects);
    break;
  }
  return s;
}

/********************************************************/
/* CLASS: Effect (<effect>)                             */
/********************************************************/

string Effect::toString(const vector<Variable> &opParameters,
                        const vector<Variable> &controlVars,
                        const vector<Function> &functions,
                        const vector<Object> &objects,
                        const vector<Type> &taskTypes) {
  string s;
  switch (type) {
  case ET_LITERAL:
    s = literal.toString(opParameters, controlVars, functions, objects);
    break;
  case ET_AND:
    s = "(AND";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(opParameters, controlVars, functions,
                                   objects, taskTypes);
    s += ")";
    break;
  case ET_NOT:
    s = "(NOT " +
        terms[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
        ")";
    break;
  case ET_FORALL: {
    s = "(FORALL (";
    vector<Variable> mergedParameters;
    for (unsigned int i = 0; i < opParameters.size(); i++)
      mergedParameters.push_back(opParameters[i]);
    for (unsigned int i = 0; i < parameters.size(); i++) {
      if (i > 0)
        s += " ";
      s += parameters[i].toString(taskTypes);
      mergedParameters.push_back(parameters[i]);
    }
    s += ") " +
         terms[0].toString(mergedParameters, controlVars, functions, objects,
                           taskTypes) +
         ")";
  } break;
  case ET_WHEN:
    s = "(WHEN " +
        goal.toString(opParameters, controlVars, functions, objects,
                      taskTypes) +
        " " +
        terms[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
        ")";
    break;
  case ET_ASSIGNMENT:
    s = "(" +
        assignment.toString(opParameters, controlVars, functions, objects) +
        ")";
    break;
  case ET_NEG_LITERAL:
    s = "~" + literal.toString(opParameters, controlVars, functions, objects);
    break;
  }
  return s;
}

/********************************************************/
/* CLASS: Action (PDDL action)                          */
/********************************************************/

string Action::toString(const vector<Function> &functions,
                        const vector<Object> &objects,
                        const vector<Type> &taskTypes) {
  vector<Variable> controlVars;
  string s = "ACTION " + name + "\n* PARAMETERS (";
  for (unsigned int i = 0; i < parameters.size(); i++) {
    if (i > 0)
      s += " ";
    s += parameters[i].toString(taskTypes);
  }
  return s + ")\n* PRECONDITION " +
         precondition.toString(parameters, controlVars, functions, objects,
                               taskTypes) +
         "\n* EFFECT " +
         effect.toString(parameters, controlVars, functions, objects,
                         taskTypes);
}

/********************************************************/
/* CLASS: Fact (PDDL initial fact)                      */
/********************************************************/

string Fact::toString(const vector<Function> &functions,
                      const vector<Object> &objects) {
  string s = "(";
  if (time != 0)
    s += "AT " + to_string(time) + " (";
  s += "= (" + functions[function].name;
  for (unsigned int i = 0; i < parameters.size(); i++)
    s += " " + objects[parameters[i]].name;
  s += ") ";
  if (valueIsNumeric)
    s += to_string(numericValue);
  else
    s += objects[value].name;
  if (time != 0)
    s += ")";
  return s + ")";
}

/********************************************************/
/* CLASS: Metric (PDDL metric expression)               */
/********************************************************/

string Metric::toString(const vector<Function> &functions,
                        const vector<Object> &objects) {
  string s;
  switch (type) {
  case MT_PLUS:
  case MT_MINUS:
  case MT_PROD:
  case MT_DIV:
    s = "(";
    if (type == MT_PLUS)
      s += "+";
    else if (type == MT_MINUS)
      s += "-";
    else if (type == MT_PROD)
      s += "*";
    else
      s += "/";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(functions, objects);
    s += ")";
    break;
  case MT_NUMBER:
    s = to_string(value);
    break;
  case MT_TOTAL_TIME:
    s = "total-time";
    break;
  case MT_IS_VIOLATED:
    s = "is-violated " + preferenceName;
    break;
  case MT_FLUENT:
    s = functions[function].name;
    for (unsigned int i = 0; i < parameters.size(); i++)
      s += " " + objects[parameters[i]].name;
  }
  return s;
}

/********************************************************/
/* CLASS: Constraint (PDDL constraint)                  */
/********************************************************/

string Constraint::toString(const vector<Function> &functions,
                            const vector<Object> &objects,
                            const vector<Type> &taskTypes) {
  vector<Variable> parameters;
  vector<Variable> controlVars;
  return toString(parameters, controlVars, functions, objects, taskTypes);
}

string Constraint::toString(const vector<Variable> &opParameters,
                            const vector<Variable> &controlVars,
                            const vector<Function> &functions,
                            const vector<Object> &objects,
                            const vector<Type> &taskTypes) {
  string s = "(";
  switch (type) {
  case RT_AND:
    s += "AND";
    for (unsigned int i = 0; i < terms.size(); i++)
      s += " " + terms[i].toString(parameters, controlVars, functions, objects,
                                   taskTypes);
    break;
  case RT_FORALL: {
    s = "FORALL (";
    vector<Variable> mergedParameters;
    for (unsigned int i = 0; i < opParameters.size(); i++)
      mergedParameters.push_back(opParameters[i]);
    for (unsigned int i = 0; i < parameters.size(); i++) {
      if (i > 0)
        s += " ";
      s += parameters[i].toString(taskTypes);
      mergedParameters.push_back(parameters[i]);
    }
    s += ") " +
         terms[0].toString(mergedParameters, controlVars, functions, objects,
                           taskTypes) +
         ")";
  } break;
  case RT_PREFERENCE:
    s += "PREFERENCE " + preferenceName + " " +
         terms[0].toString(opParameters, controlVars, functions, objects,
                           taskTypes);
    break;
  case RT_AT_END:
    s += "AT END " + goal[0].toString(opParameters, controlVars, functions,
                                      objects, taskTypes);
    break;
  case RT_ALWAYS:
    s += "ALWAYS " + goal[0].toString(opParameters, controlVars, functions,
                                      objects, taskTypes);
    break;
  case RT_SOMETIME:
    s += "SOMETIME " + goal[0].toString(opParameters, controlVars, functions,
                                        objects, taskTypes);
    break;
  case RT_WITHIN:
    s += "WITHIN " + to_string(time[0]) + " " +
         goal[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes);
    break;
  case RT_AT_MOST_ONCE:
    s += "AT-MOST-ONCE " + goal[0].toString(opParameters, controlVars,
                                            functions, objects, taskTypes);
    break;
  case RT_SOMETIME_AFTER:
    s += "SOMETIME-AFTER " +
         goal[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
         " " +
         goal[1].toString(opParameters, controlVars, functions, objects,
                          taskTypes);
    break;
  case RT_SOMETIME_BEFORE:
    s += "SOMETIME-BEFORE " +
         goal[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
         " " +
         goal[1].toString(opParameters, controlVars, functions, objects,
                          taskTypes);
    break;
  case RT_ALWAYS_WITHIN:
    s += "ALWAYS-WITHIN " + to_string(time[0]) + " " +
         goal[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes) +
         " " +
         goal[1].toString(opParameters, controlVars, functions, objects,
                          taskTypes);
    break;
  case RT_HOLD_DURING:
    s += "HOLD-DURING " + to_string(time[0]) + " " + to_string(time[1]) + " " +
         goal[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes);
    break;
  case RT_HOLD_AFTER:
    s += "HOLD-AFTER " + to_string(time[0]) + " " +
         goal[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes);
    break;
  case RT_GOAL_PREFERENCE:
    s += "PREFERENCE " + preferenceName + " " +
         goal[0].toString(opParameters, controlVars, functions, objects,
                          taskTypes);
    break;
  }
  return s + ")";
}

/********************************************************/
/* CLASS: DerivedPredicate (derived predicate)          */
/********************************************************/

string DerivedPredicate::toString(const vector<Function> &functions,
                                  const vector<Object> &objects,
                                  const vector<Type> &taskTypes) {
  vector<Variable> controlVars;
  return "(DERIVED " + function.toString(taskTypes) + " " +
         goal.toString(function.parameters, controlVars, functions, objects,
                       taskTypes) +
         ")";
}

/********************************************************/
/* CLASS: ParsedTask (PDDL planning task)               */
/********************************************************/

void ParsedTask::setError(string e) { error = e; }

// Sets the name of the domain
void ParsedTask::setDomainName(string name) {
  domainName = name;
  vector<unsigned int> parentTypes;
  BOOLEAN_TYPE = addType("#boolean", parentTypes, nullptr);
  NUMBER_TYPE = addType("number", parentTypes, nullptr);
  INTEGER_TYPE = addType("integer", parentTypes, nullptr);
  parentTypes.push_back(BOOLEAN_TYPE);
  CONSTANT_FALSE = addConstant("#false", parentTypes, nullptr);
  CONSTANT_TRUE = addConstant("#true", parentTypes, nullptr);
}

// Sets the name of the problem
void ParsedTask::setProblemName(string name) { problemName = name; }

// Sets a requirement key
void ParsedTask::setRequirement(string name) { requirements.push_back(name); }

// Returns the index of a type through its name
unsigned int ParsedTask::getTypeIndex(string const &name) {
  unordered_map<string, unsigned int>::const_iterator index =
      typesByName.find(name);
  if (index == typesByName.end()) { // Type not found
    if (name.compare("#object") == 0) {
      unsigned int i = (unsigned int)types.size();
      Type t(i, name);
      types.push_back(t);
      typesByName[name] = i;
      return i;
    } else
      return MAX_UNSIGNED_INT;
  }
  return index->second;
}

// Stores a PDDL type and returns its index
unsigned int ParsedTask::addType(string name, vector<unsigned int> &parentTypes,
                                 SyntaxAnalyzer *syn) {
  unsigned int index = getTypeIndex(name);
  Type *t;
  if (index != MAX_UNSIGNED_INT) { // Type redefined -> update its parent types
    t = &(types.at(index));
  } else {
    index = (unsigned int)types.size();
    Type newType(index, name);
    types.push_back(newType);
    typesByName[name] = index;
    t = &(types.back());
  }
  for (unsigned int i = 0; i < parentTypes.size(); i++) {
    t->parentTypes.push_back(parentTypes[i]);
  }
  return index;
}

// Returns the index of an object through its name
unsigned int ParsedTask::getObjectIndex(string const &name) {
  unordered_map<string, unsigned int>::const_iterator index =
      objectsByName.find(name);
  if (index == objectsByName.end())
    return MAX_UNSIGNED_INT;
  else
    return index->second;
}

// Stores a PDDL constant and returns its index
unsigned int ParsedTask::addConstant(string name, vector<unsigned int> &types,
                                     SyntaxAnalyzer *syn) {
  if (getObjectIndex(name) != MAX_UNSIGNED_INT)
    syn->notifyError("Constant '" + name + "' redefined");
  unsigned int index = (unsigned int)objects.size();
  Object obj(index, name, true);
  for (unsigned int i = 0; i < types.size(); i++)
    obj.types.push_back(types[i]);
  objects.push_back(obj);
  objectsByName[name] = index;
  return index;
}

// Stores a PDDL object and returns its index
unsigned int ParsedTask::addObject(string name, vector<unsigned int> &types,
                                   SyntaxAnalyzer *syn) {
  Object *obj;
  unsigned int index = getObjectIndex(name);
  if (getObjectIndex(name) != MAX_UNSIGNED_INT) {
    obj = &(objects.at(index));
  } else {
    index = (unsigned int)objects.size();
    Object newObj(index, name, false);
    objects.push_back(newObj);
    objectsByName[name] = index;
    obj = &(objects.back());
  }
  for (unsigned int i = 0; i < types.size(); i++) {
    obj->types.push_back(types[i]);
  }
  return index;
}

// Returns the index of a function through its name
unsigned int ParsedTask::getFunctionIndex(string const &name) {
  unordered_map<string, unsigned int>::const_iterator index =
      functionsByName.find(name);
  if (index == functionsByName.end())
    return MAX_UNSIGNED_INT;
  else
    return index->second;
}

// Returns the index of a preference through its name
unsigned int ParsedTask::getPreferenceIndex(std::string const &name) {
  unordered_map<string, unsigned int>::const_iterator index =
      preferencesByName.find(name);
  if (index == preferencesByName.end())
    return MAX_UNSIGNED_INT;
  else
    return index->second;
}

// Stores a predicate and returns its index
unsigned int ParsedTask::addPredicate(Function fnc, SyntaxAnalyzer *syn) {
  if (getFunctionIndex(fnc.name) != MAX_UNSIGNED_INT)
    syn->notifyError("Predicate '" + fnc.name + "' redefined");
  unsigned int index = (unsigned int)functions.size();
  fnc.index = index;
  fnc.valueTypes.push_back(BOOLEAN_TYPE);
  functions.push_back(fnc);
  functionsByName[fnc.name] = index;
  return index;
}

// Stores a function and returns its index
unsigned int ParsedTask::addFunction(Function fnc,
                                     const vector<unsigned int> &valueTypes,
                                     SyntaxAnalyzer *syn) {
  if (getFunctionIndex(fnc.name) != MAX_UNSIGNED_INT)
    syn->notifyError("Function '" + fnc.name + "' redefined");
  unsigned int index = (unsigned int)functions.size();
  fnc.index = index;
  fnc.setValueTypes(valueTypes);
  functions.push_back(fnc);
  functionsByName[fnc.name] = index;
  return index;
}

unsigned int ParsedTask::addFunction(Function fnc, SyntaxAnalyzer *syn) {
  if (getFunctionIndex(fnc.name) != MAX_UNSIGNED_INT)
    syn->notifyError("Function '" + fnc.name + "' redefined");
  unsigned int index = (unsigned int)functions.size();
  fnc.index = index;
  fnc.valueTypes.push_back(NUMBER_TYPE);
  functions.push_back(fnc);
  functionsByName[fnc.name] = index;
  return index;
}

// Stores a durative action and returns its index
unsigned int ParsedTask::addAction(string name,
                                   const vector<Variable> &parameters,
                                   const vector<Variable> &controlVars,
                                   const vector<Duration> &duration,
                                   const DurativeCondition &condition,
                                   const DurativeEffect &effect,
                                   SyntaxAnalyzer *syn) {
  for (unsigned int i = 0; i < durativeActions.size(); i++)
    if (durativeActions[i].name.compare(name) == 0)
      syn->notifyError("Action '" + name + "' redefined");
  for (unsigned int i = 0; i < actions.size(); i++)
    if (actions[i].name.compare(name) == 0)
      syn->notifyError("Action '" + name + "' redefined");
  DurativeAction a;
  a.index = (unsigned int)durativeActions.size();
  a.name = name;
  for (unsigned int i = 0; i < parameters.size(); i++)
    a.parameters.push_back(parameters[i]);
  for (unsigned int i = 0; i < controlVars.size(); i++)
    a.controlVars.push_back(controlVars[i]);
  for (unsigned int i = 0; i < duration.size(); i++)
    a.duration.push_back(duration[i]);
  a.condition = condition;
  a.effect = effect;
  durativeActions.push_back(a);
  return a.index;
}

// Stores an action and returns its index
unsigned int ParsedTask::addAction(std::string name,
                                   const vector<Variable> &parameters,
                                   const Precondition &precondition,
                                   const Effect &effect, SyntaxAnalyzer *syn) {
  for (unsigned int i = 0; i < durativeActions.size(); i++)
    if (durativeActions[i].name.compare(name) == 0)
      syn->notifyError("Action '" + name + "' redefined");
  for (unsigned int i = 0; i < actions.size(); i++)
    if (actions[i].name.compare(name) == 0)
      syn->notifyError("Action '" + name + "' redefined");
  Action a;
  a.index = (int)actions.size();
  a.name = name;
  for (unsigned int i = 0; i < parameters.size(); i++)
    a.parameters.push_back(parameters[i]);
  a.precondition = precondition;
  a.effect = effect;
  actions.push_back(a);
  return a.index;
}

// Stores a preference
unsigned int ParsedTask::addPreference(std::string name,
                                       const GoalDescription &goal,
                                       SyntaxAnalyzer *syn) {
  if (getPreferenceIndex(name) != MAX_UNSIGNED_INT)
    syn->notifyError("Preference '" + name + "' redefined");
  unsigned int index = (unsigned int)preferences.size();
  preferencesByName[name] = index;
  Constraint c;
  c.type = RT_GOAL_PREFERENCE;
  c.preferenceName = name;
  c.goal.push_back(goal);
  preferences.push_back(c);
  return index;
}

// Stores a preference
unsigned int ParsedTask::addPreference(const Constraint &c,
                                       SyntaxAnalyzer *syn) {
  if (getPreferenceIndex(c.preferenceName) != MAX_UNSIGNED_INT)
    syn->notifyError("Preference '" + c.preferenceName + "' redefined");
  unsigned int index = (unsigned int)preferences.size();
  preferencesByName[c.preferenceName] = index;
  preferences.push_back(c);
  return index;
}

// Checks whether the given function is numeric
bool ParsedTask::isNumericFunction(unsigned int fncIndex) {
  Function &f = functions[fncIndex];
  if (f.valueTypes.size() != 1)
    return false;
  return f.valueTypes[0] == NUMBER_TYPE;
}

// Checks whether the given function is boolean (a predicate)
bool ParsedTask::isBooleanFunction(unsigned int fncIndex) {
  Function &f = functions[fncIndex];
  if (f.valueTypes.size() != 1)
    return false;
  return f.valueTypes[0] == BOOLEAN_TYPE;
}

// Returns a description of this planning task
string ParsedTask::toString() {
  string res = "Domain: " + domainName;
  res += "\nRequirements:";
  for (unsigned int i = 0; i < requirements.size(); i++)
    res += " " + requirements[i];
  res += "\nTypes:";
  for (unsigned int i = 0; i < types.size(); i++)
    res += "\n* " + types[i].toString();
  res += "\nObjects:";
  for (unsigned int i = 0; i < objects.size(); i++)
    res += "\n* " + objects[i].toString();
  res += "\nFunctions:";
  for (unsigned int i = 0; i < functions.size(); i++)
    res += "\n* " + functions[i].toString(types);
  for (unsigned int i = 0; i < durativeActions.size(); i++)
    res += "\n" + durativeActions[i].toString(functions, objects, types);
  for (unsigned int i = 0; i < actions.size(); i++)
    res += "\n" + actions[i].toString(functions, objects, types);
  res += "\nInit:";
  for (unsigned int i = 0; i < init.size(); i++)
    res += "\n* " + init[i].toString(functions, objects);
  vector<Variable> parameters;
  vector<Variable> controlVars;
  res += "\nGoal:\n* " +
         goal.toString(parameters, controlVars, functions, objects, types);
  if (metricType != MT_NONE) {
    res += "\nMetric: ";
    if (metricType == MT_MINIMIZE)
      res += "MINIMIZE ";
    else
      res += "MAXIMIZE ";
    res += metric.toString(functions, objects);
  }
  for (unsigned int i = 0; i < constraints.size(); i++)
    res += "\nConstraint:\n* " +
           constraints[i].toString(functions, objects, types);
  for (unsigned int i = 0; i < derivedPredicates.size(); i++)
    res += "\n" + derivedPredicates[i].toString(functions, objects, types);
  return res;
}

float ParsedTask::ellapsedTime() { return toSeconds(startTime); }

// Checks if one of the types is compatible with one of the valid ones
bool ParsedTask::compatibleTypes(const vector<unsigned int> &types,
                                 const vector<unsigned int> &validTypes) {
  unsigned int t1, t2;
  for (unsigned int i = 0; i < types.size(); i++) {
    t1 = types[i];
    for (unsigned int j = 0; j < validTypes.size(); j++) {
      t2 = validTypes[j];
      if (compatibleTypes(t1, t2))
        return true;
    }
  }
  return false;
}

// Checks if a given type t1 is compatible with another one t2
bool ParsedTask::compatibleTypes(unsigned int t1, unsigned int t2) {
  if (t1 == t2)
    return true;
  Type &refT1 = types[t1];
  for (unsigned int i = 0; i < refT1.parentTypes.size(); i++)
    if (compatibleTypes(refT1.parentTypes[i], t2))
      return true;
  return false;
}

// Returns a string representation of a comparator
string ParsedTask::comparatorToString(Comparator cmp) {
  switch (cmp) {
  case CMP_EQ:
    return "=";
  case CMP_LESS:
    return "<";
  case CMP_LESS_EQ:
    return "<=";
  case CMP_GREATER:
    return ">";
  case CMP_GREATER_EQ:
    return ">=";
  case CMP_NEQ:
    return "!=";
  }
  return "";
}

// Returns a string representation of an assignment
string ParsedTask::assignmentToString(Assignment a) {
  switch (a) {
  case AS_ASSIGN:
    return "assign";
  case AS_INCREASE:
    return "increase";
  case AS_DECREASE:
    return "decrease";
  case AS_SCALE_UP:
    return "scale-up";
  case AS_SCALE_DOWN:
    return "scale-down";
  }
  return "";
}

// Returns a string representation of a time specifier
string ParsedTask::timeToString(TimeSpecifier t) {
  switch (t) {
  case AT_START:
    return "at start";
  case AT_END:
    return "at end";
  case OVER_ALL:
    return "over all";
  default:
    return "";
  }
}
