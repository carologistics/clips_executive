#include "intervalCalculations.h"

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Class for calculations with numeric intervals.       */
/********************************************************/

using namespace std;

// Evaluates the value [min, max] of a numeric expression
void IntervalCalculations::evaluateExpression(SASNumericExpression *e,
                                              TFloatValue *min,
                                              TFloatValue *max) {
  if (e->type == 'N') { // Constant number
    *min = e->value;
    *max = e->value;
  } else if (e->type == 'V') { // Fluent variable
    *min = fluentValues[e->var].minValue;
    *max = fluentValues[e->var].maxValue;
  } else if (e->type == '+' || e->type == '-' || e->type == '*' ||
             e->type == '/') {
    TFloatValue minLeft, maxLeft, minRight, maxRight;
    evaluateExpression(&(e->terms[0]), &minLeft, &maxLeft);
    evaluateExpression(&(e->terms[1]), &minRight, &maxRight);
    switch (e->type) {
    case '+':
      *min = minLeft + minRight;
      *max = maxLeft + maxRight;
      break;
    case '-':
      *min = minLeft - maxRight;
      *max = maxLeft - minRight;
      break;
    case '/': {
      TFloatValue d1 = minLeft / maxRight, d2 = maxLeft / minRight,
                  d3 = minLeft / minRight, d4 = maxLeft / maxRight;
      *min = std::min(std::min(d1, d2), std::min(d3, d4));
      *max = std::max(std::max(d1, d2), std::max(d3, d4));
    } break;
    case '*': {
      TFloatValue d1 = minLeft * maxRight, d2 = maxLeft * minRight,
                  d3 = minLeft * minRight, d4 = maxLeft * maxRight;
      *min = std::min(std::min(d1, d2), std::min(d3, d4));
      *max = std::max(std::max(d1, d2), std::max(d3, d4));
    } break;
    }
  } else if (e->type == 'C') {
    *min = cvarValues[e->var].minValue;
    *max = cvarValues[e->var].maxValue;
  } else if (e->type == 'D') {
    *min = this->duration.minValue;
    *max = this->duration.maxValue;
  } else {
    throwError("Numeric expression not supported in "
               "IntervalCalculations::evaluateExpression");
  }
}

// Constrains an interval according to the comparison with a numeric expression
void IntervalCalculations::constrainInterval(char comp, SASNumericExpression *e,
                                             TInterval *cvarInt) {
  if (comp == 'N')
    return;
  TFloatValue minRightSide, maxRightSide;
  evaluateExpression(e, &minRightSide, &maxRightSide);
  switch (comp) {
  case '=':
    cvarInt->minValue = minRightSide;
    cvarInt->maxValue = maxRightSide;
    break;
  case '<':
    maxRightSide -= EPSILON;
  case 'L':
    if (cvarInt->maxValue > maxRightSide)
      cvarInt->maxValue = maxRightSide;
    break;
  case '>':
    minRightSide += EPSILON;
  case 'G':
    if (cvarInt->minValue < minRightSide)
      cvarInt->minValue = minRightSide;
    break;
  }
}

// Calculates the interval values of the control parameters
void IntervalCalculations::calculateControlVarIntervals() {
  cvarValues.resize(a->controlVars.size());
  for (SASControlVar &cv : a->controlVars) {
    int index = cv.index;
    cvarValues[index].minValue = -FLOAT_INFINITY;
    cvarValues[index].maxValue = FLOAT_INFINITY;
    for (SASControlVarCondition &cvc : cv.conditions) {
      SASNumericCondition &c = cvc.condition;
      constrainInterval(c.comp, &c.terms[1], &cvarValues[index]);
    }
  }
}

// Calculates the duration interval of an action
void IntervalCalculations::calculateDuration() {
  duration.minValue = EPSILON;
  duration.maxValue = FLOAT_INFINITY;
  for (SASDurationCondition &dur : a->duration.conditions) {
    constrainInterval(dur.comp, &dur.exp, &duration);
  }
}

// Checks if a numeric expression is supported
bool IntervalCalculations::supportedCondition(SASNumericCondition *c) {
  if (c->comp == '-')
    return true;
  TFloatValue min1, max1, min2, max2;
  evaluateExpression(&c->terms[0], &min1, &max1);
  evaluateExpression(&c->terms[1], &min2, &max2);
  switch (c->comp) {
  case '=': // [2, 4] = [3, 5] -> [max(2, 3), min(4, 5)] = [3, 4] -> Ok si
            // intervalo no vacío
    return std::max(min1, min2) <= std::min(max1, max2);
  case '<': // [a, b] < [c, d] -> a < d
    return min1 < max2;
  case 'L': // [a, b] <= [c, d] -> a <= d
    return min1 <= max2;
  case '>': // [a, b] > [c, d] -> b > c
    return max1 > min2;
  case 'G': // [a, b] > [c, d] -> b >= c
    return max1 >= min2;
  case 'N': // [a, b] != [c, d] -> !(a == b == c == d)
    if (min1 != max1)
      return true;
    if (min2 != min1)
      return true;
    return max2 != min1;
  }
  return false;
}

// Constructor
IntervalCalculations::IntervalCalculations(SASAction *a, int numState,
                                           FluentIntervalData *fluentData,
                                           SASTask *task) {
  this->a = a;
  fluentValues.resize(task->numVariables.size());
  for (TVariable v = 0; v < fluentValues.size(); v++) {
    fluentValues[v].minValue = fluentData->getMinValue(v, numState);
    fluentValues[v].maxValue = fluentData->getMaxValue(v, numState);
  }
  calculateControlVarIntervals();
  calculateDuration();
}

// Checks if the start numeric conditions are supported
bool IntervalCalculations::supportedNumericStartConditions(bool *holdCondEff) {
  for (SASNumericCondition &c : a->startNumCond)
    if (!supportedCondition(&c))
      return false;
  for (SASNumericCondition &c : a->overNumCond)
    if (!supportedCondition(&c))
      return false;
  if (holdCondEff != nullptr) {
    for (unsigned int i = 0; i < a->conditionalEff.size(); i++) {
      if (holdCondEff[i]) {
        for (SASNumericCondition &c : a->conditionalEff[i].startNumCond)
          if (!supportedCondition(&c))
            return false;
      }
    }
  }
  return true;
}

// Checks if the end numeric conditions are supported
bool IntervalCalculations::supportedNumericEndConditions(bool *holdCondEff) {
  for (SASNumericCondition &c : a->overNumCond)
    if (!supportedCondition(&c))
      return false;
  for (SASNumericCondition &c : a->endNumCond)
    if (!supportedCondition(&c))
      return false;
  if (holdCondEff != nullptr) {
    for (unsigned int i = 0; i < a->conditionalEff.size(); i++) {
      if (holdCondEff[i]) {
        for (SASNumericCondition &c : a->conditionalEff[i].endNumCond)
          if (!supportedCondition(&c))
            return false;
      }
    }
  }
  return true;
}

bool IntervalCalculations::supportedNumericConditions(SASConditionalEffect *e) {
  for (SASNumericCondition &c : e->startNumCond)
    if (!supportedCondition(&c))
      return false;
  for (SASNumericCondition &c : e->endNumCond)
    if (!supportedCondition(&c))
      return false;
  return true;
}

// Constrains the intervals of all numeric variables in the start conditions
void IntervalCalculations::constrainAtStartFluents() {
  for (auto &pair : a->startNumConstrains) {
    TVariable v = pair.first;
    for (SASNumericCondition &c : pair.second) {
      constrainInterval(c.comp, &c.terms[1], &fluentValues[v]);
    }
  }
}

// Updates the variable interval with the action effect
void IntervalCalculations::applyEffect(SASNumericEffect *e) {
  TFloatValue expMin, expMax;
  TInterval &i = fluentValues[e->var];
  evaluateExpression(&e->exp, &expMin, &expMax);
  switch (e->op) { // '=' = AS_ASSIGN, '+' = AS_INCREASE, '-' = AS_DECREASE, '*'
                   // = AS_SCALE_UP, '/' = AS_SCALE_DOWN
  case '=':
    i.minValue = expMin;
    i.maxValue = expMax;
    break;
  case '+':
    i.minValue += expMin;
    i.maxValue += expMax;
    break;
  case '-':
    i.minValue -= expMax;
    i.maxValue -= expMin;
    break;
  case '*':
    i.minValue *= expMin;
    i.maxValue *= expMax;
    break;
  case '/':
    i.minValue /= expMax;
    i.maxValue /= expMin;
    break;
  }
}

// Constrains the interval of a numeric variable in the start conditions
void IntervalCalculations::constrainAtStartFluent(TVariable v) {
  if (a->startNumConstrains.find(v) != a->startNumConstrains.end()) {
    std::vector<SASNumericCondition> &cons = a->startNumConstrains[v];
    for (SASNumericCondition &c : cons) {
      if (c.comp != '-')
        constrainInterval(c.comp, &c.terms[1], &fluentValues[v]);
    }
  }
}

// Constrains the interval of a numeric variable in the end conditions
void IntervalCalculations::constrainAtEndFluent(TVariable v) {
  if (a->endNumConstrains.find(v) != a->endNumConstrains.end()) {
    std::vector<SASNumericCondition> &cons = a->endNumConstrains[v];
    for (SASNumericCondition &c : cons) {
      if (c.comp != '-')
        constrainInterval(c.comp, &c.terms[1], &fluentValues[v]);
    }
  }
}

// Updates the variables intervals with the start effects
void IntervalCalculations::applyStartEffects(Plan *p, bool *holdCondEff) {
  for (SASNumericEffect &e : a->startNumEff) {
    applyEffect(&e);
    p->startPoint.addNumericValue(e.var, fluentValues[e.var].minValue,
                                  fluentValues[e.var].maxValue);
  }
  if (holdCondEff != nullptr) {
    for (unsigned int i = 0; i < a->conditionalEff.size(); i++) {
      if (holdCondEff[i]) {
        for (SASNumericEffect &e : a->conditionalEff[i].startNumEff) {
          applyEffect(&e);
          p->startPoint.addNumericValue(e.var, fluentValues[e.var].minValue,
                                        fluentValues[e.var].maxValue);
        }
      }
    }
  }
}

// Updates the variables intervals with the start effects
void IntervalCalculations::applyStartEffects(std::vector<TNumVarChange> *v,
                                             bool *holdCondEff) {
  for (SASNumericEffect &e : a->startNumEff) {
    applyEffect(&e);
    constrainAtStartFluent(e.var);
    v->emplace_back(e.var, fluentValues[e.var].minValue,
                    fluentValues[e.var].maxValue);
  }
  if (holdCondEff != nullptr) {
    for (unsigned int i = 0; i < a->conditionalEff.size(); i++) {
      if (holdCondEff[i]) {
        for (SASNumericEffect &e : a->conditionalEff[i].startNumEff) {
          applyEffect(&e);
          constrainAtStartFluent(e.var);
          v->emplace_back(e.var, fluentValues[e.var].minValue,
                          fluentValues[e.var].maxValue);
        }
      }
    }
  }
}

// Updates the variables intervals with the end effects
void IntervalCalculations::applyEndEffects(Plan *p, bool *holdCondEff) {
  for (SASNumericEffect &e : a->endNumEff) {
    applyEffect(&e);
    p->endPoint.addNumericValue(e.var, fluentValues[e.var].minValue,
                                fluentValues[e.var].maxValue);
  }
  if (holdCondEff != nullptr) {
    for (unsigned int i = 0; i < a->conditionalEff.size(); i++) {
      if (holdCondEff[i]) {
        for (SASNumericEffect &e : a->conditionalEff[i].endNumEff) {
          applyEffect(&e);
          p->endPoint.addNumericValue(e.var, fluentValues[e.var].minValue,
                                      fluentValues[e.var].maxValue);
        }
      }
    }
  }
}

// Updates the variables intervals with the end effects
void IntervalCalculations::applyEndEffects(std::vector<TNumVarChange> *v,
                                           bool *holdCondEff) {
  for (SASNumericEffect &e : a->endNumEff) {
    applyEffect(&e);
    constrainAtEndFluent(e.var);
    v->emplace_back(e.var, fluentValues[e.var].minValue,
                    fluentValues[e.var].maxValue);
  }
  if (holdCondEff != nullptr) {
    for (unsigned int i = 0; i < a->conditionalEff.size(); i++) {
      if (holdCondEff[i]) {
        for (SASNumericEffect &e : a->conditionalEff[i].endNumEff) {
          applyEffect(&e);
          constrainAtEndFluent(e.var);
          v->emplace_back(e.var, fluentValues[e.var].minValue,
                          fluentValues[e.var].maxValue);
        }
      }
    }
  }
}

// Makes a backup of the control parameter intervals
void IntervalCalculations::copyControlVars(Plan *p) {
  if (cvarValues.empty())
    return;
  p->cvarValues = new std::vector<TInterval>();
  for (TInterval &i : cvarValues) {
    p->cvarValues->push_back(i);
  }
  p->cvarValues->shrink_to_fit();
}

// Makes a backup of the duration interval
void IntervalCalculations::copyDuration(Plan *p) {
  p->setDuration(duration.minValue, duration.maxValue);
}
