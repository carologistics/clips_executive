#ifndef STATE_H
#define STATE_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* State representation */
/********************************************************/

#include "../sas/sasTask.h"
#include "../utils/utils.h"

class TState {
public:
  unsigned int numSASVars; // Number of SAS variables
  unsigned int numNumVars; // Number of numeric variables
  TValue *state;           // Values of the SAS variables in the state
  TFloatValue *minState; // Minimum values of the numeric variables in the state
  TFloatValue *maxState; // Maximum values of the numeric variables in the state

  TState(unsigned int numSASVars, unsigned int numNumVars);
  TState(SASTask *task);
  ~TState();
  inline uint64_t getCode() {
    uint64_t code = 0;
    for (unsigned int i = 0; i < numSASVars; i++)
      code = 31 * code + state[i];
    for (unsigned int i = 0; i < numNumVars; i++)
      code = 31 * code + ((uint64_t)(100 * (minState[i] + maxState[i])));
    return code;
  }
  inline bool compareTo(TState *s) {
    for (unsigned int i = 0; i < numNumVars; i++) {
      if (abs(minState[i] - s->minState[i]) >= EPSILON)
        return false;
      if (abs(maxState[i] - s->maxState[i]) >= EPSILON)
        return false;
    }
    for (unsigned int i = 0; i < numSASVars; i++) {
      if (state[i] != s->state[i])
        return false;
    }
    return true;
  }
  TFloatValue sumNumValues() {
    TFloatValue sum = 0;
    for (unsigned int i = 0; i < numNumVars; i++) {
      sum += minState[i];
    }
    return sum;
  }
};

#endif