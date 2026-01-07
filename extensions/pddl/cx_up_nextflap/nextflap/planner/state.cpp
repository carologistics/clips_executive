#include "state.h"
using namespace std;

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* State representation */
/********************************************************/

TState::TState(unsigned int numSASVars, unsigned int numNumVars) {
  this->numSASVars = numSASVars;
  this->numNumVars = numNumVars;
  state = new TValue[numSASVars];
  minState = new TFloatValue[numNumVars];
  maxState = new TFloatValue[numNumVars];
}

TState::TState(SASTask *task)
    : TState(task->variables.size(),
             task->numVariables.size()) { // Create the initial state
  for (unsigned int i = 0; i < numSASVars; i++) {
    this->state[i] = task->initialState[i];
  }
  for (unsigned int i = 0; i < numNumVars; i++) {
    this->minState[i] = this->maxState[i] = task->numInitialState[i];
  }
}

TState::~TState() {
  delete[] state;
  delete[] minState;
  delete[] maxState;
}
