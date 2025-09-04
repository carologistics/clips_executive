#ifndef LINEARIZER_H
#define LINEARIZER_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Calculates a linear order among the steps of the     */
/* plan.                                                */
/********************************************************/

#include "../utils/priorityQueue.h"
#include "../utils/utils.h"
#include "planComponents.h"
#include <vector>

class ScheduledTimepoint : public PriorityQueueItem {
public:
  TTimePoint point;
  float scheduledTime;
  ScheduledTimepoint(TTimePoint p, float time) {
    point = p;
    scheduledTime = time;
  }
  inline int compare(PriorityQueueItem *other) {
    float otherTime = ((ScheduledTimepoint *)other)->scheduledTime;
    if (scheduledTime < otherTime)
      return -1;
    else if (scheduledTime > otherTime)
      return 1;
    else
      return 0;
  }
  virtual ~ScheduledTimepoint() {}
};

class Linearizer {
public:
  std::vector<TTimePoint> linearOrder;

  void linearize(PlanComponents &planComponents);
};

#endif