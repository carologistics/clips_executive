#ifndef SELECTOR_H
#define SELECTOR_H

/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Plan selector
 */
/********************************************************/

#include "plan.h"
#include <vector>

class SearchQueue {
private:
  const static unsigned int INITIAL_PQ_CAPACITY = 8192;
  std::vector<Plan *> pq;

  void heapify(unsigned int gap);

public:
  SearchQueue();
  void add(Plan *p);
  Plan *poll();
  inline Plan *peek() { return pq[1]; }
  inline int size() { return (int)pq.size() - 1; }
  inline Plan *getPlanAt(unsigned int i) { return pq[i]; }
  void clear();
};

#endif // !SELECTOR_H
