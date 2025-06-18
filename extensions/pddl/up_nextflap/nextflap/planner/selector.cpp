/********************************************************/
/* Oscar Sapena Vercher - DSIC - UPV                    */
/* April 2022                                           */
/********************************************************/
/* Plan selector
 */
/********************************************************/

#include "selector.h"
using namespace std;

/*******************************************/
/* SearchQueue                             */
/*******************************************/

SearchQueue::SearchQueue() {
  pq.reserve(INITIAL_PQ_CAPACITY);
  pq.push_back(nullptr); // Position 0 empty
}

// Adds a new plan to the list of open nodes
void SearchQueue::add(Plan *p) {
  unsigned int gap = (unsigned int)pq.size();
  uint32_t parent;
  pq.push_back(nullptr);
  while (gap > 1 && p->compare(pq[gap >> 1]) < 0) {
    parent = gap >> 1;
    pq[gap] = pq[parent];
    gap = parent;
  }
  pq[gap] = p;
}

// Removes and returns the best plan in the queue of open nodes
Plan *SearchQueue::poll() {
  Plan *best = pq[1];
  if (pq.size() > 2) {
    pq[1] = pq.back();
    pq.pop_back();
    heapify(1);
  } else if (pq.size() > 1)
    pq.pop_back();
  return best;
}

// Repairs the order in the priority queue
void SearchQueue::heapify(unsigned int gap) {
  Plan *aux = pq[gap];
  unsigned int child = gap << 1;
  while (child < pq.size()) {
    if (child != pq.size() - 1 && pq[child + 1]->compare(pq[child]) < 0)
      child++;
    if (pq[child]->compare(aux) < 0) {
      pq[gap] = pq[child];
      gap = child;
      child = gap << 1;
    } else
      break;
  }
  pq[gap] = aux;
}

void SearchQueue::clear() {
  pq.clear();
  pq.push_back(nullptr); // Position 0 empty
}
