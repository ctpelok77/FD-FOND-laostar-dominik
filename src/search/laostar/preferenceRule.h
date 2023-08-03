#ifndef PREFERENCES
#define PREFERENCES value
#include <memory>
#include "../search_space.h"

using std::shared_ptr;

class preferNodesWithLowerCostEstimates
{
 public:

  // lhs is prefered over rhs iff lhs has at least 
  // as low costs as rhs
  bool operator() (const shared_ptr<SearchNode> lhs, const shared_ptr<SearchNode> rhs) const
  {
	// Potential tiebreaking case
	if (abs(lhs->costEstimate - rhs->costEstimate) <= 0.01) 
	{
		if(lhs->heuristic < rhs->heuristic) return false;
		if(lhs->heuristic > rhs->heuristic) return true;
		if(lhs->index < rhs->index) return false;
		return true;
	}
	else { // Non tie-breaking case 
		if (lhs->costEstimate < rhs->costEstimate) return false;
		return true;
	}
  }
};

class preferNodesWithHigherIndex
{
 public:

  // Preference relation lhs is preferred over rhs iff lhs has at least 
  // as high costs as rhs
  bool operator() (const shared_ptr<SearchNode> lhs, const shared_ptr<SearchNode>  rhs) const
  {
    if (lhs->index > rhs->index) {
      return true;
    }
    else {
      return false;
    }
  }
};
#endif
