/**
 * This class contains componentes and helper methods 
 * for the LaostarSearch class.
 */

#ifndef LAOSTAR_UTILS_H
#define LAOSTAR_UTILS_H

#include <memory>
#include <limits>
#include <utility>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include "connector.h"
#include "util.h"
#include "preferenceRule.h"
#include <unordered_set>

using std::numeric_limits;
using std::pair;
using boost::heap::fibonacci_heap;
using std::unordered_map;
using std::queue;
using std::vector;
using std::find;
using std::unordered_set;

// This is the data type of our main queue
#ifndef HEAP
#define HEAP
typedef fibonacci_heap<shared_ptr<SearchNode>, boost::heap::compare<preferNodesWithLowerCostEstimates>> MAX_HEAP_COST;
typedef fibonacci_heap<shared_ptr<SearchNode>, boost::heap::compare<preferNodesWithHigherIndex>> MAX_HEAP_INDEX;
#endif

class LaostarUtils {
public:
	LaostarUtils(const Options &opts);
	/**
	 * Debug flag to make visible what the algortihm is currently
	 * doing. 
	 */
	bool DEBUG;
	/**
	 * Use maximal/average child estimate for marking best connectors.
	 */
	bool useMaxChildEstimate;

	/**
	* Epsilon (for double comparison)
	*
	*/
	const double EPSILON = 0.0001;

	/**
	 * Discount factor that we need for discounted MDP
	 * we solve in value iteration.
	 */
	const double DISCOUNT_FACTOR = 0.75;

	/**  
	*   A connector is promising only if it has at least one
	*   child such that the shortest path from that child to a
	*   goal is strictly shorter than the shortest path from
	*   the parent of the connector to a goal.
	*/
	bool connectorIsPromising(Connector &connector);

	/**
	* Value Iteration algorithm
	*/
	void performValueIteration(vector<shared_ptr<SearchNode>>& allNodes, shared_ptr<SearchNode>& initialNode);

	/**
	* If repeatetly called, this functions marks the 
	* the best connector among all outgoing connectors of a given 
	* SearchNode
	*/
	void markBestConnector(int candidateBestConnectorIndex, 
						 shared_ptr<SearchNode> node,
						 unordered_map<int, double>& oldCostEstimate,	
						 vector<shared_ptr<SearchNode>>& updatedNodes);
	/**
	* Compute all connectors backward reachable from 
	* the currently detected goal nodes 
	*/	 
	void computeBackwardReachableConnectors(unordered_map<StateID, shared_ptr<SearchNode>>& all_nodes_map,
										unordered_map<int, int>& distanceMap,
										vector<shared_ptr<SearchNode>>&  goalNodes, vector<shared_ptr<Connector>>& connectors);
	  
	/**
	* Trace down all marked connectors starting from a given state, collecting all unexpanded nodes
	* encountered. Prune trace at proven and disproven nodes.
	*
	* initial: node to start from
	* the set of all unexpanded nodes in the partial solution graph induced by the marked
	*         connectors which are neither proven nor disproven sorted by decreasing h-Values. //
	*         
	*/
	void traceMarkedConnectors(shared_ptr<SearchNode>& initial, vector<shared_ptr<SearchNode>>& result);  

	/**
	 * Debugging functions to visualize the state space
	 */
	void printSearchNode(shared_ptr<SearchNode> searchNode);
	void printConnector(Connector& connector);
	void printOperatorNames(vector<GlobalOperator*>& operators);
	void printSetOfNodes(std::vector<SearchNode>& v);
	void printSetOfNodes(std::vector<shared_ptr<SearchNode>>& v);
	void printSetOfNodes(std::set<shared_ptr<SearchNode>>& v, std::string name);
	void printQueue(MAX_HEAP_COST heap);
	void printQueue(std::queue<shared_ptr<Connector>>& q);
	void printSetOfConnectors(std::vector<Connector>& c);
	void printMap(unordered_map<int, int>& map);
	void printSetOfConnectors(vector<shared_ptr<Connector>>& c);
	void printSetOfNodes(std::map<size_t, shared_ptr<SearchNode>>& map);
	void printSet(std::set<int>& s);
	void printQueue(MAX_HEAP_INDEX heap);
};
#endif

