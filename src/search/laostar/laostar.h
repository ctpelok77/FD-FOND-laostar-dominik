/**
 * This LAO* search engine is based on the variant
 * proposed in Robert Mattmueller's phD thesis 
 * URL: https://www.freidok.uni-freiburg.de/dnb/download/9238
 * 
 */
#ifndef LAOSTAR_H
#define LAOSTAR_H

#include <vector>
#include <stdexcept> 
#include <map>
#include <unordered_map>
#include <cmath>
#include <math.h>
#include <set>
#include <queue>
#include <memory>
#include "connector.h"
#include "../search_engine.h"
#include "util.h"
#include "../successor_generator.h"
#include "../globals.h"
#include "../countdown_timer.h"
#include "../plugin.h"
#include "../globals.h"
#include "laostar_utils.h"
#include "graphviz_writer.h"
#include "../symmetries/graph_creator.h"
#include "preferenceRule.h"

using std::vector;
using std::map;
using std::cout;
using std::endl;
using std::queue;
using std::ceil;
using std::numeric_limits;
using std::pair;
using std::abs;
using std::set;
using std::priority_queue;
using std::pair;
using std::out_of_range;
using std::unordered_map;
using std::map;
using std::find;
using std::weak_ptr;

#ifndef HEAP
#define HEAP
typedef priority_queue<shared_ptr<SearchNode>, preferNodesWithLowerCostEstimates> MAX_HEAP_COST;
typedef priority_queue<shared_ptr<SearchNode>, preferNodesWithHigherIndex> MAX_HEAP_INDEX;
#endif

class LaostarSearch : public SearchEngine {
public:	

   bool conservative_otf;
   int MAX_SEARCH_DEPTH = 0;   
   int NUM_PROVEN_NODES = 0; 
   int NUM_DISPROVEN_NODES = 0;
   int nodes_in_same_layer = 0;	
   int expansions_in_same_layer = 0;	
   int current_search_layer_nodes = -1; 
   int current_search_layer_expansions = -1;	   
   virtual ~LaostarSearch();

  /**
   * This flag makes the algorithm output information 
   * about progress of the search.
   */
   const bool PRINT_STATUS = false;
   double oldTime;  

  /**
   * Debug flag to make visible what the algortihm is currently
   * doing. 
   */
   bool DEBUG = true;
   
  /**
   * Helper object that contains a couple of subroutines of LAO*
   */
  LaostarUtils laostarUtils;
  
  /**
   * # of node expansion
   */
  int NODE_EXPANSIONS;

  /**
   * #number of connectors
   */
  int NUM_OF_CONNECTORS;

  /**
   * #num of generated nodes
   */
  int NUM_OF_NODES;

  /**
   *
   */
  int alternatingIndex;

  /**
   * rate of nodes to be expanded in one iteration of LAO*
   */
  double rateOfNodesToExpandInOneStep;

  /**
   * number of nodes to be expanded in one iteration of LAO*
   */
  int maxNumberOfNodesToExpandInOneStep;

  /**
   * Heuristic estimator 
   */ 
  Heuristic *heuristic;

  /**
   *
   */ 
  EvaluationContext current_eval_context;

  /**
   * option to dump the solution we found
   */ 
  bool dumpPlan;

  /**
   * Queues for LAO* search
   */
  MAX_HEAP_COST openList;
  MAX_HEAP_INDEX queuePreferHighestIndex;	

  /**
   * in openList 
   */
  unordered_set<int> inOpenList;

  /**
   * in index queue
   */
  unordered_set<int> inIndexQueue;

  /**
   * Mapping from unique hash value of world states to nodes in AND/OR graph representing them.
   */
//  vector<shared_ptr<SearchNode>> stateVector;
  unordered_map<StateID, shared_ptr<SearchNode>> all_nodes_map;

  vector<shared_ptr<SearchNode>> goalNodes;
  unordered_set<int> inGoalNodes;
  LaostarSearch(const Options &opts);

  /**
   * Add options to option parser
   */
  static void add_options_to_parser(OptionParser &parser);

  /** 
   * Method for run() of the myND planner 
   *    
   */
//   void search();
   
  /**
   * This is a method which should be called in the search method 
   *
   */
   virtual void initialize() override;
   virtual SearchStatus step() override;
   virtual void print_statistics() const override;

private:
   shared_ptr<SearchNode> initialNode;
   int iteration_counter;

   /**
    * Perform one iteration of the LAO* algorithm.
    * Corresponds to doIteration() method in myND planners' implementation
    *
    */
//   SearchStatus step(std::vector<shared_ptr<SearchNode>>& nodes_to_expand);

  /**
   * Expands a given node by creating all outgoing connectors and the corresponding successor
   * states. New states and connectors are installed in the explicit game graph representation.
   *
   * node: Node to expand
   */
   void expand(shared_ptr<SearchNode>& node);
  

  /**
   * Find nodes to expand.
   *
   * returns a set of nodes which will be expanded in the next step.
   */
   void compute_nodes_to_expand(std::vector<shared_ptr<SearchNode>>& nodes_to_expand);
   
  /**
   * Update nodes by propagating cost estimates and proof/disproof information backward through the
   * graph.
   *
   * nodes: Nodes to start the update with
   */
   void updateUntilFixpoint(vector<shared_ptr<SearchNode>>& nodes);
   
  /**
   *
   */ 
   void computeWeakDiscretePlanSteps(vector<shared_ptr<SearchNode>>& nodes);

  /**
   *
   */
   void unsafeAndProvenLabelling();
   
  /**
   *
   */
   int deleteUnprovenConnectors(vector<shared_ptr<Connector>>& backwardReachableConnectors);
      
  /**
   * Test whether a node for a given state has already been allocated or not. If there is already
   * such a node, return it, otherwise create a new node for the given state and associate it with
   * the state in the state-node mapping setOfAllNodes. Return the new node.
   *
   * state State to be represented by a node in the game graph
   * returns The unique node corresponding to the given state, either newly created or old.
   */ 
   shared_ptr<SearchNode> lookupAndInsert(GlobalState state, shared_ptr<SearchNode> parentNode, const GlobalOperator *parent_op);

	/**
	 * Dump the strong cyclic plan
	 * by BFS starting at initial node.
	 *
	 */
	void dump_plan(Plan& plan);

   /**
    * Dumping the policy for the orbit search case. The policy is defined by the canonical policy by setting
    * the operator to be the inverse image of the operator for the canonical representative, under the mapping
    * from the state to its canonical representative.
    *
    */
   void dump_plan_from_canonical(Plan& plan);
   void dumpStateSpace(std::string outfile);
   void printSearchStats(Timer& timer);
   void handle_node_expansion(int h_value);
   void handle_node_generations(int h_value);
   void remove_last_f_layer();
};
#endif
