#include <assert.h>
#include "laostar.h"

static SearchEngine *_parse(OptionParser &parser) {
    parser.document_synopsis(
        "LAO* search",
        "LAO* is a  heuristic search algorithm for nondeterministic planning that finds solutions with loops.");
    parser.add_option<Heuristic *>("eval", "evaluator for h-value");
    parser.add_option<bool>(
        "dump_deterministic_operators",
        "dumping operators with one determinization for creating successor generator", "false");
    parser.add_option<bool>(
        "debug",
        "dumping debug output", "false");
    parser.add_option<bool>(
        "useMaxChildEstimate",
        "use max child estimate", "false");

    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();
    LaostarSearch* engine = nullptr;
	
    if (!parser.dry_run()) {
        engine = new LaostarSearch(opts);
    }
    return engine;
}

LaostarSearch::LaostarSearch(const Options &opts): SearchEngine(opts),
		  DEBUG(opts.get<bool>("debug")),
		  laostarUtils(opts),
	      heuristic(opts.get<Heuristic *>("eval")),
	      current_eval_context(get_initial_state(), &statistics)
{
	// Setup parameters 
    rateOfNodesToExpandInOneStep = 1.0;
    maxNumberOfNodesToExpandInOneStep = 1;
	alternatingIndex = 0;
    NODE_EXPANSIONS = 0;
    NUM_OF_CONNECTORS = 0;
    NUM_OF_NODES = 0;
	dumpPlan = false;
	iteration_counter = 0;
	oldTime = -1.0;
}

void LaostarSearch::compute_nodes_to_expand(std::vector<shared_ptr<SearchNode>>& nodes_to_expand) {
	assert(nodes_to_expand.size() == 0);
	laostarUtils.traceMarkedConnectors(initialNode, nodes_to_expand);

	if (!nodes_to_expand.empty()) { // Case 1: There is an unexpanded node
//		if (DEBUG) {
//			cout << "LaostarSearch::nodesToExpand(): There is at least one unexpaneded node in current best solution graph." << endl;
//		}

	} else { // Case 2: Alternate between queues 
	      
//		if (DEBUG) {
//			cout << "\nLaostarSearch::nodesToExpand(): Case 2: Tracing was not successful! \n" << endl;
//		}
		// We alternate between the normal queue (best costEstimate)
		// and the queue that takes the other queue (highest index)
		int size = 0;
   
		if (rateOfNodesToExpandInOneStep == 1.0) {
			size = fmin(maxNumberOfNodesToExpandInOneStep, openList.size());
		} else {
			if (openList.size() > 0) {
				size = (int) (rateOfNodesToExpandInOneStep * openList.size()) + 1;
			}
		}

		for (int i = 0; i < size; i++) {
			shared_ptr<SearchNode> n;

			do {
				if (alternatingIndex == 0) {
					n = openList.top();
					openList.pop();
				} else {
					n = queuePreferHighestIndex.top();
					queuePreferHighestIndex.pop();
				}
			} while (n->is_closed() && !openList.empty() && !queuePreferHighestIndex.empty());

			nodes_to_expand.push_back(n);
			n->close();	
		}
		alternatingIndex = (alternatingIndex + 1) % 2;
//		cout << "No nodes to expand from tracing marked connectors. Got " << nodes_to_expand.size() << " nodes, asked for " << size << endl;
	}
}

void LaostarSearch::computeWeakDiscretePlanSteps(vector<shared_ptr<SearchNode>>& nodes) 
{
  if (DEBUG) {
	cout << "LaostarSearch::computeWeakDiscretePlanSteps" << flush<< endl;
  }	

  queue<shared_ptr<SearchNode>> nodesQueue;
  unordered_set<int> seen;
	
  // for all nodes consider children of outgoing connectors
  for (auto nodesPtr = nodes.begin();
      nodesPtr != nodes.end();
      ++nodesPtr)
  {
    shared_ptr<SearchNode>& node = *nodesPtr; 
    bool weakGoalDistanceReduced = false;

    for (auto connectorPtr = node->outgoingConnectors.begin();
         connectorPtr != node->outgoingConnectors.end();
         ++connectorPtr)
    {
        for (auto c = (*connectorPtr)->children.begin(); 
            c != (*connectorPtr)->children.end();
            ++c) 
        {
            shared_ptr<SearchNode> child = (*c).lock() ;  
			
			// if the weak goal distance (distance to the goal assuming deterministic setting)
			if (child->weakGoalDistance + 1.0 < node->weakGoalDistance) {
                node->weakGoalDistance = child->weakGoalDistance + (*connectorPtr)->baseCost;
				weakGoalDistanceReduced = true;
            }
        }

    }
    if(weakGoalDistanceReduced) {
        nodesQueue.push(node);
        seen.insert(node->index);
    }
  }
  //debug(2, _ARGS);

  // Update parents
  while(!nodesQueue.empty()) {
    shared_ptr<SearchNode> next = nodesQueue.front();
    nodesQueue.pop();
    seen.erase(next->index);

    for (auto connectorPtr = next->incomingConnectors.begin();
          connectorPtr != next->incomingConnectors.end();
          ++connectorPtr) 
     {
        shared_ptr<SearchNode> parent = (*connectorPtr)->parent;

        if (next->weakGoalDistance + 1.0 < parent->weakGoalDistance) {
            parent->weakGoalDistance = next->weakGoalDistance + 1;

            if (seen.find(parent->index) == seen.end()) {
                nodesQueue.push(parent);
                seen.insert(parent->index);
            }
        }
     }
  }
}

void LaostarSearch::updateUntilFixpoint(vector<shared_ptr<SearchNode>>& nodes) {
  if (DEBUG) {
     cout << "LaostarSearch::updateUntilFixpoint: Update until fixpoint." << endl << flush;
	 cout << "Nodes.size " << nodes.size() << endl << flush;
  }

  assert(!nodes.empty());  
  computeWeakDiscretePlanSteps(nodes);
  queue<shared_ptr<SearchNode>> nodes_queue;
  set<int>queueLogger;
  set<int> seen;
  vector<shared_ptr<SearchNode>> seenVector;
  
  for (auto nodesIt = nodes.begin();
       nodesIt != nodes.end(); 
      ++nodesIt) 
  {
     nodes_queue.push(*nodesIt);
     queueLogger.insert((*nodesIt)->index);
  }

   while (!nodes_queue.empty()) {
      shared_ptr<SearchNode> nodePtr = nodes_queue.front();
      nodes_queue.pop();
      queueLogger.erase(nodePtr->index);
      assert (!seen.find(nodePtr->index) != seen.end());
	  
	  if (seen.find(nodePtr->index) == seen.end()) {
		  seen.insert(nodePtr->index);
		  seenVector.push_back(nodePtr);
	  }
	  
      for (auto cPtr = nodePtr->incomingConnectors.begin();
		 cPtr != nodePtr->incomingConnectors.end();
	     ++cPtr) 
      {
		  shared_ptr<Connector> connector = *cPtr;	
		  shared_ptr<SearchNode> parent = connector->parent;

		  if (!parent->existsMarkedConnector() || ! ((*connector) == parent->getMarkedConnector())) {
              continue;	
		  }

		  int index = connector->parent->index;
		  bool seenContains = seen.find(index) != seen.end();
		  bool queueContains = queueLogger.find(index) != queueLogger.end();

		  if (!parent->isProven && 
			  !parent->isDisproven && 
			  !seenContains &&
			  !queueContains) 
		  {
			  nodes_queue.push(parent);
			  queueLogger.insert(parent->index);
		  }
      }
	}

	laostarUtils.performValueIteration(seenVector, initialNode);
}

 void LaostarSearch::unsafeAndProvenLabelling() {
    if (DEBUG) 
    {
      cout << "LaostarSearch::unsafeAndProvenLabelling: Unsafe and proven labelling." << flush << endl;
    }
	int num_safe = 0;
	
	// Mark all outgoing connectors of all nodes safe  	
	// Track on how many connectors are safe (initially all are assumed to be safe)
	// counts the # of connectors
    for (auto& nodePtr : all_nodes_map)
    {
		shared_ptr<SearchNode>& node = nodePtr.second;

        for (auto connectorPtr = node->outgoingConnectors.begin();
	     connectorPtr != node->outgoingConnectors.end();
	     ++connectorPtr)
		{
		  (*connectorPtr)->isSafe = true;
		   num_safe++;
		}
    }

    int old_num_safe;

	// Iterate as long as the number of safe connectors stays the same (fixed point algorithm)
	// Intuition: Initially we assume all connectors to be safe, then we remove all connectors
	//			  that are not backward reachable and start the repeat it until no changes occur.			  
	unordered_map<int, int> layers;
    do {
        old_num_safe = num_safe;
		vector<shared_ptr<Connector>> backwardReachable;
		unordered_map<int,int> distanceMap;
		laostarUtils.computeBackwardReachableConnectors(all_nodes_map, distanceMap, goalNodes, backwardReachable);
		num_safe = deleteUnprovenConnectors(backwardReachable);
		layers.swap(distanceMap);	
    }
    while (num_safe != old_num_safe);
	
	// for all nodes consider all outgoing connectors
	// if the connector is 
    for (auto& nodePtr : all_nodes_map)
    {
		  shared_ptr<SearchNode> node = nodePtr.second;
		  int bestConnector = -1;
		  int bestDistance = -1;
		  int index = 0;
		  
		  for (auto conPtr = node->outgoingConnectors.begin();
			   conPtr != node->outgoingConnectors.end();
				++conPtr)
		  {
		   
			if ((*conPtr)->isSafe) {
				  if (!node->isProven) {
						node->isProven = true;
						NUM_PROVEN_NODES++;
				  }
				  
				  if (bestConnector == -1 || bestDistance > layers.at((*conPtr)->id)) {
						bestConnector = index;
						bestDistance = layers.at((*conPtr)->id);
				  }
			}

			++index;
		 }

      if (node->isProven) {
		node->setMarkedConnector(bestConnector);
      }
    }
}

void LaostarSearch::initialize() {
	std::cout << "Algorithm: LAO*-search" << flush << std::endl;


    GlobalState initialState = get_initial_state();
    initialNode = lookupAndInsert(initialState, NULL, NULL);

	initialNode->depth = 0;
	MAX_SEARCH_DEPTH = 0;
	cout << "Initial heuristic value is " << initialNode->heuristic << flush<< endl;
}

SearchStatus LaostarSearch::step() {
	if (DEBUG) {
		cout << "LaostarSearch::search(): Performing iteration " << iteration_counter++ << " of LAO* algorithm."<< endl;
		cout << "Number of state action table " << all_nodes_map.size() << " elements" << endl;
		dumpStateSpace("state_space"+std::to_string(iteration_counter)+".dot");
	}
	if (openList.empty() || queuePreferHighestIndex.empty()) {
		initialNode->isDisproven = true;
	}

	if (initialNode->isProven){
		if (DEBUG) {
		}


		Plan plan;
		if (dumpPlan) {
			dump_plan(plan);
		}
		set_plan(plan);
		std::cout << "\nExpanded " << NODE_EXPANSIONS << " state(s)." << flush <<endl;
		std::cout << "Generated " << NUM_OF_NODES << " state(s)." << endl;
		return SOLVED;
	}
	if (initialNode->isDisproven) {
		if (DEBUG) {
			dumpStateSpace("state_space.dot");
		}
		std::cout << "\nExpanded " << NODE_EXPANSIONS << " state(s)."<< flush << endl;
		std::cout << "Generated " << NUM_OF_NODES << " state(s)." << flush << endl;
		return SOLVED;
		return FAILED;
	}

    std::vector<shared_ptr<SearchNode>> nodes_to_expand;
    compute_nodes_to_expand(nodes_to_expand);
	if (openList.empty() || queuePreferHighestIndex.empty()) {
		return IN_PROGRESS;
	}

	assert(nodes_to_expand.size() > 0);
    for (auto nodePtr = nodes_to_expand.begin();
    		nodePtr != nodes_to_expand.end();
    		++nodePtr) 
	{
    	expand(*nodePtr);
    }
    updateUntilFixpoint(nodes_to_expand);
	if (!initialNode->isProven && !initialNode->isDisproven) {
		unsafeAndProvenLabelling();
	}
	if (PRINT_STATUS) {				
		printSearchStats(g_timer);
	}

    return IN_PROGRESS;
}

void LaostarSearch::dump_plan(Plan& plan) {
	if(do_orbit_search) {
		dump_plan_from_canonical(plan);
		return;
	}

    std::queue<shared_ptr<SearchNode>> queue;
    queue.push(initialNode);
	unordered_set<int> visited;
    string operatorString = "";
	int numOfOperators = 0;
	int numPolicyEntries = 0;
	int numOfPredicates = 0;
	std::string stateString = " ";
	std::string markedConString = " ";

    while (!queue.empty()) {
		shared_ptr<SearchNode> node = queue.front();
		queue.pop();

        if (visited.find(node->index) != visited.end() || node->isGoalNode)
        	continue;
		visited.insert(node->index);
		numPolicyEntries++;

		const GlobalOperator* current_op = node->getMarkedConnector().nondeterministic_operator;
		markedConString += std::to_string(node->index)  +" "+  std::to_string(node->markedConnectorIndex) + " ";
		operatorString += " (" + current_op->get_name() + ")";
		plan.push_back(current_op);
		int n = 0;
		stateString +=  node->get_state().toString(n);
		numOfPredicates += n;
		numOfOperators++;

		// Adding the successors to the queue
		vector<weak_ptr<SearchNode>>& successors = node->getMarkedConnector().children;
		for (weak_ptr<SearchNode> n : successors) {
		    queue.push(n.lock());
		}
    }

	std::cout << numOfPredicates << " " <<  stateString << flush << std::endl;
	std::cout << "Policy entries: " << numPolicyEntries << flush << std::endl;
	std::cout << "%%" << flush << std::endl;
	std::cout <<  numOfOperators << " " << operatorString << flush << std::endl;
	std::cout << "%%" << flush << std::endl;
	std::cout << "policy " << markedConString << flush << std::endl;
}


void LaostarSearch::dump_plan_from_canonical(Plan& plan) {
	// Doing a BFS to pass all reachable by policy states, getting the policy from canonical
    PerStateInformation<bool> visited;
    PerStateInformation<const GlobalOperator*> policy;
    PerStateInformation<Permutation> symmetry;

    StateRegistry tmp_registry;
    // Add state to the registry
    GlobalState init_state = tmp_registry.add_state(g_initial_state());
    const GlobalState& canonical_init = initialNode->get_state();
    const Permutation& sigma = g_symmetry_graph->create_permutation_from_state_to_state(init_state, canonical_init);
    symmetry[init_state] = sigma;
	if (DEBUG) {
		cout << "Initial state:" << endl;
		init_state.dump_pddl();
	}
    const GlobalOperator* can_init_op = initialNode->getMarkedConnector().nondeterministic_operator;
    if (DEBUG) {
    	cout << "Operator from canonical policy: " << can_init_op->get_name() << endl;
    }

    // Passing inverse of the sigma
	const GlobalOperator& init_op = g_symmetry_graph->get_operator_permutation(can_init_op, Permutation(sigma, true));
    if (DEBUG) {
    	cout << "Reconstructed operator: " << init_op.get_name() << endl;
    }
	policy[init_state] = &init_op;

    // Add state to the queue
    std::queue<StateID> queue;
    queue.push(init_state.get_id());

    size_t all_nodes_map_size = all_nodes_map.size();
	if (DEBUG) {
		cout << "=====================================================================" << endl;
		cout << "Number of nodes so far is " << all_nodes_map_size << endl;
	}
    string operatorString = "";
	int numOfOperators = 0;
	int numPolicyEntries = 0;

	operatorString += " (" +init_op.get_name() + ")";
	plan.push_back(&init_op);
	numOfOperators++;

    while (!queue.empty()) {
        // Get state from the queue
		StateID state_id = queue.front();
		queue.pop();
		const GlobalState& s = tmp_registry.lookup_state(state_id);

        visited[s] = true;
		numPolicyEntries++;

		// Adding the successors to the queue
		vector<GlobalState> successors;
		tmp_registry.get_successor_states(s, *policy[s], successors);
		for (size_t i=0; i < successors.size(); i++) {
			const GlobalState& t = successors[i];
	        if (visited[t] || test_goal(t)) {
	        	continue;
	        }

	        const GlobalState& t_prime = tmp_registry.get_state_permutation(t, symmetry[s]);
		    const GlobalState& canonical_t_prime = g_state_registry->get_canonical_representative(t_prime);

		    const Permutation& sigma_t_prime = g_symmetry_graph->create_permutation_from_state_to_state(t_prime, canonical_t_prime);

	        const Permutation& rho_t = Permutation(symmetry[s], sigma_t_prime);
//	        const Permutation& rho_t = Permutation(sigma_t_prime, symmetry[s]);
	        symmetry[t] = rho_t;

			shared_ptr<SearchNode> node = lookupAndInsert(canonical_t_prime, NULL, NULL);

			if (DEBUG) {
				std::cout << "Canonical node index " << node->index << ", actual state:" << std::endl;
				canonical_t_prime.dump_pddl();
				if (node->is_new() || all_nodes_map.size() > all_nodes_map_size) {
					std::cout << "=============================== The canonical representative is a new node!!! Should not happen!" << std::endl;
				}
			}

			// if there is no marked connector the traced strong cyclic plan is not proper!
			// this case should never happen when LAO* is working properly.
			if (!node->existsMarkedConnector()) {
				if(DEBUG) {
	    			std::cout << "LaostarSearch::dump_plan_from_canonical: No marked connector" << std::endl;
	            	cout << "=============================== Node has no marked outgoing connector" << endl;
	            	laostarUtils.printSearchNode(node);
				}
				exit_with(EXIT_CRITICAL_ERROR);
			}

			Connector& c = node->getMarkedConnector();
			const GlobalOperator* op = c.nondeterministic_operator;
			const GlobalOperator& current_op = g_symmetry_graph->get_operator_permutation(op, Permutation(rho_t, true));

			operatorString += " (" +current_op.get_name() + ")";
			plan.push_back(&current_op);
			if(DEBUG) {
				cout << "======> Action: " << current_op.get_name() << endl;
			}
			numOfOperators++;

			policy[t] = &current_op;
		    queue.push(t.get_id());
		}
    }
	std::cout << "Policy entries: " << numPolicyEntries << flush << std::endl;
	std::cout << "%%" << flush << std::endl;
	std::cout <<  numOfOperators << " " << operatorString << flush << std::endl;
	std::cout << "%%" << flush << std::endl;
}


int LaostarSearch::deleteUnprovenConnectors(vector<shared_ptr<Connector>>& backwardReachableConnectors) 
{
	if(DEBUG) {
		std::cout << "LaostarSearch::deleteUnprovenConnectors()" << std::endl;
	}

    queue<shared_ptr<Connector>> deleteQueue;
    unordered_map<int,int> counter;
    vector<shared_ptr<SearchNode>> candidateParents;
	unordered_set<int> inCandidateParents;
	
	// Candidate parents are initially all search nodes 
	// that are backward reachable from the goal  
	for (auto conIt = backwardReachableConnectors.begin();
		 conIt != backwardReachableConnectors.end();
		 ++conIt) 
	{
		  shared_ptr<SearchNode> node =  (*conIt)->parent;

		  if (inCandidateParents.find(node->index) == inCandidateParents.end()) {
				candidateParents.push_back(node);
				inCandidateParents.insert(node->index);
		  }	 
    }
		
	// all candidate parents are mapped 0
    for (auto parentPtr = candidateParents.begin();
         parentPtr != candidateParents.end();
         ++parentPtr)
    {
		shared_ptr<SearchNode> parentNode = (*parentPtr);
        counter[parentNode->index] = 0;
    }
	
	// map the backward reachable connectors to 1 + their parent node?  
    // Compute the number of times each of the nodes is a parent
    for (auto cPtr = backwardReachableConnectors.begin();
         cPtr != backwardReachableConnectors.end();
         ++cPtr)
    {
		int index = (*cPtr)->parent->index;
        counter[index] = counter.at(index) + 1;
    }

	// for every backward reachable connector
	// consider its children search nodes  
    for (auto cPtr = backwardReachableConnectors.begin();
         cPtr != backwardReachableConnectors.end();
         ++cPtr)
    {
        for(auto childPtr = (*cPtr)->children.begin();
            childPtr != (*cPtr)->children.end();
            ++childPtr)
        {
			  shared_ptr<SearchNode> node = (*childPtr).lock();
			  // if the node we look at is no goal node and is not contained in candidate parents
			  // let it be unsafe	 
//	          bool goalContains = inGoalNodes.find(node->index) != inGoalNodes.end();
	          bool candidateContains = inCandidateParents.find(node->index) != inCandidateParents.end(); 

			  if (!candidateContains && !node->isGoalNode) {
				deleteQueue.push(*cPtr);

				if (DEBUG) {
						std::cout << "LaostarSearch::deleteUnprovenConnectors:deleted connector with id" << (*cPtr)->id << std::endl;	
				}
				break;
			  } 
        }
    }
  			
	// As long as the delete queue is nonempty 
	// pop a connector from the delete queue, set it unsafe
	// if the connector is not yet contained in the delete queue 
	// add it 
	// delete queue: every connector on that queue will be made unsafe
	int num_nonSafe = 0;
    while(!deleteQueue.empty()) { 
        shared_ptr<Connector>& c = deleteQueue.front();
        deleteQueue.pop();
		num_nonSafe++;
        c->isSafe = false;
        shared_ptr<SearchNode> parent = c->parent;
	    pair<int, int> p(parent->index, counter.at(parent->index) - 1);
		counter.insert(p);
        if (counter.at(parent->index) == 0) {
              for (auto dPtr = parent->incomingConnectors.begin();
                   dPtr != parent->incomingConnectors.end();
                   ++dPtr)
              {
                deleteQueue.push(*dPtr);
				if (DEBUG) {
					std::cout << "LaostarSearch::deleteUnprovenConnectors:deleted connector with id" << (*dPtr)->id << std::endl;	
                }
              }
        }
    }

	int num_safe = backwardReachableConnectors.size() - num_nonSafe;
    return num_safe;
}

void LaostarSearch::expand(shared_ptr<SearchNode>& node) {
	if (node->isExpanded)
		return;
	handle_node_expansion(node->heuristic);

	if (DEBUG) {
		cout << "LaostarSearch::expand: Node " << node->index << " is expanded now" << endl;
		if (alternatingIndex == 0) {
			cout << "\nLaostarSearch::expand: Node " << node->index << " with h/f-value "
					<< node->heuristic << "/"<< node->costEstimate << " from queue BestCostEstimate is expanded now" << endl;
		} else {
			cout << "\nLaostarSearch::expand: Node " << node->index << " with h/f-value "
					<< node->heuristic << "/" << node->costEstimate << " from queue HighestIndex is expanded now" << endl;
		}
	}

	assert(!node->isExpanded);
	node->isExpanded = true;
	vector<const GlobalOperator *> applicable_ops;
	shared_ptr<SearchNode> weakParent(node);
	bool hasSuccessor; 

	if(!(node->isProven || node->isDisproven)) {
		g_successor_generator->generate_applicable_ops(node->get_state(), applicable_ops);
		hasSuccessor = false; // which is different from node (no self-loop)	
	}
	else { 
		cout << "LaostarSearch::expand(): Node to expand is already decided." << endl;
		exit_with(EXIT_CRITICAL_ERROR);
	}	

	if (DEBUG) {
		cout << "LaostarSearch::expand: applicable operators are generated" << endl;
	}

	for (auto opPtr = applicable_ops.begin();
		   opPtr != applicable_ops.end();
		   ++opPtr) 
	{
		vector<weak_ptr<SearchNode>> children;
		const GlobalOperator* op = *opPtr;

        vector<GlobalState> successorStates;
        if (do_orbit_search) {
        	g_state_registry->get_canonical_successor_states(node->get_state(), *op, successorStates);
        } else {
        	g_state_registry->get_successor_states(node->get_state(), *op, successorStates);
        }
        statistics.inc_generated(successorStates.size());
    	if (DEBUG) {
    		cout << "LaostarSearch::expand: successor states are created for the operator " << op->get_name() << endl;
    	}

        for (auto statePtr = successorStates.begin(); statePtr != successorStates.end(); ++statePtr) {
	    	if (node->get_state().get_id() == statePtr->get_id())
	    		continue;
	    	shared_ptr<SearchNode> newNode = lookupAndInsert(*statePtr, node, op);
			newNode->depth = node->depth + 1;
			if (MAX_SEARCH_DEPTH < newNode->depth) MAX_SEARCH_DEPTH = newNode->depth;			

	    	if (search_progress.check_progress(current_eval_context)) {
	    		statistics.print_basic_statistics();
	    	}
	    	shared_ptr<SearchNode> weakNode(newNode);
	    	if(!(*newNode == *node)) {
	    		hasSuccessor = true;
	    	}
	    	children.push_back(weakNode);
		}

		shared_ptr<Connector> c(new Connector(weakParent, NUM_OF_CONNECTORS, children, op));
		++NUM_OF_CONNECTORS;
		
		// Outgoing connectors of the current node are all "applicable operators" from that node
		node->outgoingConnectors.push_back(c);
	
		// Current connector should be the incoming connector  
		for (auto childPtr = children.begin();
			 childPtr != children.end();
		     ++childPtr)
		{
			shared_ptr<SearchNode> child = (*childPtr).lock();
			(*child).incomingConnectors.push_back(c);
		}
	}

	if (!hasSuccessor) {
		node->isDisproven = true;
		NUM_DISPROVEN_NODES++;
		if (DEBUG) {
			std::cout << node->index << " is deadend" << std::endl;
		}
	}

	NODE_EXPANSIONS++;
	statistics.inc_expanded();
	if (DEBUG) {
		cout << "LaostarSearch::expand finished" << endl;
	}
}

shared_ptr<SearchNode> LaostarSearch::lookupAndInsert(GlobalState state, shared_ptr<SearchNode> parentNode, const GlobalOperator *parent_op) {
	if (DEBUG) {
		cout << "LaostarSearch::lookupAndInsert starting for state "  << endl<< state.toString() << endl << "with a hash value " << state.get_id().hash() << endl;
	}
	unordered_map<StateID, shared_ptr<SearchNode>>::iterator node_it = all_nodes_map.find(state.get_id());
	if (DEBUG) {
		cout << "LaostarSearch::lookupAndInsert the node is " << (node_it != all_nodes_map.end() ? "" : "not ") << "known" << endl;
	}

	if (node_it == all_nodes_map.end()) {
		shared_ptr<SearchNode> node = search_space.get_node_shared_ptr(state);
		node->index = NUM_OF_NODES;
		all_nodes_map[state.get_id()] = node;
		
		// Heuristic evaluation
		current_eval_context = EvaluationContext(state, &statistics);
		node->heuristic = (double) current_eval_context.get_heuristic_value_or_infinity(heuristic);
		node->isDisproven = heuristic->dead_ends_are_reliable() && current_eval_context.is_heuristic_infinite(heuristic);
		node->isDisproven ? NUM_DISPROVEN_NODES++ : NUM_DISPROVEN_NODES;
		statistics.inc_evaluated_states();
		node->costEstimate = node->heuristic;

		if (!parentNode && !parent_op) {
			node->open_initial(node->heuristic);
		}	
		else {
			node->open(node->heuristic, *parentNode, parent_op);
		}
		if (DEBUG) {
			cout << "LaostarSearch::lookupAndInsert: evaluating heuristic for the new node "
					<< node->index << " - got the value " << node->heuristic << endl;
		}
		++NUM_OF_NODES;
	
		if (node->isProven || node->isDisproven) {
			node->isExpanded = true;
			node->close();
		
			// if node is initial node	
			if (node->index == 0) return node;  
		} else {
			openList.push(node);
			queuePreferHighestIndex.push(node);
		}

		if (node->isGoalNode) {
			goalNodes.push_back(node);
			inGoalNodes.insert(node->index);
		}

		if (DEBUG) {
			cout << "LaostarSearch::lookupAndInsert: New node: " << node->index << endl;
		}

		handle_node_generations(node->heuristic);

		return node;
	} else {
		if (DEBUG) {
			cout << "LaostarSearch::lookupAndInsert: old node at index " << state.get_id().hash()
					<< " out of " << all_nodes_map.size() << " entries" << endl;
		}
		assert(state.get_id() == node_it->second->get_state().get_id());
		return node_it->second;
	}
}

void LaostarSearch::dumpStateSpace(std::string outfile)
{
	GraphvizWriter gw;
	gw.createOutputStateSpace(true, initialNode, outfile);
}

void LaostarSearch::printSearchStats(Timer& timer) {
	if ((timer.current_clock() - oldTime) <= 2.0) return; 
	oldTime = timer.current_clock();
	std::cout << "-------------------------------------------------" << flush <<std::endl;	
	std::cout << "-              Search Statistics                -" << flush << std::endl;
	std::cout << "elapsed time: " << timer                           << flush << std::endl;	
	std::cout << "nodes generated: " << NUM_OF_NODES << "\t nodes expanded: " << NODE_EXPANSIONS << flush << std::endl;
	std::cout << "nodes proven: " <<  NUM_PROVEN_NODES << " nodes disproven: "<< NUM_DISPROVEN_NODES << flush << std::endl;
	std::cout << ""													 << flush << std::endl;
	std::cout << "max search depth: " << MAX_SEARCH_DEPTH << flush   << flush << std::endl;
	std::cout << "-------------------------------------------------" << flush << std::endl;	
}

void LaostarSearch::print_statistics() const {
    search_progress.print_initial_h_values();
    statistics.print_detailed_statistics();
    search_space.print_statistics();
}


void LaostarSearch::handle_node_generations(int h_value)
{
	if (h_value != current_search_layer_nodes) {
		nodes_in_same_layer = 1; 
		current_search_layer_nodes = h_value;
	}		
	else {
		nodes_in_same_layer++;	
	}
}

void LaostarSearch::handle_node_expansion(int h_value)
{
	if (h_value != current_search_layer_expansions) {
		expansions_in_same_layer = 1; 
		current_search_layer_nodes = h_value;
	}		
	else {
		expansions_in_same_layer++;	
	}
}
void LaostarSearch::remove_last_f_layer()
{
	NUM_OF_NODES = NUM_OF_NODES - expansions_in_same_layer;	
	NODE_EXPANSIONS = NODE_EXPANSIONS - expansions_in_same_layer; 
}

LaostarSearch::~LaostarSearch() {
}
static Plugin<SearchEngine> _plugin("laostar", _parse);
