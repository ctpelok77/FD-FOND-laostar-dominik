#include "laostar_utils.h"

using std::cout;
using std::endl;
using std::flush;
using std::string;
using std::map;

LaostarUtils::LaostarUtils(const Options &opts) :
								DEBUG(opts.get<bool>("debug")),
								useMaxChildEstimate(opts.get<bool>("useMaxChildEstimate"))
{

}

void LaostarUtils::performValueIteration(vector<shared_ptr<SearchNode>>& allNodes, shared_ptr<SearchNode>& initialNode)
{
  if (DEBUG) {
    cout << "LaostarUtils::performValueIteration: performValueIteration on" << endl;
  }

  // Auxiliary data structures.
  unordered_map<int, double> oldCostEstimate;
  vector<shared_ptr<SearchNode>> updatedNodes;  
 
  // Initialization.
  for (auto allNodesPtr = allNodes.begin();
       allNodesPtr != allNodes.end();
       ++allNodesPtr)
  {
      shared_ptr<SearchNode>& node = *allNodesPtr;

      if (node->isProven) {
        node->costEstimate = 0.0;
      }
      else if(node->isDisproven) {
        node->costEstimate = numeric_limits<double>::infinity();
      }
      else {
        node->costEstimate = node->heuristic;
      }
	
	  updatedNodes.push_back(node);
  }
  
  // main loop
  bool converged = false;
  do {

    // update improved
    for (auto it = updatedNodes.begin(); it != updatedNodes.end(); ++it) {
		shared_ptr<SearchNode>& node = *it;	
		oldCostEstimate[node->index] = node->costEstimate;
    }
	
	vector<shared_ptr<SearchNode>> newNodes;  
	newNodes.swap(updatedNodes);

    for (auto allNodesPtr = allNodes.begin();
        allNodesPtr != allNodes.end();
        ++allNodesPtr) 
    {
      shared_ptr<SearchNode> node = *allNodesPtr;
	
	  // We do not change marked connectors of proven nodes.
      if (!node->isProven) { 
        node->setMarkedConnector(-1);
        
        int index = 0;
        for (auto outPtr = node->outgoingConnectors.begin();
            outPtr != node->outgoingConnectors.end();
            ++outPtr) 
        {
              // A connector is promising only if it has at least one
              // child such that the shortest path from that child to a
              // goal is strictly shorter than the shortest path from
              // the parent of the connector to a goal.
              // Assert that at least one outgoing connector of
              // should be marked as losing. 
              // each expanded node is promising. otherwise the node
              if (connectorIsPromising(*(*outPtr))) {
                markBestConnector(index, node, oldCostEstimate, updatedNodes);
              }

			  ++index;
        }

		if (!node->existsMarkedConnector() && !node->outgoingConnectors.empty()) {
		   int index = 0; 
		   for (auto cPtr = node->outgoingConnectors.begin();
				 cPtr != node->outgoingConnectors.end();
				 ++cPtr)
			 {
				  markBestConnector(index, node, oldCostEstimate, updatedNodes);
				  ++index;
			 }
		 }
         if (!node->existsMarkedConnector()) 
         {
            // No outgoing connector or only connectors where at least one child is a dead end.
            node->isDisproven = true;
			updatedNodes.push_back(node);	
            node->costEstimate = INFINITY;

            if(node == initialNode)
            {
              break;
            }
          }
      }
    }

    // convergence test
    converged = true;
    for (auto allNodesPtr = allNodes.begin();
         allNodesPtr != allNodes.end();
         ++allNodesPtr) 
    {
      shared_ptr<SearchNode> node = *allNodesPtr;
      double error = (node->costEstimate - oldCostEstimate.at(node->index));

      if (abs (error) > EPSILON)
      {
        converged = false;
        break;  
      }
    }
  }
  while(!converged);
}

bool LaostarUtils::connectorIsPromising(Connector &connector) 
{
  vector<weak_ptr<SearchNode>>& connectorChildren = connector.children;
  for (auto childPtr = connectorChildren.begin(); 
       childPtr != connectorChildren.end();
       ++childPtr) 
  {
    shared_ptr<SearchNode> child = (*childPtr).lock();
    if (child->weakGoalDistance < connector.parent->weakGoalDistance) {
      return true;
    }
  }
  return false; 
}


void LaostarUtils::markBestConnector(int candidateBestConnectorIndex, 
                                      shared_ptr<SearchNode> node,
                                      unordered_map<int, double>& oldCostEstimate,
									  vector<shared_ptr<SearchNode>>& updatedNodes) 
{
    shared_ptr<Connector> candidateBestConnector = node->outgoingConnectors.at(candidateBestConnectorIndex);
    double avgChildEstimate = 0.0;
    double maxChildEstimate = 0.0;
    vector<weak_ptr<SearchNode>>& connectorChildren = candidateBestConnector->children;
    assert (connectorChildren.size() != 0);
    
    for (auto childPtr = connectorChildren.begin(); 
         childPtr != connectorChildren.end();
         ++childPtr) 
    {
        shared_ptr<SearchNode> child = (*childPtr).lock();
        
        // Do not mark a connector which leads to a dead end.
        if (child->isDisproven) {
          return;
        }
        
        double childEstimate = child->costEstimate;
        
        if (oldCostEstimate.find(child->index) != oldCostEstimate.end()) {
           childEstimate = oldCostEstimate.at(child->index);    
        }
        
        if (childEstimate > maxChildEstimate) {
          maxChildEstimate = childEstimate;
        }
        
        avgChildEstimate += childEstimate; 
    }
    avgChildEstimate /= candidateBestConnector->children.size();
  
    // Experiments show that it seems to be preferable to average about
    // child nodes.
    double childEstimate = useMaxChildEstimate ? maxChildEstimate : avgChildEstimate;

    if (!node->existsMarkedConnector()
    		|| candidateBestConnector->baseCost + DISCOUNT_FACTOR * childEstimate < node->costEstimate) {
    	node->costEstimate = candidateBestConnector->baseCost + DISCOUNT_FACTOR * childEstimate;
        node->setMarkedConnector(candidateBestConnectorIndex);
		updatedNodes.push_back(node);
    }
}

void LaostarUtils::computeBackwardReachableConnectors(unordered_map<StateID, shared_ptr<SearchNode>>& all_nodes_map,
												 unordered_map<int, int>&  distanceMap,
												 vector<shared_ptr<SearchNode>>&  goalNodes, vector<shared_ptr<Connector>>& backwardReachableConnectors)
{
	if (DEBUG) {
		cout << "LaostarUtils::computeBackwardReachableConnectors(): Compute backward reachable connectors" << endl;
	}

    vector<shared_ptr<Connector>> connectorsAtCurrentDistance;
	unordered_set<int> inAtCurrentDistance;
	unordered_set<int> inBackwardReachable;
    int currentDistance = 1;

	// Collect all goal nodes in goalNodes
	// for all incoming connectors of the goal nodes, save the incoming connector
	// in the distanceMap (mapping Connector.id -> distance from goal)
	// In the distance is fixed to 1 because we start with currentDistance = 1.
	for (auto it = goalNodes.begin(); it != goalNodes.end(); ++it) 
	{
		shared_ptr<SearchNode>& node = *it;
		
		if(DEBUG) {
			cout << "LaostarUtils::computeBackwardReachableConnectors(): Goal node " << node->index <<" detected" << flush << endl;
		}

		for (auto connectorPtr = node->incomingConnectors.begin();
            connectorPtr != node->incomingConnectors.end();
            ++connectorPtr)
        {
			shared_ptr<Connector> connector = *connectorPtr;
            if (!connector->parent->isGoalNode && connector->isSafe) 
            {
			  if(inAtCurrentDistance.find(connector->id) == inAtCurrentDistance.end()) {
					connectorsAtCurrentDistance.push_back(connector);
					inAtCurrentDistance.insert(connector->id);
			  }

			  if(inBackwardReachable.find(connector->id) == inBackwardReachable.end()) 
			  {
					backwardReachableConnectors.push_back(connector);
					inBackwardReachable.insert(connector->id);
			  } 

              pair<int, int> p (connector->id, currentDistance);
			  distanceMap.insert(p);
	
		   	  if (DEBUG){
					cout << "LaostarUtils::computeBackwardReachableConnectors(): Inserting p" << endl;
					cout << "LaostarUtils::computeBackwardReachableConnectors(): " << connector->id << " " << currentDistance << endl;
		   	  }
            }
        }
	 }
     
     // If we have a connector at currentDistance we go into this while loop 
     // so at start we need to have at least one goal node
     // We start looking at all connectors which have distance 2 to the goal and further away from the goal
//	 vector<shared_ptr<Connector>> newlyFoundConnectors;
	 while (!connectorsAtCurrentDistance.empty()) {
        currentDistance++;
		vector<shared_ptr<Connector>> newlyFoundConnectors;
        for (auto cPtr = connectorsAtCurrentDistance.begin();
            cPtr != connectorsAtCurrentDistance.end();
            ++cPtr)
        {
		   shared_ptr<Connector> connector = *cPtr;
           for (auto dPtr = connector->parent->incomingConnectors.begin();
                dPtr != connector->parent->incomingConnectors.end();
                ++dPtr) 
		   {
				shared_ptr<Connector> otherConnector = *dPtr;
				bool backwardReachableContains = inBackwardReachable.find(otherConnector->id) !=
												 inBackwardReachable.end(); 				

				// Check whether backwardReachable connectors already contain the connector
				// we currently look at
				// if not, insert it into backwardReachableConnectors as well as  
				// report it to be backwardReachable at current distance
				if (!backwardReachableContains
					&& !otherConnector->parent->isGoalNode 
					&& otherConnector->isSafe)
				{
					newlyFoundConnectors.push_back(otherConnector);
					backwardReachableConnectors.push_back(otherConnector);
					inBackwardReachable.insert(otherConnector->id);
					pair<int, int> p(otherConnector->id, currentDistance);
					distanceMap.insert(p);
				}
          }
        }
        connectorsAtCurrentDistance.swap(newlyFoundConnectors); 
     }
	
	 // For all nodes we have seen so far, consider the outgoing connectors
     for (auto& statePtr : all_nodes_map)
	 {
		shared_ptr<SearchNode>& node = statePtr.second;
		for (auto cPtr = node->outgoingConnectors.begin();
            cPtr != node->outgoingConnectors.end();
            cPtr++)
        {
		   shared_ptr<Connector> connector = *cPtr;
		   if(inBackwardReachable.find(connector->id) != inBackwardReachable.end())
		   {	          
				connector->isSafe = true;	
		   } 
		   else {
				connector->isSafe = false;
		   }
		}
   }
}

void LaostarUtils::traceMarkedConnectors(shared_ptr<SearchNode>& initial, vector<shared_ptr<SearchNode>>& result) 
{
	if (DEBUG) {
		cout << " " << endl;
		cout << "LaostarUtils::traceMarkedConnectors: Tracing..." << endl;
	}
    
    unordered_set<shared_ptr<SearchNode>> seen;
	unordered_set<int> containedInResult;
    queue<shared_ptr<SearchNode>> queue;
    unordered_set<shared_ptr<SearchNode>> set_for_queue;
    queue.push(initial);       
    set_for_queue.insert(initial);

    while (!queue.empty()) {
      shared_ptr<SearchNode> next = queue.front();
      queue.pop();
	  set_for_queue.erase(next);
      seen.insert(next);

      if (!next->isProven && !next->isDisproven) {
        if (!next->isExpanded) {
	
          if (DEBUG) {
             cout << "LaostarUtils::traceMarkedConnectors: node " << next->index << " is added to result... (not expanded)." << endl;
          }

		  if(containedInResult.find(next->index) == containedInResult.end()) {
				result.push_back(next);
				containedInResult.insert(next->index);
		  }	
        }
        else {
           if (next->existsMarkedConnector()) {
              if (DEBUG) {
                std::cout << "Node " << next->index << " has a marked connector. Add children of marked connector to queue." << flush << endl;
              }
              
              Connector con = next->getMarkedConnector();
              vector<weak_ptr<SearchNode>>& children = con.children;

              for (auto childPtr = children.begin(); 
                  childPtr != children.end();
                  ++childPtr) 
              {
                  bool seenContainsChild = find_if(seen.begin(), seen.end(), [=](const shared_ptr<SearchNode> s)->bool
										  {
											  if ((*(*childPtr).lock()) == *s) { 
												return true;
											  }
											  return false;
										   }) != seen.end();   
                  bool queueContainsChild = find_if(set_for_queue.begin(), set_for_queue.end(), [=](const shared_ptr<SearchNode> s)->bool
											{
											  if ((*(*childPtr).lock()) == *s) { 
												return true;
											  }
											  return false;
										   }) != set_for_queue.end(); 
                  if (!seenContainsChild && !queueContainsChild) {
						queue.push((*childPtr).lock());	
						set_for_queue.insert((*childPtr).lock());
                  }
              }
          } 
          else {
            cout << "LaostarUtils::traceMarkedConnectors: Strange case where no connector is marked while tracing..." << endl;
            exit_with(EXIT_CRITICAL_ERROR);
          }          
        }
      }
   }
}

void LaostarUtils::printSearchNode(shared_ptr<SearchNode> searchNode) {
  std::cout << "PrintSearchNode" << std::endl;
  cout << "########## Search Node with index " << searchNode->index << "##########" << flush << endl;
  cout << "heuristic = " << searchNode->heuristic << flush << endl;
  cout << "costEstimate = " << searchNode->costEstimate << flush << endl;
  cout << "weakGoalDistance  = " << searchNode->weakGoalDistance << flush << endl;
  cout << "incomingConnectors.size() = " << searchNode->incomingConnectors.size() << flush << endl;
  cout << "incomingConnectors contains: ";
  cout << "outgoingConnectors.size() = " << searchNode->outgoingConnectors.size() << flush << endl;
  cout << "outgoingConnector contains: ";
  cout << "\nmarkedConnector " << searchNode->markedConnectorIndex << flush << endl;
  if (searchNode->existsMarkedConnector()) {
		Connector& c = searchNode->getMarkedConnector();
		cout << "Children nodes: " << flush ;
        vector<weak_ptr<SearchNode>>& children = c.children;
        for (auto childPtr = children.begin();
            childPtr != children.end();
            ++childPtr)
        {
        	cout << (*childPtr).lock()->index<< " ";
        }
  }
  cout << endl << endl;
       
  if (searchNode->isGoalNode) {
      cout << "isGoalNode = true"<< flush << endl;
  }
  else {
      cout << "isGoalNode = false"<< flush << endl;
  }
  
   if (searchNode->isProven) {
      cout << "isProven = true"<< flush << endl;
  }
  else {
      cout << "isProven = false"<< flush << endl;
  }
  
  if (searchNode->isDisproven) {
      cout << "isDisproven = true"<< flush << endl;
  }
  else {
      cout << "isDisproven = false"<< flush << endl;
  }
  
  if (searchNode->isExpanded) {
      cout << "isExpanded = true"<< flush << endl;
  }
  else {
      cout << "isExpanded = false"<< flush << endl;
  }
  
   searchNode->get_state().dump_pddl();
   cout << " " << flush << endl;
}

void LaostarUtils::printConnector(Connector& connector) {
  cout << "####### Connector with id " << connector.id << "#######" << flush << endl;
  if (connector.parent != NULL) {
       cout << "parent (SearchNode) with id: " << connector.parent->index << flush << endl;
  }
  else {
      cout << "parent is NULL" << flush << endl;
  }
 
  cout << "children: " << flush;
   for (auto it = connector.children.begin(); it != connector.children.end(); ++it) {
       cout << (*it).lock()->index << " " << flush;
   }

  /*for (auto it = connector.children.begin(); it != connector.children.end(); ++it) {
		 cout << " " << endl;
		(*it).lock()->get_state().dump_fdr(); 
		cout << " " << endl;
   }*/
  cout << " " << endl;
  
  cout << "nondeterministic operator " << connector.nondeterministic_operator->name << flush << endl;
  cout << "baseCost " << connector.baseCost << flush << endl;
  cout << "isProven " << connector.isProven << flush << endl;
  cout << "isDisproven " << connector.isDisproven << flush << endl;
  cout << "isSafe " << connector.isSafe << flush << endl;
  cout << " " << flush << endl;
}

void LaostarUtils::printOperatorNames(std::vector<GlobalOperator* >& operators) {
  cout << "#operators: " << operators.size() << flush << endl;
  
  for(auto op = operators.begin(); op != operators.end(); ++op) {
    cout << "operator: " << (*op)->name << flush <<endl;
  }
  cout << " " << flush << endl;
}

void LaostarUtils::printSetOfNodes(std::vector<SearchNode>& v) {
  cout << "Print Set of nodes: ";
  for (auto it = v.begin(); it != v.end(); ++it) {
    cout << it->index << " ";
  }
  cout << endl;
}

void LaostarUtils::printSetOfNodes(std::vector<shared_ptr<SearchNode>>& v) {
  cout << "Print Set of nodes: ";
  for (auto it = v.begin(); it != v.end(); ++it) {
    cout << (*it)->index << " ";
  }
  cout << endl;
}

void LaostarUtils::printSetOfNodes(std::set<shared_ptr<SearchNode>>& v, string nameOfSet) {
  cout << nameOfSet <<" Print Set of nodes: ";

  for (auto it = v.begin(); it != v.end(); ++it) {
    cout << (*it)->index << " ";
  }
  cout << endl;
}


void LaostarUtils::printQueue(MAX_HEAP_COST heap) {
	cout << "Print min heap size() " << heap.size() << endl;
	cout << "Print min heap: ";
	//int counter = 0;
 /*for (auto it = heap.begin(); it != heap.end(); ++it) {
   if(counter == 10) break;
   cout << (*it)->index << "/" <<  (*it)->costEstimate << " ";
 }*/
	for (size_t i = 0; i < heap.size(); ++i) {
		shared_ptr<SearchNode> it = heap.top();
		heap.pop();
		cout << it->index << "/" <<  it->costEstimate << " ";
	}
	cout << " " << endl;
}

void LaostarUtils::printQueue(MAX_HEAP_INDEX heap) {
	cout << "Print min heap size() " << heap.size() << endl;
	cout << "Print min heap: ";
// int counter = 0;
 /*for (auto it = heap.begin(); it != heap.end(); ++it) {
	if(counter == 10) break; 
   cout << (*it)->index << " ";
 }*/
	for (size_t i = 0; i < heap.size(); ++i) {
		shared_ptr<SearchNode> it = heap.top();
		heap.pop();
		cout << it->index << "/" <<  it->costEstimate << " ";
	}
	cout << " " << endl;
}


void LaostarUtils::printQueue(std::queue<shared_ptr<Connector>>& conQueue) {
 cout << "Print conQueue size() " << conQueue.size() << endl;
 cout << "Print conQueue: ";
 while(!conQueue.empty()) {
	  shared_ptr<Connector> con = conQueue.front();
	  conQueue.pop();
	  cout << con->id<< " ";
 }
 cout << " " << endl;
}


void LaostarUtils::printSetOfConnectors(vector<Connector>& c)
{
		for (auto it = c.begin(); it != c.end(); ++it) {
		cout << it->id << " " << flush <<endl;	
	}	
}

void LaostarUtils::printSetOfConnectors(vector<shared_ptr<Connector>>& c)
{
	std::cout << " \n PRINT SET OF CONNECTORS " << flush;
	for (auto it = c.begin(); it != c.end(); ++it) {
		cout << (*it)->id << " " << flush;
	}	
	std::cout << "" << std::endl;
}

void LaostarUtils::printMap(unordered_map<int, int>& map)
{
	for (auto it = map.begin(); it != map.end(); ++it) {
		cout << "Key " << it->first << " Value " << it->second <<endl;
	}		
}

void LaostarUtils::printSetOfNodes(map<size_t, shared_ptr<SearchNode>>& map)
{
	for (auto it = map.begin(); it != map.end(); ++it) {
		cout << (*it).second->index << " " << endl;
		(*it).second->get_state().dump_fdr(); 
	}	
	cout << " " << endl;
}

void LaostarUtils::printSet(std::set<int>& s)
{
	std::cout << "Begin Set" << std::endl;
	for (auto it = s.begin(); it != s.end(); ++it) {
		std::cout << *it << std::endl;	
	}	
	std::cout << "End Set" << std::endl;
}

