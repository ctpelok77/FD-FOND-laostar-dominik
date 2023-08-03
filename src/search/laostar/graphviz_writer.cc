#include "graphviz_writer.h"

void GraphvizWriter::createOutputStateSpace(bool complete, shared_ptr<SearchNode>& initialNode, 
											std::string outfile)
{
	std::vector<shared_ptr<SearchNode>> seenNodes;	
	std::set<int> seenNodeIds;
	std::vector<Connector> seenConnectors;
	std::queue<shared_ptr<SearchNode>> queue;
	std::set<int> queueLogger;
	queue.push(initialNode);	
	queueLogger.insert(initialNode->index);
	while(!queue.empty()) {
		shared_ptr<SearchNode> node = queue.front();
		queue.pop();
		queueLogger.erase(node->index);
		seenNodes.push_back(node);
		seenNodeIds.insert(node->index);
		
		// if not complete only dump the state space where
		// connectors lead to
		vector<Connector> connectors;
		if (complete) {
			for (auto it = node->outgoingConnectors.begin(); it != node->outgoingConnectors.end(); ++it) {
				connectors.push_back(*(*it));
			}	
		}	
		else {
			if (node->existsMarkedConnector()) {
			  connectors.push_back(node->getMarkedConnector());
			}
		}
	
		// Iterate over all connectors and append all their children if
		// not previously added	
		for (auto it = connectors.begin(); it != connectors.end(); ++it) {
			seenConnectors.push_back(*it);	
			Connector& connector = *it;	
			
			for (auto next = connector.children.begin(); next != connector.children.end(); ++next) {
		        bool seenContains = find_if(seenNodeIds.begin(), seenNodeIds.end(),
								  [=](const int index)->bool
								  {
										  int i = ((*next).lock())->index;
										  if (i == index) { 
											return true;
										  }
										  return false;
								  }) != seenNodeIds.end(); 
	
				bool queueContains = find_if(queueLogger.begin(), queueLogger.end(),
								  [=](const int index) ->bool
								  {
										  int i = ((*next).lock())->index;
										  if (i == index) { 
											return true;
										  }
										  return false;
								  }) != queueLogger.end();

						  		
				if (!seenContains && !queueContains) {
					queue.push((*next).lock());			
					queueLogger.insert((*next).lock()->index);
				}
			}			
		}	
	}

	std::stringstream stream;	
	stream << "digraph {\n";
	for (auto node = seenNodes.begin(); node != seenNodes.end(); ++node) {
		stream << to_string((*node)->index);	
		stream <<  " [ peripheries=\"1\", shape=\"rectangle\", ";				
		if((*node)->isGoalNode) {
			stream <<"fontcolor=\"white\", style=\"filled\", fillcolor=\"blue\", ";
		}			
		
		if (!(*node)->isProven) {
			  if ((*node)->isDisproven && !(*node)->isExpanded) {
					stream << "style=\"filled\", fillcolor=\"red\", ";
			  } else if (((*node)->isDisproven && (*node)->isExpanded)) {
					stream << "style=\"filled,rounded\", fillcolor=\"red\", ";
			  } else if (!(*node)->isExpanded) {
					stream << "style=\"filled\", fillcolor=\"yellow\", ";
			  } else {
					stream << "style=\"rounded\", ";
			  }
		} else { // Proven nodes are labelled
			if (!(*node)->isExpanded) {
				stream << "style=\"filled\", fillcolor=\"green\", ";
			} else {
				stream << "style=\"filled,rounded\", fillcolor=\"green\", ";
			}
		}
		stream << "label=\" ";
		stream << "index: " +  to_string((*node)->index) + "\\n";
		stream << "cost estimate: " + to_string((*node)->costEstimate) + "\\n";
		stream << "heuristic estimate: " + to_string((*node)->heuristic) + "\\n";
		stream << "proven: "+to_string((*node)->isProven) + "\\n";
		//stream << (*node)->get_state().toString();
	   
	   //stream << (*node)->get_state().stateString; 
		stream << "\" ]\n";
	}

	for (auto connector = seenConnectors.begin(); connector != seenConnectors.end(); ++connector) {
		for (auto next = connector->children.begin(); next != connector->children.end(); ++next) {
			stream << std::to_string(connector->parent->index);
			stream << " -> ";
			stream <<  std::to_string(next->lock()->index);
			stream <<" [ label=\"";
			stream << connector->nondeterministic_operator->name +": "+ std::to_string(connector->id);
			stream << "\"";
			bool parent_has_marked_connector = connector->parent->existsMarkedConnector();
			int parent_marked_id = -1;
			if (parent_has_marked_connector)
				parent_marked_id = connector->parent->getMarkedConnector().id;
	
			if (complete && parent_has_marked_connector && connector->id == parent_marked_id && connector->isSafe) {
				stream << ", style=\"bold\", color=\"green\" ";
			} else if (complete && parent_has_marked_connector && connector->id == parent_marked_id && !connector->isSafe) {
				stream << ", style=\"bold\", color=\"red\" ";
			} else if (connector->isSafe) {
				stream <<  ", style=\"bold\", color=\"blue\" ";
			}
			stream << " ]\n";
		}
	}

	stream << "}\n";
  
	// Write state space to dot file
	ofstream myfile;
	myfile.open(outfile, std::ofstream::out);
	myfile << stream.rdbuf();
	myfile.close();
}

