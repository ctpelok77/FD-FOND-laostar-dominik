/************************************************
 *  This is the Graphviz state space dump tool  
 * 
 * # Nodes 
 * red node: DISPROVEN
 * green node: PROVEN
 * yellow: nor PROVEN nor DISPROVEN and unexpanded 
 * white: not PROVEN not DISPROVEN and expanded   
 * 
 * # Arcs
 * red arc: non-safe connector
 * blue arc: safe connector 
 * green arc: marked connector
 ************************************************/

#ifndef GVWRITER 
#define GVWRITER 
#include <sstream>
#include <memory>
#include "../search_space.h"
#include <queue>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "util.h"

using std::find;
using std::to_string;
using std::ofstream;

class GraphvizWriter
{

public:	
	void createOutputStateSpace(bool complete, 
								shared_ptr<SearchNode>& initialNode, 
								std::string outfile); 
};
#endif
