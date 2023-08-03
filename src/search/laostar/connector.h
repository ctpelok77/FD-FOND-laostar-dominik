/**
 * A connector associates with a node node a set of successor nodes
 * succ_1, ..., succ_n. One connector corresponds to an AND
 * conjunction, whereas several outgoing connectors from one node are
 * interpreted disjunctively. Hence, a list of outgoing connectors corresponds
 * to a disjunction over conjunctions over possible successor states.
 */
#ifndef CONNECTOR_H
#define CONNECTOR_H
#include <vector>
#include "../global_operator.h"
#include "util.h"
#include <set>
#include <memory>
#include <functional>

using std::vector;
using std::set;
using std::weak_ptr;
using std::shared_ptr;

class SearchNode;
class Connector {
public:
   int id;
  
   bool operator==(const Connector &connector) const;
    
   Connector(shared_ptr<SearchNode>& parent, int id, vector<weak_ptr<SearchNode>> &children, const GlobalOperator* op);
    
   Connector();
   
   /**
    * Parent node to which this connector is attached
    */
   shared_ptr<SearchNode> parent;
    
   /**
   *  Child nodes
   */
   vector<weak_ptr<SearchNode>> children;

   
  /**
   * Operator which corresponds to this connector.
   */
   const GlobalOperator* nondeterministic_operator;
  
  /**
   * Base cost of this connector
   */
  double baseCost;

  bool isProven;
   
  bool isDisproven;
  
  bool isSafe;

  
private:
};

#endif
