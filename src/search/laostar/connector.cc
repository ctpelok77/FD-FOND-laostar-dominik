/**
 * Connector class like in myND. 
 * A connector corresponds to an application
 * of an operator connecting the state where
 * the operator is applied with the states 
 * that result from the operator application.
 */

#include "connector.h"
#include "util.h"
#include "../search_space.h"

bool Connector::operator==(const Connector &connector) const{
	if (id != connector.id) {
			return false;
	}
	return true;
}

Connector::Connector(shared_ptr<SearchNode>& parent,
					 int id, vector<weak_ptr<SearchNode>> &children, 
					 const GlobalOperator* op) 
{
    this->parent = parent;
    this->id = id;
    this->children = children;
    this->nondeterministic_operator = op;
    this->baseCost = op->cost;
	this->isSafe = true;
}
