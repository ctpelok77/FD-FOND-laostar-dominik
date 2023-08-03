#ifndef SEARCH_SPACE_H
#define SEARCH_SPACE_H

#include "global_state.h"
#include "operator_cost.h"
#include "per_state_information.h"
#include "search_node_info.h"
#include "laostar/connector.h"
#include <vector>
#include <memory>
#include <unordered_set>
#include <functional>

using std::shared_ptr;

class GlobalOperator;
class GlobalState;

class SearchNode {
    StateID state_id;
    SearchNodeInfo &info;
    OperatorCost cost_type;
public:
    bool operator==(const SearchNode &otherNode);

    SearchNode(StateID state_id_, SearchNodeInfo &info_,
               OperatorCost cost_type_);

    StateID get_state_id() const {
        return state_id;
    }
    GlobalState get_state() const;

    bool is_new() const;
    bool is_open() const;
    bool is_closed() const;
    bool is_dead_end() const;

    bool is_h_dirty() const;
    void set_h_dirty();
    void clear_h_dirty();
    int get_g() const;
    int get_real_g() const;
    int get_h() const;

    void open_initial(int h);
    void open(int h, const SearchNode &parent_node,
              const GlobalOperator *parent_op);
    void reopen(const SearchNode &parent_node,
                const GlobalOperator *parent_op);
    void update_parent(const SearchNode &parent_node,
                       const GlobalOperator *parent_op);
    void increase_h(int h);
    void close();
    void mark_as_dead_end();

    void dump() const;

  /**
   * 
   */ 
   int depth;

  /**
   * Heuristic value of this node
   */
  double heuristic;

  /**
   * Cost estimate of this node
   */
  double costEstimate;

  /**
   * Weak goal distance of this node.
   */
  double weakGoalDistance;
  
  /**
   * Incoming connectors
   */
  std::vector<shared_ptr<Connector>> incomingConnectors;

  shared_ptr<Connector> sharedPtrHandle;
  
  /**
   * Outgoing connectors
   */
   std::vector<shared_ptr<Connector>> outgoingConnectors;
  
   Connector& getMarkedConnector();
   void setMarkedConnector(int connectorIndex);
   bool existsMarkedConnector(); 

   /**
   * Indicates if this is a goal node.
   */
  bool isGoalNode;
  
  /**
   * Flag indicating that this node has been proven, i.e., the protagonist has a winning strategy
   * for this node.
   */
  bool isProven;
  
  /**
   * Flag indicating that this node has been disproven, i.e., the antagonist has a winning strategy
   * for this node.
   */
  bool isDisproven;
  
  /**
   * Flag indicating that this node has already been expanded.
   */
  bool isExpanded;
  
  int markedConnectorIndex;

  int index;
private:
  
};

class SearchSpace {
    PerStateInformation<SearchNodeInfo> search_node_infos; 
    OperatorCost cost_type;

    void trace_path_with_symmetries(const GlobalState &goal_state,
                    std::vector<const GlobalOperator *> &path) const;
public:
    SearchSpace(OperatorCost cost_type_);
    SearchNode get_node(const GlobalState &state);
    std::shared_ptr<SearchNode> get_node_shared_ptr(const GlobalState &state);
    void trace_path(const GlobalState &goal_state,
                    std::vector<const GlobalOperator *> &path) const;

    void dump() const;
    void print_statistics() const;
}; 

#endif
