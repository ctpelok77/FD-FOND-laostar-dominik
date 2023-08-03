#include "search_space.h"

#include "global_operator.h"
#include "global_state.h"
#include "globals.h"
#include "successor_generator.h"
#include "symmetries/graph_creator.h"
#include <cassert>
#include "search_node_info.h"

using namespace std;

bool SearchNode::operator==(const SearchNode &otherNode) {
  if (otherNode.index != this->index){
    return false;
  }
  return true;
}


SearchNode::SearchNode(StateID state_id_, SearchNodeInfo &info_,
                       OperatorCost cost_type_)
    : state_id(state_id_), info(info_), cost_type(cost_type_) {
	assert(state_id != StateID::no_state);

	this->markedConnectorIndex = -1;
	
	isGoalNode = test_goal(get_state());
	if (isGoalNode) {
		weakGoalDistance = 0;
	} else {
		weakGoalDistance = INFINITY;
	}

   	isProven = isGoalNode;
	isDisproven = false;	
	isExpanded = false;
}

Connector& SearchNode::getMarkedConnector() {
  return *(outgoingConnectors.at(markedConnectorIndex));
}

void SearchNode::setMarkedConnector(int connectorIndex) {
  markedConnectorIndex = connectorIndex;
}

bool SearchNode::existsMarkedConnector() {
  if (markedConnectorIndex != -1) return true;
  return false;
}

GlobalState SearchNode::get_state() const {
    return g_state_registry->lookup_state(state_id);
}

bool SearchNode::is_open() const {
    return info.status == SearchNodeInfo::OPEN;
}

bool SearchNode::is_closed() const {
    return info.status == SearchNodeInfo::CLOSED;
}

bool SearchNode::is_dead_end() const {
    return info.status == SearchNodeInfo::DEAD_END;
}

bool SearchNode::is_new() const {
    return info.status == SearchNodeInfo::NEW;
}

int SearchNode::get_g() const {
    assert(info.g >= 0);
    return info.g;
}

int SearchNode::get_real_g() const {
    return info.real_g;
}

int SearchNode::get_h() const {
    return info.h;
}

bool SearchNode::is_h_dirty() const {
    return info.h_is_dirty;
}

void SearchNode::set_h_dirty() {
    info.h_is_dirty = true;
}

void SearchNode::clear_h_dirty() {
    info.h_is_dirty = false;
}

void SearchNode::open_initial(int h) {
    assert(info.status == SearchNodeInfo::NEW);
    info.status = SearchNodeInfo::OPEN;
    info.g = 0;
    info.real_g = 0;
    info.h = h;
    info.parent_state_id = StateID::no_state;
    info.creating_operator = 0;
}

void SearchNode::open(int h, const SearchNode &parent_node,
                      const GlobalOperator *parent_op) {
    assert(info.status == SearchNodeInfo::NEW);
    info.status = SearchNodeInfo::OPEN;
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.h = h;
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = parent_op;
}

void SearchNode::reopen(const SearchNode &parent_node,
                        const GlobalOperator *parent_op) {
    assert(info.status == SearchNodeInfo::OPEN ||
           info.status == SearchNodeInfo::CLOSED);

    // The latter possibility is for inconsistent heuristics, which
    // may require reopening closed nodes.
    info.status = SearchNodeInfo::OPEN;
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = parent_op;
}

// like reopen, except doesn't change status
void SearchNode::update_parent(const SearchNode &parent_node,
                               const GlobalOperator *parent_op) {
    assert(info.status == SearchNodeInfo::OPEN ||
           info.status == SearchNodeInfo::CLOSED);
    // The latter possibility is for inconsistent heuristics, which
    // may require reopening closed nodes.
    info.g = parent_node.info.g + get_adjusted_action_cost(*parent_op, cost_type);
    info.real_g = parent_node.info.real_g + parent_op->get_cost();
    info.parent_state_id = parent_node.get_state_id();
    info.creating_operator = parent_op;
}

void SearchNode::increase_h(int h) {
    assert(h >= info.h);
    info.h = h;
}

void SearchNode::close() {
    assert(info.status == SearchNodeInfo::OPEN);
    info.status = SearchNodeInfo::CLOSED;
}

void SearchNode::mark_as_dead_end() {
    info.status = SearchNodeInfo::DEAD_END;
}

void SearchNode::dump() const {
    cout << state_id << ": ";
    g_state_registry->lookup_state(state_id).dump_fdr();
    if (info.creating_operator) {
        cout << " created by " << info.creating_operator->get_name()
             << " from " << info.parent_state_id << endl;
    } else {
        cout << " no parent" << endl;
    }
}

SearchSpace::SearchSpace(OperatorCost cost_type_)
    : cost_type(cost_type_) {
}


SearchNode SearchSpace::get_node(const GlobalState &state) {
    return SearchNode(state.get_id(), search_node_infos[state], cost_type);
}

std::shared_ptr<SearchNode> SearchSpace::get_node_shared_ptr(const GlobalState &state) {
	return std::shared_ptr<SearchNode>(new SearchNode(state.get_id(), search_node_infos[state], cost_type));
}


void SearchSpace::trace_path(const GlobalState &goal_state,
                             vector<const GlobalOperator *> &path) const {
    if (g_symmetry_graph && !g_symmetry_graph->is_trivial()) {
        trace_path_with_symmetries(goal_state, path);
        return;
    }
    GlobalState current_state = goal_state;
    assert(path.empty());
    for (;;) {
        const SearchNodeInfo &info = search_node_infos[current_state];
        const GlobalOperator *op = info.creating_operator;
        if (op == 0) {
            assert(info.parent_state_id == StateID::no_state);
            break;
        }
        path.push_back(op);
        current_state = g_state_registry->lookup_state(info.parent_state_id);
    }
    reverse(path.begin(), path.end());
}

void SearchSpace::trace_path_with_symmetries(const GlobalState &goal_state,
                                     vector<const GlobalOperator *> &path) const {
    vector<Permutation> permutations;
    vector<GlobalState> state_trace;
    GlobalState current_state = goal_state;
    assert(path.empty());
    for (;;) {
        const SearchNodeInfo &info = search_node_infos[current_state];
        assert(info.status != SearchNodeInfo::NEW);
        const GlobalOperator *op = info.creating_operator;
        state_trace.push_back(current_state);
        GlobalState parent_state = g_initial_state();
        GlobalState new_state = g_initial_state();
        if (op != 0) {
            parent_state = g_state_registry->lookup_state(info.parent_state_id);
            new_state = g_state_registry->get_successor_state(parent_state, *op);
        }
        Permutation p;
        if (new_state.get_id() != current_state.get_id()){
            p = g_symmetry_graph->create_permutation_from_state_to_state(current_state, new_state);
        }
        permutations.push_back(p);
        if (op == 0)
            break;
        current_state = parent_state;
    }
    assert(state_trace.size() == permutations.size());
    vector<Permutation> reverse_permutations;
    Permutation temp_p, p;
    while (permutations.begin() != permutations.end()) {
        p = permutations.back();
        temp_p = Permutation(p, temp_p);
        reverse_permutations.push_back(temp_p);
        permutations.pop_back();
    }
    for (size_t i = 0; i < state_trace.size(); ++i){
        Permutation &permutation = reverse_permutations[state_trace.size() - i-1];
        state_trace[i] = g_state_registry->get_state_permutation(state_trace[i],
                                                                 permutation);
    }
    int trace_size = state_trace.size();
    for (int i = trace_size - 1; i > 0; i--) {
        vector<const GlobalOperator *> applicable_ops;
        g_successor_generator->generate_applicable_ops(state_trace[i], applicable_ops);
        bool found = false;
        int min_cost_op=0;
        int min_cost=numeric_limits<int>::max();

        for (size_t o = 0; o < applicable_ops.size(); o++) {
            const GlobalOperator *op = applicable_ops[o];
            GlobalState succ_state = g_state_registry->get_successor_state(state_trace[i], *op);
            if (succ_state.get_id() == state_trace[i-1].get_id()) {
                found = true;
                if (op->get_cost() < min_cost) {
                    min_cost = op->get_cost();
                    min_cost_op = o;
                }
            }
        }
        if (!found) {
            cout << "No operator is found!!!" << endl
                 << "Cannot reach the state " << endl;
            state_trace[i-1].dump_pddl();
            cout << endl << "From the state" << endl;
            state_trace[i].dump_pddl();
            exit_with(EXIT_CRITICAL_ERROR);
        }
        path.push_back(applicable_ops[min_cost_op]);
    }
}

void SearchSpace::dump() const {
    for (PerStateInformation<SearchNodeInfo>::const_iterator it =
             search_node_infos.begin(g_state_registry);
         it != search_node_infos.end(g_state_registry); ++it) {
        StateID id = *it;
        GlobalState s = g_state_registry->lookup_state(id);
        const SearchNodeInfo &node_info = search_node_infos[s];
        cout << id << ": ";
        s.dump_fdr();
        if (node_info.creating_operator && node_info.parent_state_id != StateID::no_state) {
            cout << " created by " << node_info.creating_operator->get_name()
                 << " from " << node_info.parent_state_id << endl;
        } else {
            cout << "has no parent" << endl;
        }
    }
}

void SearchSpace::print_statistics() const {
//cout << "Number of registered states: "
 //        << g_state_registry->size() << endl;
}
