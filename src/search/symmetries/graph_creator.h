#ifndef SYMMETRIES_GRAPH_CREATOR_H
#define SYMMETRIES_GRAPH_CREATOR_H

#ifdef USE_BLISS
#include <graph.hh>
#endif
#include "group.h"
#include "../plugin.h"
#include "../global_state.h"

enum SymmetryBasedSearchType {
    NO_SYMMETRIES,
    INIT_AND_GOAL_STABILIZED_REGULAR_SEARCH,
    GOAL_ONLY_STABILIZED_REGULAR_SEARCH,
    GOAL_ONLY_STABILIZED_ORBIT_SEARCH,
    INIT_AND_GOAL_STABILIZED_STRIPS_NO_SEARCH,
    NOT_STABILIZED_NO_SEARCH,
    INIT_STABILIZED_NO_SEARCH
};

//TODO: Add vertex for axioms.
enum color_t {PREDICATE_VERTEX, VALUE_VERTEX, PRECOND_VERTEX, EFFECT_VERTEX,
              GOAL_VERTEX, INIT_AND_GOAL_VERTEX, INIT_NON_GOAL_VERTEX, CONDITIONAL_EFFECT_VERTEX, CONDITIONAL_DELETE_EFFECT_VERTEX,
			  NONE_OF_THOSE_VERTEX, NEGATED_VERTEX, NON_DETERMINISTIC_OPTION_VERTEX, AXIOM_VERTEX, MAX_VALUE};


class GraphCreator  {
	SymmetryBasedSearchType search_type;
	bool add_generators_powers;
	bool generate_whole_group;
	bool perfect_canonical;
	bool landmarks_stable_only;
	std::vector<int> landmark_stable_ids;
	bool reverse_lexicographical_state_comparison;
	bool use_landmarks;
	LandmarkGraph *lgraph;
	bool no_search;
	bool initialized;

    int time_bound;
    int generators_bound;
    bool statistics;
    bool evaluate_real_successor;
    bool remove_redundand_operators;
public:

    GraphCreator(const Options &opts);
    virtual ~GraphCreator();

    void initialize();

    bool is_trivial() const { return group.get_num_generators() == 0; }

    Permutation create_permutation_from_trace(Trace& auth) const { return group.compose_permutation(auth); }

    void get_trace(const GlobalState& state, Trace& full_trace);
    PackedStateBin* get_canonical_state(const GlobalState& state) ;

    Permutation create_permutation_from_state_to_state(const GlobalState& from_state, const GlobalState& to_state);

    static void add_options_to_parser(OptionParser &parser);
    void free_memory() { group.free_memory(); }

    SymmetryBasedSearchType get_search_type() const { return search_type; }

    bool is_using_landmarks() const { return use_landmarks; }
    bool is_stable_landmarks_only() const { return landmarks_stable_only; }
    const vector<int>& get_stable_landmark_ids() const { return landmark_stable_ids; }
    bool is_evaluate_real_successor() const { return evaluate_real_successor; }

    void get_operators_equivalence_relation(vector<vector<size_t> >& equiv) const {
    	group.get_operators_equivalence_relation(equiv);
    }
    bool is_non_redundand_operator(const GlobalOperator* op) const;

    const GlobalOperator& get_operator_permutation(const GlobalOperator* op, const Permutation& perm) const {
    	return group.get_operator_permutation(op, perm);
    }

private:
    Group group;
    std::vector<bool> non_redundand_operators;

#ifdef USE_BLISS
    bliss::Digraph* create_bliss_directed_graph() const;
    void add_operator_directed_graph(bliss::Digraph* g, const GlobalOperator& op) const;
    bool effect_can_be_overwritten(size_t ind, const std::vector<GlobalEffect>& prepost) const;
    void add_condition_directed_graph(bliss::Digraph* g, int to_idx, const GlobalCondition& cond) const;
    void add_effect_directed_graph(bliss::Digraph* g, int from_idx, const GlobalEffect& eff) const;
#endif
    int get_operator_color(const GlobalOperator& op) const;
    int get_fact_index(int var, int val) const;

};

#endif

