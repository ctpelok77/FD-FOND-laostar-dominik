#include "graph_creator.h"
#include <vector>
#include "../global_operator.h"
#include "permutation.h"
#include "../timer.h"
#include <cassert>

using namespace std;

static inline int get_op_index(const GlobalOperator *op) {
	if (op->is_axiom()) {
	    int op_index = op - &*g_axioms.begin();
	    assert(op_index >= 0 && op_index < g_axioms.size());
	    return op_index + g_operators.size();
	}
    int op_index = op - &*g_operators.begin();
    assert(op_index >= 0 && op_index < g_operators.size());
    return op_index;
}

GraphCreator::GraphCreator(const Options &opts) : group(opts) {
    initialized = false;
    search_type = SymmetryBasedSearchType(opts.get_enum("symmetries"));

    // Setting static for Permutation
    reverse_lexicographical_state_comparison = opts.get<bool>("reverse_lexicographical");

    lgraph = 0;
    if (opts.contains("lm_graph")) {
    	lgraph = opts.get<LandmarkGraph *>("lm_graph");
    }
    use_landmarks = (lgraph != 0);

    landmarks_stable_only = opts.get<bool>("landmarks_stable_only");
    if (landmarks_stable_only) {
        if (!lgraph) {
        	cout << "Cannot use stable landmarks without specifying the landmark graph!" << endl;
        	::exit(1);
        }
    }
    add_generators_powers = opts.get<bool>("add_powers");
    generate_whole_group = opts.get<bool>("generate_whole_group");
    perfect_canonical = opts.get<bool>("perfect_canonical");

    no_search = opts.get<bool>("no_search");

    time_bound = opts.get<int>("time_bound");
    generators_bound =  opts.get<int>("generators_bound");
    statistics = opts.get<bool>("statistics");
    evaluate_real_successor = opts.get<bool>("evaluate_real_successor");
    remove_redundand_operators = opts.get<bool>("remove_redundand_operators");
}


void GraphCreator::initialize() {
#ifdef USE_BLISS
    if (initialized)
        return;

    initialized = true;
    cout << "Initializing symmetry " << endl;
    group.initialize();
    // Setting static for Permutation
    Permutation::reverse_lexicographical_state_comparison = reverse_lexicographical_state_comparison;

    bliss::AbstractGraph* graph;
    graph = create_bliss_directed_graph();
    ((bliss::Digraph*)graph)->set_splitting_heuristic(bliss::Digraph::shs_flm);
    graph->set_time_bound(time_bound);
    graph->set_generators_bound(generators_bound);

    bliss::Stats stats1;
    cout << "Using Bliss to find group generators" << endl;
    graph->canonical_form(stats1,&(Group::add_permutation),&group);
    cout << "Got " << group.get_num_generators() << " group generators, time step: [t=" << g_timer << "]" << endl;

    if (lgraph) {
    	cout << "Using landmark graph to set a connection between landmarks per generator" << endl;
    	group.set_landmark_ids_for_generators(*lgraph);
    }
	Permutation::use_landmarks = (lgraph != 0);

    if (landmarks_stable_only) {
    	group.get_landmark_stable_ids(landmark_stable_ids);
    }

    if (add_generators_powers) {
    	cout << "Adding the powers of all generators" << endl;
    	group.add_generators_powers();
//  	cout << "Finished adding the powers of all generators" << endl;
    }
    if (generate_whole_group) {
    	cout << "Creating all elements" << endl;
    	group.create_whole_group();
    }

    group.default_direct_product();

    vector<vector<size_t> > equiv;
    group.get_operators_equivalence_relation(equiv);
    if (statistics) {
    	// Dumping generators
    	group.dump_generators();
    	group.dump_operators_equivalence(equiv);
    	cout << "Number of redundant due to symmetry operators: " << group.count_redundant_operators_equivalence(equiv) << endl;
    	vector<int> orders;
    	int max_order = group.get_generators_orders(orders);
    	vector<int> count_orders(max_order+1,0);
    	for (size_t i=0; i < orders.size(); i++) {
    		int ord = orders[i];
    		cout<<"Generator " << i << " has order " << ord << endl;
    		count_orders[ord]++;
    	}

    	for (size_t i=2; i < count_orders.size(); i++) {
    		if (i > 2 && count_orders[i] == 0)
    			continue;
    		cout << "Number of generators of order " << i << ": " << count_orders[i] << endl;
    	}
		cout<<"Maximal generator order is " << max_order << endl;

//    	cout << "Number of object symmetries: " << group.get_number_object_symmetries() << endl;

		// Dumping generators by cycles
		group.dump_generators_cycles();
    }

    cout<<"Number of generators: "<< group.get_num_generators()<<endl;
    cout << "Number of identity generators: " << Group::get_num_identity_generators() << endl;

    if (no_search)
    	::exit(0);

    if (remove_redundand_operators) {
    	// equiv consists of equivalence classes of size > 1
    	// Thus we first mark all as needed, and then mark redundant (second and onwards in each equivalence class)
    	non_redundand_operators.assign(g_operators.size(), true);

        for (size_t i = 0; i < equiv.size(); ++i) {
        	const vector<size_t>& eq_class = equiv[i];
            for (size_t j = 1; j < eq_class.size(); ++j) {
            	int op_no = eq_class[j];
            	non_redundand_operators[op_no] = false;
            }
        }
    }

    // Deleting the graph
    delete graph;
    cout << "Done initializing symmetries [t=" << g_timer << "]" << endl;
#else
    cerr << "You must build the planner with the USE_BLISS symbol defined" << endl;
    ::exit(1);
#endif
}



GraphCreator::~GraphCreator() {
    // Freeing the group
    free_memory();
}

#ifdef USE_BLISS
bliss::Digraph* GraphCreator::create_bliss_directed_graph() const {
    // Differ from create_bliss_graph() in (a) having one node per action (incoming arcs from pre, outgoing to eff),
    //                                 and (b) not having a node for goal, recoloring the respective values.
   // initialization

	bool stabilize_init = false;
	bool stabilize_goal = true;
	bool stabilize_non_of_those = false;

    if (get_search_type() == NOT_STABILIZED_NO_SEARCH || get_search_type() == INIT_STABILIZED_NO_SEARCH) {
    	stabilize_goal = false;
    }
    if (get_search_type() == INIT_AND_GOAL_STABILIZED_REGULAR_SEARCH ||
    		get_search_type() == INIT_AND_GOAL_STABILIZED_STRIPS_NO_SEARCH ||
    		get_search_type() == INIT_STABILIZED_NO_SEARCH) {
    	stabilize_init = true;
    }
    if (get_search_type() == INIT_AND_GOAL_STABILIZED_STRIPS_NO_SEARCH) {
    	stabilize_non_of_those = true;
    }

    int num_of_vertex = g_variable_domain.size();
    for (size_t num_of_variable = 0; num_of_variable < g_variable_domain.size(); num_of_variable++){
        Permutation::dom_sum_by_var.push_back(num_of_vertex);
        num_of_vertex+=g_variable_domain[num_of_variable];
        for(int num_of_value = 0; num_of_value < g_variable_domain[num_of_variable]; num_of_value++){
            Permutation::var_by_val.push_back(num_of_variable);
        }
    }

//    Permutation::length = num_of_vertex;
    Permutation::num_variable_and_value_nodes = num_of_vertex;

    bliss::Digraph* g = new bliss::Digraph();
    int idx = 0;
    // add vertex for each varaible
    for (size_t i = 0; i < g_variable_domain.size(); i++){
       idx = g->add_vertex(PREDICATE_VERTEX);
    }
    // now add values vertices for each predicate
    for (size_t i = 0; i < g_variable_domain.size(); i++){
       for (int j = 0; j < g_variable_domain[i]; j++){
          idx = g->add_vertex(VALUE_VERTEX);
          g->add_edge(idx,i);
       }
    }

    // now add vertices for operators
    for (size_t op_no = 0; op_no < g_operators.size(); op_no++){
        const GlobalOperator& op = g_operators[op_no];
        g->add_vertex(get_operator_color(op));
//        Group::operator_node_index.push_back(op_idx);
    }
    // now add vertices for axioms
    for (size_t op_no = 0; op_no < g_axioms.size(); op_no++){
        const GlobalOperator& op = g_axioms[op_no];
        g->add_vertex(get_operator_color(op));
//        Group::operator_node_index.push_back(op_idx);
    }

    Permutation::length = num_of_vertex + g_operators.size() + g_axioms.size();

    for (size_t op_no = 0; op_no < g_operators.size(); op_no++){
        const GlobalOperator& op = g_operators[op_no];
        add_operator_directed_graph(g, op);
    }
    for (size_t op_no = 0; op_no < g_axioms.size(); op_no++){
        const GlobalOperator& op = g_axioms[op_no];
        add_operator_directed_graph(g, op);
    }

    // Recoloring the initial values
    if (stabilize_init) {
        for (size_t var = 0; var < g_variable_domain.size(); var++){
        	int val = g_initial_state()[var];
            int init_idx = get_fact_index(var, val);
            g->change_color(init_idx, INIT_NON_GOAL_VERTEX);
        }
    }

    // Recoloring the goal values
    if (stabilize_goal) {
    	for (size_t i = 0; i < g_goal.size(); i++){
    		int var = g_goal[i].first;
    		int val = g_goal[i].second;
    		int goal_idx = get_fact_index(var, val);
            int init_val = g_initial_state()[var];
            if (stabilize_init && val == init_val)
            	g->change_color(goal_idx, INIT_AND_GOAL_VERTEX);
            else
            	g->change_color(goal_idx, GOAL_VERTEX);
    	}
    }

    // Recoloring the <none of those> values and negated atoms
    if (stabilize_non_of_those) {
        for (size_t var = 0; var < g_variable_domain.size(); var++) {
        	// Going over all values
        	//TODO: check whether that is always the last value
            for (int value = 0; value < g_variable_domain[var]; value++){
            	const string &fact_name = g_fact_names[var][value];
                int idx = get_fact_index(var, value);
            	if (fact_name == "<none of those>") {
                    g->change_color(idx, NONE_OF_THOSE_VERTEX);
            	} else if (fact_name.substr(0, 12) == "NegatedAtom ") {
                    g->change_color(idx, NEGATED_VERTEX);
            	}
            }
        }
    }

    return g;
}

void GraphCreator::add_operator_directed_graph(bliss::Digraph* g, const GlobalOperator& op) const {
	// One vertex for the operator itself
	// Edges from facts in the precondition to the operator
	// Edges from the operator to facts in the unconditional effects
	// One vertex for each conditional effect with non-empty condition
	//    Edge from the operator to the conditional effect,
	//    Edge from the facts in its condition to the conditional effect
	//    Edge from the conditional effect to the effect fact

//    int op_idx = g->add_vertex(get_operator_color(op));
	int op_no = get_op_index(&op);
    int op_idx = Group::get_operator_node_index(op_no);

    const std::vector<GlobalCondition>& pre = op.get_preconditions();
    for (size_t i = 0; i < pre.size(); ++i) {
    	add_condition_directed_graph(g, op_idx, pre[i]);
    }
    // For non-deterministic effects
    const std::vector<std::vector<GlobalEffect> > &nd_effs = op.get_non_deterministic_effects();
    for (size_t i = 0; i < nd_effs.size(); ++i) {
    	// Adding a node for each non-deterministic option
    	// Edge from the operator to non-deterministic option node
    	// The rest is as for deterministic effects, replacing operator node with non-deterministic option node
        int nd_eff_idx = g->add_vertex(NON_DETERMINISTIC_OPTION_VERTEX);
        g->add_edge(op_idx, nd_eff_idx);
        const std::vector<GlobalEffect>& eff = nd_effs[i];
        for (size_t idx1 = 0; idx1 < eff.size(); idx1++) {
        	add_effect_directed_graph(g, nd_eff_idx, eff[idx1]);
        }
    }
}

void GraphCreator::add_condition_directed_graph(bliss::Digraph* g, int to_idx, const GlobalCondition& cond) const {
    int cond_idx = get_fact_index(cond.var, cond.val);
    g->add_edge(cond_idx, to_idx);
}

void GraphCreator::add_effect_directed_graph(bliss::Digraph* g, int from_idx, const GlobalEffect& eff) const {
	int var = eff.var;

    int eff_val = eff.val;
    int eff_idx = get_fact_index(var, eff_val);

    if (eff.conditions.size() == 0) {
        g->add_edge(from_idx, eff_idx);
        return;
    }
//    cout << "Adding a node for conditional effect" << endl;
    // Adding a node for each condition. An edge from op to node, an edge from node to eff,
    // for each cond, an edge from cond to node.
    color_t effect_color = CONDITIONAL_EFFECT_VERTEX;
    int cond_op_idx = g->add_vertex(effect_color);
    g->add_edge(from_idx, cond_op_idx); // Edge from operator to conditional effect
    g->add_edge(cond_op_idx, eff_idx); // Edge from conditional effect to effect
    // Adding edges for conds
    for (size_t c = 0; c < eff.conditions.size(); c++) {
    	add_condition_directed_graph(g, cond_op_idx, eff.conditions[c]);
    }
}
#endif

int GraphCreator::get_operator_color(const GlobalOperator& op) const {
    if (op.is_axiom()) {
    	return AXIOM_VERTEX;
    }
    return  MAX_VALUE + op.get_cost();
}

int GraphCreator::get_fact_index(int var, int val) const {
    return Permutation::dom_sum_by_var[var] + val;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////


void GraphCreator::get_trace(const GlobalState& state, Trace& full_trace) {
	if (generate_whole_group) {
		group.get_trace_one_pass(state, full_trace);
		return;
	}
	if (perfect_canonical) {
		group.get_trace_bfs(state, full_trace);
		return;
	}
    group.get_trace(state, full_trace);
}

PackedStateBin* GraphCreator::get_canonical_state(const GlobalState& state)  {
	if (generate_whole_group)
		return group.get_canonical_state_one_pass(state);

	if (perfect_canonical)
		return group.get_canonical_state_bfs(state);

    return group.get_canonical_state(state);
}

Permutation GraphCreator::create_permutation_from_state_to_state(const GlobalState& from_state, const GlobalState& to_state) {
    Trace new_trace;
    Trace curr_trace;
    get_trace(from_state, curr_trace);
    get_trace(to_state, new_trace);

    Permutation p1(create_permutation_from_trace(new_trace), true);  //inverse
    Permutation p2 = create_permutation_from_trace(curr_trace);
    return  Permutation(p2, p1);
}

bool GraphCreator::is_non_redundand_operator(const GlobalOperator* op) const {

	assert(non_redundand_operators.size() == g_operators.size());
	if (non_redundand_operators.size() == 0)
		return true;
	return non_redundand_operators[get_op_index(op)];
}


void GraphCreator::add_options_to_parser(OptionParser &parser) {
    vector<string> sym_types;
    sym_types.push_back("none");
    sym_types.push_back("init_and_goal_reg");
    sym_types.push_back("goal_only_reg");
    sym_types.push_back("goal_only_orbit");
    sym_types.push_back("init_and_goal_patrik");
    sym_types.push_back("no_stabilization_no_search");
    sym_types.push_back("init_no_search");
    parser.add_enum_option("symmetries",
                           sym_types,
                           "use symmetries",
						   "goal_only_orbit"
                           );

    parser.add_option<bool>("evaluate_real_successor",
                           "heuristic evaluation of the real successor state", "false");

    parser.add_option<bool>("reverse_lexicographical",
                           "lexicographical compare of states", "true");

    parser.add_option<bool>("add_powers",
                           "Add the powers of found generators to the generators list", "false");

    parser.add_option<bool>("generate_whole_group",
                           "Create all elements of the symmetry group", "false");

    parser.add_option<int>("stop_after_false_generated",
                           "Stopping after the Bliss software generated too many false generators","-1");

    parser.add_option<bool>("no_search",
                           "No search is performed, exiting after creating the symmetries", "false");

    parser.add_option<LandmarkGraph *>("lm_graph", "Landmark graph", "", OptionFlags(false));
//    parser.add_list_option<LandmarkGraph *>("lm_graph", "Landmark graph", "[]");

    parser.add_option<bool>("landmarks_stable_only",
                           "Update only stable landmarks when reaching symmetrical state", "false");

    parser.add_option<int>("time_bound",
                           "Stopping after the Bliss software reached the time bound", "0");

    parser.add_option<int>("generators_bound",
                           "Stopping after the Bliss software reached the bound on the number of generators", "0");

    parser.add_option<bool>("statistics",
                           "Dump statistics", "false");

    parser.add_option<bool>("independent_variables_partition",
                           "Use variable independence for group decomposition", "false");

    parser.add_option<bool>("store_identity_generators",
                           "Store generators that stabilize all states", "false");

    parser.add_option<bool>("remove_redundand_operators",
                           "Remove operators with pre/eff/cost identical to other existing operators", "false");

    parser.add_option<bool>("perfect_canonical",
                           "Find perfect canonical state", "false");

}



static GraphCreator *_parse(OptionParser &parser) {
//	cout << "=========================================================================================================" << endl;
//	cout << "======     Running symmetry parser" << endl;
//	cout << "=========================================================================================================" << endl;
    GraphCreator::add_options_to_parser(parser);
    Options opts = parser.parse();

    SymmetryBasedSearchType type = SymmetryBasedSearchType(opts.get_enum("symmetries"));

    if (type == NO_SYMMETRIES) {
    	return 0;
    }

    if (type == INIT_AND_GOAL_STABILIZED_REGULAR_SEARCH || type == GOAL_ONLY_STABILIZED_REGULAR_SEARCH) {
    	cout << "DKS is not supported in this version" << endl;
        exit_with(EXIT_UNSUPPORTED);
    }

    if (!parser.dry_run()) {

        bool statistics = opts.get<bool>("statistics");
        bool remove_redundand_operators = opts.get<bool>("remove_redundand_operators");
        if (statistics || remove_redundand_operators) {
        	opts.set<bool>("store_identity_generators", true);
        }
        if (type == INIT_AND_GOAL_STABILIZED_STRIPS_NO_SEARCH ||
        		type == NOT_STABILIZED_NO_SEARCH ||
        		type == INIT_STABILIZED_NO_SEARCH) {
        	// Ensure that there is no search
        	opts.set<bool>("no_search", true);
        }

    	GraphCreator* gr = new GraphCreator(opts);

    	if (gr) {
    		if (gr->get_search_type() == GOAL_ONLY_STABILIZED_REGULAR_SEARCH) {
        		cout << "Creating symmetry graph stabilizing goal only and using regular search" << endl;
    		} else if (gr->get_search_type() == INIT_AND_GOAL_STABILIZED_REGULAR_SEARCH) {
        		cout << "Creating symmetry graph stabilizing initial and goal and using regular search" << endl;
    		} else if (gr->get_search_type() == GOAL_ONLY_STABILIZED_ORBIT_SEARCH) {
        		cout << "Creating symmetry graph stabilizing goal only and using orbit search" << endl;
    		} else if (gr->get_search_type() == INIT_AND_GOAL_STABILIZED_STRIPS_NO_SEARCH) {
        		cout << "Creating symmetry graph stabilizing initial state, goal and <none of those> values, no search" << endl;
    		} else if (gr->get_search_type() == NOT_STABILIZED_NO_SEARCH) {
        		cout << "Creating symmetry graph without stabilizing states, no search" << endl;
    		} else if (gr->get_search_type() == INIT_STABILIZED_NO_SEARCH) {
        		cout << "Creating symmetry graph stabilizing initial state, no search" << endl;
    		} else {
    			cout << "Illegal option!" << endl;
        		::exit(1);
    		}
    	}

    	return gr;
    } else {
    	return 0;
    }

}


static Plugin<GraphCreator> _plugin("symmetry_state_pruning", _parse);
