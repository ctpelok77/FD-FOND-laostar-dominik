#include "group.h"
#include "../globals.h"
#include "../graph_algorithms/scc.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <set>

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

bool first_index_compare ( const vector<int>& l, const vector<int>& r) { return l[0] < r[0]; }

int compare_states(const GlobalState& l, const GlobalState& r) {
	if (l.get_id() == r.get_id())
		return 0;

    for (size_t i = g_variable_domain.size(); i > 0; --i) {
    	if (l[i-1] < r[i-1])
    		return -1;
    	if (l[i-1] > r[i-1])
    		return 1;
    }
    return 0;
}


bool Group::safe_to_add_generators;
int Group::num_identity_generators;
int Group::stop_after_false_generated;
int Group::store_identity_generators;

//std::vector<int> Group::operator_node_index;

Group::Group(const Options &opts) : packed_state(0), unpacked_state(0), permuted_state(0), best_current(StateID::no_state) {
	use_independent_variables_decomposition = opts.get<bool>("independent_variables_partition");    stop_after_false_generated = opts.get<int>("stop_after_false_generated");
    if (stop_after_false_generated == -1)
    	stop_after_false_generated = numeric_limits<int>::max();
    store_identity_generators = opts.get<bool>("store_identity_generators");

//    operator_node_index.clear();
    generate_whole_group = opts.get<bool>("generate_whole_group");
}

void Group::initialize() {
    if (!packed_state)
        packed_state = new PackedStateBin[g_state_packer->get_num_bins()];
    if (!unpacked_state)
        unpacked_state = new int[g_variable_domain.size()];
    if (generate_whole_group && !permuted_state)
    	permuted_state = new int[g_variable_domain.size()];

    safe_to_add_generators = true;
    num_identity_generators = 0;
}

Group::~Group(){
    //Empty - all vectors expected to be deleted by default destructor of vector
}


void Group::free_memory() {
    if (packed_state) {
        delete packed_state;
        packed_state = 0;
    }
    if (unpacked_state) {
        delete unpacked_state;
        unpacked_state = 0;
    }
    // Removing the permutations
    generators.clear();
}


/**
 * Add new permutation to the list of permutations
 * The function will be called from bliss
 */
void Group::add_permutation(void* param, unsigned int, const unsigned int * full_perm){
    assert(safe_to_add_generators);
    Permutation perm(full_perm);
    //Only if we have non-identity permutation we need to save it into the list of generators
    if(!perm.identity()){
        ((Group*) param)->add_generator(perm);
    } else {
        num_identity_generators++;
        if (store_identity_generators)
        	((Group*) param)->add_identity_generator(full_perm);

        if (num_identity_generators > stop_after_false_generated) {
            cout << endl << "Problems with generating symmetry group! Too many false generators." << endl;
            cout << "Number of generators: 0" << endl;
            cout << "Actually found " << ((Group*) param)->get_num_generators() << " generators." << endl;
            exit(1);
        }
    }
}

void Group::add_generator(Permutation gen) {
    assert(safe_to_add_generators);
    generators.push_back(gen);
#ifdef DEBUGMODE
    cout << "Added generator number " << get_num_generators();
#endif

}

void Group::add_identity_generator(const unsigned int * full_perm) {
	// Getting the full permutation. We know that the permutation is identity on states.
    assert(safe_to_add_generators);

	// Extracting the mapping of identical operators
    vector<pair<size_t, size_t> > ops;
	for (size_t op_no=0; op_no < g_operators.size(); ++op_no) {
//		unsigned int idx = Permutation::length + op_no;
		unsigned int idx = get_operator_node_index(op_no);
		if (full_perm[idx] == idx)
			continue;

		size_t to_op_no = get_op_no_by_index(full_perm[idx]);
		ops.push_back(make_pair(op_no, to_op_no));
	}
    identity_generators.push_back(ops);
}

unsigned int Group::get_operator_node_index(int op_no) {
	return Permutation::num_variable_and_value_nodes + op_no;
}

unsigned int Group::get_op_no_by_index(unsigned int index) {
	return index - Permutation::num_variable_and_value_nodes;
}

int Group::get_num_generators() const {
    return generators.size();
}

void Group::add_generators_powers(){
	// Going over all generators, for each add all its powers, putting all new generators into a separate set,
	// making sure that no generator appears twice
	Gens new_gens;
	for (int i=0; i < get_num_generators(); i++) {
		// Starting with power 2 to save memory allocation
		Permutation p(get_permutation(i), get_permutation(i));
		while (!p.identity()) {
			new_gens.push_back(p);
			p = Permutation(p, get_permutation(i));
		}
	}


	// Save the permutations that are not in the generators
	for (size_t i=0; i < new_gens.size(); i++) {
		bool in_generators = false;
		for (int j=0; j < get_num_generators(); j++) {
			if (get_permutation(j) == new_gens[i]) {
				in_generators = true;
				break;
			}
		}
		if (in_generators)
			continue;

		add_generator(new_gens[i]);
		cout<<"Added new generator" << endl;
	}

}

int Group::get_generators_orders(vector<int>& orders) const {
	int max_order=0;

	for (int i=0; i < get_num_generators(); i++) {
		// Starting with power 2 to save memory allocation
		Permutation p(get_permutation(i), get_permutation(i));
		int ord=2;
		while (!p.identity()) {
			p = Permutation(p, get_permutation(i));
			ord++;
		}
		orders.push_back(ord);
		if (ord > max_order) {
			max_order = ord;
		}
	}
	return max_order;
}

int Group::get_number_object_symmetries() const {
	int count=0;

	for (int i=0; i < get_num_generators(); i++) {
		if ( get_permutation(i).is_object_symmetry() )
			count++;
	}
	return count;
}

void Group::create_whole_group(){
	// We start with our set of generators. We choose a pair of elements (might be the same),
	// multiply them and add to the set of generators. Continue until no new element is added.
	set<Permutation> elems;
	elems.insert(generators.begin(), generators.end());
	bool change = true;
	while (change) {
		Gens new_gens;
		for (set<Permutation>::iterator it1=elems.begin(); it1 != elems.end(); it1++) {
			Permutation p(*it1, *it1);
			if (!p.identity()) {
				new_gens.push_back(p);
			}
			set<Permutation>::iterator it2=it1;
			it2++;
			for (; it2 != elems.end(); it2++) {
				Permutation p1(*it1, *it2);
				if (!p1.identity()) {
					new_gens.push_back(p1);
				}
				Permutation p2(*it2, *it1);
				if (!p2.identity()) {
					new_gens.push_back(p2);
				}
			}
		}
		change = false;

		for (size_t i=0; i < new_gens.size(); i++) {
			std::pair<std::set<Permutation>::iterator,bool> ret = elems.insert(new_gens[i]);
			change = change || ret.second;
		}
	}
	generators.clear();
	std::copy(elems.begin(), elems.end(), std::back_inserter(generators));
}


bool Group::is_commutative(const Permutation& gen1, const Permutation& gen2) const{
	for (size_t i = 0; i < gen1.num_variable_and_value_nodes; ++i) {
		if (gen1.get_value(gen2.get_value(i)) != gen2.get_value(gen1.get_value(i)))
			return false;
	}
	return true;
}

void Group::group_division() const {

}


void Group::group_independent_variables_decomposition() {
	assert(subgrpoups.size()==0);
	//Setting an edge between two permutations that are variable dependent
	if (generators.size() == 0)
		return;
	// Finding connected components

	// TODO: Check whether a better algorithm exists for undirected graphs
	vector<vector<int> > graph(generators.size(), vector<int>());
    for (size_t i = 0; i < generators.size() - 1; ++i) {
        for (size_t j = i+1; j < generators.size(); ++j) {
        	const Permutation& from_perm = get_permutation(i);
        	const Permutation& to_perm = get_permutation(j);

        	if (!from_perm.variables_disjoint(to_perm)) {
    			// Add bidirectional edges.
    			graph[i].push_back(j);
    			graph[j].push_back(i);
        	}
        }
    }

	SCC scc(graph);
    const vector<vector<int> >& result = scc.get_result();
    subgrpoups.clear();
    for (size_t i = 0; i < result.size(); ++i) {
        // Need to sort each component
        vector<int> component = result[i];

        if(component.size() == 0)
        	continue;
         sort(component.begin(), component.end());
        subgrpoups.push_back(component);
    }
    // Sorting by the first element
    sort(subgrpoups.begin(), subgrpoups.end(),first_index_compare);

}


void Group::default_direct_product(){
    safe_to_add_generators = false;  // From this point on it is not safe to add generators
//    dump_generators();
    if (use_independent_variables_decomposition) {
    	group_independent_variables_decomposition();
    }
}

Permutation Group::compose_permutation(Trace& perm_index) const {
    Permutation new_perm;
    for (size_t i = 0; i < perm_index.size(); ++i) {
        new_perm = Permutation(new_perm, get_permutation(perm_index[i]));
    }
    return new_perm;
}

const Permutation& Group::get_permutation(int index) const {
    return generators[index];
}

PackedStateBin* Group::get_canonical_state(const GlobalState& state) {
	if (use_independent_variables_decomposition)
		return get_canonical_state_independent_vars(state);

	return get_canonical_state_ehc(state);
}

PackedStateBin* Group::get_canonical_state_ehc(const GlobalState& state) {
    size_t size = get_num_generators();
    copy_buff(unpacked_state, state);

    bool changed = (size > 0);
    while (changed) {
        changed = false;
        for (size_t i=0; i < size; i++) {
            if (generators[i].replace_if_less(unpacked_state)) {
                changed =  true;
            }
        }
    }
    pack_state(packed_state, unpacked_state);
    return packed_state;
}

PackedStateBin* Group::get_canonical_state_bfs(const GlobalState& /*state*/) {
	exit_with(EXIT_UNSUPPORTED);
/*
	// Doing a BFS to pass all symmetrical states, keeping the currently best state
    size_t size = get_num_generators();
    StateRegistry tmp_registry;

    // Add state to the registry
    GlobalState tmp_state = tmp_registry.add_state(state);
    best_current = tmp_state.get_id();
    bfs_info[tmp_state].visited = true;
    // Add state to the queue
    set<StateID> queue;
    queue.insert(tmp_state.get_id());
//	cout << "=========================================================" << endl;

    while (!queue.empty()) {
        // Get state from the queue
    	set<StateID>::iterator it = queue.begin();
    	GlobalState s = tmp_registry.lookup_state(*it);
//    	s.dump_pddl();
//    	cout << "------------------------------------------------------" << endl;
    	queue.erase(it);

        for (size_t i=0; i < size; i++) {
        	Permutation& perm = generators[i];
            GlobalState t = tmp_registry.get_state_permutation(s, perm);
            if (bfs_info[t].visited)
            	continue;

            // Adding to queue
            queue.insert(t.get_id());
            bfs_info[t].visited = true;
            if (compare_states(tmp_registry.lookup_state(best_current), t) == 1) {
            	best_current = t.get_id();
            }
        }
    }
//    cout << "========Best found:==========" << endl;
//    tmp_registry.lookup_state(best_current).dump_pddl();
//	cout << "=========================================================" << endl;

    pack_state(packed_state, tmp_registry.lookup_state(best_current));
    return packed_state;
    */
}

PackedStateBin* Group::get_canonical_state_independent_vars(const GlobalState& state) {
    copy_buff(unpacked_state, state);

    for (size_t i=0; i < subgrpoups.size(); ++i) {
    	// Going over each subgroup, finding minima for that group
    	const vector<int>& subgroup = subgrpoups[i];
        size_t size = subgroup.size();

        bool changed = (size > 0);
        while (changed) {
            changed = false;
            for (size_t j=0; j < size; ++j) {
            	int perm_index = subgroup[j];
                if (generators[perm_index].replace_if_less(unpacked_state)) {
                    changed =  true;
                }
            }
        }
    }
    pack_state(packed_state, unpacked_state);
    return packed_state;
}

PackedStateBin* Group::get_canonical_state_one_pass(const GlobalState& state) {
    copy_buff(unpacked_state, state);
    if (generators.size() == 0) {
        pack_state(packed_state, unpacked_state);
        return packed_state;
    }

    for (size_t i=0; i < generators.size(); ++i) {
    	generators[i].permute_state_unpacked(state, permuted_state);
    	copy_buffer_if_less(unpacked_state, permuted_state);
    }
    pack_state(packed_state, unpacked_state);
    return packed_state;
}

void Group::get_operators_equivalence_relation(vector<vector<size_t> >& equiv) const {
	// Going over the identity generators, adding edges to a graph if operators identical.
	// Connected components of the graph correspond to the equivalence classes of operators.
	if (identity_generators.size() == 0)
		return;

	vector<vector<int> > graph(g_operators.size(), vector<int>());

    for (size_t i = 0; i < identity_generators.size(); ++i) {
        const vector<pair<size_t, size_t> >& ops = identity_generators[i];
        for (size_t j = 0; j < ops.size(); ++j) {
        	size_t op_no = ops[j].first;
        	size_t to_op_no = ops[j].second;
        	graph[op_no].push_back(to_op_no);
        }
    }
	SCC scc(graph);
	const vector<vector<int> >& result = scc.get_result();
    for (size_t i = 0; i < result.size(); ++i) {
    	const vector<int>& eq_class = result[i];
    	if (eq_class.size() <= 1)
    		continue;
    	vector<size_t> ops;
        for (size_t j = 0; j < eq_class.size(); ++j) {
        	size_t op_no = eq_class[j];
        	ops.push_back(op_no);
        }
    	equiv.push_back(ops);
    }
}

void Group::dump_operators_equivalence(const vector<vector<size_t> >& equiv) const {
	cout << "--------------------------------------------------------------------------" << endl;
    for (size_t i = 0; i < equiv.size(); ++i) {
    	const vector<size_t>& eq_class = equiv[i];
    	if (eq_class.size() <= 1)
    		continue;
    	cout << "Equivalence Class " << i << endl;
        for (size_t j = 0; j < eq_class.size(); ++j) {
        	size_t op_no = eq_class[j];
        	g_operators[op_no].dump();
        }
    }
	cout << "--------------------------------------------------------------------------" << endl;
}


int Group::count_redundant_operators_equivalence(const vector<vector<size_t> >& equiv) const {
	int num_ops_to_prune = 0;
    for (size_t i = 0; i < equiv.size(); ++i) {
    	const vector<size_t>& eq_class = equiv[i];
    	if (eq_class.size() <= 1)
    		continue;
    	num_ops_to_prune += eq_class.size() - 1;
    }
    return num_ops_to_prune;
}

void Group::dump_generators() const {
//    if (get_num_generators() == 0)
//        return;
    for (int i = 0; i < get_num_generators(); i++) {
        cout << "Generator " << i << endl;
        get_permutation(i).print_cycle_notation();
    }

    cout << "-------------------------------------------------------------------------" << endl;
    cout << "Extra group info:" << endl;
    cout << "Permutation of variable values length: " << Permutation::num_variable_and_value_nodes << endl;
    cout << "Permutation total length: " << Permutation::length << endl;
    cout << "Permutation variables by values (" << g_variable_domain.size() << "): " << endl;
    for (unsigned int i = g_variable_domain.size(); i < Permutation::num_variable_and_value_nodes; i++)
        cout << Permutation::get_var_by_index(i) << "  " ;
    cout << endl;

    cout << "Number of identity on states generators found: " << Group::num_identity_generators << endl;
    for (size_t i = 0; i < identity_generators.size(); ++i) {
        cout << "Identity generator " << i << endl;
        const vector<pair<size_t, size_t> >& ops = identity_generators[i];
        for (size_t j = 0; j < ops.size(); ++j) {
        	// Dumping the pairs
        	size_t op_no = ops[j].first;
        	const GlobalOperator& op = g_operators[op_no];
        	size_t to_op_no = ops[j].second;
        	const GlobalOperator& to_op = g_operators[to_op_no];
        	cout << "[" << op_no << "]" << op.get_name() << " ->" << "[" << to_op_no << "]" << to_op.get_name() << endl;
        	op.dump();
        	to_op.dump();
        	cout << "--------------------------------------------------------------------------" << endl;
        }
    }

    if (use_independent_variables_decomposition) {
    	cout << "Number of subgroups: " << subgrpoups.size() << endl;

    	cout << "Permutation indices by subgroups:" << endl;

    	for (size_t i = 0; i < subgrpoups.size(); ++i) {
    		// Printing subgroup i
    		cout << " [ ";
    		for (size_t j = 0; j < subgrpoups[i].size(); ++j) {
    			cout << subgrpoups[i][j] << " ";
    		}
    		cout << " ]";
    	}
    	cout << endl;
    }
}

static string get_atom_for_fact(int var_no, int value) {
    const string &fact_name = g_fact_names[var_no][value];
    if (fact_name == "<none of those>") {
        cerr << "error: fact name should not be <none of those>"
             << fact_name << endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    int predicate_pos = 0;
    if (fact_name.substr(0, 5) == "Atom ") {
        predicate_pos = 5;
    } else if (fact_name.substr(0, 12) == "NegatedAtom ") {
//        predicate_pos = 12;
        cerr << "error: fact name should not be NegatedAtom ..."
             << fact_name << endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    size_t paren_pos = fact_name.find('(', predicate_pos);
    size_t closing_paren_pos = fact_name.find(')', paren_pos);
    if (predicate_pos == 0 || paren_pos == string::npos || closing_paren_pos == string::npos) {
        cerr << "error: cannot extract predicate from fact: "
             << fact_name << endl;
        exit_with(EXIT_INPUT_ERROR);
    }
    // Constructing string from pieces
    string res = string("(") +
      	   string(fact_name.begin() + predicate_pos, fact_name.begin() + paren_pos) +
		   string(" ") +
		   string(fact_name.begin() + paren_pos + 1, fact_name.begin() + closing_paren_pos) +
 		   string(")");
    res.erase(std::remove(res.begin(), res.end(), ','), res.end());

    if (predicate_pos == 5)
    	return res;

    return string("(not ") + res + string(")");
}

void Group::dump_generators_cycles() const {
	ofstream os("symmetries.pddl");
	os << "(define (problem strips-symmetry)" << endl;
	os << "  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;" << endl;
	os << "  ;; Symmetry group generators" << endl;
    for (int i = 0; i < get_num_generators(); i++) {
    	os << "  ;; Generator " << i << endl;
//        cout << get_permutation(i).get_cycle_notation_patrik() << endl;

    	// (:set [:name <some symbol>] (atom ..) ... (atom ..))
        vector<vector<pair<int, int> > > cycles;
        get_permutation(i).get_cycle_notation_indexes(cycles);
        for (size_t j = 0; j < cycles.size(); ++j) {
        	const vector<pair<int, int> >& cycle = cycles[j];
        	os << "  (:set :name g" << i << " ";
            for (size_t k = 0; k < cycle.size(); ++k) {
            	os << get_atom_for_fact(cycle[k].first, cycle[k].second) << " ";
//            	const string &fact_name = g_fact_names[cycle[k].first][cycle[k].second];
//            	os << "(" <<fact_name << ") ";
            }
            os << ")" << endl;
        }
        os << endl;
    }
	os << "  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;" << endl;
	os << ")" << endl;
	os.close();
}

void Group::copy_buff(int* buffer, const GlobalState& s2) const{
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
    	buffer[i] = s2[i];
    }
}

bool Group::copy_buffer_if_less(int* s1, const int* s2) const{
	// We can choose here any comparison method we like, as long as we stick to it.
	size_t from = 0;
    for (; from < g_variable_domain.size(); ++from) {
    	if (s1[from] > s2[from])
    		break;
    	if (s1[from] < s2[from])
    		return false;
    }
    if (from == g_variable_domain.size())
    	return false;
    // We get here when s2 is smaller, and from points to the first different variable
    for (size_t i = from; i < g_variable_domain.size(); ++i)
    	s1[i] = s2[i];
    return true;
}

void Group::pack_state(PackedStateBin* buffer, const int* s2) const{
	int num_variables = g_variable_domain.size();
    for (int i = 0; i < num_variables; ++i) {
    	int val = s2[i];
    	g_state_packer->set(buffer, i, val);
    }
}

void Group::pack_state(PackedStateBin* buffer, const GlobalState& s2) const{
	int num_variables = g_variable_domain.size();
    for (int i = 0; i < num_variables; ++i) {
    	int val = s2[i];
    	g_state_packer->set(buffer, i, val);
    }
}

void Group::get_trace(const GlobalState& state, Trace& full_trace) {
    int size = get_num_generators();
    if (size == 0)
        return;

    copy_buff(unpacked_state, state);
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i=0; i < size; i++) {
            if (generators[i].replace_if_less(unpacked_state)) {
                full_trace.push_back(i);
                changed = true;
            }
        }
    }
}

void Group::get_trace_independent_vars(const GlobalState& state, Trace& full_trace) {
    int size = get_num_generators();
    if (size == 0)
        return;

    copy_buff(unpacked_state, state);
    for (size_t i=0; i < subgrpoups.size(); ++i) {
    	// Going over each subgroup, finding minima for that group
    	const vector<int>& subgroup = subgrpoups[i];
        size_t size = subgroup.size();

        bool changed = (size > 0);
        while (changed) {
            changed = false;
            for (size_t j=0; j < size; ++j) {
            	int perm_index = subgroup[j];
                if (generators[perm_index].replace_if_less(unpacked_state)) {
                    full_trace.push_back(perm_index);
                    changed =  true;
                }
            }
        }
    }
}

void Group::get_trace_bfs(const GlobalState& /*state*/, Trace& /*full_trace*/) {
	exit_with(EXIT_UNSUPPORTED);
	/*
	// Doing a BFS to pass all states, keeping the currently best state.
	// Keeping the creating permutation for each state, reconstructing when done

    size_t size = get_num_generators();
    if (size == 0)
        return;

    StateRegistry tmp_registry;

    // Add state to the registry
    GlobalState tmp_state = tmp_registry.add_state(state);
    best_current = tmp_state.get_id();
    bfs_info[tmp_state].visited = true;
    // Add state to the queue
    set<StateID> queue;
    queue.insert(tmp_state.get_id());

    while (!queue.empty()) {
        // Get state from the queue
    	set<StateID>::iterator it = queue.begin();
    	GlobalState s = tmp_registry.lookup_state(*it);
    	queue.erase(it);

        for (size_t i=0; i < size; i++) {
        	Permutation& perm = generators[i];
            GlobalState t = tmp_registry.get_state_permutation(s, perm);
            if (bfs_info[t].visited)
            	continue;

            // Adding to queue
            queue.insert(t.get_id());
            bfs_info[t].visited = true;
            bfs_info[t].creating_generator = i;
            bfs_info[t].parent = s.get_id();
            if (compare_states(tmp_registry.lookup_state(best_current), t) == 1) {
            	best_current = t.get_id();
            }
        }
    }
    // Reconstructing the path backwards from the best state
    Trace full_trace_backwards;
    GlobalState curr = tmp_registry.lookup_state(best_current);
    while (bfs_info[curr].parent != StateID::no_state) {
    	full_trace_backwards.push_back(bfs_info[curr].creating_generator);
    	curr = tmp_registry.lookup_state(bfs_info[curr].parent);
    }
    // Inverting the trace
    size_t trace_len = full_trace_backwards.size();
    for (size_t i = 0; i < trace_len; i++) {
    	short int elem = full_trace_backwards[trace_len-1-i];
    	full_trace.push_back(elem);
    }
    */
}

void Group::get_trace_one_pass(const GlobalState& state, Trace& full_trace) {
    int size = get_num_generators();
    if (size == 0)
        return;

    copy_buff(unpacked_state, state);
    size_t best_index = generators.size();
    for (size_t i=0; i < generators.size(); ++i) {
    	generators[i].permute_state_unpacked(state, permuted_state);
    	if (copy_buffer_if_less(unpacked_state, permuted_state))
    		best_index = i;
    }

    if (best_index < generators.size()) {
    	full_trace.push_back(best_index);
    }
}

const GlobalOperator& Group::get_operator_permutation(const GlobalOperator* op, const Permutation& perm) const {
//	std::cout << "DEBUG: Permuting operator " << op->name << std::endl;
	int op_no = get_op_index(op);
//	std::cout << "DEBUG: Permuting operator " << op_no << std::endl;
	unsigned int idx = get_operator_node_index(op_no);

//	std::cout << "DEBUG: Operator node index " << idx << std::endl;
	unsigned int to_idx = perm.get_value(idx);
//	std::cout << "DEBUG: Permuted operator node index " << to_idx << std::endl;
	int to_op_no = get_op_no_by_index(to_idx);
//	std::cout << "DEBUG: Permuted operator " << to_op_no << std::endl;

	if (op->is_axiom()) {
		return g_axioms[to_op_no - g_operators.size()];
	}
	return g_operators[to_op_no];
}

////////////////////////////////////////////////////////////////////////////////////////////
void Group::set_landmark_ids_for_generators(LandmarkGraph &lm_graph) {
//	cout << "Number of landmarks for setting IDs: " << lm_graph.number_of_landmarks() << ", number of generators to set IDs: " << get_num_generators() << endl;
	for (int i = 0; i < get_num_generators(); i++) {
		generators[i].set_landmarks_connections(lm_graph);
	}
}

void Group::get_landmark_stable_ids(vector<int>& landmark_ids) {
	if (get_num_generators() == 0)
		return;
	int num_landmarks = get_permutation(0).get_landmark_ids().size();
	vector<bool> stable(num_landmarks, true);
	for (int i = 0; i < get_num_generators(); i++) {
		const vector<int> &lms = get_permutation(i).get_landmark_ids();
		assert(lms.size() == num_landmarks);

		// Going over the indices, setting false to each landmark that is not copied into itself
	    for (int id = 0; id < num_landmarks; ++id) {
	    	if (lms[id] != id)
	    		stable[id] = false;
	    }

	}
	// Getting the stable ids to return
	int stable_ids = 0;
    for (int id = 0; id < num_landmarks; ++id) {
    	if (stable[id]) {
    		landmark_ids.push_back(id);
    		stable_ids++;
    	}
    }

	cout << "Number of stable landmarks: " << stable_ids << " out of total " << num_landmarks << " landmarks." << endl;
}
