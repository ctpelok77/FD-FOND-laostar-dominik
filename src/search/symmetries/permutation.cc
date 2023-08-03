#include "permutation.h"
#include "../globals.h"
#include <iomanip>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>

using namespace std;
#include <sstream>

unsigned int Permutation::length;
unsigned int Permutation::num_variable_and_value_nodes;

vector<int> Permutation::var_by_val;
vector<int> Permutation::dom_sum_by_var;
bool Permutation::reverse_lexicographical_state_comparison;
bool Permutation::use_landmarks;

void Permutation::_allocate() {
    borrowed_buffer = false;
    value = new unsigned int[length];
    inverse_value = new unsigned int[length];
    affected.assign(g_variable_domain.size(), false);
	vars_affected.clear();
	from_vars.assign(g_variable_domain.size(), -1);
//	buff_for_state_copy = new PackedStateBin[g_state_packer->get_num_bins()];
	affected_vars_cycles.clear();
}

void Permutation::_deallocate() {
    if (!borrowed_buffer) {
        delete[] value;
        delete[] inverse_value;
//        delete[] buff_for_state_copy;
    }
}

void Permutation::_copy_value_from_permutation(const Permutation &perm) {
    for (unsigned int i = 0; i < length; i++)
    	set_value(i, perm.get_value(i));
}

void Permutation::_inverse_value_from_permutation(const Permutation &perm) {
    for (unsigned int i = 0; i < length; i++)
		set_value(perm.get_value(i), i);
}

void Permutation::_copy_landmarks_ids_from_permutation(const Permutation &perm) {
	to_landmark_id = vector<int>(perm.get_landmark_ids());
}

void Permutation::_inverse_landmarks_ids_from_permutation(const Permutation &perm) {
	const vector<int> &to_ids = perm.get_landmark_ids();
	to_landmark_id.assign(to_ids.size(), -1);

	for (size_t i = 0; i < to_ids.size(); i++) {
		if (to_ids[i] != -1)
			to_landmark_id[to_ids[i]] = i;
	}
}



Permutation &Permutation::operator=(const Permutation &other) {
    if (this != &other) {
        if (borrowed_buffer) {
            _allocate();
        } else {
        	affected.assign(g_variable_domain.size(), false);
        	vars_affected.clear();
        	from_vars.assign(g_variable_domain.size(), -1);
        	affected_vars_cycles.clear();
        }
        _copy_value_from_permutation(other);
        if (use_landmarks) {
        	// Copying landmark info
        	_copy_landmarks_ids_from_permutation(other);
        }
    }
    this->finalize();
    return *this;
}

Permutation::Permutation(){
	_allocate();
    for (unsigned int i = 0; i < length; i++)
    	set_value(i,i);
    finalize();
}


Permutation::Permutation(const unsigned int* full_permutation){
	_allocate();
	for (unsigned int i = 0; i < length; i++){
    	set_value(i,full_permutation[i]);
	}
	finalize();
}

Permutation::Permutation(const Permutation& perm, bool invert){
    _allocate();
    if (invert) {
    	_inverse_value_from_permutation(perm);
        if (use_landmarks) {
        	// Reverting indices
        	_inverse_landmarks_ids_from_permutation(perm);
        }
    } else {
        _copy_value_from_permutation(perm);
        if (use_landmarks) {
        	// Copying landmark info
        	_copy_landmarks_ids_from_permutation(perm);
        }
    }
    finalize();
}

// New constructor to use instead of * operator
Permutation::Permutation(const Permutation& perm1, const Permutation& perm2){
    _allocate();

	for (unsigned int i = 0; i < length; i++) {
		set_value(i, perm2.get_value(perm1.get_value(i)));
	}
	if (use_landmarks) {
		// Setting landmarks info
		const vector<int> &to_ids1 = perm1.get_landmark_ids();
		if (to_ids1.size() == 0) { // Has to be identity
			if (perm1.identity()) {
				_copy_landmarks_ids_from_permutation(perm2);
				return;
			}
			cout << "Landmark IDs connections are not set for non-identity permutation!!!" << endl;
			exit(1);
		}

		const vector<int> &to_ids2 = perm2.get_landmark_ids();
		if (to_ids2.size() == 0) { // Has to be identity
			if (perm2.identity()) {
				_copy_landmarks_ids_from_permutation(perm1);
				return;
			}
			cout << "Landmark IDs connections are not set for non-identity permutation!!!" << endl;
			exit(1);
		}
		to_landmark_id.assign(to_ids1.size(), -1);
		assert(to_ids1.size() == to_ids2.size());

		for (size_t i = 0; i < to_ids1.size(); i++) {
			int to_id1 = to_ids1[i];
			if (to_id1 != -1) {
				to_landmark_id[i] = to_ids2[to_id1];
			}
		}
	}
	finalize();
}



Permutation::~Permutation(){
	_deallocate();
}

void Permutation::finalize(){
	// Sorting the vector of affected variables
	::sort(vars_affected.begin(), vars_affected.end());

	// Going over the vector from_vars of the mappings of the variables and finding cycles
	vector<bool> marked;
	marked.assign(g_variable_domain.size(), false);
	int num_vars = from_vars.size();
	for (int i = 0; i < num_vars; i++) {
		if (marked[i] || from_vars[i] == -1)
			continue;

		int current = i;
		marked[current] = true;
		vector<int> cycle;
		cycle.push_back(current);

        while (from_vars[current] != i){
        	current = from_vars[current];
        	marked[current] = true;
        	cycle.insert(cycle.begin(),current);
        }
        // Get here when from_vars[current] == i.
        affected_vars_cycles.push_back(cycle);
	}
}

bool Permutation::identity() const{
	return vars_affected.size() == 0;
}

bool Permutation::is_object_symmetry() const{
	// Assuming that each object is mapped into a single multi-valued variable.
	// Then, it is sufficient to check for 1 variable cycle that is not a singleton.

	return ((affected_vars_cycles.size() == 1) &&
			(affected_vars_cycles[0].size() > 1));
}

bool Permutation::operator ==(const Permutation &other) const{

	for (unsigned int i = 0; i < length; i++) {
        if (get_value(i) != other.get_value(i)) return false;
    }

	return true;
}


void Permutation::print_cycle_notation() const {
	vector<int> done;
	int num_variables = g_variable_domain.size();
	for (unsigned int i = num_variables; i < num_variable_and_value_nodes; i++){
		if (find(done.begin(), done.end(), i) == done.end()){
			unsigned int current = i;
	        if(get_value(i) == i) continue; //don't print cycles of size 1

	        pair<int, int> varval = get_var_val_by_index(i);
	        cout<<"("<< g_fact_names[varval.first][(int) varval.second]  <<" ";

	        while(get_value(current) != i){
	            done.push_back(current);
	            current = get_value(current);

		        pair<int, int> currvarval = get_var_val_by_index(current);
	            cout<< g_fact_names[currvarval.first][(int) currvarval.second] <<" ";
	        }
	        done.push_back(current);
	        cout<<") ";
		}
	}
	cout << endl << "Variables:  ";
	for (size_t i = 0; i < vars_affected.size(); i++) cout << vars_affected[i] << "  ";
	cout << endl << "Variables permuted:  ";

	for (size_t i = 0; i < vars_affected.size(); i++) cout << from_vars[vars_affected[i]] << " -> " << vars_affected[i] << "  ";
	cout << endl;

	cout << "Affected variables by cycles: " << endl;
	for (size_t i=0; i < affected_vars_cycles.size(); i++) {
		cout << "( " ;
		for (size_t j=0; j < affected_vars_cycles[i].size(); j++) {
			cout << affected_vars_cycles[i][j] << " ";
		}
		cout << ")  ";
	}
	cout << endl;
}


string Permutation::get_cycle_notation_patrik() const {
	std::stringstream ss;
	ss << "  ";
	vector<bool> marked(num_variable_and_value_nodes, false);

	int num_variables = g_variable_domain.size();
	for (unsigned int i = num_variables; i < num_variable_and_value_nodes; i++){
		if (marked[i])
			continue;

		unsigned int current = i;
        if(get_value(i) == i)
        	continue; //don't print cycles of size 1

        pair<int, int> varval = get_var_val_by_index(i);
    	const string &fact_name = g_fact_names[varval.first][(int) varval.second];
    	if (fact_name == "<none of those>")
    		continue;  // skipping

        // Starting a new cycle
        ss <<"("<< fact_name  <<" ";

        while(get_value(current) != i) {
        	marked[current] = true;
            current = get_value(current);
            varval = get_var_val_by_index(current);
            ss << "  " << g_fact_names[varval.first][varval.second];
        }
    	marked[current] = true;
        ss << " ), ";
	}
	string first = ss.str();
	return first.substr(0, first.size()-2);
}

void Permutation::get_cycle_notation_indexes(vector<vector<pair<int, int> > >& cycles) const {
	assert(cycles.size() == 0);
//	std::stringstream ss;
//	ss << "  ";
	vector<bool> marked(num_variable_and_value_nodes, false);

	int num_variables = g_variable_domain.size();
	for (unsigned int i = num_variables; i < num_variable_and_value_nodes; i++){
		if (marked[i])
			continue;

		unsigned int current = i;
        if(get_value(i) == i)
        	continue; //don't print cycles of size 1

        pair<int, int> varval = get_var_val_by_index(i);
    	const string &fact_name = g_fact_names[varval.first][(int) varval.second];
    	if (fact_name == "<none of those>" || fact_name.substr(0, 12) == "NegatedAtom ")
    		continue;  // skipping

        // Starting a new cycle
//    	ss <<"("<< fact_name  <<" ";
        vector<pair<int, int> > cycle;
        cycle.push_back(varval);

        while(get_value(current) != i) {
        	marked[current] = true;
        	current = get_value(current);
        	varval = get_var_val_by_index(current);
//        	ss << "  " << g_fact_names[varval.first][varval.second];
        	cycle.push_back(varval);
        }
    	marked[current] = true;
//    	ss << " ), ";
    	cycles.push_back(cycle);
	}
//	string first = ss.str();
//	return first.substr(0, first.size()-2);
}

string Permutation::get_cycle_notation() const {
	std::stringstream ss;
	ss << "  ";
	vector<int> done;
	for (unsigned int i = g_variable_domain.size(); i < num_variable_and_value_nodes; i++){
		if (find(done.begin(), done.end(), i) == done.end()){
	        int current = i;
	        if(get_value(i) == i) continue; //don't print cycles of size 1

            pair<int, int> varval = get_var_val_by_index(i);
            ss << "( " << g_fact_names[varval.first][varval.second];
	        while(get_value(current) != i){
	            done.push_back(current);
	            current = get_value(current);
	            varval = get_var_val_by_index(current);
	            ss << "  " << g_fact_names[varval.first][varval.second];
	        }
	        done.push_back(current);
	        ss << " ), ";
		}
	}
	string first = ss.str();
	return first.substr(0, first.size()-2);
}


void Permutation::dump() const {
	for (unsigned int i = 0; i < length; i++){
		if (get_value(i) != i)
			cout << setw(4) << i;
	}
	cout << endl;
	for (unsigned int i = 0; i < length; i++){
		if (get_value(i) != i)
			cout << setw(4) << get_value(i);
	}
	cout << endl;
	dump_landmark_ids();
}

void Permutation::dump_landmark_ids() const {
	cout << "Landmark IDs:   ";
	for (size_t i = 0; i < to_landmark_id.size() ; i++){
		cout << i << " -> " << to_landmark_id[i] << "   ";
	}
	cout << endl;
}

///////////////////////////////////////////////////////////////////////////////
// Changes added by Michael on 30.1.12
int Permutation::get_var_by_index(const unsigned int ind) {
	// In case of ind < g_variable_domain.size(), returns the index itself, as this is the variable part of the permutation.
	if (ind < g_variable_domain.size()) {
		cout << "=====> WARNING!!!! Check that this is done on purpose!" << endl;
		return ind;
	}
	return var_by_val[ind-g_variable_domain.size()];
}

pair<int, int> Permutation::get_var_val_by_index(const unsigned int ind) {
	assert(ind>=g_variable_domain.size());
	int var =  var_by_val[ind-g_variable_domain.size()];
	int val = ind - dom_sum_by_var[var];
	assert(val >=0 && val<g_variable_domain.size());

	return make_pair(var, val);
}

unsigned int Permutation::get_index_by_var_val_pair(int var, int val) {
	return dom_sum_by_var[var] + val;
}

void Permutation::set_value(unsigned int ind, unsigned int val) {
	value[ind] = val;
	inverse_value[val] = ind;
	set_affected(ind, val);
}

void Permutation::set_affected(unsigned int ind, unsigned int val) {
	// Skipping nodes that don't correspond to values or stabilized
	if (ind < g_variable_domain.size()
			|| ind >= num_variable_and_value_nodes
			|| ind == val)
		return;

	int var = get_var_by_index(ind);
	int to_var = get_var_by_index(val);

	if (!affected[var]) {
		vars_affected.push_back(var);
		affected[var] = true;
	}
	if (!affected[to_var]) {
		vars_affected.push_back(to_var);
		affected[to_var] = true;
	}
	// Keeping the orig. var for each var.
	from_vars[to_var] = var;
}


pair<int, int> Permutation::get_new_var_val_by_old_var_val(const int var, const int val) const {
	unsigned int old_ind = get_index_by_var_val_pair(var, val);
	unsigned int new_ind = get_value(old_ind);
	return get_var_val_by_index(new_ind);
}

pair<int, int> Permutation::get_old_var_val_by_new_var_val(const int var, const int val) const {
	unsigned int new_ind = get_index_by_var_val_pair(var, val);
	unsigned int old_ind = get_inverse_value(new_ind);
	return get_var_val_by_index(old_ind);
}

bool Permutation::replace_if_less(int* state) {
	if (reverse_lexicographical_state_comparison) {
		return replace_if_less_reverse(state);
	}
	return replace_if_less_regular(state);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// This method compares the state to the state resulting from permuting it.
// If the original state is bigger than the resulted one, it is rewritten with the latter and true is returned.
////////////////////  New version - no extra buffer is needed, faster copy ///////////////
bool Permutation::replace_if_less_reverse(int* state) {
	if (identity())
		return false;

	size_t from_here = vars_affected.size(); // Will be set to value below vars_affected.size() if there is a need to overwrite the state,
	// starting from that index in the vars_affected vector.

	// Going over the affected variables, comparing the resulted values with the state values.
	int num_vars_affected = vars_affected.size();
	for (int i = num_vars_affected-1; i>=0; i--) {
		int to_var =  vars_affected[i];
		int from_var = from_vars[to_var];

		pair<int, int> to_pair = get_new_var_val_by_old_var_val(from_var, state[from_var]);
		assert( to_pair.first == to_var);

		// Check if the values are the same, then continue to the next aff. var.
		if (to_pair.second == state[to_var])
			continue;

		if (to_pair.second < state[to_var])
			from_here = i;

		break;
	}
	if (from_here == vars_affected.size())
		return false;

	for (size_t i = 0; i < affected_vars_cycles.size(); i++) {
		if (affected_vars_cycles[i].size() == 1) {
			int var = affected_vars_cycles[i][0];
			pair<int, int> to_pair = get_new_var_val_by_old_var_val(var, state[var]);
			state[var] = to_pair.second;
			continue;
		}
		// Remembering one value to be rewritten last
		int last_var = affected_vars_cycles[i][affected_vars_cycles[i].size()-1];
		int last_val = state[last_var];

		for (size_t j=affected_vars_cycles[i].size()-1; j>0; j--) {
			// writing into variable affected_vars_cycles[i][j]
			int to_var = affected_vars_cycles[i][j];
			int from_var = affected_vars_cycles[i][j-1];
			int from_val = state[from_var];
			pair<int, int> to_pair = get_new_var_val_by_old_var_val(from_var, from_val);
			state[to_var] = to_pair.second;
		}
		// writing the last one
		pair<int, int> to_pair = get_new_var_val_by_old_var_val(last_var, last_val);
		state[affected_vars_cycles[i][0]] = to_pair.second;
	}

	return true;
}

bool Permutation::replace_if_less_regular(int* state) {
	if (identity())
		return false;

	int from_here = -1; // Will be set to non-negative value if there is a need to overwrite the state, starting from that index
	// in the vars_affected vector.

	// Going over the affected variables, comparing the resulted values with the state values.
	for (size_t i = 0; i < vars_affected.size(); i++) {
		int to_var =  vars_affected[i];
		int from_var = from_vars[to_var];

		pair<int, int> to_pair = get_new_var_val_by_old_var_val(from_var, state[from_var]);
		assert( to_pair.first == to_var);
		int to_val = to_pair.second;

		// Check if the values are the same, then continue to the next aff. var.
		if (to_val == state[to_var])
			continue;

		if (to_val < state[to_var])
			from_here = i;

		break;
	}
	if (from_here < 0)
		return false;

	for (size_t i = 0; i < affected_vars_cycles.size(); i++) {
		if (affected_vars_cycles[i].size() == 1) {
			int var = affected_vars_cycles[i][0];
			int from_val = state[var];
			pair<int, int> to_pair = get_new_var_val_by_old_var_val(var, from_val);
			state[var] = to_pair.second;
			continue;
		}
		// Remembering one value to be rewritten last
		int last_var = affected_vars_cycles[i][affected_vars_cycles[i].size()-1];
		int last_val = state[last_var];

		int num_affected_vars_cycles = affected_vars_cycles[i].size();
		for (int j=num_affected_vars_cycles-1; j>0; j--) {
			// writing into variable affected_vars_cycles[i][j]
			int to_var = affected_vars_cycles[i][j];
			int from_var = affected_vars_cycles[i][j-1];
			int from_val = state[from_var];
			pair<int, int> to_pair = get_new_var_val_by_old_var_val(from_var, from_val);
			state[to_var] = to_pair.second;
		}
		// writing the last one
		pair<int, int> to_pair = get_new_var_val_by_old_var_val(last_var, last_val);
		state[affected_vars_cycles[i][0]] = to_pair.second;
	}

	return true;
}


void Permutation::permute_state(const GlobalState& from_state, PackedStateBin* to_state) {
	// Does not assume anything about to_state

	int num_variables = g_variable_domain.size();
	for(int from_var = 0; from_var < num_variables; from_var++) {
		int from_val = from_state[from_var];
		pair<int, int> to_pair = get_new_var_val_by_old_var_val(from_var, from_val);

		// Copying the values to the new state
//		to_state[to_pair.first] = to_pair.second;
		g_state_packer->set(to_state, to_pair.first, to_pair.second);
	}
}

void Permutation::permute_state_unpacked(const GlobalState& from_state, int* to_state) {
	// Does not assume anything about to_state

	int num_variables = g_variable_domain.size();
	for(int from_var = 0; from_var < num_variables; from_var++) {
		int from_val = from_state[from_var];
		pair<int, int> to_pair = get_new_var_val_by_old_var_val(from_var, from_val);

		// Copying the values to the new state
		to_state[to_pair.first] = to_pair.second;
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Permutation::set_landmarks_connections(LandmarkGraph &lm_graph) {
	int num_landmarks = lm_graph.number_of_landmarks();
	to_landmark_id.assign(num_landmarks, -1);
//	cout << "Setting the initial indices, number of landmarks is " << to_landmark_id.size() << endl;
    for (int id = 0; id < num_landmarks; ++id) {
    	LandmarkNode *node = lm_graph.get_lm_for_index(id);
        for (int to_id = 0; to_id < num_landmarks; ++to_id) {
        	LandmarkNode *to_node = lm_graph.get_lm_for_index(to_id);
        	if (is_permutation_of(node, to_node)) {
        		to_landmark_id[id] = to_id;
        		break;
        	}
        }
//		cout << "==> " << id << " -> " << to_landmark_id[id] << endl;
    }
}



bool Permutation::is_permutation_of(const LandmarkNode* from_node, const LandmarkNode* to_node) {
	// Finds whether to_node is a permutation of from_node by direct comparison. Used once, to create the vector of indices.
	if (from_node->conjunctive != to_node->conjunctive)
		return false;

	if (from_node->disjunctive != to_node->disjunctive)
		return false;


	const std::vector<int> &from_vars = from_node->vars;
	const std::vector<int> &to_vars = to_node->vars;

	if (from_vars.size() != to_vars.size())
		return false;

	const std::vector<int> &from_vals = from_node->vals;
	const std::vector<int> &to_vals = to_node->vals;

	for (size_t v = 0; v < from_vars.size(); v++) {
		int from_var = from_vars[v];
		int from_val = from_vals[v];
		pair<int, int> to_pair = get_new_var_val_by_old_var_val(from_var, from_val);
		int var = to_pair.first;
		int val = to_pair.second;

		// Checking whether a part of to_node, if not, return false.
		bool is_part_of = false;
		for (size_t to_v = 0; to_v < to_vars.size(); to_v++) {
			if (to_vars[to_v] == var && to_vals[to_v] == val) {
				is_part_of = true;
			}
		}
		if (!is_part_of)
			return false;
	}

	return true;

}

void Permutation::permute_landmark_ids(const vector<bool>& from_ids, vector<bool>& to_ids) const {
	// The landmark in to_ids is considered to be not reached if its origin is not reached in from_ids

	assert(to_ids.empty());
	to_ids.assign(from_ids.size(), true);
	for (size_t i = 0; i < from_ids.size(); i++) {
		// Go over the ids, if false, mark as false the permuted id.
		if (from_ids[i])
			continue;
		if (to_landmark_id[i] == -1)
			continue;
		to_ids[to_landmark_id[i]] = false;
	}
}


bool Permutation::variables_disjoint(const Permutation& p) const {
	const vector<int> &vars = p.vars_affected;
	for (size_t i=0; i < vars.size(); i++) {
		if (affected[vars[i]])
			return false;
	}
	return true;
}
