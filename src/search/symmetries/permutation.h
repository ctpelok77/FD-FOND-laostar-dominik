#ifndef SYMMETRIES_PERMUTATION_H
#define SYMMETRIES_PERMUTATION_H
#include "../global_state.h"
#include <vector>
#include <string>
#include <fstream>
#include "../landmarks/landmark_graph.h"

class Permutation{
public:
    Permutation();
    Permutation(const unsigned int*);
    Permutation(const Permutation&, bool invert=false);
    Permutation(const Permutation& perm1, const Permutation& perm2);
    ~Permutation();

    Permutation& operator =(const Permutation&);
    bool operator ==(const Permutation&) const;
    bool operator <( const Permutation& other ) const {
    	for (unsigned int i = 0; i < length; i++) {
            if (get_value(i) < other.get_value(i)) return true;
            if (get_value(i) > other.get_value(i)) return false;
        }
    	return false;
    };

    bool identity() const;
    bool is_object_symmetry() const;

    // Michael: version for the trace_back
    void permute_state(const GlobalState& from_state, PackedStateBin* to_state);
    void permute_state_unpacked(const GlobalState& from_state, int* to_state);
    void print_cycle_notation() const;
    std::string get_cycle_notation_patrik() const;
    void get_cycle_notation_indexes(std::vector<std::vector<std::pair<int, int> > >& cycles) const;
	std::string get_cycle_notation() const;
    void set_value(unsigned int ind, unsigned int val);
    unsigned int get_value(unsigned int ind) const { return value[ind]; }
    unsigned int get_inverse_value(unsigned int ind) const { return inverse_value[ind]; }
    void dump() const;
	void dump_landmark_ids() const;

    static unsigned int length;
    static unsigned int num_variable_and_value_nodes;

    static std::vector<int> var_by_val;
    static std::vector<int> dom_sum_by_var;

    static int get_var_by_index(const unsigned int val);
    static std::pair<int, int> get_var_val_by_index(const unsigned int ind);
    static unsigned int get_index_by_var_val_pair(const int var, const int val);

    std::pair<int, int> get_new_var_val_by_old_var_val(const int var, const int val) const;
    std::pair<int, int> get_old_var_val_by_new_var_val(const int var, const int val) const;

    bool replace_if_less(int*);

	//For Landmarks
	void set_landmarks_connections(LandmarkGraph &lm_graph);
	void permute_landmark_ids(const std::vector<bool>& from_ids, std::vector<bool>& to_ids) const;
    const std::vector<int>& get_landmark_ids() const { return to_landmark_id; }

    // For faster canonical computation
    bool variables_disjoint(const Permutation& p) const;

	// Used for removing the global variable
	static bool reverse_lexicographical_state_comparison;
	static bool use_landmarks;
private:
	unsigned int* value;
	unsigned int* inverse_value;
    std::vector<int> vars_affected;
    std::vector<bool> affected;
    bool borrowed_buffer;
    // Need to keep the connection between affected vars, ie which var goes into which.
    std::vector<int> from_vars;
//    PackedStateBin* buff_for_state_copy;
    // Affected vars by cycles
    std::vector<std::vector<int> > affected_vars_cycles;
	// Keeping connections between Landmark ids.
    std::vector<int> to_landmark_id;


    void set_affected(unsigned int ind, unsigned int val);

    void finalize();
    void _allocate();
    void _deallocate();
    void _copy_value_from_permutation(const Permutation&);
    void _inverse_value_from_permutation(const Permutation &perm);

    void _copy_landmarks_ids_from_permutation(const Permutation &perm);
    void _inverse_landmarks_ids_from_permutation(const Permutation &perm);

    bool replace_if_less_regular(int* state);
    bool replace_if_less_reverse(int* state);

    bool is_permutation_of(const LandmarkNode* from_node, const LandmarkNode* to_node);
};

#endif
