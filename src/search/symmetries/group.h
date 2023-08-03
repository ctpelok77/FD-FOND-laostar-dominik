#ifndef SYMMETRIES_GROUP_H
#define SYMMETRIES_GROUP_H
#include <vector>
#include "permutation.h"
#include "../global_state.h"
#include "../option_parser.h"
#include "../per_state_information.h"


using namespace std;
using namespace __gnu_cxx;
typedef std::vector<short int> Trace;
typedef std::vector<std::vector<int> > GroupDivision; // each entry consists of the indices of the generators that form this commutative subgroup
typedef std::vector<Permutation> Gens; // the generators for the automorphism


class Group{
public:
    Group(const Options &opts);
    ~Group();
    void initialize();

    static void add_permutation(void*, unsigned int, const unsigned int *);
    void add_generators_powers();
    void create_whole_group();

    void set_landmark_ids_for_generators(LandmarkGraph &lm_graph);
    void get_landmark_stable_ids(vector<int>& ids);

    // Direct product creation methods.
    void default_direct_product();

    void add_generator(Permutation gen);
    void add_identity_generator(const unsigned int * full_perm);
    int get_num_generators() const;
    void dump_generators() const;
    void dump_generators_cycles() const;

    Permutation compose_permutation(Trace&) const;
    PackedStateBin* get_canonical_state(const GlobalState& state);
    PackedStateBin* get_canonical_state_bfs(const GlobalState& state);
    PackedStateBin* get_canonical_state_one_pass(const GlobalState& state);
    void get_trace(const GlobalState& state, Trace& full_trace);
    void get_trace_one_pass(const GlobalState& state, Trace& full_trace);
    void get_trace_bfs(const GlobalState& state, Trace& full_trace);

    void free_memory();

    void get_operators_equivalence_relation(vector<vector<size_t> >& equiv) const;

    void dump_operators_equivalence(const vector<vector<size_t> >& equiv) const;
    int count_redundant_operators_equivalence(const vector<vector<size_t> >& equiv) const;
    static int get_num_identity_generators() {return num_identity_generators;}

    int get_generators_orders(vector<int>& orders) const;
    int get_number_object_symmetries() const;
    static int store_identity_generators;
    static unsigned int get_operator_node_index(int op_no);
    static unsigned int get_op_no_by_index(unsigned int index);

    const GlobalOperator& get_operator_permutation(const GlobalOperator* op, const Permutation& perm) const;

private:
    Gens generators;
    vector<vector<pair<size_t, size_t> > > identity_generators;
	GroupDivision subgrpoups;
    bool use_independent_variables_decomposition;
    bool generate_whole_group;

    PackedStateBin* packed_state;
    int* unpacked_state;
    int* permuted_state;
    static bool safe_to_add_generators;

    static int num_identity_generators;
    static int stop_after_false_generated;
//    static std::vector<unsigned int> operator_node_index;

    class bfs_node_info {
    public:
		StateID parent;
		bool visited;
		int creating_generator;
    	bfs_node_info() : parent(StateID::no_state), visited(false), creating_generator(-1) {
    	}
    };

    PerStateInformation<bfs_node_info> bfs_info;
    StateID best_current;

    const Permutation& get_permutation(int) const;
    void copy_buff(int* s1, const GlobalState& s2) const;
    bool copy_buffer_if_less(int* s1, const int* s2) const;
    void pack_state(PackedStateBin* s1, const int* s2) const;
    void pack_state(PackedStateBin* buffer, const GlobalState& s2) const;

    bool is_commutative(const Permutation& gen1, const Permutation& gen2) const;
    void group_division() const;
    void group_independent_variables_decomposition();
    PackedStateBin* get_canonical_state_independent_vars(const GlobalState& state);
    PackedStateBin* get_canonical_state_ehc(const GlobalState& state);
    void get_trace_independent_vars(const GlobalState& state, Trace& full_trace);

};

#endif
