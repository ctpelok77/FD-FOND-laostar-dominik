#include "relaxation_heuristic.h"

#include "global_operator.h"
#include "global_state.h"
#include "globals.h"
#include "task_proxy.h"
#include "utilities.h"
#include "utilities_hash.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <unordered_map>
#include <vector>

using namespace std;

// construction and destruction
RelaxationHeuristic::RelaxationHeuristic(const Options &opts)
    : Heuristic(opts) {
}

RelaxationHeuristic::~RelaxationHeuristic() {
}

bool RelaxationHeuristic::dead_ends_are_reliable() const {
    return !has_axioms();
}

// initialization
void RelaxationHeuristic::initialize() {
    // Build propositions.
    int prop_id = 0;
    VariablesProxy variables = task_proxy.get_variables();
    propositions.resize(variables.size());
    for (FactProxy fact : variables.get_facts()) {
//    	std::cout << "Fact: var=" << fact.get_variable().get_id() << ", value=" <<  propositions[fact.get_variable().get_id()].size() << ", proposition id: "<< prop_id<<std::endl;
        propositions[fact.get_variable().get_id()].push_back(Proposition(prop_id++));
    }

    // Build goal propositions.
    for (FactProxy goal : task_proxy.get_goals()) {
        Proposition *prop = get_proposition(goal);
//    	std::cout << "Goal: var=" << goal.get_variable().get_id() << ", value=" <<  goal.get_value() << ", proposition id: "<< prop->id<< std::endl;
        prop->is_goal = true;
        goal_propositions.push_back(prop);
    }

    // Build unary operators for operators and axioms.
    int op_no = 0;
    for (OperatorProxy op : task_proxy.get_operators())
        build_unary_operators(op, op_no++);
    for (OperatorProxy axiom : task_proxy.get_axioms())
        build_unary_operators(axiom, -1);

    // Simplify unary operators.
    simplify();

    // Cross-reference unary operators.
    for (size_t i = 0; i < unary_operators.size(); ++i) {
        UnaryOperator *op = &unary_operators[i];
//  	std::cout << "Simplified unary operator " << i << ": " << op->operator_no << ", eff " << op->effect->id << ", pre " << std::flush;
        for (size_t j = 0; j < op->precondition.size(); ++j) {
//        	std::cout << op->precondition[j]->id << " ";
            op->precondition[j]->precondition_of.push_back(op);
        }
 //       std::cout << std::endl;
    }
}

Proposition *RelaxationHeuristic::get_proposition(const FactProxy &fact) {
    int var = fact.get_variable().get_id();
    int value = fact.get_value();
    assert(in_bounds(var, propositions));
    assert(in_bounds(value, propositions[var]));
    return &propositions[var][value];
}

int RelaxationHeuristic::build_unary_operators(const OperatorProxy &op, int op_no) {
//	std::cout << "Creating unary operators for operator " << op.get_id() << std::endl;
    int base_cost = op.get_cost();
    vector<Proposition *> precondition_props;
    for (FactProxy precondition : op.get_preconditions()) {
        precondition_props.push_back(get_proposition(precondition));
    }
    // Note: The following code is a nasty hack for the non-deterministic tasks,
    // that brakes the applicability to abstract tasks.
    const vector<vector<GlobalEffect> > &nd_effs = op.get_global_operator()->get_non_deterministic_effects();

    for (size_t i=0; i < nd_effs.size(); ++i) {
    	// For each possible outcome we create unary operators as for deterministic operators
    	const vector<GlobalEffect>& effs = nd_effs[i];
        for (size_t j=0; j < effs.size(); ++j) {
        	const GlobalEffect &effect = effs[j];

            Proposition *effect_prop = &propositions[effect.var][effect.val];
            for (size_t c=0; c < effect.conditions.size(); ++c) {
            	const GlobalCondition &eff_cond = effect.conditions[c];
                precondition_props.push_back(&propositions[eff_cond.var][eff_cond.val]);
            }
            unary_operators.push_back(UnaryOperator(precondition_props, effect_prop, op_no, base_cost));
//            std::cout << "Created unary operator for operator " << op_no << ", effect " << effect_prop->id << ", non deterministic effect number " << i << ", effect " << j << std::endl;
            precondition_props.erase(precondition_props.end() - effect.conditions.size(), precondition_props.end());
        }
    }
    return nd_effs.size();
}

void RelaxationHeuristic::simplify() {
    // Remove duplicate or dominated unary operators.

    /*
      Algorithm: Put all unary operators into an unordered_map
      (key: condition and effect; value: index in operator vector.
      This gets rid of operators with identical conditions.

      Then go through the unordered_map, checking for each element if
      none of the possible dominators are part of the unordered_map.
      Put the element into the new operator vector iff this is the case.

      In both loops, be careful to ensure that a higher-cost operator
      never dominates a lower-cost operator.
    */


    cout << "Simplifying " << unary_operators.size() << " unary operators..." << flush;

    typedef pair<vector<Proposition *>, Proposition *> HashKey;
    typedef unordered_map<HashKey, int> HashMap;
    HashMap unary_operator_index;
    unary_operator_index.reserve(unary_operators.size());

    for (size_t i = 0; i < unary_operators.size(); ++i) {
        UnaryOperator &op = unary_operators[i];
        sort(op.precondition.begin(), op.precondition.end(),
             [] (const Proposition * p1, const Proposition * p2) {
                 return p1->id < p2->id;
             }
             );
        HashKey key(op.precondition, op.effect);
        pair<HashMap::iterator, bool> inserted = unary_operator_index.insert(
            make_pair(key, i));
        if (!inserted.second) {
            // We already had an element with this key; check its cost.
            HashMap::iterator iter = inserted.first;
            int old_op_no = iter->second;
            int old_cost = unary_operators[old_op_no].base_cost;
            int new_cost = unary_operators[i].base_cost;
            if (new_cost < old_cost)
                iter->second = i;
            assert(unary_operators[unary_operator_index[key]].base_cost ==
                   min(old_cost, new_cost));
        }
    }

    vector<UnaryOperator> old_unary_operators;
    old_unary_operators.swap(unary_operators);

    for (HashMap::iterator it = unary_operator_index.begin();
         it != unary_operator_index.end(); ++it) {
        const HashKey &key = it->first;
        int unary_operator_no = it->second;
        int powerset_size = (1 << key.first.size()) - 1; // -1: only consider proper subsets
        bool match = false;
        if (powerset_size <= 31) { // HACK! Don't spend too much time here...
            for (int mask = 0; mask < powerset_size; ++mask) {
                HashKey dominating_key = make_pair(vector<Proposition *>(), key.second);
                for (size_t i = 0; i < key.first.size(); ++i)
                    if (mask & (1 << i))
                        dominating_key.first.push_back(key.first[i]);
                HashMap::iterator found = unary_operator_index.find(
                    dominating_key);
                if (found != unary_operator_index.end()) {
                    int my_cost = old_unary_operators[unary_operator_no].base_cost;
                    int dominator_op_no = found->second;
                    int dominator_cost = old_unary_operators[dominator_op_no].base_cost;
                    if (dominator_cost <= my_cost) {
                        match = true;
                        break;
                    }
                }
            }
        }
        if (!match)
            unary_operators.push_back(old_unary_operators[unary_operator_no]);
    }

    sort(unary_operators.begin(), unary_operators.end(),
         [&] (const UnaryOperator &o1, const UnaryOperator &o2) {
            if (o1.operator_no != o2.operator_no)
                return o1.operator_no < o2.operator_no;
            if (o1.effect != o2.effect)
                return o1.effect->id < o2.effect->id;
            if (o1.base_cost != o2.base_cost)
                return o1.base_cost < o2.base_cost;
            return lexicographical_compare(o1.precondition.begin(), o1.precondition.end(),
                                           o2.precondition.begin(), o2.precondition.end(),
                                           [] (const Proposition *p1, const Proposition *p2) {
                return p1->id < p2->id;
            });
        });

    cout << " done! [" << unary_operators.size() << " unary operators]" << endl;
}
