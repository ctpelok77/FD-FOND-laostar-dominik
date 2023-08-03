#ifndef GLOBAL_OPERATOR_H
#define GLOBAL_OPERATOR_H

#include "global_state.h"

#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

struct GlobalCondition {
    int var;
    int val;
    explicit GlobalCondition(std::istream &in);
    GlobalCondition(int variable, int value);

    bool is_applicable(const GlobalState &state) const {
        return state[var] == val;
    }

    bool operator==(const GlobalCondition &other) const {
        return var == other.var && val == other.val;
    }

    bool operator!=(const GlobalCondition &other) const {
        return !(*this == other);
    }

    bool operator<(const GlobalCondition &other) const {
    	return (var < other.var) || (var ==  other.var && val < other.val);
    }
    void dump() const;
};

struct GlobalEffect {
    int var;
    int val;
    std::vector<GlobalCondition> conditions;
    explicit GlobalEffect(std::istream &in);
    GlobalEffect(int variable, int value, const std::vector<GlobalCondition> &conds);

    bool does_fire(const GlobalState &state) const {
        for (size_t i = 0; i < conditions.size(); ++i)
            if (!conditions[i].is_applicable(state))
                return false;
        return true;
    }

    void dump() const;
};

class GlobalOperator {
    bool is_an_axiom;
    std::vector<GlobalCondition> preconditions;
    // These effects correspond to the effects of a nondeterministic operator for FOND planning!
    std::vector<std::vector<GlobalEffect> > nd_effects;
    

    mutable bool marked; // Used for short-term marking of preferred operators
    void read_pre_post(std::istream &in, std::vector<GlobalEffect>& effects);
    void read_nd_pre_post(std::istream &in);

    // For non-deterministic version
    bool is_in_precondition(int var, int val) const;

public:
    std::string name;
	int index;
    int cost;
    explicit GlobalOperator(std::istream &in, bool is_axiom, int index);
    void dump() const;
    void dump_effects(const std::vector<GlobalEffect>& eff) const;

    const std::string &get_name() const {return name; }

    bool is_axiom() const {return is_an_axiom; }

    const std::vector<GlobalCondition> &get_preconditions() const {return preconditions; }
    const std::vector<GlobalEffect> &get_effects() const {
		std::cout << "get_effects" << std::flush << std::endl;
		throw std::runtime_error("This function is not implemented");
		return nd_effects[0]; 
	}
    const std::vector<std::vector<GlobalEffect> > &get_non_deterministic_effects() const {return nd_effects; }
    bool is_applicable(const GlobalState &state) const {
        for (size_t i = 0; i < preconditions.size(); ++i)
            if (!preconditions[i].is_applicable(state))
                return false;
        return true;
    }

	bool is_truly_nondeterministic() const;
	bool is_deterministic() const;
	size_t degree() const;
    bool is_marked() const {
        return marked;
    }
    void mark() const {
        marked = true;
    }
    void unmark() const {
        marked = false;
    }

    int get_cost() const {return cost; }

    friend std::ostream& operator<< (std::ostream& out, const GlobalOperator& op);
};

#endif
