#include "global_operator.h"

#include "globals.h"
#include "utilities.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <set>

using namespace std;

static void check_fact(int var, int val) {
    if (!in_bounds(var, g_variable_domain)) {
        cerr << "Invalid variable id: " << var << endl;
        exit_with(EXIT_INPUT_ERROR);
    }
    if (val < 0 || val >= g_variable_domain[var]) {
        cerr << "Invalid value for variable " << var << ": " << val << endl;
        exit_with(EXIT_INPUT_ERROR);
    }
}

GlobalCondition::GlobalCondition(istream &in) {
    in >> var >> val;
    check_fact(var, val);
}

GlobalCondition::GlobalCondition(int variable, int value)
    : var(variable),
      val(value) {
    check_fact(var, val);
}

GlobalEffect::GlobalEffect(int variable, int value, const vector<GlobalCondition> &conds)
    : var(variable),
      val(value),
      conditions(conds) {
    check_fact(var, val);
}

// Michael: Reading non-deterministic pre_post
void GlobalOperator::read_nd_pre_post(istream &in) {
    int count;
    in >> count;
	std::vector<GlobalEffect> effects;
    for (int i = 0; i < count; ++i)
    	read_pre_post(in, effects);

    nd_effects.push_back(effects);
}

void GlobalOperator::read_pre_post(istream &in, std::vector<GlobalEffect>& effects) {
    int cond_count, var, pre, post;
    in >> cond_count;
    vector<GlobalCondition> conditions;
    conditions.reserve(cond_count);
    for (int i = 0; i < cond_count; ++i)
        conditions.push_back(GlobalCondition(in));
    in >> var >> pre >> post;
    if (pre != -1)
        check_fact(var, pre);
    check_fact(var, post);
    if (pre != -1)
        preconditions.push_back(GlobalCondition(var, pre));
    effects.push_back(GlobalEffect(var, post, conditions));
}

GlobalOperator::GlobalOperator(istream &in, bool axiom, int index): index(index)
{
    marked = false;

    is_an_axiom = axiom;
    if (!is_an_axiom) {
        check_magic(in, "begin_operator");
        in >> ws;
        getline(in, name);
        int count;
        in >> count;
        for (int i = 0; i < count; ++i)
            preconditions.push_back(GlobalCondition(in));
        in >> count;


        for (int i = 0; i < count; ++i)
            read_nd_pre_post(in);

        int op_cost;
        in >> op_cost;
        cost = g_use_metric ? op_cost : 1;

        g_min_action_cost = min(g_min_action_cost, cost);
        g_max_action_cost = max(g_max_action_cost, cost);
		
        check_magic(in, "end_operator");
    } else {
        name = "<axiom>";
        cost = 0;
        check_magic(in, "begin_rule");
        read_nd_pre_post(in);
        check_magic(in, "end_rule");
    }
    
	set<GlobalCondition> s( preconditions.begin(), preconditions.end() );
    preconditions.assign( s.begin(), s.end() );
}

bool GlobalOperator::is_truly_nondeterministic() const {
	if (is_deterministic()) return false; 
	int num_nontrivial_outcomes = 0;
	for (auto& eff : nd_effects) {
		bool is_nontrivial = false;				
		if (!eff.empty()) {
		   	++num_nontrivial_outcomes; 
		}
	}

	if (num_nontrivial_outcomes >= 2) return true;
	return false;
}

bool GlobalOperator::is_in_precondition(int var, int val) const {
	for (size_t i = 0; i < preconditions.size(); ++i) {
		if (var == preconditions[i].var && val == preconditions[i].val)
			return true;
	}
	return false;
}

bool GlobalOperator::is_deterministic() const {
	if (get_non_deterministic_effects().size() > 1) {
		return false;	
	}		
	return true;
}

size_t GlobalOperator::degree() const {
	return nd_effects.size();	
} 

void GlobalCondition::dump() const {
    cout << g_variable_name[var] << ": " << val;
}

void GlobalEffect::dump() const {
    cout << g_variable_name[var] << ":= " << val;
    if (!conditions.empty()) {
        cout << " if";
        for (size_t i = 0; i < conditions.size(); ++i) {
            cout << " ";
            conditions[i].dump();
        }
    }
}

void GlobalOperator::dump() const {
    cout << name << ":";
    for (size_t i = 0; i < preconditions.size(); ++i) {
        cout << " [";
        preconditions[i].dump();
        cout << "]";
    }
    if (nd_effects.size() == 0) {
        cout << endl;
        return;
    }
	
    for (size_t i = 0; i < nd_effects.size(); i++) {
		cout << " {";
        dump_effects(nd_effects[i]);
		cout << "}";
    }
    cout << endl;
}

void GlobalOperator::dump_effects(const std::vector<GlobalEffect>& eff) const {
	for (size_t i = 0; i < eff.size(); i++) {
		cout << " [";
		eff[i].dump();
		cout << "]";
	}
}

std::ostream& operator<< (std::ostream& out, const GlobalOperator& op) {
	// Dumping operator in a deterministic setting - getting the first determinization only
	if (op.is_axiom()) {
		return out;
	}
	out << "begin_operator" << std::endl;
	out << op.get_name() << std::endl;

	std::vector<bool> affected_variables(g_variable_domain.size(), false);
	std::vector<int> affected_variable_value(g_variable_domain.size(), -1);
	for (auto e : op.get_effects()) {
		affected_variables[e.var] = true;
	}
	std::vector<GlobalCondition> prevail;

	for (auto cond : op.get_preconditions()) {
		if (affected_variables[cond.var])
			affected_variable_value[cond.var] = cond.val;
		else
			prevail.push_back(cond);
	}
	// Dumping prevail
	out << prevail.size() << endl;
	for (auto cond : prevail) {
    	out << cond.var  << " " << cond.val << std::endl;
	}
	// Dumping prepost
	out << op.get_effects().size() << endl;
	for (auto e : op.get_effects()) {
		// Dumping condition first
    	out << e.conditions.size() << " ";
    	for (auto cond : e.conditions) {
        	out << cond.var  << " " << cond.val << " ";
    	}
		// Dumping effect
    	out << e.var << " " << affected_variable_value[e.var] << " " << e.val << std::endl;
	}

	if (g_use_metric)
    	out << op.get_cost() << std::endl;
	else
    	out << 0 << std::endl;

	out << "end_operator" << std::endl;
	return out;
}
