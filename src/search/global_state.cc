#include "global_state.h"

#include "globals.h"
#include "utilities.h"
#include "state_registry.h"

#include <algorithm>
#include <iostream>
#include <cassert>
using namespace std;


GlobalState::GlobalState(const PackedStateBin *buffer_, const StateRegistry &registry_,
                         StateID id_)
    : buffer(buffer_),
      registry(&registry_),
      id(id_) {
    assert(buffer);
    assert(id != StateID::no_state);
}

GlobalState::~GlobalState() {
}

int GlobalState::operator[](size_t index) const {
    return g_state_packer->get(buffer, index);
}

std::string GlobalState::pddl_representation() {
	std::string s;
	for (size_t i = 0; i < g_variable_domain.size(); ++i) {
			const string &fact_name = g_fact_names[i][(*this)[i]];
			if (fact_name != "<none of those>")
				s += " "+ fact_name; 
	}
	return s; 
}


void GlobalState::dump_pddl() const {
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
        const string &fact_name = g_fact_names[i][(*this)[i]];
        if (fact_name != "<none of those>")
            cout << fact_name << endl;
    }
}

void GlobalState::dump_fdr() const {
    for (size_t i = 0; i < g_variable_domain.size(); ++i)
        cout << "  #" << i << " [" << g_variable_name[i] << "] -> "
             << (*this)[i] << endl;
}

std::string GlobalState::toString(int& num_of_props) 
{
	 num_of_props = 0;
	 std::string s = "";
	 for (size_t i = 0; i < g_variable_domain.size(); ++i) {
		 s += g_fact_names[i][(*this)[i]] + " ";
		 num_of_props++;
	 }
     
	return s;	
}


std::string GlobalState::toString() 
{
	 std::string s = "";
	 for (size_t i = 0; i < g_variable_domain.size(); ++i) {
		 s += g_fact_names[i][(*this)[i]] + " " + "\n";
	 }
     
	return s;	
}

