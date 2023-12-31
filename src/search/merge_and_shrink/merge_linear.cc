#include "merge_linear.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../utilities.h"

#include <cassert>
#include <cstdlib>

using namespace std;

MergeLinear::MergeLinear(const Options &opts)
    : MergeStrategy(),
      order(VariableOrderType(opts.get_enum("variable_order"))),
      need_first_index(true) {
}

pair<int, int> MergeLinear::get_next(const vector<TransitionSystem *> &all_transition_systems) {
    assert(!done() && !order.done());

    int first;
    if (need_first_index) {
        need_first_index = false;
        first = order.next();
        cout << "First variable: " << first << endl;
    } else {
        // The most recent composite transition system is appended at the end of
        // all_transition_systems in merge_and_shrink.cc
        first = all_transition_systems.size() - 1;
    }
    int second = order.next();
    cout << "Next variable: " << second << endl;
    assert(all_transition_systems[first]);
    assert(all_transition_systems[second]);
    --remaining_merges;
    if (done() && !order.done()) {
        cerr << "Variable order finder not done, but no merges remaining" << endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    return make_pair(first, second);
}

void MergeLinear::dump_strategy_specific_options() const {
    cout << "Linear merge strategy: ";
    order.dump();
}

string MergeLinear::name() const {
    return "linear";
}

static MergeStrategy *_parse(OptionParser &parser) {
    vector<string> merge_strategies;
    merge_strategies.push_back("CG_GOAL_LEVEL");
    merge_strategies.push_back("CG_GOAL_RANDOM");
    merge_strategies.push_back("GOAL_CG_LEVEL");
    merge_strategies.push_back("RANDOM");
    merge_strategies.push_back("LEVEL");
    merge_strategies.push_back("REVERSE_LEVEL");
    parser.add_enum_option("variable_order", merge_strategies,
                           "the order in which atomic transition systems are merged",
                           "CG_GOAL_LEVEL");

    Options opts = parser.parse();
    if (parser.dry_run())
        return 0;
    else
        return new MergeLinear(opts);
}

static Plugin<MergeStrategy> _plugin("merge_linear", _parse);
