#include "util.h"

using std::string;
using std::cout;
using std::flush;
using std::endl;
using std::set;
using std::map;
using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::shared_ptr;
using std::ofstream;

void print_in_red(string str) 
{
	cout << "\033[1;31m" << str << "\033[0m\n";
}

void print_in_green(string str)
{
	cout << "\033[1;32m" << str << "\033[0m\n";
}

void print_in_blue(string str)
{
	cout << "\033[1;34m" << str << "\033[0m\n";
}


void debug(int index, const char* func = __PRETTY_FUNCTION__,  int line = __LINE__)
{
  cout << "\033[1;31mDEBUG_STATEMENT " << index << flush;
  cout << " in function "<< func << " at line " << line << " " << " \033[0m\n"<<flush << endl;
}

void print_set_of_operators(vector<GlobalOperator>& ops, string description)
{
	print_in_green("Begin" + description);
	int index = 0;

	for (auto& o: ops) {
		std::cout << "Operator" << index  << std::endl;
		o.dump();	
		++index;
	}	

	print_in_green("End" + description);
}

void print_set(std::set<int> s)
{
	for (auto& e : s) {
		std::cout << e << " "<< std::endl;	
	}	
}

void print_set_of_operators(vector<const GlobalOperator*>& ops, string description)
{
	print_in_green("Begin" + description);

	for (auto& o: ops) {
		o->dump();	
	}	

	print_in_green("End" + description);
}

void update_maximum(int& current_value, int& old_value) {
	if (current_value > old_value) 
		old_value = current_value;	
}

void print_vector(vector<int>& v)
{
	print_in_red("Begin vector");
	for (int i : v) {
		std::cout << i << std::endl;	
	}	
	print_in_red("End vector");
}


