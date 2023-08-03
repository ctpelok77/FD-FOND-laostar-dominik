/**
 * This bunch of functions makes live a bit easier when debugging  
 * and also provides functions to visualize the state space, operators etc
 * 
 */
#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <string>
#include <set>
#include <map>
#include <limits>
#include <memory>
#include <algorithm>
#include <fstream>

#include "../global_operator.h"

#ifndef INFINITY
#define INFINITY numeric_limits<double>::infinity()
#endif
#define _ARGS  __PRETTY_FUNCTION__, __LINE__

void print_in_red(std::string str);
void print_in_green(std::string str);
void print_in_green(std::string str);
void print_set_of_operators(std::vector<GlobalOperator>& ops, std::string description);
void print_set_of_operators(std::vector<const GlobalOperator*>& ops, std::string description);
void debug(int index, const char* func, int line);
void update_maximum(int& current_value, int& old_value);
void print_vector(std::vector<int>& v);
//void dump_unaries(const std::vector<UnaryOperator *> vec);

/**
 * Checks whether a given pointer points to NULL
 */
template<typename T>
void nullPointer(T pointer, const char* func, int line) {
   if (pointer == NULL) {
	   std::cout << "\033[1;31mPOINTER NULL" << std::flush;
	   std::cout << " in function "<< func << " at line " << line << " " << " \033[0m\n"<< std::flush << std::endl;

   }
   else {
	   std::cout << "\n \033[1;32mPOINTER NOT NULL" << std::flush;
	   std::cout << " in function "<< func << " at line " << line << " " << " \033[0m\n"<< std::flush << std::endl;
   } 
}

template<typename T>
void printValue(T var, std::string s , const char* func, int line) {
	std::cout << "\033[1;34mValue of " << s << "=" << var << std::flush;
	std::cout << " in function "<< func << " at line "<<" "<< line << " " << " \033[0m\n"<< std::flush << std::endl;

}
/**
 * Check set membership
 */
template<typename T> 
bool contains(std::set<T> container, T element) {
  return container.find(element) != container.end();
}

/**
 * Check set membership
 */
template<typename T> 
bool contains(std::vector<T> container, T element) {
  return std::find(container.begin(), container.end(), element) != container.end();
}

/**
 * Check membership in map
 */
template<typename T,typename U, typename Y> 
bool containsKey(std::map<T,U,Y> container, T element) {
  return container.find(element) != container.end();
}
void print_set(std::set<int> s);

#endif
