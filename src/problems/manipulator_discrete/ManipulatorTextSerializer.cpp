/** @file RockSampleTextSerializer.cpp
 *
 * Contains the implementations of the serialization methods for RockSample.
 */
#include "ManipulatorTextSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include "global.hpp"                     // for make_unique

#include "solver/abstract-problem/Action.hpp"

#include "solver/mappings/actions/enumerated_actions.hpp"
#include "ManipulatorAction.hpp"
#include "position_history.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace manipulator {


ManipulatorTextSerializer::ManipulatorTextSerializer(solver::Solver *solver) :
        Serializer(solver) {
}



void ManipulatorTextSerializer::saveAction(solver::Action const *action, std::ostream &os) {
    if (action == nullptr) {
        os << "NULL";
        return;
    }
    ManipulatorAction const &a = static_cast<ManipulatorAction const &>(*action);
    long bin_number = a.getBinNumber();
    os << bin_number << " ";
    for (auto &k: a.max_joint_inputs_) {
    	os << k << " ";
    }
    os << a.num_input_steps_ << " END";
}

std::unique_ptr<solver::Action> ManipulatorTextSerializer::loadAction(std::istream &is) {	
	std::vector<std::string> strings;
	std::string s;	
	long code;
	std::vector<double> max_joint_inputs;
	int num_input_steps;	
	is >> s;
	if (s == "NULL") {
		return nullptr;
	}
	
	while (s != "END") {		
		strings.push_back(s);
		is >> s;		
	}
	
	std::istringstream(strings[0]) >> code;	
	for (size_t i = 1; i < strings.size() - 1; i++) {
		double val;
		std::istringstream(strings[i]) >> val;
		max_joint_inputs.push_back(val);
	}
	
	std::istringstream(strings[strings.size() - 1]) >> num_input_steps;	
	return std::make_unique<ManipulatorAction>(code, max_joint_inputs, num_input_steps);
}

} /* namespace rocksample */
