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

#include "solver/mappings/actions/continuous_actions.hpp"

#include "ManipulatorAction.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace manipulator_continuous {

ManipulatorTextSerializer::ManipulatorTextSerializer(solver::Solver *solver, unsigned int &action_space_dim) :
        Serializer(solver),
        action_space_dim_(action_space_dim){
}

void ManipulatorTextSerializer::saveAction(solver::Action const *action, std::ostream &os) {
    if (action == nullptr) {
        os << "NULL";
        return;
    }
    ManipulatorAction const &a = static_cast<ManipulatorAction const &>(*action);
    std::vector<double> jv = a.getJointInputs();
    for (size_t i = 0; i < jv.size(); i++) {
        os << jv[i];        
        os << " ";        
    }
    os << "END";
    
}

std::unique_ptr<solver::Action> ManipulatorTextSerializer::loadAction(std::istream &is) {
	std::vector<double> action_vec;
	std::string s;
	is >> s;
	if (s == "NULL") {
		return nullptr;
	}
	
	while (s != "END") {
		double val;
		std::istringstream(s) >> val;
		action_vec.push_back(val);
		is >> s;
	}
	
	return std::make_unique<ManipulatorAction>(action_vec);
}

void ManipulatorTextSerializer::saveConstructionData(const ThisActionConstructionDataBase* baseData, std::ostream& os) {
	os << (baseData != nullptr) << " ";
	if (baseData != nullptr) {
		const ConstructionData& data = static_cast<const ConstructionData&>(*baseData);
		for (size_t i = 0; i < action_space_dim_; i++) {
			os << data[i] << " ";
		}
		os << "END";
	}
}

std::unique_ptr<ManipulatorTextSerializer::ThisActionConstructionDataBase> ManipulatorTextSerializer::loadConstructionData(std::istream& is) {	
	bool notNull;
	is >> notNull;
	if (notNull) {
		std::vector<double> input;
		std::string s;
		is >> s;
		while (s != "END") {
			double val;
			std::istringstream(s) >> val;
			input.push_back(val);
			is >> s;
		}
		
		return std::make_unique<ConstructionData>(input);
	} else {
		return nullptr;
	}
}

} /* namespace rocksample */
