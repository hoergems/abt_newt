#include "ManipulatorSerializer.hpp"

#include <iostream>                     // for operator<<, basic_ostream, basic_istream<>::__istream_type, basic_ostream<>::__ostream_type, endl
#include <string>                       // for operator>>, string
#include <vector>                       // for vector

#include <fstream>

#include <sstream>

#include "problems/shared/ManipulatorObservation.hpp"    
#include "problems/shared/ManipulatorState.hpp"  

namespace shared{
void saveVector(std::vector<long> values, std::ostream &os) {
    os << "(";
    for (auto it = values.begin(); it != values.end(); it++) {
        os << *it;
        if ((it + 1) != values.end()) {
            os << ", ";
        }
    }
    os << ")";
}

std::vector<long> loadVector(std::istream &is) {
    std::vector<long> values;
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ')');
    std::istringstream sstr(tmpStr);
    while (std::getline(sstr, tmpStr, ',')) {
        long value;
        std::istringstream(tmpStr) >> value;
        values.push_back(value);
    }
    return values;
}

ManipulatorSerializer::ManipulatorSerializer(solver::Solver *solver) :
        Serializer(solver) {
}

void ManipulatorSerializer::saveState(solver::State const *state, std::ostream &os) {    
    if (state == nullptr) {
        os << "NULL";
        return;
    }    
    shared::ManipulatorState const &manipulatorState = static_cast<shared::ManipulatorState const &>(*state);    
    manipulatorState.serialize(os);
}

std::unique_ptr<solver::State> ManipulatorSerializer::loadState(std::istream &is) {	
	std::vector<std::string> strings;
	std::string s;
	is >> s;
	if (s == "NULL") {
		return nullptr;
	}
	while (s != "END") {
		strings.push_back(s);
		is >> s;		
	}	
	
	std::vector<double> joint_values;
	std::vector<double> ee_position;	
	for (size_t i = 0; i < strings.size() - 3; i++) {
		double val;
		std::istringstream(strings[i]) >> val;
		joint_values.push_back(val);
	}
	
	for (size_t i = strings.size() - 3; i < strings.size(); i++) {
		double val;
		std::istringstream(strings[i]) >> val;
		ee_position.push_back(val);
	}
	
	JointValues jv(joint_values);	
	return std::make_unique<shared::ManipulatorState>(jv, ee_position);
}

void ManipulatorSerializer::saveObservation(solver::Observation const *obs, std::ostream &os) {    
    if (obs == nullptr) {
        os << "NULL";
        return;
    }
    shared::ManipulatorObservation const &o = static_cast<shared::ManipulatorObservation const &>(*obs);
    o.serialize(os);    
}

std::unique_ptr<solver::Observation> ManipulatorSerializer::loadObservation(std::istream &is) {
	std::vector<double> strings;
    std::string s;
    is >> s;
    if (s == "NULL") {
    	return nullptr;
    }
    while (s != "END") {
    	double val;
    	std::istringstream(s) >> val;
    	strings.push_back(val);
    	is >> s;    			
    }    
    return std::make_unique<shared::ManipulatorObservation>(strings);    
}


}
