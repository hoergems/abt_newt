/** @file ManipulatorAction.cpp
 *
 * Contains the implementations for the methods of the RockSampleAction class.
 */
#include "ManipulatorAction.hpp"

#include <cstddef>                      // for size_t
#include <cstdint>
#include <math.h>

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <iostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "problems/shared/GridPosition.hpp"  // for GridPosition, operator==, operator<<
#include "solver/abstract-problem/State.hpp"             // for State

using std::cout;
using std::endl;

namespace manipulator_continuous {

std::unique_ptr<solver::Action> ManipulatorAction::copy() const {
    return std::make_unique<ManipulatorAction>(storage);
}

double ManipulatorAction::distanceTo(solver::Action const &other) const {
    ManipulatorAction const &otherAction = static_cast<ManipulatorAction const &>(other);
    const std::vector<double> jv = getJointInputs();
    const std::vector<double> jv_other = otherAction.getJointInputs();
    double distance = 0;
    for (size_t i = 0; i < jv.size(); i++) {
    	distance += std::pow(jv[i] - jv_other[i], 2);
    }
    return std::sqrt(distance);
}
    
bool ManipulatorAction::equals(const Point &otherPoint) const {
    	ManipulatorAction const &otherAction = static_cast<ManipulatorAction const &>(otherPoint);
    	const std::vector<double> jv = getJointInputs();
        const std::vector<double> jv_other = otherAction.getJointInputs();
        for (size_t i = 0; i < jv.size(); i++) {
            if (jv[i] != jv_other[i]) {
            	return false;
            }
        }
        return true;
}

std::size_t ManipulatorAction::hash() const {
    std::size_t hashValue = 0;
    std::vector<double> jv = getJointInputs();
    for (auto &k: jv) {
    	tapir::hash_combine(hashValue, k);
    }  
    return hashValue;
}

void ManipulatorAction::print(std::ostream &os) const {
    std::vector<double> jv = getJointInputs();
    os << "action input {(";
    for (size_t i = 0; i < jv.size(); i++) {
    	os << jv[i];
    	if (i < jv.size()) {
    		os << ", ";
    	}
    }
    os << ")}";    
}

std::vector<double> ManipulatorAction::getJointInputs() const {
    return storage.asVector();
}

} /* namespace manipulator */
