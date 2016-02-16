/** @file RockSampleState.cpp
 *
 * Contains the implementation for the methods of RockSampleState.
 */
#include "ManipulatorState.hpp"

#include <cstddef>                      // for size_t

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <iostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"
#include "solver/abstract-problem/State.hpp"             // for State

using std::cout;
using std::endl;

namespace shared {
ManipulatorState::ManipulatorState(JointValues values, boost::shared_ptr<shared::Robot> &robot):
    solver::Vector(),    
    jointValues_(values),
    end_effector_position_(),
	weight_(1.0),
	previous_state_(nullptr){
	std::vector<double> ee_pos;
	std::vector<double> joint_angles;
	std::vector<double> jv_vec = jointValues_.asVector();
	for (size_t i = 0; i < jv_vec.size() / 2; ++i) {
	    joint_angles.push_back(jv_vec[i]);
	}
	
	robot->getEndEffectorPosition(joint_angles, ee_pos);
	/**if (!ee_pos[0]) {
		cout << "AHHHHHHHHHHHHHHHHHHHHHH" << endl;
	}*/
	end_effector_position_ = std::make_shared<std::vector<double>>(ee_pos);
}

ManipulatorState::ManipulatorState(JointValues values, std::vector<double> ee_position):
    solver::Vector(),
    jointValues_(values),
    end_effector_position_(std::make_shared<std::vector<double>>(ee_position)),
	weight_(1.0),
	previous_state_(nullptr){
    
}

ManipulatorState::ManipulatorState(JointValues values, std::vector<double> ee_position, double weight):
	solver::Vector(),
	jointValues_(values),
	end_effector_position_(std::make_shared<std::vector<double>>(ee_position)),
	weight_(weight),
	previous_state_(nullptr){
	
}

std::unique_ptr<solver::Point> ManipulatorState::copy() const {	
	return std::make_unique<ManipulatorState>(jointValues_, *(end_effector_position_), weight_);
}

double ManipulatorState::distanceTo(solver::State const &otherState) const {
    
    ManipulatorState const &otherManipulatorState =
        static_cast<ManipulatorState const &>(otherState);
    double distance = jointValues_.euclideanDistanceTo(
                otherManipulatorState.jointValues_);    
    return distance;
}

std::vector<double> ManipulatorState::asVector() const {    
    return jointValues_.asVector();    
}

std::shared_ptr<std::vector<double> > ManipulatorState::getEndEffectorPosition() const {
    return end_effector_position_;
}

bool ManipulatorState::equals(solver::State const &otherState) const {
    ManipulatorState const &otherManipulatorState =
        static_cast<ManipulatorState const &>(otherState);
    std::vector<double> jointValues = jointValues_.asVector();
    std::vector<double> otherValues = otherManipulatorState.getJointValues().asVector();    
    for (size_t i = 0; i < jointValues.size(); i++) {
        if (round_(jointValues[i], roundingPrecision_) != round_(otherValues[i], roundingPrecision_)) {
            return false;            
        }
    }
    return true;
    /**if (hash() == otherManipulatorState.hash()) {
        return true;    
    }   
    return false;*/
}

double ManipulatorState::round_(const double &value, const int &precision) const {
    double ceil_value = ceil(value*pow(10, precision)) / pow(10, precision);
    double floor_value = floor(value*pow(10, precision)) / pow(10, precision);    
    if (fabs(value - floor_value) < fabs(ceil_value - value)) {
        return floor_value;
    }
    return ceil_value;
    
    
}

std::size_t ManipulatorState::hash() const {
    std::size_t hashValue = 0;
    std::vector<double> jointValues = jointValues_.asVector();
    for (size_t i = 0; i < jointValues_.size(); i++) {
        tapir::hash_combine(hashValue, round_(jointValues[i], roundingPrecision_));
    }        
    return hashValue;
}

std::size_t ManipulatorState::hash(const JointValues &values) const {
    std::size_t hashValue = 0;
    std::vector<double> jointValues = values.asVector();
    for (size_t i = 0; i < jointValues_.size(); i++) {
        tapir::hash_combine(hashValue, round_(jointValues[i], roundingPrecision_));
    }        
    return hashValue;
    

}

void ManipulatorState::serialize(std::ostream &os) const {	
	std::vector<double> jointValues = jointValues_.asVector();
	for (size_t i = 0; i < jointValues.size(); i++) {
	    os << jointValues[i];	    
	    os << " ";	            
	}
	
	for (size_t i = 0; i < end_effector_position_->size(); i++) {
	    os << end_effector_position_->at(i);	    
	    os << " ";	    
	}
	os << "END";
}

void ManipulatorState::print(std::ostream &os) const {	
    std::vector<double> jointValues = jointValues_.asVector();
    for (size_t i = 0; i < jointValues.size(); i++) {
        os << jointValues[i];
        if (i != jointValues.size() - 1) {
            os << " ";
        }        
    }
    os << endl;
    os << "End effector position: ";
    for (size_t i = 0; i < end_effector_position_->size(); i++) {
        os << end_effector_position_->at(i);
        if (i != end_effector_position_->size() - 1) {
            os << " ";
        }
    }  
    os << endl;    
}


JointValues ManipulatorState::getJointValues() const {     
     return jointValues_;
}

void ManipulatorState::setWeight(const double &weight) const{
	weight_ = weight;
}
    
double ManipulatorState::getWeight() const {
	return weight_;
}
 
} /* namespace rocksample */
