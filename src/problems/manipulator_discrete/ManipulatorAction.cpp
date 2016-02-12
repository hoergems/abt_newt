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

namespace manipulator {
JointInputs::JointInputs():
	joint_inputs_()
{
	
}

JointInputs::JointInputs(std::vector<double> &joint_inputs) :
    joint_inputs_(joint_inputs)
{   
}

ManipulatorAction::ManipulatorAction(JointInputs jointInputs, std::vector<double> max_joint_inputs, int num_input_steps) :        
        jointInputs_(jointInputs),       
        max_joint_inputs_(max_joint_inputs),
        num_input_steps_(num_input_steps) {
}

ManipulatorAction::ManipulatorAction(long code, std::vector<double> max_joint_inputs, int num_input_steps) : 
        jointInputs_(),
        max_joint_inputs_(max_joint_inputs),
        num_input_steps_(num_input_steps)
{       
	    std::vector<double> ks;
        int base = num_input_steps_; 
        std::vector<double> ss;
        for (auto &k: max_joint_inputs_) {
        	ks.push_back(2.0 * k / (num_input_steps_ - 1));
        }
        
        double j = code; 
        double j_old = code;
        double s = 0;        
        for(size_t i = max_joint_inputs_.size() - 1; i != (size_t) - 0; i--) {
        	double s;        	
        	j = j_old / std::pow(base, i);        	
        	modf(j, &s);        	
        	ss.push_back(s);   
        	if (i != 1) {
        	    j = (int)(j_old) % (int)std::pow(base, i);
        	    j_old = j;        	    
        	}
        }        
        ss.push_back((int)j_old % base);        
        std::vector<double> joint_inputs;
        for (size_t i = 0; i < max_joint_inputs_.size(); i++) {
        	joint_inputs.push_back(-max_joint_inputs_[i] + ss[i] * ks[i]);
        }
        
        jointInputs_ = JointInputs(joint_inputs);          
}

std::unique_ptr<solver::Action> ManipulatorAction::copy() const {    
    return std::make_unique<ManipulatorAction>(jointInputs_, max_joint_inputs_, num_input_steps_);
}

double ManipulatorAction::distanceTo(solver::Action const &other) const {
    ManipulatorAction const &otherAction = static_cast<ManipulatorAction const &>(other);
    double distance = 0.0;
    for (size_t i = 0; i < jointInputs_.joint_inputs_.size(); i++) {
    	distance += std::pow(jointInputs_.joint_inputs_[i] - otherAction.getJointInputs()[i], 2);
    }
    return std::sqrt(distance); 
}

void ManipulatorAction::print(std::ostream &os) const {
	os << "action inputs {(";
	for (size_t i = 0; i < jointInputs_.joint_inputs_.size(); i++) {
		os << jointInputs_.joint_inputs_[i];
		if (i < jointInputs_.joint_inputs_.size() - 1) {
			os << ", ";
		}
	}
	os << ")}";
}

long ManipulatorAction::getBinNumber() const {	
	std::vector<double> ks;
	double vel = 0.0;
	for (size_t i = 0; i < jointInputs_.joint_inputs_.size(); i++) {
		vel = 2.0 * max_joint_inputs_[i] / (num_input_steps_ - 1);		
		ks.push_back((jointInputs_.joint_inputs_[i] + max_joint_inputs_[i]) / vel);
	}
	std::reverse(ks.begin(), ks.end());    
    int base = num_input_steps_;
    long bin_num = 0;
    for (size_t i = 0; i < jointInputs_.joint_inputs_.size(); i++) {
    	bin_num += std::pow(base, i) * ks[i];
    }
    return bin_num;
}

std::vector<double> ManipulatorAction::getJointInputs() const {
	std::vector<double> jv;
	for (auto &k: jointInputs_.joint_inputs_) {
		jv.push_back(k);		
	}
	return jv;
}

} /* namespace manipulator */
