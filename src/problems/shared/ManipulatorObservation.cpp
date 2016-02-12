/** @file ManipulatorObservation.cpp
 *
 * Contains the implementations for the methods of RockSampleObservation.
 */
#include "ManipulatorObservation.hpp"

#include <cstddef>                      // for size_t
#include <cmath>
#include <random>

#include <algorithm>                    // for copy
#include <iterator>                     // for ostream_iterator
#include <ostream>                      // for operator<<, ostream
#include <vector>                       // for vector, operator==, _Bit_const_iterator, _Bit_iterator_base, hash, vector<>::const_iterator

#include "global.hpp"  
#include "solver/abstract-problem/Observation.hpp"             // for Observation

using std::cout;
using std::endl;

namespace shared {

ManipulatorObservation::ManipulatorObservation() :
    observedJointValues_()    
{
    
}

ManipulatorObservation::ManipulatorObservation(std::vector<double> &thetas) :
    observedJointValues_(thetas)    
{
    
}

ManipulatorObservation::ManipulatorObservation(const ManipulatorState &manipulatorState,
		                                       std::shared_ptr<shared::EigenMultivariateNormal<double>> &observation_distribution)     
{	
    std::vector<double> stateValues = manipulatorState.getJointValues().asVector();
    Eigen::MatrixXd state(stateValues.size(), 1);
    Eigen::MatrixXd sample(stateValues.size(), 1);
    observation_distribution->nextSample(sample);
    std::vector<double> observedValues;    
    for (size_t i = 0; i < stateValues.size(); i++) {           	
    	observedValues.push_back(stateValues[i] + sample(i));    	
    }
    
    observedJointValues_ = JointValues(observedValues);    
}

std::unique_ptr<solver::Observation> ManipulatorObservation::copy() const {
    std::vector<double> vec = observedJointValues_.asVector();   
    return std::make_unique<ManipulatorObservation>(vec);
}

double ManipulatorObservation::distanceTo(solver::Observation const &otherObs) const {    
    ManipulatorObservation const &otherManipulatorObservation = static_cast<ManipulatorObservation const &>(otherObs);
    std::vector<double> observedJointValuesVector = observedJointValues_.asVector();
    std::vector<double> otherObservedValues = otherManipulatorObservation.getObservedJointValues().asVector();
     
    double sum = 0;
    for (size_t i = 0; i < observedJointValuesVector.size(); i++) {
        sum += pow(observedJointValuesVector[i] - otherObservedValues[i], 2);
    }
    return sqrt(sum);    
}

bool ManipulatorObservation::equals(solver::Observation const &otherObs) const {    
    return distanceTo(otherObs) < 0.01;
}

std::size_t ManipulatorObservation::hash() const {    
    std::size_t hashValue = 0;
    std::vector<double> observedJointValuesVector = observedJointValues_.asVector();
    for (size_t i = 0; i < observedJointValuesVector.size(); i++) {
        tapir::hash_combine(hashValue, observedJointValuesVector[i]);
    }        
    return hashValue;
}

void ManipulatorObservation::print(std::ostream &os) const { 
    std::vector<double> observedJointValuesVector = observedJointValues_.asVector();
    os << "(";
    for (size_t i = 0; i < observedJointValuesVector.size(); i++) {
        os << observedJointValuesVector[i];
        if (i != observedJointValuesVector.size() - 1) {
            os << ", ";
        }
    }
    os << ")";    
}

void ManipulatorObservation::serialize(std::ostream &os) const {
	std::vector<double> observedJointValuesVector = observedJointValues_.asVector();
	for (auto &k: observedJointValuesVector) {
		os << k << " ";
	}
	os << "END";
}

long ManipulatorObservation::getBinNumber() const {     
    return 0;
}

JointValues ManipulatorObservation::getObservedJointValues() const {
    return observedJointValues_;
}

}
/* namespace manipulator */
