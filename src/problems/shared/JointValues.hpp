/** @file JointValues.hpp
 * 
 */
#ifndef JOINTVALUES_HPP_
#define JOINTVALUES_HPP_

#include <cmath>                        // for abs, pow, sqrt and PI

#include <iostream>
#include <sstream>

#include "global.hpp"

#include "prob.hpp"

using std::cout;
using std::endl;

struct JointValues {	
    std::vector<double> values_;

    JointValues() :
        values_() {

    }
   
    JointValues(int num) :
        values_() {
        for (int i=0; i < num; i++) {
            values_.push_back(0.0);
        }
    }    

    JointValues(std::vector<double> values) :
        values_() {
        for (size_t i=0; i < values.size(); i++) {
            values_.push_back(values[i]);
        }
        
        for (size_t i = 0; i < values_.size() / 2; i++) {
        	if (values_[i] > M_PI) {
        		values_[i] = -2.0 * M_PI + values_[i];
        	}
        	else if (values_[i] < -M_PI) {
        		values_[i] = 2.0 * M_PI + values_[i];
        	}
        }
    }  

    /** Returns the Euclidean distance from this GridPosition to the given GridPosition. */
    double euclideanDistanceTo(JointValues const &other) const {
        double sum = 0.0;
        std::vector<double> other_ja = other.asVector();
        for (size_t i=0; i < values_.size(); i++) {
            sum += std::pow(values_[i] - other_ja[i], 2);
        }
        return std::sqrt(sum);        
    } 
    
    bool isLegal() {        
        /**
        We do not enforce joint limits at the moment
        
        if (theta1 > M_PI || theta1 < -M_PI) {            
            return false;
        }
        if (theta2 > M_PI || theta2 < -M_PI) {            
            return false;
        }*/        
        return true;    
    } 
    
    void sampleUninformed(std::vector<double> &position_lower_constraints, 
                          std::vector<double> &position_upper_constraints,
                          std::vector<double> &velocity_constraints) {
        int seed = 123456789; 
        for (size_t i = 0; i < values_.size() / 2; i++) {
            values_[i] = uniform_sample(position_lower_constraints[i],
                                        position_upper_constraints[i],
                                        seed);
        }      
        for (size_t i=values_.size() / 2; i < values_.size(); i++) {
            values_[i] = uniform_sample(-velocity_constraints[i], velocity_constraints[i], seed);            
        }
                
    }

    std::vector<double> asVector() const{        
        return values_;        
    }

    size_t size() const {
        return values_.size();
    }
};

namespace std {
/** We define a hash function directly in the std:: namespace, so that this will be the
 * default hash function for GridPosition.
 */
template<> struct hash<JointValues> {
    /** Returns the hash value for the given GridPosition. */
    std::size_t operator()(JointValues const &values) const {
        std::size_t hashValue = 0;
        for (size_t i = 0; i < values.size(); i++) {
            tapir::hash_combine(hashValue, values.asVector()[i]);
        }        
        return hashValue;
    }
};
} /* namespace std */

/** A handy insertion operator for printing grid positions. */
inline std::ostream &operator<<(std::ostream &os, JointValues const &obj) {
    os << "(";
    for (size_t i = 0; i < obj.size(); i++) {
        os << obj.asVector()[i];
        if (i != obj.size() - 1) {
            os << " "; 
        }
    }
    os << ")";    
    return os;
}

inline std::istream &operator>>(std::istream &is, JointValues &obj) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ',');
    for (size_t i = 0; i < obj.size(); i++) {
        std::istringstream(tmpStr) >> obj.asVector()[i];
        if (i != obj.size() - 1) {
            std::getline(is, tmpStr, 'j');
        }
    }
    std::getline(is, tmpStr, ')');
    return is;
}

#endif /* JOINTVALUES_HPP_ */
