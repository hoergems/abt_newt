#include "ManipulatorTransitionParameters.hpp"
#include <iostream>

using std::cout;
using std::endl;

namespace shared{

ManipulatorTransitionParameters::ManipulatorTransitionParameters(bool &collided, std::shared_ptr<shared::ManipulatorState> &collided_state) :
    collided_(collided),
    collided_state_(collided_state)
{
    
}

void ManipulatorTransitionParameters::print(std::ostream &os) const {    
    std::string col_str("False");
    if (collided_) {
       col_str = "True";
       os << "Collision detected: " << col_str << endl;
       os << "Colliding state: " << *(collided_state_.get()) << endl;
    }
    
    else {
    	os << "Collision detected: " << col_str << endl;
    }
}

}
