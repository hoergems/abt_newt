#ifndef MANIPULATOR_TRANSITION_HPP_
#define MANIPULATOR_TRANSITION_HPP_

#include <unistd.h>
#include "solver/abstract-problem/TransitionParameters.hpp"
#include "ManipulatorState.hpp"

namespace shared {

class ManipulatorTransitionParameters: public solver::TransitionParameters {

    public:
        ManipulatorTransitionParameters(bool &collided, std::shared_ptr<shared::ManipulatorState> &collided_state);
    
        virtual void print(std::ostream &os) const override;
        
    private:
        bool collided_;
        
        std::shared_ptr<shared::ManipulatorState> collided_state_;

};

}

#endif
