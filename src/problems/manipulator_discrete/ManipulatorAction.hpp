/** @file RockSampleAction.hpp
 *
 * Defines the RockSampleAction class, which represents an action for the RockSample problem, and
 * also the ActionType enumeration, which enumerates the different types of actions for RockSample.
 */
#ifndef MANIPULATOR_ACTION_HPP_
#define MANIPULATOR_ACTION_HPP_

#include <cstddef>                      // for size_t
#include <cstdint>

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "f_point_test.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/DiscretizedPoint.hpp"             // for DiscretizedPoint

namespace manipulator {
/** An enumeration of the possible action types in RockSample.
 */

class JointInputs {
     public:
	     JointInputs();
	
	     JointInputs(std::vector<double> &joint_inputs);
         
         std::vector<double> joint_inputs_;
};

/** A class representing an action in the RockSample POMDP
 *
 * This class also implements solver::DiscretizedPoint so that the solver can use a simplistic
 * enumerated action mapping approach (EnumeratedActionPool) to store the available actions from
 * each belief node.
 */
class ManipulatorAction : public solver::DiscretizedPoint {    
  public:
    /** Constructs a new action from the given ActionType. */
    ManipulatorAction(JointInputs joint_inputs, std::vector<double> max_joint_inputs, int num_input_steps);
    
    ManipulatorAction(long code, std::vector<double> max_joint_inputs, int num_input_steps);
    
    virtual ~ManipulatorAction() = default;

    std::unique_ptr<solver::Action> copy() const override;
    double distanceTo(solver::Action const &other) const override;
    void print(std::ostream &os) const override;

    long getBinNumber() const override; 

    std::vector<double> getJointInputs() const;
    
    int num_input_steps_;
    
    std::vector<double> max_joint_inputs_;

  private:    
    JointInputs jointInputs_; 
    
    long bin_number_;   
    
};
} /* namespace rocksample */

#endif /* MANIPULATOR_ACTION_HPP_ */
