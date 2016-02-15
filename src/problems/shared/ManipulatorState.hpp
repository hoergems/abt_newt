/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef MANIPULATOR_STATE_HPP_
#define MANIPULATOR_STATE_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include <cmath>


#include "problems/shared/JointValues.hpp"
#include "problems/shared/robot.hpp"
#include "solver/abstract-problem/State.hpp"             // for State
#include "solver/abstract-problem/VectorState.hpp"             // for VectorState

namespace shared {
/** A class representing a state in the RockSample POMDP.
 *
 * The state contains the position of the robot, as well as a boolean value for each rock
 * representing whether it is good (true => good, false => bad).
 *
 * This class also implements solver::VectorState in order to allow the state to be easily
 * converted to a vector<double>, which can then be used inside the standard R*-tree implementation
 * of StateIndex to allow spatial lookup of states.
 */
class ManipulatorState : public solver::VectorState { 
    friend class ManipulatorTextSerializer;   
  public:
    /** Constructs a new RockSampleState with the given robot position, and the given goodness states
     * for all of the rocks.
     */
    ManipulatorState(JointValues values, boost::shared_ptr<shared::Robot> &robot);
    ManipulatorState(JointValues values, std::vector<double> ee_position);
    ManipulatorState(JointValues values, std::vector<double> ee_position, double weight);
    ManipulatorState(JointValues values, const boost::shared_ptr<shared::Robot> &robot, double weight);
    virtual ~ManipulatorState() = default;

    std::unique_ptr<solver::State> copy() const override;
    double distanceTo(solver::State const &otherState) const override;
    bool equals(solver::State const &otherState) const override;    
    std::size_t hash() const override;
    std::size_t hash(const JointValues &values) const;
    void print(std::ostream &os) const override;
    
    void serialize(std::ostream &os) const;

    std::vector<double> asVector() const override;

    /** Returns the position of the robot. */
    JointValues getJointValues() const;

    std::shared_ptr<std::vector<double> > getEndEffectorPosition() const;
    
    void setWeight(const double &weight) const;
    
    double getWeight() const;
    
    void setPreviousState(std::shared_ptr<ManipulatorState> &previous_state) {
    	previous_state_ = previous_state;
    } 
    
    std::shared_ptr<ManipulatorState> getPreviousState() const{
    	return previous_state_;
    }

  private:
    /** The position of the robot. */
    JointValues jointValues_;
                                                 
    double round_(const double &value, const int &precision) const;
    
    void make_end_effector_position() const;
    
    int roundingPrecision_ = 4;

    mutable std::shared_ptr<std::vector<double> > end_effector_position_;
    
    mutable double weight_;
    
    std::shared_ptr<ManipulatorState> previous_state_;
    
    boost::shared_ptr<shared::Robot> robot_;
    
};
} /* namespace manipulator */

// We define a hash function directly in the std namespace.
namespace std {
/** A struct in the std namespace to define a standard hash function for the
 * RockSampleState class.
 */
template<> struct hash<shared::ManipulatorState> {
    /** Returns the hash value for the given RockSampleState. */
    std::size_t operator()(shared::ManipulatorState const &state) const {
        return state.hash();
    }
};
} /* namespace std */

#endif /* MANIPULATORSTATE_HPP_ */
