/** @file ManipulatorObservation.hpp
 *
 * Defines the ManipulatorObservation class, which represents an observation in the Manipulator POMDP.
 */
#ifndef MANIPULATOR_OBSERVATION_HPP_
#define MANIPULATOR_OBSERVATION_HPP_

#include <cstddef>                      // for size_t

#include <ostream>                      // for ostream
#include <vector>                       // for vector

#include "solver/abstract-problem/DiscretizedPoint.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "global.hpp"                     // for RandomGenerator

#include "JointValues.hpp"
#include "ManipulatorState.hpp"
#include "prob.hpp"
#include "mult_normal.hpp"
#include <boost/random.hpp>

namespace shared {
/** A class representing an observation in the RockSample POMDP.
 *
 * This is represented by two boolean flags; one for whether or not any observation was made,
 * and one for whether the rock was seen as good or bad if an observation was, in fact, made.
 *
 * This class also implements solver::DiscretizedPoint so that the solver can use a simplistic
 * enumerated observation mapping approach (EnumeratedObservationPool) to store the possible
 * observations from each ActionNode.
 */
class ManipulatorObservation : public solver::DiscretizedPoint {    

  public:
    /** Constructs a new, empty observation. */
    ManipulatorObservation();
    
    ManipulatorObservation(std::vector<double> &thetas);    
    
    ManipulatorObservation(const ManipulatorState &manipulatorState, 
    		               std::shared_ptr<shared::EigenMultivariateNormal<double>> &observation_distribution);

    virtual ~ManipulatorObservation() = default;
    _NO_COPY_OR_MOVE(ManipulatorObservation);

    std::unique_ptr<solver::Observation> copy() const override;
    double distanceTo(solver::Observation const &otherObs) const override;
    bool equals(solver::Observation const &otherObs) const override;
    std::size_t hash() const override;
    void print(std::ostream &os) const override;
    
    void serialize(std::ostream &os) const;

    long getBinNumber() const override;
    
    JointValues getObservedJointValues() const;    
    
  private:
    JointValues observedJointValues_;
  
};
} /* namespace manipulator */

#endif /* MANIPULATOR_OBSERVATION_HPP_ */
