/** @file RockSampleTextSerializer.hpp
 *
 * Contains text-based serialization methods for the core classes implementing RockSample, that is:
 * RockSampleChange, RockSampleState, RockSampleAction, and RockSampleObservation.
 */
#ifndef MANIPULATOR_TEXTSERIALIZER_HPP_
#define MANIPULATOR_TEXTSERIALIZER_HPP_

#include "solver/abstract-problem/Action.hpp"

#include "solver/mappings/actions/continuous_actions.hpp"
#include "solver/mappings/observations/approximate_observations.hpp"
#include "problems/shared/ManipulatorSerializer.hpp"

#include "position_history.hpp"
#include "global.hpp"

namespace solver {
class Solver;
} /* namespace solver */

namespace manipulator_continuous {

/** A serialization class for the RockSample problem.
 *
 * This contains serialization methods for RockSampleChange, RockSampleState, RockSampleAction,
 * and RockSampleObservation.
 * this class also inherits from solver::EnumeratedObservationTextSerializer in order to serialize
 * the observation mappings.
 *
 * Note that this class does not implement any method to serialize the action mappings; this is
 * because different serialization is needed depending on what type of history-based heuristic
 * information is being used.
 */
class ManipulatorTextSerializer: virtual public shared::ManipulatorSerializer, 
    virtual solver::ContinuousActionTextSerializer, public PositionDataTextSerializer
    {
    //typedef ManipulatorModel Model;	
    typedef ManipulatorAction::ConstructionData ConstructionData;
public:    
    /** Creates a new RockSampleTextSerializer instance, associated with the given solver. */
    ManipulatorTextSerializer() = default;
    ManipulatorTextSerializer(solver::Solver *solver, unsigned int &action_space_dim);
    virtual ~ManipulatorTextSerializer() = default;
    _NO_COPY_OR_MOVE(ManipulatorTextSerializer);
   
    void saveAction(solver::Action const *action, std::ostream &os) override;
    std::unique_ptr<solver::Action> loadAction(std::istream &is) override;
    
    void saveConstructionData(const ThisActionConstructionDataBase* baseData, std::ostream& os) override;
    
    std::unique_ptr<ManipulatorTextSerializer::ThisActionConstructionDataBase> loadConstructionData(std::istream& is) override;

    //virtual int getActionColumnWidth() override;
    //virtual int getTPColumnWidth() override;
    //virtual int getObservationColumnWidth() override;
private:
    unsigned int action_space_dim_;
};

} /* namespace manipulator */

#endif /* MANIPULATOR_TEXTSERIALIZER_HPP_ */
