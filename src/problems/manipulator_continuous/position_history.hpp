/** @file position_history.hpp
 *
 * Defines a class to keep track of the position of the robot in RockSample.
 *
 * This is useful, since the position is fully observable but is not included in observations.
 */
#ifndef MANIPULATOR_POSITION_HISTORY_HPP_
#define MANIPULATOR_POSITION_HISTORY_HPP_

#include <memory>
#include <vector>

#include "solver/abstract-problem/HistoricalData.hpp"

#include "solver/serialization/TextSerializer.hpp"

//#include "problems/shared/GridPosition.hpp"
//#include "problems/shared/CartesianCoordinates.hpp"
#include "ManipulatorModel.hpp"
#include "ManipulatorAction.hpp"
#include "problems/shared/JointValues.hpp"

namespace manipulator_continuous {
class ManipulatorAction;
class ManipulatorModel;

/** An implementation of the serialization methods for the PositionData class. */
class PositionDataTextSerializer : virtual public solver::TextSerializer {
public:
    void saveHistoricalData(solver::HistoricalData const *data, std::ostream &os) override;
    std::unique_ptr<solver::HistoricalData> loadHistoricalData(std::istream &is) override;
};

/** A class to store the robot position associated with a given belief node.
 *
 * Since the robot position in RockSample is fully observable, all particles in any belief will
 * in fact have the same position, which is stored here.
 */
class PositionData : public solver::HistoricalData {
    friend class PositionDataTextSerializer;
public:
    /** Creates a new PositionData instance for the given model, and located in the given grid
     * square.
     */
    PositionData(ManipulatorModel *model, 
                 const shared::ManipulatorState &manipulator_state_,
                 int old_step);
    PositionData(ManipulatorModel *model, 
                 const shared::ManipulatorState &manipulator_state_,
                 int old_step, 
                 bool /*copy_const*/);
    virtual ~PositionData() = default;
    _NO_COPY_OR_MOVE(PositionData);

    std::unique_ptr<solver::HistoricalData> copy() const;

    std::unique_ptr<solver::HistoricalData> createChild(
            solver::Action const &action,
            solver::Observation const &observation) const override;

    int getCurrentStep() const;

    const shared::ManipulatorState* getManipulatorState() const;

    void print(std::ostream &os) const override;

private:
    /** The ManipulatorModel instance this PositionData instance is associated with. */
    ManipulatorModel *model_;
    /** The grid position of this PositionData. */    
    
    const shared::ManipulatorState manipulator_state_;

    int current_step_;     
    
};


} /* namespace manipulator_continuous */

#endif /* MANIPULATOR_POSITION_HISTORY_HPP_ */
