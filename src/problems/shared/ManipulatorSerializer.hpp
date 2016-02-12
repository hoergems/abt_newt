#ifndef MANIPULATOR_SERIALIZER_HPP_
#define MANIPULATOR_SERIALIZER_HPP_

#include <iosfwd>                       // for istream, ostream
#include <memory>                       // for unique_ptr
#include <fstream>

#include "problems/shared/JointValues.hpp"

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/State.hpp"
#include "solver/abstract-problem/Observation.hpp"

#include "solver/serialization/TextSerializer.hpp"    // for TextSerializer
#include "solver/mappings/observations/enumerated_observations.hpp"
#include "solver/mappings/observations/approximate_observations.hpp"


namespace solver {
class Solver;
} /* namespace solver */

namespace shared {

/** A simple method to serialize a vector of longs to an output stream. */
void saveVector(std::vector<long> values, std::ostream &os);
/** A simple method to de-serialize a vector of longs from an input stream. */
std::vector<long> loadVector(std::istream &is);

class ManipulatorSerializer: virtual public solver::TextSerializer, 
    virtual public solver::ApproximateObservationTextSerializer {
    
    public:
        ManipulatorSerializer() = default;
        ManipulatorSerializer(solver::Solver *solver);
        virtual ~ManipulatorSerializer() = default;
        _NO_COPY_OR_MOVE(ManipulatorSerializer);
        
        void saveState(solver::State const *state, std::ostream &os) override;
        std::unique_ptr<solver::State> loadState(std::istream &is) override;
        
        void saveObservation(solver::Observation const *obs, std::ostream &os) override;
        std::unique_ptr<solver::Observation> loadObservation(std::istream &is) override;
};

}

#endif
