/** @file Model.cpp
 *
 * Contains default implementations for some of the methods of Model.
 *
 * These should be overridden as needed, in order to further customize the ABT functionality.
 */
#include "solver/abstract-problem/Model.hpp"

#include <functional>

#include "solver/cached_values.hpp"
#include "solver/ActionNode.hpp"
#include "solver/BeliefNode.hpp"

#include "solver/abstract-problem/Action.hpp"        // for Action
#include "solver/abstract-problem/HistoricalData.hpp"
#include "solver/abstract-problem/State.hpp"                    // for State
#include "solver/abstract-problem/Observation.hpp"              // for Observation
#include "solver/abstract-problem/TransitionParameters.hpp"
#include "solver/abstract-problem/heuristics/HeuristicFunction.hpp"

#include "solver/belief-estimators/estimators.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"
#include "solver/mappings/actions/ActionPool.hpp"
#include "solver/mappings/observations/discrete_observations.hpp"
#include "solver/mappings/observations/ObservationMapping.hpp"
#include "solver/mappings/observations/ObservationPool.hpp"

#include "solver/indexing/StateIndex.hpp"
#include "solver/indexing/RTree.hpp"

#include "solver/changes/DefaultHistoryCorrector.hpp"
#include "solver/changes/HistoryCorrector.hpp"

#include "solver/search/search_interface.hpp"
#include "solver/search/steppers/ucb_search.hpp"

#include "solver/serialization/Serializer.hpp"

using std::cout;
using std::endl;

namespace solver {
Model::Model(std::string problemName, RandomGenerator *randGen, std::unique_ptr<Options> options) :
        problemName_(problemName),
        randGen_(randGen),
        options_(std::move(options)) {
}

/* -------------------- Simple getters ---------------------- */
RandomGenerator *Model::getRandomGenerator() const {
    return randGen_;
}

Options const *Model::getOptions() const {
    return options_.get();
}

void Model::updateModel(StepResult &stepResult, std::vector<State const *> &particles) {

}

/* -------------------- Black box dynamics ---------------------- */
// The more detailed methods are optional.

std::unique_ptr<TransitionParameters> Model::generateTransition(
        State const &/*state*/,
        Action const &/*action*/) {
    return nullptr;
}

std::unique_ptr<State> Model::generateNextState(
        State const &/*state*/,
        Action const &/*action*/,
        TransitionParameters const */*transitionParameters*/ // optional
        ) {
    return nullptr;
}

std::unique_ptr<Observation> Model::generateObservation(
          State const */*state*/, // optional
          Action const &/*action*/,
          TransitionParameters const */*transitionParameters*/, // optional
          State const &/*nextState*/
          ) {
    return nullptr;
}

double Model::generateReward(
          State const &/*state*/,
          Action const &/*action*/,
          TransitionParameters const */*transitionParameters*/, // optional
          State const */*nextState*/ // optional
          ) {
    return 0;
}


/* -------------- Methods for handling model changes ---------------- */
// Default = no changes.
void Model::applyChanges(std::vector<std::unique_ptr<ModelChange>> const &/*changes*/,
        Solver */*solver*/) {
}


/* ------------ Methods for handling particle depletion -------------- */
std::vector<std::unique_ptr<State>> Model::generateParticles(
        BeliefNode *previousBelief, Action const &action, Observation const &obs, long nParticles,
        std::vector<State const *> const &previousParticles) {    
    double startTime = tapir::clock_ms();
    double endTime = startTime + 10000.0;
    std::vector<std::unique_ptr<State>> particles;
    ObservationMapping *obsMap = previousBelief->getMapping()->getActionNode(action)->getMapping();    
    BeliefNode *childNode = obsMap->getBelief(obs);    
    // Use rejection sampling to generate the required number of particles.
    while ((long)particles.size() < nParticles) {
        if (tapir::clock_ms() >= endTime) {
            cout << "WARNING: Timeout while replenishing particles" << endl;
            return particles;
        }
        // Sample a random particle.
        if (previousParticles.size() == 0) {
            cout << "Previous particle set is empty!!!!!" << endl;
        }
        long index = std::uniform_int_distribution<long>(0,
                previousParticles.size() - 1)(*getRandomGenerator());
        State const *state = previousParticles[index];

        // Now generate a step in the model, and compare the observation to the actual observation.
        // Note that this comparison is done implicitly via the observation mapping, to ensure
        // that approximate observations are treated cleanly.
        StepResult result = generateStep(*state, action);
        if (obsMap->getBelief(*result.observation) == childNode) {
            particles.push_back(std::move(result.nextState));
        }
    }
    return particles;
}

std::vector<std::unique_ptr<State>> Model::generateParticles(
        BeliefNode *previousBelief, Action const &action, Observation const &obs, long nParticles) {
    std::vector<std::unique_ptr<State>> particles;
    ObservationMapping *obsMap = previousBelief->getMapping()->getActionNode(action)->getMapping();
    BeliefNode *childNode = obsMap->getBelief(obs);
    
    double startTime = tapir::clock_ms();
    double endTime = startTime + 10000.0;

    // Use rejection sampling to generate the required number of particles.
    while ((long)particles.size() < nParticles) {
        if (tapir::clock_ms() >= endTime) {
            cout << "WARNING: Timeout while replenishing particles" << endl;
            return particles;
        }
        // Sample a state uniformly at random        
        std::unique_ptr<State> state = sampleStateUninformed();

        // Now generate a step in the model, and compare the observation to the actual observation.
        // Note that this comparison is done implicitly via the observation mapping, to ensure
        // that approximate observations are treated cleanly.
        
        StepResult result = generateStep(*state, action);        
        if (obsMap->getBelief(*result.observation) == childNode) {
            particles.push_back(std::move(result.nextState));
        }
    }
    return particles;
}


/* --------------- Pretty printing methods ----------------- */
void Model::drawEnv(std::ostream &/*os*/) {
    // Default = do nothing.
}
void Model::drawSimulationState(BeliefNode const */*belief*/, State const &/*state*/,
        std::ostream &/*os*/) {
    // Default = do nothing.
}

/* ---------------------- Basic customizations  ---------------------- */
/** The default implementation simply returns zero. */
HeuristicFunction Model::getHeuristicFunction() {
    return [] (solver::HistoryEntry const *, solver::State const *,
            solver::HistoricalData const *) {
        return 0;
    };
}

/** Optional; not implemented. */
std::unique_ptr<Action> Model::getRolloutAction(HistoryEntry const */*entry*/,
        State const */*state*/, HistoricalData const */*data*/) {
    return nullptr;
}

/* ------- Customization of more complex solver functionality  --------- */
std::unique_ptr<StateIndex> Model::createStateIndex() {
    // Use an RTree, with the correct # of state variables.
    return std::make_unique<RTree>(getOptions()->numberOfStateVariables);
}

std::unique_ptr<HistoryCorrector> Model::createHistoryCorrector(Solver *solver) {
    // Create a DefaultHistoryCorrector.
    return std::make_unique<DefaultHistoryCorrector>(solver, getHeuristicFunction());
}

std::unique_ptr<ObservationPool> Model::createObservationPool(Solver *solver) {
    // Create a DiscreteObservationPool
    return std::make_unique<DiscreteObservationPool>(solver);
}

std::unique_ptr<SearchStrategy> Model::createSearchStrategy(Solver *solver) {
    // Create a basic search strategy using UCB with a coefficient of 1.0, and the default
    // heuristic function for this model.
    return std::make_unique<BasicSearchStrategy>(solver,
            std::make_unique<UcbStepGeneratorFactory>(solver, 1.0),
            getHeuristicFunction());
}

std::unique_ptr<SelectRecommendedActionStrategy> Model::createRecommendationSelectionStrategy(Solver */*solver*/) {
    // Create a basic search strategy using max the Q-value function
    return std::make_unique<MaxRecommendedActionStrategy>();
}

std::unique_ptr<EstimationStrategy> Model::createEstimationStrategy(Solver */*solver*/) {
    // Returns an estimation strategy using a simple average.
    return std::make_unique<EstimationFunction>(estimators::average);
}

std::unique_ptr<HistoricalData> Model::createRootHistoricalData() {
    // Optional; not implemented.
    return nullptr;
}

std::unique_ptr<Serializer> Model::createSerializer(Solver */*solver*/) {
    // Optional; not implemented.
    return nullptr;
}

solver::StateInfo * Model::sampleParticle(const std::vector<solver::StateInfo *> &stateInfos) {
    RandomGenerator *randGen = getRandomGenerator();
    long index = std::uniform_int_distribution<long>(0, stateInfos.size() - 1)(*randGen);
    return stateInfos[index];
}
} /* namespace solver */
