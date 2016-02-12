/** @file nn_rollout.cpp
 *
 * Contains the implementation for an approximate-nearest-neighbor-based rollout strategy.
 */
#include "solver/search/steppers/nn_rollout.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/BeliefTree.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Solver.hpp"

#include "solver/mappings/actions/ActionMapping.hpp"

namespace solver {

NnRolloutFactory::NnRolloutFactory(Solver *solver, long maxNnComparisons, double maxNnDistance) :
            solver_(solver),
            maxNnComparisons_(maxNnComparisons),
            maxNnDistance_(maxNnDistance),
            nnMap_() {
}

BeliefNode* NnRolloutFactory::findNeighbor(BeliefNode *belief) {
    // A maximum distance of 0 means this function is disabled.
    if (maxNnDistance_ < 0) {
        return nullptr;
    }

    // Initially there is no minimum distance, unless we've already stored a neighbor.
    double minDist = std::numeric_limits<double>::infinity();
    BeliefNode *nearestBelief = nnMap_[belief].neighbor;
    if (nearestBelief != nullptr) {
        minDist = belief->distL1Independent(nearestBelief);
    }

    long numTried = 0;
    for (BeliefNode *otherBelief : solver_->getPolicy()->getNodes()) {
        // Obviously we don't want the belief itself.
        if (belief == otherBelief) {
            continue;
        }

        if (numTried >= maxNnComparisons_) {
            // Stop if we reach the maximum # of comparisons.
            break;
        } else {
            double distance = belief->distL1Independent(otherBelief);
            if (distance < minDist) {
                minDist = distance;
                nearestBelief = otherBelief;
            }
            numTried++;
        }
    }

    // If it's not near enough, we've failed.
    if (minDist > maxNnDistance_) {
        return nullptr;
    }

    // Otherwise update the mapping with the new neighbor.
    nnMap_[belief].neighbor = nearestBelief;
    return nearestBelief;
}

std::unique_ptr<StepGenerator> NnRolloutFactory::createGenerator(SearchStatus &status,
        HistoryEntry const *entry, State const */*state*/, HistoricalData const */*data*/) {
    // Find a neighbor, and use it to make a new generator.
    BeliefNode *neighbor = findNeighbor(entry->getAssociatedBeliefNode());
    return std::make_unique<NnRolloutGenerator>(status, solver_, neighbor);
}

NnRolloutGenerator::NnRolloutGenerator(SearchStatus &status, Solver *solver, BeliefNode *neighbor) :
            StepGenerator(status),
            model_(solver->getModel()),
            currentNeighborNode_(neighbor) {
    // Set the initial status appropriately.
    if (currentNeighborNode_ == nullptr) {
        status_ = SearchStatus::UNINITIALIZED;
    } else{
        status_ = SearchStatus::INITIAL;
    }
}

Model::StepResult NnRolloutGenerator::getStep(HistoryEntry const */*entry*/, State const *state,
        HistoricalData const */*data*/) {
    if (currentNeighborNode_ == nullptr) {
        // If we have no neighbor, the NN rollout is finished.
        status_ = SearchStatus::OUT_OF_STEPS;
        return Model::StepResult { };
    }

    // Generate a step using the recommended action from the neighboring node.
    std::unique_ptr<Action> action = currentNeighborNode_->getRecommendedAction();
    Model::StepResult result = model_->generateStep(*state, *action);

    // getChild() will return nullptr if the child doesn't yet exist => this will be the last step.
    currentNeighborNode_ = currentNeighborNode_->getChild(*action, *result.observation);
    return std::move(result);
}
} /* namespace solver */
