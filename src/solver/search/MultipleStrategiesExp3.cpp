/** @file MultipleStrategiesExp3.cpp
 *
 * Contains the implementation of MultipleStrategiesExp3, which defines an EXP3-based approach
 * to combining multiple strategies.
 */
#include "solver/search/MultipleStrategiesExp3.hpp"

#include "solver/abstract-problem/Model.hpp"

#include "solver/BeliefNode.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/HistoryEntry.hpp"
#include "solver/Solver.hpp"

namespace solver {
MultipleStrategiesExp3::MultipleStrategiesExp3(Solver *solver,
        double strategyExplorationCoefficient,
        std::vector<std::unique_ptr<SearchStrategy>> strategies) :
            solver_(solver),
            model_(solver_->getModel()),
            strategyExplorationCoefficient_(strategyExplorationCoefficient),
            strategies_() {
    // Initialize the StrategyInfo for each strategy.
    for (unsigned long index = 0; index < strategies.size(); index++) {
        StrategyInfo info;
        info.strategyNo = index;
        info.strategy = std::move(strategies[index]);
        info.weight = 1.0 / strategies.size();
        info.probability = 1.0 / strategies.size();
        strategies_.push_back(std::move(info));
    }
}

MultipleStrategiesExp3::StrategyInfo *MultipleStrategiesExp3::sampleAStrategy(
        std::unordered_set<long> strategiesToExclude) {
    // If all the strategies are excluded, we can't sample one.
    if (strategiesToExclude.size() == strategies_.size()) {
        return nullptr;
    }

    // Create a vector of weights for use in std::discrete_distribution
    std::vector<double> weights;
    for (StrategyInfo &info : strategies_) {
        // Only add nonzero weights for strategies that have not been excluded.
        if (strategiesToExclude.count(info.strategyNo) == 0) {
            weights.push_back(info.probability);
        } else {
            weights.push_back(0);
        }
    }

    // Sample the strategy number using the distribution, and return a StrategyInfo*
    std::discrete_distribution<long> distribution(weights.begin(), weights.end());
    long strategyNo = distribution(*model_->getRandomGenerator());
    return &strategies_[strategyNo];
}

void MultipleStrategiesExp3::updateStrategyWeights(long strategyNo, double timeUsed,
        double deltaValue) {
    StrategyInfo &strategyInfo = strategies_[strategyNo];
    // Ignore negative changes.
    if (deltaValue < 0.0) {
        deltaValue = 0.0;
    }

    // Update the parameters for this strategy.
    strategyInfo.timeSpent += timeUsed;
    strategyInfo.numberOfTimesUsed++;
    strategyInfo.weight *= std::exp(
            strategyExplorationCoefficient_ * (deltaValue / model_->getOptions()->maxVal)
                    / (2 * strategyInfo.probability));

    // Recalculate the total weight.
    double weightTotal = 0.0;
    for (StrategyInfo &info : strategies_) {
        weightTotal += info.weight;
    }

    // Now we calculate actual probabilities for each strategy.
    double probabilityTotal = 0.0;
    for (StrategyInfo &info : strategies_) {
        info.probability = ((1 - strategyExplorationCoefficient_) * info.weight / weightTotal
                + strategyExplorationCoefficient_ / 2);

        // Scale the probability by the inverse of the time it costs to use this strategy.
        info.probability *= (info.numberOfTimesUsed + 1) / (info.timeSpent + 1);

        probabilityTotal += info.probability;
    }

    // Renormalize to make the probabilities actual probabilities.
    for (StrategyInfo &info : strategies_) {
        info.probability /= probabilityTotal;
    }
}

SearchStatus MultipleStrategiesExp3::extendAndBackup(HistorySequence *sequence, long maximumDepth) {
    BeliefNode *rootNode = sequence->getFirstEntry()->getAssociatedBeliefNode();
    double initialRootValue = rootNode->getCachedValue();

    // Keep track of which strategies have failed outright.
    std::unordered_set<long> failedStrategies;
    while (true) {
        // Sample a new strategy.
        StrategyInfo *info = sampleAStrategy(failedStrategies);

        // No more strategies => give up.
        if (info == nullptr) {
            return SearchStatus::UNINITIALIZED;
        }

        // Try using a strategy.
        double startTime = tapir::clock_ms();
        SearchStatus status = info->strategy->extendAndBackup(sequence, maximumDepth);
        double timeUsed = tapir::clock_ms() - startTime;

        // If the strategy initialized successfully, we backup, update weights, and we're done.
        if (status != SearchStatus::UNINITIALIZED) {
            // Update the weights for EXP3 and return the new status.
            rootNode->recalculateValue(); // Make sure the root node recalculates its value.
            double newRootValue = rootNode->getCachedValue();
            updateStrategyWeights(info->strategyNo, timeUsed, newRootValue - initialRootValue);
            return status;
        }

        // The strategy failed to initialize -> update the weights, but keep trying other strategies.
        updateStrategyWeights(info->strategyNo, timeUsed, 0.0);
        failedStrategies.insert(info->strategyNo);
    }
    return SearchStatus::UNINITIALIZED;
}
} /* namespace solver */
