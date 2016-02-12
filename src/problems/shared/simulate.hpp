/** @file simulate.hpp
 *
 * Contains a generic function for running ABT simulations, which can be used to form the main
 * method of a problem-specific "simulate" executable.
 */
#ifndef SIMULATE_HPP_
#define SIMULATE_HPP_

#include <fstream>                      // for operator<<, basic_ostream, basic_ostream<>::__ostream_type, ofstream, endl, ostream, ifstream
#include <iostream>                     // for cout
#include <map>
#include <memory>                       // for unique_ptr
#include <string>                       // for string, char_traits, operator<<
#include <utility>                      // for move                // IWYU pragma: keep
#include <vector>                       // for vector, vector<>::iterator

#include "global.hpp"                     // for RandomGenerator, make_unique

#include "solver/abstract-problem/Action.hpp"
#include "solver/abstract-problem/ModelChange.hpp"       // for ModelChange
#include "solver/abstract-problem/Observation.hpp"       // for Observation
#include "solver/abstract-problem/State.hpp"             // for operator<<, State

#include "solver/serialization/Serializer.hpp"        // for Serializer

#include "solver/HistoryEntry.hpp"
#include "solver/HistorySequence.hpp"
#include "solver/Simulator.hpp"            // for Simulator
#include "solver/Solver.hpp"            // for Solver

#ifdef GOOGLE_PROFILER
#include <google/profiler.h>
#endif

using std::cout;
using std::endl;

/** A template method to run a simulation for the given model and options classes. */
template<typename ModelType, typename OptionsType>
int simulate(int argc, char const *argv[]) {
    std::unique_ptr<options::OptionParser> parser = OptionsType::makeParser(true);

    OptionsType options;
    std::string workingDir = tapir::get_current_directory();    
    try {
        parser->setOptions(&options);
        parser->parseCmdLine(argc, argv);
        if (!options.baseConfigPath.empty()) {            
            tapir::change_directory(options.baseConfigPath);
        }
        if (!options.configPath.empty()) {
            parser->parseCfgFile(options.configPath);
        }
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(workingDir);
        }
        parser->finalize();
    } catch (options::OptionParsingException const &e) {
        std::cerr << e.what() << std::endl;
        return 2;
    }

    if (options.seed == 0) {
        options.seed = std::time(nullptr);
    }
    cout << "Global seed: " << options.seed << endl << endl;
    RandomGenerator randGen;
    randGen.seed(options.seed);
    randGen.discard(10);

    

    if (options.rngState > 0) {
        std::stringstream sstr;
        sstr << options.rngState;
        sstr >> randGen;
        cout << "Loaded PRNG state " << options.rngState << endl;
    }

    double totalReward = 0;
    double totalTime = 0;
    double totalNSteps = 0;
    double totalNHistories = 0;
    double totalNSucRuns = 0;    

#ifdef GOOGLE_PROFILER
    ProfilerStart("simulate.prof");
#endif

    for (long runNumber = 0; runNumber < options.nRuns; runNumber++) {
        cout << "Run #" << runNumber+1 << endl;
        cout << "PRNG engine state: " << randGen << endl;

        // We want the simulated history to be independent of the solver's searching,
        // so we create a different random generator here.
        RandomGenerator solverGen(randGen);
        // Advance it forward a long way to avoid correlation between the solver and simulator.
        solverGen.discard(10000);

        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(options.baseConfigPath);
        }
        std::unique_ptr<ModelType> solverModel = std::make_unique<ModelType>(&solverGen,
                std::make_unique<OptionsType>(options));;
        solver::Solver solver(std::move(solverModel));        

        if (options.loadInitialPolicy) {
            cout << "Loading policy... " << endl;
            std::ifstream inFile;
            inFile.open(options.policyPath);
            if (!inFile.is_open()) {
                std::ostringstream message;
                message << "Failed to open " << options.policyPath;
                debug::show_message(message.str());
                return 1;
            }
            solver.getSerializer()->load(inFile);
            inFile.close();
        } else {
        	cout << "Starting from empty policy. " << endl;
        	solver.initializeEmpty();
        }

        std::unique_ptr<ModelType> simulatorModel = std::make_unique<ModelType>(&randGen,
                std::make_unique<OptionsType>(options));
        solver::Simulator simulator(std::move(simulatorModel), &solver, options.areDynamic);
        if (options.hasChanges) {
            simulator.loadChangeSequence(options.changesPath);
        }
        if (!options.baseConfigPath.empty()) {
            tapir::change_directory(workingDir);
        }

        simulator.setMaxStepCount(options.nSimulationSteps);
        cout << "Running..." << endl;

        double tStart = tapir::clock_ms();
        std::pair<double, std::pair<long, bool>> simulationResult = simulator.runSimulation();        
        double totT = tapir::clock_ms() - tStart;      
        long actualNSteps = simulator.getStepCount();

        totalReward += simulationResult.first;
        totalTime += totT;
        totalNSteps += actualNSteps;
        totalNHistories += simulationResult.second.first;
        
        if (simulationResult.second.second) {
            totalNSucRuns += 1;
        }

        std::ofstream os(options.logPath, std::ios_base::app | std::ios_base::out);
        os << "Run #" << runNumber+1 << endl;
        os << "Reward: " << simulationResult.first << endl;

        solver::HistorySequence *sequence = simulator.getHistory();
        std::vector<std::vector<solver::State const *> > *visitedBeliefs = simulator.getVisitedBeliefs();        
        size_t j = 0;        
        for (solver::HistoryEntry::IdType entryNo = 0;
                entryNo < sequence->getLength() - 1; entryNo++) {
            solver::HistoryEntry *entry = sequence->getEntry(entryNo);
            
            os << "t = " << entryNo << endl;
            os << "S: " << *entry->getState() << endl;
            os << "A: " << *entry->getAction() << endl;
            os << "O: " << *entry->getObservation() << endl;
            os << "R: " << entry->getImmediateReward() << endl;
            if (entry->getTransitionParameters() != nullptr) {
                os << "Trans: " << *entry->getTransitionParameters() << endl;
            }
            os << "PARTICLES BEGIN" << endl; 
            //cout << "saving particles: " << options.saveParticles << endl;
            if (options.saveParticles) {
                for (size_t k = 0; k < visitedBeliefs->at(j).size(); k++)
                {
                    os << "p: " << *(visitedBeliefs->at(j)[k]) << endl;
                }
            }           
                       
            os << "PARTICLES END" << endl;
            j++;
        } 
               
        os << "Final State: " << *sequence->getLastEntry()->getState();        
        os << endl;
        
        os << "Total discounted reward: " << simulationResult.first << endl;
        os << "Actual of steps: " << actualNSteps << endl;
        os << "Time spent on changes: " << simulator.getTotalChangingTime() << "ms" << endl;        
        os << "Time spent on policy updates: " << simulator.getTotalImprovementTime() << "ms" << endl;        
        os << "Time spent replenishing particles: " << simulator.getTotalReplenishingTime() << "ms" << endl;       
        os << "Time spent pruning: " << simulator.getTotalPruningTime() << "ms" << endl;        
        os << "Total time taken: " << totT << "ms" << endl;
        os.close();
        if (options.savePolicy) {
            // Write the final policy to a file.
            cout << "Saving final policy..." << endl;
            std::ofstream outFile;
            std::ostringstream sstr;
            sstr << "final-" << runNumber << ".pol";
            outFile.open(sstr.str());
            solver.getSerializer()->save(outFile);
            outFile.close();
            cout << "Finished saving." << endl;
        }
        cout << "Run complete!" << endl << endl;
    }

#ifdef GOOGLE_PROFILER
    ProfilerStop();
#endif
    std::ofstream os(options.logPath, std::ios_base::app | std::ios_base::out);
    os << "##################################" << endl;
    os << "inc_covariance: " << options.inc_covariance << endl;
    os << "Process covariance: " << options.process_covariance << endl;
    os << "Observation covariance: " << options.observation_covariance << endl;
    os << "Mean number of steps: " << totalNSteps / options.nRuns << endl;
    os << "Mean planning time per step: " << totalTime / totalNSteps << " ms" << endl;
    os << "Mean planning time per run: " << totalTime / options.nRuns << " ms" << endl;
    os << "Mean number of histories per run: " << totalNHistories / options.nRuns << endl;
    os << "Mean number of histories per step: " << totalNHistories / totalNSteps << endl;
    os << "Num successes: " << totalNSucRuns << endl;
    os << "Percentage of successful runs: " << (100.0 / options.nRuns) * totalNSucRuns << endl;
    os << "Mean rewards: " << totalReward / options.nRuns << endl;
    os.close();

    cout << options.nRuns << " runs completed." << endl;
    cout << "Mean reward: " << totalReward / options.nRuns << endl;
    cout << "Mean number of steps: " << totalNSteps / options.nRuns << endl;
    cout << "Mean time taken: " << totalTime / options.nRuns << "ms" << endl;
    cout << "Mean time per step: " << totalTime / totalNSteps << "ms" << endl;
    return 0;
}

#endif /* SIMULATE_HPP_ */

