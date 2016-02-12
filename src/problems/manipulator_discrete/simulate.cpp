/** @file rocksample/simulate.cpp
 *
 * Defines the main method for the "simulate" executable for the RockSample POMDP, which runs
 * online simulations to test the performance of the solver.
 */
#include "problems/shared/simulate.hpp"

#include "ManipulatorModel.hpp"          // for RockSampleModel
#include "ManipulatorOptions.hpp"        // for RockSampleOptions

/** The main method for the "simulate" executable for RockSample. */
int main(int argc, char const *argv[]) {
    return simulate<manipulator::ManipulatorModel, manipulator::ManipulatorOptions>(argc, argv);
}
