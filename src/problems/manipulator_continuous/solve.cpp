/** @file rocksample/solve.cpp
 *
 * Defines the main method for the "solve" executable for the RockSample POMDP, which generates an
 * initial policy.
 */
#include "problems/shared/solve.hpp"

#include "ManipulatorModel.hpp"          // for ManipulatorModel
#include "ManipulatorOptions.hpp"        // for ManipulatorOptions

/** The main method for the "solve" executable for RockSample. */
int main(int argc, char const *argv[]) {
    return solve<manipulator_continuous::ManipulatorModel, manipulator_continuous::ManipulatorOptions>(argc, argv);
}
