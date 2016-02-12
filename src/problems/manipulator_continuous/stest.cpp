/** @file manipulator/stest.cpp
 *
 * Defines the main method for the "stest" executable for the RockSample POMDP, which tests the
 * serialization methods for RockSample by deserializing and re-serializing a policy file.
 */
#include "problems/shared/stest.hpp"

#include "ManipulatorModel.hpp"          // for RockSampleModel
#include "ManipulatorModel.hpp"        // for RockSampleOptions

/** The main method for the "stest" executable for RockSample. */
int main(int argc, char const *argv[]) {
    return stest<manipulator_continuous::ManipulatorModel, manipulator_continuous::ManipulatorOptions>(argc, argv);
}
