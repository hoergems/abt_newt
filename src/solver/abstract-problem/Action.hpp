/** @file Action.hpp
 *
 * Defines the Action interface; currently this is the same as Point, so this is just a straight
 * typedef.
 */
#ifndef SOLVER_ACTION_HPP_
#define SOLVER_ACTION_HPP_

#include "solver/abstract-problem/Point.hpp"

namespace solver {
    /** Currently Action has no requirements over and above Point, so this is just a typedef. */
    typedef Point Action;
} /* namespace solver */

#endif /* SOLVER_ACTION_HPP_ */
