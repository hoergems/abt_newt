/** @file GridPosition.hpp
 *
 * Defines a handy struct to represent 2-D grid indices, in the form of (i, j) == (row, column),
 * as well as some useful methods for dealing with them.
 */
#ifndef CARTESIANCOORDINATES_HPP_
#define CARTESIANCOORDINATES_HPP_

#include <cmath>                        // for abs, pow, sqrt

#include <iostream>
#include <sstream>

#include "global.hpp"

/** Represents a position within a 2-D grid, using a zero-based row and column index.
 *
 * i = 0, 1, ... is the row, from top to bottom.
 * j = 0, 1, ... is the column, from left to right.
 */
struct CartesianCoordinates {
	/** The row number, starting from 0 for the topmost row. */
    long x;
    /** The column number, starting from 0 for the leftmost column. */
    long y;
    /** Makes a new GridPosition with i=0 and j=0. */
    CartesianCoordinates() :
        x(0),
        y(0) {
    }
    /** Makes a new GridPosition with the given row (_i) and column (_j). */
    CartesianCoordinates(long _x, long _y) :
        x(_x),
        y(_y) {
    }

    /** Returns the Euclidean distance from this GridPosition to the given GridPosition. */
    double euclideanDistanceTo(CartesianCoordinates const &other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }    
};

namespace std {
/** We define a hash function directly in the std:: namespace, so that this will be the
 * default hash function for GridPosition.
 */
template<> struct hash<CartesianCoordinates> {
    /** Returns the hash value for the given GridPosition. */
    std::size_t operator()(CartesianCoordinates const &pos) const {
        std::size_t hashValue = 0;
        tapir::hash_combine(hashValue, pos.x);
        tapir::hash_combine(hashValue, pos.y);
        return hashValue;
    }
};
} /* namespace std */

/** A handy insertion operator for printing grid positions. */
inline std::ostream &operator<<(std::ostream &os, CartesianCoordinates const &obj) {
    os << "(" << obj.x << ", " << obj.y << ")";
    return os;
}

/** A handy extraction operator for reading GridPositions from a file. */
inline std::istream &operator>>(std::istream &is, CartesianCoordinates &obj) {
    std::string tmpStr;
    std::getline(is, tmpStr, '(');
    std::getline(is, tmpStr, ',');
    std::istringstream(tmpStr) >> obj.x;
    std::getline(is, tmpStr, ')');
    std::istringstream(tmpStr) >> obj.y;
    return is;
}

/** Two grid positions are equal iff they have the same row and column. */
inline bool operator==(CartesianCoordinates const &lhs, CartesianCoordinates const &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

/** Two grid positions are equal iff they have the same row and column. */
inline bool operator!=(CartesianCoordinates const &lhs, CartesianCoordinates const &rhs) {
    return lhs.x != rhs.x || lhs.y != rhs.y;
}

#endif /* CARTESIANCOORDINATES_HPP_ */
