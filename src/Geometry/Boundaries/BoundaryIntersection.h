#ifndef PMPL_BOUNDARY_INTERSECTION_H_
#define PMPL_BOUNDARY_INTERSECTION_H_

#include <cstddef>
#include <vector>

class Boundary;


/// Use rejection sampling to find a point on the intersection of two
/// boundaries.
/// @param _b1 The first boundary.
/// @param _b2 The second boundary.
/// @param _maxAttempts The maximum number of attempts.
/// @param _attempts Optionally returns the number of attempts made.
/// @return A point within both boundaries, or an empty vector if none was
///         found.
std::vector<double>
RejectionSampleIntersection(const Boundary* const _b1, const Boundary* const _b2,
    const size_t _maxAttempts = 10, size_t* const _attempts = nullptr) noexcept;


#endif
