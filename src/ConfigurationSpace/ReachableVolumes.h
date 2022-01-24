#ifndef PMPL_REACHABLE_VOLUMES_H_
#define PMPL_REACHABLE_VOLUMES_H_

class Chain;
class WorkspaceBoundingSphericalShell;


/// Compute the reachable volume of a chain.
/// @param _dimension The workspace dimension.
/// @param _chain The chain.
/// @return The reachable volume of _chain's end relative to its root.
WorkspaceBoundingSphericalShell
ComputeReachableVolume(const size_t _dimension,
    const std::vector<double>& _center, const Chain& _chain);

#endif
