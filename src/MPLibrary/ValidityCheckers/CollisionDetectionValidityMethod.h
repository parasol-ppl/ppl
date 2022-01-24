#ifndef PMPL_COLLISION_DETECTOR_VALIDITY_METHOD_H_
#define PMPL_COLLISION_DETECTOR_VALIDITY_METHOD_H_

#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"

#include "nonstd/io.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for Collision Detectors.
///
/// Collision detectors check for collisions between obstacles and multibodies.
/// They evaluate the validity of a configuration in terms of whether it is
/// in collision with another body.
///
/// A CDInfo object may carry additional information about the check.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CollisionDetectionValidityMethod : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfg;

    ///@}
    ///@name Construction
    ///@{

    CollisionDetectionValidityMethod() = default;

    CollisionDetectionValidityMethod(XMLNode& _node) 
      : ValidityCheckerMethod<MPTraits>(_node) {}

    virtual ~CollisionDetectionValidityMethod() = default;

    ///@}
    ///@name CollisionDetection Interface
    ///@{

    /// @return Collision Detection object
    virtual CollisionDetectionMethod* GetCDMethod() const noexcept 
      {return nullptr;};

    /// Determine whether a workspace point lies inside of an obstacle.
    /// @param _p The workspace point.
    /// @return True if _p is inside an obstacle.
    virtual bool IsInsideObstacle(const Point3d& _p) = 0;

    /// Check if two workspace points are mutually visible.
    /// @param _a The first point.
    /// @param _b The second point.
    /// @return True if _a is visible from _b and vice versa.
    virtual bool WorkspaceVisibility(const Point3d& _a, const Point3d& _b) = 0;

    /// Check for collision between two multibodies.
    /// @param _cdInfo CDInfo
    /// @param _a The first multibody.
    /// @param _b The second multibody.
    /// @param _caller Function calling validity checker.
    /// @return True if _a and _b collide in their present configurations.
    virtual bool IsMultiBodyCollision(CDInfo& _cdInfo, const MultiBody* const _a,
        const MultiBody* const _b, const std::string& _caller) = 0;

    ///@}
};

#endif