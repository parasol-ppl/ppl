#include "Geometry/Bodies/Chain.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphericalShell.h"
#include "MPProblem/Robot/Robot.h"

#include "ReachableVolumes.h"
#include "Transformation.h"

#include <algorithm>


/// Compute the unconstrained reachable distance of one joint in a linear chain,
/// relative to its parent.
/// @param _dimension The workspace dimension.
/// @param _joint The joint to compute the reachable distance for.
/// @param _parent The parent of _joint in the current ordering of _chain.
/// @param _chain The chain which holds _joint and _parent (may be forward or
///               reverse oriented relative to the multibody).
/// @return The reachable distance of _joint relative to _parent.
double
ComputeReachableDistanceOfSingleLink(const size_t _dimension,
    const Connection* _joint, const Connection* _parent, const Chain* _chain) {
  // Check that the joint type is supported.
  const Connection::JointType frontJointType = _joint->GetConnectionType();
  switch(frontJointType) {
    case Connection::JointType::Revolute:
    {
      if(_dimension == 3)
        throw RunTimeException(WHERE) << "Revolute joints are not supported "
                                      << "for 3D workspaces (requires "
                                      << "Directed Reachable Volumes).";
      break;
    }
    case Connection::JointType::Spherical:
    {
      if(_dimension == 2)
        throw RunTimeException(WHERE) << "Spherical joints are not supported "
                                      << "for 2D workspaces.";
      break;
    }
    default:
    {
      /// @todo We can probably support some non-actuated joints here, but
      ///       need to verify with the theory.
      throw RunTimeException(WHERE) << "Unsupported joint type '"
                                    << frontJointType
                                    << "' connecting bodies "
                                    << _joint->GetPreviousBodyIndex()
                                    << " and "
                                    << _joint->GetNextBodyIndex() << ".";
    }
  }

  // Make sure the joint does not have a translational offset in the DH frame.
  const auto& dh = _joint->GetDHParameters();
  if(dh.m_a != 0 or dh.m_d != 0)
    throw RunTimeException(WHERE) << "Reachable volumes do not handle joints "
                                  << "with offsets in the DH params. "
                                  << "Connection between bodies "
                                  << _joint->GetNextBodyIndex()
                                  << " and "
                                  << _joint->GetPreviousBodyIndex()
                                  << " has a = " << dh.m_a
                                  << ", d = " << dh.m_d << ".";

  // _joint is OK for reachable volumes. Compute its reachable distance
  // relative to _parent.
  if(_chain->IsForward())
    // If the chain is forward-oriented, then _parent is the parent of
    // _joint in the multibody.
    return _joint->GetTransformationToDHFrame().translation().norm() +
           _parent->GetTransformationToBody2().translation().norm();
  else
    // If the chain is backward-oriented, then _joint is the parent of
    // _parent in the multibody.
    return _joint->GetTransformationToBody2().translation().norm() +
           _parent->GetTransformationToDHFrame().translation().norm();
}


WorkspaceBoundingSphericalShell
ComputeReachableVolume(const size_t _dimension,
    const std::vector<double>& _center, const Chain& _chain) {
  // Initialize minimum and maximum reachable distances.
  double min = 0,
         max = 0;

  // If the chain includes a front body, include it in the reachable distance
  // computation.
  if(_chain.GetFrontBody()) {

    if(_chain.GetFrontBody()->IsBase())
      max += _chain.GetFirstJoint()->GetTransformationToDHFrame().translation().
             norm();
    else
      max += _chain.GetFirstJoint()->GetTransformationToBody2().translation().
             norm();
    min = max;
  }

  // Include the reachable distances for each joint in the chain.
  for(auto parentJoint = _chain.begin(), childJoint = _chain.begin() + 1;
      childJoint != _chain.end(); ++parentJoint, ++childJoint) {
    // Compute the unconstrained reachable distance of the child joint relative
    // to its parent.
    const double rd = ComputeReachableDistanceOfSingleLink(
        _dimension, *childJoint, *parentJoint, &_chain);

    // Update the chain's minimum and maximum reachable distance with Minknowski
    // sum.
    min = std::max(0., (min > rd) ? min - rd : rd - max);
    max += rd;
  }

  // If the chain includes the end-effector, include it in the reachable
  // distance computation.
  if(_chain.GetBackBody()) {
    double rd;
    if(_chain.GetBackBody()->IsBase())
      rd = _chain.GetLastJoint()->GetTransformationToDHFrame().translation().
          norm();
    else
      rd = _chain.GetLastJoint()->GetTransformationToBody2().translation().
          norm();
    min = std::max(0., (min > rd) ? min - rd : rd - max);
    max += rd;
  }

  return WorkspaceBoundingSphericalShell(_center, max, min);
}
