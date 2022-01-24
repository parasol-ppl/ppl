#ifndef PMPL_SKELETON_CLEARANCE_UTILITY_H_
#define PMPL_SKELETON_CLEARANCE_UTILITY_H_

#include <string>

#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/MPTools/MPTools.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceSkeleton.h"


////////////////////////////////////////////////////////////////////////////////
/// A utility for algorithms that improve the obstacle clearance of a workspace
/// skeleton.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SkeletonClearanceUtility final : public MPBaseObject<MPTraits> {

  ///@name Motion Planning Types
  ///@{

  typedef typename MPTraits::CfgType CfgType;

  ///@}
  ///@name Internal State
  ///@{

  std::string m_mauLabel; ///< The label of the medial axis tool to use.

  ///@}

  public:

    ///@name Construction
    ///@{

    SkeletonClearanceUtility();

    SkeletonClearanceUtility(XMLNode& _node);

    virtual ~SkeletonClearanceUtility() = default;

    ///@}
    ///@name Skeleton Fix
    ///@{

    /// Fix the skeleton clearance by pushing vertices and edge points to the
    /// medial axis.
    /// @param _skeleton The skeleton to adjust.
    void operator()(WorkspaceSkeleton& _skeleton) const;

    /// Fix the skeleton clearance by attempting to push vertices and edge points
    /// away from nearby obstacles.
    /// @note This is a hacky way to fix clearance that we used in the WAFR'16
    ///       paper Dynamic Region RRT. A medial axis push is the preferred way to
    ///       do this - this version is retained for reference and reproduction of
    ///       prior work only. It should not be used in new methods.
    /// @param _skeleton The skeleton to adjust.
    void HackFix(WorkspaceSkeleton& _skeleton) const;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
SkeletonClearanceUtility<MPTraits>::
SkeletonClearanceUtility() {
  this->SetName("SkeletonClearanceUtility");
}


template <typename MPTraits>
SkeletonClearanceUtility<MPTraits>::
SkeletonClearanceUtility(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("SkeletonClearanceUtility");

  m_mauLabel = _node.Read("mauLabel", true, "", "The medial axis utility to "
      "use.");
}

/*------------------------------- Skeleton Fix -------------------------------*/

template <typename MPTraits>
void
SkeletonClearanceUtility<MPTraits>::
operator()(WorkspaceSkeleton& _skeleton) const {
  MethodTimer mt(this->GetStatClass(), "SkeletonClearanceUtility");

  auto g = _skeleton.GetGraph();

  if(this->m_debug)
    std::cout << "Skeleton has " << g.get_num_vertices() << " vertices "
              << "and " << g.get_num_edges() << " edges."
              << "\n\tPushing vertices and edge points to the medial axis.\n";

  auto mau = this->GetMPTools()->GetMedialAxisUtility(m_mauLabel);
  auto boundary = this->GetEnvironment()->GetBoundary();
  auto robot = this->GetMPProblem()->GetRobot("point");

  // Define the push function
  auto push = [&](Point3d& _p) {
    CfgType cfg(_p, robot);

    // If success we use the new point instead
    // Else we failed to push current vertex to MA
    if(mau->PushToMedialAxis(cfg, boundary)) {
      if(this->m_debug)
        std::cout << "\tPushed " << _p << " to " << cfg.GetPoint() << ".\n";
      _p = cfg.GetPoint();
    }
    else if(this->m_debug)
      std::cout << "\tFailed to push " << _p << " to the medial axis.\n";
  };

  // Push flow graph vertices.
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    push(vit->property());

  // Push flowgraph edges.
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)
    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
      push(*pit);

  if(this->m_debug)
    std::cout << "\tSkeleton medial axis push complete." << std::endl;
}


template <typename MPTraits>
void
SkeletonClearanceUtility<MPTraits>::
HackFix(WorkspaceSkeleton& _skeleton) const {
  MethodTimer mt(this->GetStatClass(), "SkeletonClearanceUtility::HackFix");

  auto g = _skeleton.GetGraph();

  if(this->m_debug)
    std::cout << "Skeleton has " << g.get_num_vertices() << " vertices "
              << "and " << g.get_num_edges() << " edges."
              << "\n\tPushing vertices and edge points with low clearance away "
              << "from nearest obstacles.";

  const double robotRadius = this->GetTask()->GetRobot()->GetMultiBody()->
      GetBoundingSphereRadius();
  auto boundary   = this->GetEnvironment()->GetBoundary();
  auto pointRobot = this->GetMPProblem()->GetRobot("point");
  auto vc         = this->GetValidityChecker("pqp_solid");

  // This is a cheaper version of our clearance utility that is optimized for a
  // point and doesn't compute a witness. It finds the minimum clearance of the
  // input point _p.
  auto getClearanceInfo = [&](const Point3d& _p) -> pair<double, Point3d> {
    // Check against obstacles using a point robot.
    CfgType cfg(_p, pointRobot);
    const bool onlyConsiderBoundary = true;
    CDInfo cdInfo(!onlyConsiderBoundary);
    vc->IsValid(cfg, cdInfo, "Skeleton Push");

    // Check against boundary.
    const double boundaryClearance = boundary->GetClearance(_p);
    if(boundaryClearance < cdInfo.m_minDist) {
      cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
      cdInfo.m_minDist = boundaryClearance;
    }

    // Return the minimum clearance and nearest obstacle point.
    return make_pair(cdInfo.m_minDist, cdInfo.m_objectPoint);
  };

  auto push = [&](Point3d& _p) {
    if(this->m_debug)
      std::cout << "\n\t\tPushing from: " << setprecision(4) << _p
                << "\n\t\t          to: ";

    // Get clearance info for this point.
    auto initialClearance = getClearanceInfo(_p);
    const Point3d& objectPoint = initialClearance.second;

    // Check if we are at least one robot radius away from the nearest obstacle.
    const Vector3d w = _p - objectPoint;   // From obstacle to original point.
    const double wMag = w.norm();
    Vector3d t = w * (robotRadius / wMag); // As w, but one robot radius long.
    if(wMag >= t.norm()) {
      if(this->m_debug)
        std::cout << "(already clear)" << std::endl;
      return;
    }

    // Try to improve the clearance if we are too close.
    int tries = 3;
    while(tries-- && wMag < t.norm()) {
      // Set the new point as the nearest object point plus t.
      const Point3d newPoint = t + objectPoint;
      auto newClearance = getClearanceInfo(newPoint);

      // If t has better clearance than _p, push _p to t and quit.
      if(newClearance.first > initialClearance.first) {
        _p = t + objectPoint;
        if(this->m_debug)
          std::cout << _p << std::endl;
        return;
      }
      // Otherwise, cut the difference between t and w in half and retry.
      else
        t = (w + t) / 2.;
    }
    if(this->m_debug)
      std::cout << "(FAILED)" << std::endl;
  };

  // Push flowgraph vertices.
  for(auto vit = g.begin(); vit != g.end(); ++vit)
    push(vit->property());

  // Push flowgraph edges.
  for(auto eit = g.edges_begin(); eit != g.edges_end(); ++eit)
    for(auto pit = eit->property().begin(); pit < eit->property().end(); ++pit)
      push(*pit);

  if(this->m_debug)
    std::cout << "\n\tSkeleton clearance adjustment complete." << std::endl;
}

/*----------------------------------------------------------------------------*/

#endif
