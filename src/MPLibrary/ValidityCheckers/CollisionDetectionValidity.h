#ifndef PMPL_COLLISION_DETECTION_VALIDITY_H_
#define PMPL_COLLISION_DETECTION_VALIDITY_H_

#include "MPLibrary/ValidityCheckers/CollisionDetectionValidityMethod.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"

#include "Utilities/MetricUtils.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/SpheresCollisionDetection.h"

#include "nonstd/io.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// Classifies validity based on collisions with other objects in the workspace.
///
/// There are three types of collision that can occur:
/// 1. Self collision. Two or more pieces of the robot's geometry are
///    overlapping in workspace.
/// 2. Obstacle collision. The robot's geometry overlaps with some workspace
///    obstacle.
/// 3. Inter-robot collision. The robot's geometry overlaps with some other
///    robot in its current configuration.
/// For group configurations, collisions between robots within the group are
/// considered as self-collisions, while collisions with robots not in the group
/// are classified as inter-robot collisions. This distinction is made because
/// considering self-collisions usually means that we want (group)
/// configurations which are valid without considering the obstacles and other
/// robots.
///
/// Any robot can be omitted from the collision checks by setting it as virtual.
/// When collision checking a virtual robot, other robots are also ignored.
///
/// @note This class interfaces with external CD libraries to determine
///       collision information, sometimes including clearance and penetration
///       information.
///
/// @todo Remove the 'GetCDMethod' function after re-implementing
///       ObstacleClearanceValidity as a subtype of CollisionDetectionValidity.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CollisionDetectionValidity 
  : public CollisionDetectionValidityMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfg;
    typedef typename GroupCfg::Formation    Formation;

    ///@}
    ///@name Construction
    ///@{

    CollisionDetectionValidity();

    CollisionDetectionValidity(XMLNode& _node);

    virtual ~CollisionDetectionValidity();

    ///@}
    ///@name CollisionDetection Interface
    ///@{

    /// @return Collision Detection object
    virtual CollisionDetectionMethod* GetCDMethod() const noexcept override;

    /// Determine whether a workspace point lies inside of an obstacle.
    /// @param _p The workspace point.
    /// @return True if _p is inside an obstacle.
    virtual bool IsInsideObstacle(const Point3d& _p) override;

    /// Check if two workspace points are mutually visible.
    /// @param _a The first point.
    /// @param _b The second point.
    /// @return True if _a is visible from _b and vice versa.
    virtual bool WorkspaceVisibility(const Point3d& _a, const Point3d& _b) override;

    /// Check for collision between two multibodies.
    /// @param _cdInfo CDInfo
    /// @param _a The first multibody.
    /// @param _b The second multibody.
    /// @param _caller Function calling validity checker.
    /// @return True if _a and _b collide in their present configurations.
    virtual bool IsMultiBodyCollision(CDInfo& _cdInfo, const MultiBody* const _a,
        const MultiBody* const _b, const std::string& _caller) override;

    ///@}

  protected:

    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) override;

    virtual bool IsValidImpl(GroupCfg& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Orchestrate collision computation between robot and environment
    /// multibodies
    /// @param _cdInfo Output for collision detection info.
    /// @param _cfg Configuration of interest.
    /// @param _caller Name of the calling function.
    /// @return True if the robot is in collision at _cfg.
    virtual bool IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg,
        const std::string& _caller);

    /// Orchestrate collision computation between robots in a group cfg
    /// and environment multibodies
    /// @param _cdInfo Output for collision detection info.
    /// @param _cfg Group configuration of interest.
    /// @param _caller Name of the calling function.
    /// @return True if the robot group is in collision at _cfg.
    virtual bool IsInCollision(CDInfo& _cdInfo, const GroupCfg& _cfg,
        const std::string& _caller);

    /// Check if any of the robot's bodies are in collision with each other.
    /// @param _cdInfo Output for collision detection info. It will only be
    ///                updated if the detected collision is closer than the
    ///                previous.
    /// @param _multibody The robot's MultiBody.
    /// @param _caller Name of the calling function.
    /// @return True if the robot is in self-collision.
    virtual bool IsInSelfCollision(CDInfo& _cdInfo,
        const MultiBody* const _multibody, const std::string& _caller);

    /// Check if any of the robot's bodies are in collision with or outside the
    /// environment boundary.
    /// @param _cdInfo Output for collision detection info. It will only be
    ///                updated if the detected collision is closer than the
    ///                previous.
    /// @param _cfg The robot configuration.
    /// @return True if the robot is in self-collision.
    virtual bool IsInBoundaryCollision(CDInfo& _cdInfo, const CfgType& _cfg);

    /// Check if any of the robot's bodies are in collision with an obstacle.
    /// @param _cdInfo Output for collision detection info. It will only be
    ///                updated if the detected collision is closer than the
    ///                previous.
    /// @param _multibody The robot's MultiBody.
    /// @param _caller Name of the calling function.
    /// @return True if the robot is in collision with an obstacle.
    virtual bool IsInObstacleCollision(CDInfo& _cdInfo,
        const MultiBody* const _multibody, const std::string& _caller);

    /// Check for a collision between a query robot and a specific set of other
    /// robots.
    /// @param _robot The query robot.
    /// @param _robots The other robots.
    /// @param _caller Name of the calling function.
    virtual bool IsInInterRobotCollision(CDInfo& _cdInfo, Robot* const _robot,
        const std::vector<Robot*>& _robots, const std::string& _caller);

    ///@}
    ///@name Internal State
    ///@{

    ///< Underlying collision detection object.
    std::unique_ptr<CollisionDetectionMethod> m_cdMethod;

    bool m_ignoreSelfCollision{false};    ///< Check self collisions
    bool m_interRobotCollision{false};    ///< Check inter-robot collisions
    bool m_ignoreAdjacentLinks{false};    ///< Ignore adj links in self collisions
    bool m_ignoreSiblingCollisions{false}; ///< Ignore sibling links in self collisions

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity() 
  : CollisionDetectionValidityMethod<MPTraits>() {
  this->SetName("CollisionDetection");
}


template <typename MPTraits>
CollisionDetectionValidity<MPTraits>::
CollisionDetectionValidity(XMLNode& _node)
    : CollisionDetectionValidityMethod<MPTraits>(_node) {
  this->SetName("CollisionDetection");

  m_ignoreSelfCollision = _node.Read("ignoreSelfCollision", false,
      m_ignoreSelfCollision,
      "Ignore collisions between bodies of the same robot.");
  m_interRobotCollision = _node.Read("interRobotCollision", false,
      m_interRobotCollision,
      "Check for collisions between robots.");
  m_ignoreAdjacentLinks = _node.Read("ignoreAdjacentLinks", false,
      m_ignoreAdjacentLinks,
      "Ignore adjacent bodies in self-collision checks.");
  m_ignoreSiblingCollisions = _node.Read("ignoreSiblingCollisions", false,
      m_ignoreSiblingCollisions,
      "Ignore bodies that share a parent in self-collision checks.");

  const std::string cdLabel = _node.Read("method", true, "", "method");

  if(cdLabel == "BoundingSpheres")
    m_cdMethod.reset(new BoundingSpheres());
  else if(cdLabel == "InsideSpheres")
    m_cdMethod.reset(new InsideSpheres());
  else if(cdLabel == "RAPID")
    m_cdMethod.reset(new Rapid());
  else if(cdLabel == "PQP")
    m_cdMethod.reset(new PQP());
  else if(cdLabel == "PQP_SOLID")
    m_cdMethod.reset(new PQPSolid());
  else
    throw ParseException(_node.Where()) << "Unknown collision detection library '"
                                        << cdLabel << "' requested.";
}


template <typename MPTraits>
CollisionDetectionValidity<MPTraits>::
~CollisionDetectionValidity() = default;

/*---------------------- Collision Detection Interface -----------------------*/

template <typename MPTraits>
CollisionDetectionMethod*
CollisionDetectionValidity<MPTraits>::
GetCDMethod() const noexcept {
  return m_cdMethod.get();
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInsideObstacle(const Point3d& _p) {
  /// @todo Implement a bounding box check (per multibody and body) before
  ///       calling m_cdMethod.

  auto env = this->GetEnvironment();
  const size_t numObstacles = env->NumObstacles();

  // Check each obstacle.
  for(size_t i = 0; i < numObstacles; ++i) {
    auto obst = env->GetObstacle(i);

    // Check each body.
    for(size_t j = 0; j < obst->GetNumBodies(); ++j) {
      this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(),
          "IsInsideObstacle");

      const Body* const b = obst->GetBody(j);
      if(m_cdMethod->IsInsideObstacle(_p, b->GetPolyhedron(),
            b->GetWorldTransformation()))
        return true;
    }
  }

  return false;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
WorkspaceVisibility(const Point3d& _a, const Point3d& _b) {
  // Generate the third point for the triangle with _a, _b as side
  Vector3d p;
  auto direction = (_b -_a).normalize();

  // If line segment is not parallel to z-axis
  if(abs(direction*Vector3d(0,0,1))< 0.9)
    p = _a + Vector3d(0,0,1e-8);
  else
    p = _a + Vector3d(0,1e-8,0);

  // Create body geometry with a single triangle
  GMSPolyhedron line;
  line.GetVertexList() = std::vector<Vector3d>{{_a[0], _a[1], _a[2]},
      {_b[0], _b[1], _b[2]}, p};
  line.GetPolygonList() = std::vector<GMSPolygon>{GMSPolygon(0, 1, 2,
      line.GetVertexList())};
  mathtool::Transformation t; // I think this is identity?

  // Check collision of the triangle against every obstacle in the environment
  Environment* env = this->GetEnvironment();
  const size_t num = env->NumObstacles();
  CDInfo cdInfo;

  for(size_t i = 0; i < num; ++i) {
    auto obst = env->GetObstacle(i);

    for(size_t j = 0; j < obst->GetNumBodies(); ++j) {
      this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(),
          "WorkspaceVisibility");

      const Body* const b = obst->GetBody(j);
      if(m_cdMethod->IsInCollision(line, t,
                                   b->GetPolyhedron(),
                                   b->GetWorldTransformation(),
                                   cdInfo))
        return false;
    }
  }

  return true;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsMultiBodyCollision(CDInfo& _cdInfo, const MultiBody* const _a,
    const MultiBody* const _b, const std::string& _caller) {
  bool collision = false;
  const bool allInfo = _cdInfo.m_retAllInfo;
  CDInfo cdInfo(allInfo);

  // Check each body in _a against each body in _b.
  for(size_t i = 0; i < _a->GetNumBodies(); ++i) {
    const Body* const b1 = _a->GetBody(i);

    for(unsigned int j = 0; j < _b->GetNumBodies(); ++j) {
      cdInfo.ResetVars(allInfo);
      const Body* const b2 = _b->GetBody(j);
      collision |= m_cdMethod->IsInCollision(b1->GetPolyhedron(),
                                             b1->GetWorldTransformation(),
                                             b2->GetPolyhedron(),
                                             b2->GetWorldTransformation(),
                                             cdInfo);
      this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _caller);

      // Retain minimum distance information.
      if(cdInfo < _cdInfo) {
        _cdInfo = cdInfo;
        _cdInfo.m_collidingObstIndex = -1;
      }

      // Early quit if we do not care for distance information.
      if(collision and !allInfo)
        return true;
    }
  }

  return collision;
}

/*--------------------- ValidityCheckerMethod Overrides ----------------------*/

template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  this->GetStatClass()->IncCfgIsColl(_caller);

  // Position the robot within the environment.
  _cfg.ConfigureRobot();

  // Check for collisions.
  const bool valid = !IsInCollision(_cdInfo, _cfg, _caller);
  _cfg.SetLabel("VALID", valid);

  return valid;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsValidImpl(GroupCfg& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  this->GetStatClass()->IncCfgIsColl(_caller);

  // Position the robots within the environment.
  _cfg.ConfigureRobot();

  // Check for collisions.
  const bool valid = !IsInCollision(_cdInfo, _cfg, _caller);
  //_cfg.SetLabel("VALID");

  return valid;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const CfgType& _cfg, const std::string& _caller) {
  auto robot = _cfg.GetRobot();
  auto multibody = _cfg.GetMultiBody();
  const bool allInfo = _cdInfo.m_retAllInfo;
  _cdInfo.ResetVars(allInfo);

  if(this->m_debug)
    std::cout << "Checking robot '" << robot->GetLabel() << "' for collision."
              << "\n\tCfg: " << _cfg.PrettyPrint()
              << std::endl;

  bool collision = false;

  // Check for self-collision between the robot's bodies.
  if(!m_ignoreSelfCollision
      and (collision |= IsInSelfCollision(_cdInfo, multibody, _caller))
      and !allInfo)
    return true;

  // Check for containment within the environment boundary.
  if((collision |= IsInBoundaryCollision(_cdInfo, _cfg)) and !allInfo)
    return true;

  // Check for obstacle collisions.
  if((collision |= IsInObstacleCollision(_cdInfo, multibody, _caller))
      and !allInfo)
    return true;

  // Check for collision with all other robots.
  if(m_interRobotCollision and !robot->IsVirtual()) {
    // Unfortunately this involves an extra copy to avoid templating
    // IsInterRobotCollision, perhaps we can remove it in the future.
    std::vector<Robot*> robots;
    const auto& allRobots = this->GetMPProblem()->GetRobots();
    std::for_each(allRobots.begin(), allRobots.end(),
        [&robots](const std::unique_ptr<Robot>& _robot) {
          robots.push_back(_robot.get());
        }
    );

    collision |= IsInInterRobotCollision(_cdInfo, robot, robots, _caller);
  }

  return collision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInCollision(CDInfo& _cdInfo, const GroupCfg& _cfg, const std::string& _caller) {
  const bool allInfo = _cdInfo.m_retAllInfo;
  _cdInfo.ResetVars(allInfo);

  auto group = _cfg.GetGroupRoadmap()->GetGroup();
  const auto& robots = group->GetRobots();

  if(this->m_debug)
    std::cout << "Checking robot group '" << group->GetLabel()
              << "' for collision."
              << "\n\tCfg: " << _cfg.PrettyPrint()
              << std::endl;

  bool collision = false;

  // Check the group for self-collisions.
  if(!m_ignoreSelfCollision) {
    // Check each robot for self-collision.
    for(Robot* const robot : robots) {
      collision |= IsInSelfCollision(_cdInfo, robot->GetMultiBody(), _caller);

      // Early quit if we don't care for all collision info.
      if(!allInfo and collision)
        return true;
    }

    // Check for inter-robot collisions within the group.
    std::vector<Robot*> remainingRobots = robots;
    while(remainingRobots.size() > 1) {
      // Take the last remaining robot out of the set and check it against the
      // others.
      Robot* const r = remainingRobots.back();
      remainingRobots.pop_back();
      collision |= IsInInterRobotCollision(_cdInfo, r, remainingRobots, _caller);

      // Early quit if we don't care for all collision info.
      if(!allInfo and collision)
        return true;
    }
  }

  // Check the group for boundary containment.
  for(Robot* const robot : robots) {
    collision |= IsInBoundaryCollision(_cdInfo, _cfg.GetRobotCfg(robot));

    // Early quit if we don't care for all collision info.
    if(!allInfo and collision)
      return true;
  }


  // Check for obstacle collisions.
  for(Robot* const robot : robots) {
    collision |= IsInObstacleCollision(_cdInfo, robot->GetMultiBody(), _caller);

    // Early quit if we don't care for all collision info.
    if(!allInfo and collision)
      return true;
  }

  // We can quit now if we aren't using inter-robot collision, or if this group
  // has all robots in the problem.
  const auto& allRobots = this->GetMPProblem()->GetRobots();
  const size_t otherRobotCount = allRobots.size() - group->Size();
  if(!m_interRobotCollision or otherRobotCount == 0)
    return collision;

  // Find the set of robots which are not in this group.
  std::vector<Robot*> otherRobots;
  otherRobots.reserve(otherRobotCount);
  for(const auto& robot : allRobots)
    if(!group->VerifyRobotInGroup(robot.get()))
      otherRobots.push_back(robot.get());

  // Create a clearance map to track clearance info with otherRobots.
  CDClearanceMap clearanceMap;

  // Check for inter-robot collisions.
  for(Robot* const robot : robots) {
    collision |= IsInInterRobotCollision(_cdInfo, robot, otherRobots, _caller);

    // Early quit if we don't care for all collision info.
    if(!allInfo and collision)
      return true;

    // Update self-clearance.
    clearanceMap.Merge(_cdInfo.m_clearanceMap);
  }

  _cdInfo.m_clearanceMap = clearanceMap;

  return collision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInSelfCollision(CDInfo& _cdInfo, const MultiBody* const _multibody,
    const std::string& _caller) {
  // If the robot has only one body, then it cannot be in self-collision.
  if(_multibody->GetNumBodies() == 1)
    return false;

  const size_t numBody = _multibody->GetNumBodies();
  const bool allInfo = _cdInfo.m_retAllInfo;
  bool collision = false;

  for(size_t i = 0; i < numBody - 1; ++i) {
    const Body* const body1 = _multibody->GetBody(i);
    for(size_t j = i + 1; j < numBody; ++j) {
      const Body* const body2 = _multibody->GetBody(j);

      // Check for ignoring adjacent links.
      if(m_ignoreAdjacentLinks and body1->IsAdjacent(body2))
        continue;

      // Check for ignoring siblings.
      if(m_ignoreSiblingCollisions and body1->SameParent(body2))
        continue;

      CDInfo cdInfo(allInfo);
      const bool c = m_cdMethod->IsInCollision(body1->GetPolyhedron(),
                                               body1->GetWorldTransformation(),
                                               body2->GetPolyhedron(),
                                               body2->GetWorldTransformation(),
                                               cdInfo);
      collision |= c;
      this->GetStatClass()->IncNumCollDetCalls(m_cdMethod->GetName(), _caller);

      if(this->m_debug and c)
        std::cout << "\tSelf-collision detected between bodies " << i << " and "
                  << j << "." << std::endl;

      // Retain minimum distance information.
      if(cdInfo < _cdInfo) {
        _cdInfo = cdInfo;
        /// @todo -1 is the default value here, so this doesn't really work. We
        ///       need to expand our CDInfo object to handle this properly.
        _cdInfo.m_collidingObstIndex = -1;
      }

      // If we don't need all information, terminate on the first collision.
      if(!allInfo and collision)
        return true;
    }
  }

  return collision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInBoundaryCollision(CDInfo& _cdInfo, const CfgType& _cfg) {
  auto env = this->GetEnvironment();

  const bool inBounds = _cfg.InBounds(env);
  if(inBounds)
    return false;

  /// @todo This also doesn't work, we can't distinguish between no collision,
  ///       self collision, and boundary collision since we use -1 for all
  ///       three.
  _cdInfo.m_collidingObstIndex = -1;

  if(this->m_debug)
    std::cout << "\tOut-of-bounds detected."
              << "\n\t\tEnvironment boundary: " << *env->GetBoundary()
              << std::endl;

  return true;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInObstacleCollision(CDInfo& _cdInfo, const MultiBody* const _multibody,
    const std::string& _caller) {
  auto env = this->GetEnvironment();

  bool collision = false;
  const bool allInfo = _cdInfo.m_retAllInfo;
  const size_t numObst = env->NumObstacles();

  for(size_t i = 0; i < numObst; ++i) {
    CDInfo cdInfo(allInfo);
    const bool c = IsMultiBodyCollision(cdInfo, _multibody, env->GetObstacle(i),
        _caller);
    collision |= c;

    if(this->m_debug and c)
      std::cout << "\tCollision with obstacle " << i << " detected."
                << std::endl;

    // Store the nearest obstacle information. This is not really correct
    // because CDInfo::operator< is looking at m_minDist, which might represent
    // a distance to a self collision.
    if(cdInfo < _cdInfo) {
      _cdInfo = cdInfo;
      _cdInfo.m_nearestObstIndex = i;
      if(c)
        _cdInfo.m_collidingObstIndex = i;
    }

    // Early quit if we don't care for distance information.
    if(!allInfo and collision)
      return true;
  }

  return collision;
}


template <typename MPTraits>
bool
CollisionDetectionValidity<MPTraits>::
IsInInterRobotCollision(CDInfo& _cdInfo, Robot* const _robot,
    const std::vector<Robot*>& _robots, const std::string& _caller) {
  // If the robot is virtual, it cannot be in inter-robot collision.
  if(_robot->IsVirtual())
    return false;

  const bool allInfo = _cdInfo.m_retAllInfo;
  bool collision = false;

  for(Robot* const otherRobot : _robots) {
    // Skip self-checks and checks against virtual robots.
    if(_robot == otherRobot or otherRobot->IsVirtual())
      continue;

    // Perform the collision check.
    CDInfo cdInfo(allInfo);
    const bool c = IsMultiBodyCollision(cdInfo,
        _robot->GetMultiBody(), otherRobot->GetMultiBody(), _caller);
    collision |= c;

    if(this->m_debug and c) {
      CfgType cfg2(otherRobot);
      cfg2.SetData(otherRobot->GetMultiBody()->GetCurrentDOFs());

      std::cout << "\tInter-robot collision detected:"
                << "\n\t\tRobot: " << otherRobot->GetLabel()
                << "\n\t\tCfg: " << cfg2.PrettyPrint()
                << std::endl;
    }

    // Retain lowest CD info.
    if(cdInfo < _cdInfo) {
      _cdInfo = cdInfo;
      /// @todo Arg, more ambiguous data. See above.
      _cdInfo.m_collidingObstIndex = -1;
    }

    // Early quit if we don't care for distance information.
    if(!allInfo and collision)
      return true;
  }

  return collision;
}

/*----------------------------------------------------------------------------*/

#endif
