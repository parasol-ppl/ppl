#ifndef PMPL_COMPOSE_COLLISION_H_
#define PMPL_COMPOSE_COLLISION_H_

#include "CollisionDetectionValidityMethod.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"

#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"

#include "nonstd/io.h"

#include <algorithm>


////////////////////////////////////////////////////////////////////////////////
/// Composed collision detector which applies two or more detection conditions.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ComposeCollision : public CollisionDetectionValidityMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfg;

    enum LogicalOperator {AND, OR}; ///< The supported logical operators.

    ///@}
    ///@name Construction
    ///@{

    ComposeCollision();

    ComposeCollision(XMLNode& _node);

    virtual ~ComposeCollision() = default;

    ///@}
    ///@name CollisionDetection Interface
    ///@{

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
    ///@name Internal State
    ///@{

    LogicalOperator m_operator; ///< The logical operator joining CD's.
    std::vector<std::string> m_cdLabels; ///< The CD labels to combine.

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ComposeCollision<MPTraits>::
ComposeCollision() {
  this->SetName("ComposeCollision");
}


template <typename MPTraits>
ComposeCollision<MPTraits>::
ComposeCollision(XMLNode& _node) 
  : CollisionDetectionValidityMethod<MPTraits>(_node) {
  this->SetName("ComposeCollision");

  std::string logicalOperator = _node.Read("operator", true, "", "operator");
  std::transform(logicalOperator.begin(), logicalOperator.end(),
                 logicalOperator.begin(), ::tolower);

  if(logicalOperator == "and")
    m_operator = AND;
  else if(logicalOperator == "or")
    m_operator = OR;
  else
    throw ParseException(_node.Where()) << "Operator '" << logicalOperator
                                        << "' is unknown.";

  for(auto& child : _node)
    if(child.Name() == "CollisionDetector")
      m_cdLabels.push_back(child.Read("label", true, "",
          "CollisionDetection method to include in this set."));

  if(m_cdLabels.size() < 2)
    throw ParseException(_node.Where()) << "Must specify at least two methods.";
}

/*---------------------- Collision Detection Overrides -----------------------*/

template <typename MPTraits>
bool
ComposeCollision<MPTraits>::
IsInsideObstacle(const Point3d& _p) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking inside obstacle..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(basevc);
        const bool inside = vc->IsInsideObstacle(_p);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (inside ? "" : "not ")
                    << "inside obstacle"
                    << std::endl;

        if(!inside)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(basevc);
        const bool inside = vc->IsInsideObstacle(_p);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (inside ? "" : "not ")
                    << "inside obstacle"
                    << std::endl;

        if(inside)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}


template <typename MPTraits>
bool
ComposeCollision<MPTraits>::
WorkspaceVisibility(const Point3d& _a, const Point3d& _b) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking workspace visibility..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(basevc);
        const bool visible = vc->WorkspaceVisibility(_a, _b);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (visible ? "" : "not ")
                    << "visible"
                    << std::endl;

        if(!visible)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(basevc);
        const bool visible = vc->WorkspaceVisibility(_a, _b);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (visible ? "" : "not ")
                    << "visible"
                    << std::endl;

        if(visible)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}


template <typename MPTraits>
bool
ComposeCollision<MPTraits>::
IsMultiBodyCollision(CDInfo& _cdInfo, const MultiBody* const _a,
    const MultiBody* const _b, const std::string& _caller) {

  if(this->m_debug)
    std::cout << "ComposeCollision:: checking multibody collision..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(basevc);
        const bool collision = vc->IsMultiBodyCollision(_cdInfo, _a, _b, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (collision ? "" : "not ")
                    << "collision"
                    << std::endl;

        if(!collision)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod<MPTraits>*>(basevc);
        const bool collision = vc->IsMultiBodyCollision(_cdInfo, _a, _b, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (collision ? "" : "not ")
                    << "collision"
                    << std::endl;

        if(collision)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}

/*--------------------- ValidityCheckerMethod Overrides ----------------------*/

template <typename MPTraits>
bool
ComposeCollision<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking validity..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto vc = this->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto vc = this->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}


template <typename MPTraits>
bool
ComposeCollision<MPTraits>::
IsValidImpl(GroupCfg& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking validity..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto vc = this->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto vc = this->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}

#endif
