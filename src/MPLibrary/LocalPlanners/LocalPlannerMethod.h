#ifndef PMPL_LOCAL_PLANNER_METHOD_H_
#define PMPL_LOCAL_PLANNER_METHOD_H_

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/GroupLPOutput.h"
#include "MPLibrary/MPBaseObject.h"
#include "MPProblem/Environment/Environment.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref LocalPlanners.
///
/// LocalPlannerMethod has two main functions: @c IsConnected and @c BlindPath.
///
/// @c IsConnected takes as input two configurations \f$c_1\f$, \f$c_2\f$, an
/// LPOutput, validation resolutions, and optional booleans dictating whether to
/// check collision and save the path. The function both returns true or false
/// to validate the simple path between \f$c_1\f$ and \f$c_2\f$, but also
/// populates the LPOutput structure with useful information.
///
/// @c BlindPath is a helper to plan between waypoints without collision
/// checking. This is useful for working with pre-validated paths.
///
/// @todo All local planners need to use a distance metric to set their edge
///       weights properly; currently many of them are simply using the number
///       of intermediate steps as a weight.
/// @todo Local planners and Extenders represent the same concepts and should be
///       merged into a single class with both an Extend and LocalPlan function.
///       This will help simplify several other objects within PMPL as well,
///       such as bi-directional RRT.
///
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LocalPlannerMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;
    typedef typename GroupCfgType::Formation Formation;

    ///@}
    ///@name Construction
    ///@{

    LocalPlannerMethod(bool _saveIntermediates = false);

    LocalPlannerMethod(XMLNode& _node);

    virtual ~LocalPlannerMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Local Planner Interface
    ///@{

    /// Validate a simple path between two nodes.
    /// @param _start The starting configuration.
    /// @param _end The ending configuration.
    /// @param _col The witness configuration on failure.
    /// @param _lpOutput Weight and path computed from local plan.
    /// @param _posRes Positional DOF resolution.
    /// @param _oriRes Rotational DOF resolution.
    /// @param _checkCollision Use validity checking?
    /// @param _savePath Save all configurations along the path?
    /// @return A boolean indicating whether the connection succeeded.
    ///
    /// @usage
    /// @code
    /// LocalPlannerPointer lp = this->GetLocalPlanner(m_lpLabel);
    /// Environment* env = this->GetEnvironment();
    /// CfgType c1, c2, col;
    /// LPOutput<MPTraits> lpOut;
    /// lp->IsConnected(c1, c2, col, &lpOut, env->GetPositionRes(),
    ///     env->GetOrientationRes());
    /// @endcode
    virtual bool IsConnected(const CfgType& _start, const CfgType& _end,
        CfgType& _col, LPOutput<MPTraits>* _lpOutput, double _posRes,
        double _oriRes, bool _checkCollision = true, bool _savePath = false) = 0;
    ///@example LocalPlanner_UseCase.cpp
    /// This is an example of how to use the local planner methods.

    /// This version does not return a collision node.
    /// @overload
    bool IsConnected(const CfgType& _start, const CfgType& _end,
        LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false);

    /// This version is for group configurations.
    /// @overload
    virtual bool IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _col, GroupLPOutput<MPTraits>* _lpOutput, double _posRes,
        double _oriRes, bool _checkCollision = true, bool _savePath = false,
        const Formation& _formation = Formation());

    /// This version for group configurations does not return a collision node.
    /// @overload
    bool IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupLPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false,
        const Formation& _formation = Formation());

    /// Blind-plan between a set of waypoints. Also useful for reconstructing
    /// previously validated edges.
    /// @param _waypoints The waypoint configurations.
    /// @param _posRes    Positional DOF resolution.
    /// @param _oriRes    Rotational DOF resolution.
    /// @return Configurations along path through _waypoints up to a resolution
    ///         (_posRes, _oriRes). Does not include the first or last
    ///         configuration.
    ///
    /// @usage
    /// @code
    /// LocalPlannerPointer lp = this->GetLocalPlanner(m_lpLabel);
    /// Environment* env = this->GetEnvironment();
    /// CfgType c1, c2;
    /// lp->BlindPath({c1, c2}, env->GetPositionRes(), env->GetOrientationRes());
    /// @endcode
    std::vector<CfgType> BlindPath(const std::vector<CfgType>& _waypoints,
        const double _posRes, const double _oriRes);

    /// @overload This version assumes the environment's resolution values.
    std::vector<CfgType> BlindPath(const std::vector<CfgType>& _waypoints);

    /// @overload This version is for group configurations.
    /// @param _formation A (possibly improper) subset of the robot group which
    ///                   should remain in its relative formation while moving.
    ///                   Only makes sense if these robots are in the same
    ///                   formation at both _start and _end.
    std::vector<GroupCfgType> BlindPath(
        const std::vector<GroupCfgType>& _waypoints, const double _posRes,
        const double _oriRes, const Formation& _formation = Formation());

    /// @overload This version assumes the environment's resolution values.
    std::vector<GroupCfgType> BlindPath(
        const std::vector<GroupCfgType>& _waypoints,
        const Formation& _formation = Formation());

    ///@}

  protected:

    ///@name Internal State
    ///@{

    bool m_saveIntermediates{false}; ///< Save the intermediates in the roadmap?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LocalPlannerMethod<MPTraits>::
LocalPlannerMethod(const bool _saveIntermediates)
  : m_saveIntermediates(_saveIntermediates)
{ }


template <typename MPTraits>
LocalPlannerMethod<MPTraits>::
LocalPlannerMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  m_saveIntermediates = _node.Read("saveIntermediates", false,
      m_saveIntermediates, "Save intermediate nodes");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
LocalPlannerMethod<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tSave intermediates: " << m_saveIntermediates
      << std::endl;
}

/*------------------------ LocalPlanner Interface ----------------------------*/

template <typename MPTraits>
inline
bool
LocalPlannerMethod<MPTraits>::
IsConnected(const CfgType& _start, const CfgType& _end,
    LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath) {
  CfgType col(_start.GetRobot());
  return IsConnected(_start, _end, col, _lpOutput, _posRes,
      _oriRes, _checkCollision, _savePath);
}


template <typename MPTraits>
bool
LocalPlannerMethod<MPTraits>::
IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _col,
    GroupLPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, const Formation& _formation) {
  throw NotImplementedException(WHERE) << "No default implementation provided.";
}


template <typename MPTraits>
inline
bool
LocalPlannerMethod<MPTraits>::
IsConnected(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupLPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
    bool _checkCollision, bool _savePath, const Formation& _formation) {
  GroupCfgType col(_start.GetGroupRoadmap());
  return IsConnected(_start, _end, col, _lpOutput, _posRes,
                     _oriRes, _checkCollision, _savePath, _formation);
}


template <typename MPTraits>
std::vector<typename MPTraits::CfgType>
LocalPlannerMethod<MPTraits>::
BlindPath(const std::vector<CfgType>& _waypoints,
    const double _posRes, const double _oriRes) {
  // Blind local-plan between each intermediate,
  bool first = true;
  std::vector<CfgType> out;
  LPOutput<MPTraits> lpOutput;
  for(auto iter = _waypoints.begin(); iter + 1 != _waypoints.end(); ++iter) {
    // If this isn't the first configuration, then its an intermediate and must
    // be added to the path.
    if(!first)
      out.push_back(*iter);
    else
      first = false;

    // Reconstruct the resolution-level edge and append it to the output.
    lpOutput.Clear();
    IsConnected(*iter, *(iter + 1), &lpOutput, _posRes, _oriRes, false, true);
    out.insert(out.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  }
  return out;
}


template <typename MPTraits>
std::vector<typename MPTraits::CfgType>
LocalPlannerMethod<MPTraits>::
BlindPath(const std::vector<CfgType>& _waypoints) {
  auto env = this->GetEnvironment();
  return BlindPath(_waypoints, env->GetPositionRes(), env->GetOrientationRes());
}


template <typename MPTraits>
std::vector<typename MPTraits::GroupCfgType>
LocalPlannerMethod<MPTraits>::
BlindPath(const std::vector<GroupCfgType>& _waypoints, const double _posRes,
    const double _oriRes, const Formation& _formation) {
  // Blind local-plan between each intermediate,
  bool first = true;
  std::vector<GroupCfgType> out;
  GroupLPOutput<MPTraits> lpOutput(_waypoints.at(0).GetGroupRoadmap());
  for(auto iter = _waypoints.begin(); iter + 1 != _waypoints.end(); ++iter) {
    // If this isn't the first configuration, then its an intermediate and must
    // be added to the path.
    if(!first)
      out.push_back(*iter);
    else
      first = false;

    // Reconstruct the resolution-level edge and append it to the output.
    lpOutput.Clear();
    IsConnected(*iter, *(iter + 1), &lpOutput, _posRes, _oriRes, false, true);
    out.insert(out.end(), lpOutput.m_path.begin(), lpOutput.m_path.end());
  }
  return out;
}


template <typename MPTraits>
std::vector<typename MPTraits::GroupCfgType>
LocalPlannerMethod<MPTraits>::
BlindPath(const std::vector<GroupCfgType>& _waypoints,
    const Formation& _formation) {
  auto env = this->GetEnvironment();
  return BlindPath(_waypoints, env->GetPositionRes(), env->GetOrientationRes(),
      _formation);
}

/*----------------------------------------------------------------------------*/
#endif
