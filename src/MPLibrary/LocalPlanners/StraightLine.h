#ifndef PMPL_STRAIGHT_LINE_H_
#define PMPL_STRAIGHT_LINE_H_

#include "ConfigurationSpace/GroupRoadmap.h"

#include "LocalPlannerMethod.h"
#include "GroupLPOutput.h"
#include "LPOutput.h"
#include "MPProblem/IsClosedChain.h"
#include "Transformation.h"
#include "Vector.h"
#include "nonstd.h"

#include <queue>

#include <boost/utility/enable_if.hpp>


////////////////////////////////////////////////////////////////////////////////
/// Check a straight-line path in c-space for valididty.
///
/// This local planner validates straight line paths which is a direct linear
/// interpolation between two configurations in @cspace.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class StraightLine : public LocalPlannerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::WeightType       WeightType;

    typedef typename MPTraits::GroupCfgType     GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename GroupCfgType::Formation    Formation;

    ///@}
    ///@name Construction
    ///@{

    StraightLine(const std::string& _vcLabel = "", bool _binary = false,
        bool _saveIntermediates = false);

    StraightLine(XMLNode& _node);

    virtual ~StraightLine() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name LocalPlannerMethod Overrides
    ///@{

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false) override;

    virtual bool IsConnected(
        const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
        GroupLPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false,
        const Formation& _robotIndexes = Formation()) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Default for non closed chains
    bool IsConnectedFunc(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Check if two Cfgs could be connected by straight line.
    /// This method implements straight line connection local planner
    /// by checking collision of each Cfg along the line.
    /// If the is any Cfg causes Robot collides with any obstacle,
    /// false will be returned.
    virtual bool IsConnectedSLSequential(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    /// Check if two Cfgs could be connected by straight line
    /// This method uses binary search to check clearances of Cfgs between _c1
    /// and _c2.
    virtual bool IsConnectedSLBinary(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_dmLabel;          ///< The metric for measuring edge length.
    std::string m_vcLabel;          ///< The validity checker.
    bool m_binaryEvaluation{false}; ///< Use binary search?

		double m_selfEdgeSteps{1};

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
StraightLine<MPTraits>::
StraightLine(const std::string& _vcLabel, bool _binary, bool _saveIntermediates)
  : LocalPlannerMethod<MPTraits>(_saveIntermediates),
    m_vcLabel(_vcLabel), m_binaryEvaluation(_binary) {
  this->SetName("StraightLine");
}


template <typename MPTraits>
StraightLine<MPTraits>::
StraightLine(XMLNode& _node) : LocalPlannerMethod<MPTraits>(_node) {
  this->SetName("StraightLine");

  m_binaryEvaluation = _node.Read("binaryEvaluation", false, m_binaryEvaluation,
      "Use binary search to evaluate the edge, or linear scan if false.");

  m_dmLabel = _node.Read("dmLabel", false, "euclidean",
      "The distance metric for computing edge length.");

  m_vcLabel = _node.Read("vcLabel", true, "", "The validity checker to use.");

	m_selfEdgeSteps = _node.Read("selfEdgeSteps", false, m_selfEdgeSteps,0.0,10000.0, 
																"Number of increments in a self edge.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
StraightLine<MPTraits>::
Print(std::ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tbinary evaluation = " << (m_binaryEvaluation ? "true" : "false")
      << "\n\tdmLabel = " << m_dmLabel
      << "\n\tvcLabel = " << m_vcLabel
      << std::endl;
}

/*----------------------- LocalPlannerMethod Overrides -----------------------*/

template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  const std::string id = this->GetNameAndLabel();
  MethodTimer mt(this->GetStatClass(), id + "::IsConnected");

  if(this->m_debug)
    std::cout << id
              << "\n\tChecking line from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << "\n\tUsing " << (m_binaryEvaluation ? "binary" : "sequential")
              << " evaluation."
              << std::endl;

  // Initialize the LPOutput object.
  _lpOutput->Clear();
  _lpOutput->SetLPLabel(this->GetLabel());

  const bool connected = IsConnectedFunc(_c1, _c2,
      _col, _lpOutput, _positionRes, _orientationRes, _checkCollision,
      _savePath);

  auto stats = this->GetStatClass();
  stats->IncLPAttempts(id);
  stats->IncLPConnections(id, connected);

  if(this->m_debug)
    std::cout << "\n\tLocal Plan is "
              << (connected ? "valid" : "invalid at " + _col.PrettyPrint())
              << std::endl;

  return connected;
}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnected(const GroupCfgType& _c1, const GroupCfgType& _c2, GroupCfgType& _col,
    GroupLPOutput<MPTraits>* _lpOutput, double _positionRes,
    double _orientationRes, bool _checkCollision, bool _savePath,
    const Formation& _robotIndexes) {
  const std::string id = this->GetNameAndLabel();
  auto stats = this->GetStatClass();
  MethodTimer(stats, id + "::IsConnectedFunc");

  if(this->m_debug) {
    std::cout << id
              << "\n\tChecking line from " << _c1.PrettyPrint()
              << " to " << _c2.PrettyPrint()
              << std::endl;
    if(!_robotIndexes.empty())
      std::cout << "\tUsing formation: " << _robotIndexes << std::endl;
  }

  // Initialize the LPOutput object.
  _lpOutput->Clear();
  _lpOutput->SetLPLabel(this->GetLabel());

  auto env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto groupMap = _c1.GetGroupRoadmap();

  // Determine whether multiple robots are moving and whether this is a
  // formation rotation (rotation about some leader robot).
  const bool multipleParts = _robotIndexes.size() > 1;
  const bool isRotational = _c1.OriDOF() > 0;
  const bool formationRotation = multipleParts && isRotational;
  const size_t leaderRobotIndex = _robotIndexes.empty() ? size_t(-1)
                                                        : _robotIndexes[0];

  // Will find all the straight-line increments for each robot independently.
  // (Though the numSteps calculation is coupled with all moving robots).
  int numSteps;
  GroupCfgType increment(groupMap);
  increment.FindIncrement(_c1, _c2, &numSteps, _positionRes, _orientationRes);

  const GroupCfgType originalIncrement = increment;

  // Set up increment for all translating bodies, should there be more than one.
  if(multipleParts) {
    // Remove the rotational bits, as increment should only do the translation
    // and then RotateFormationAboutLeader() will handle all rotations:
    increment = GroupCfgType(groupMap, true);

    // Overwrite all positional dofs from the leader's cfg for all active robots
    increment.OverwriteDofsForRobots(
        originalIncrement.GetRobotCfg(leaderRobotIndex).GetLinearPosition(),
        _robotIndexes);
  }

  bool connected = true;
  int cdCounter = 0;
  GroupCfgType currentStep(_c1),
               leaderStep(_c1),
               previousStep(groupMap);
  for(int i = 1; i < numSteps; ++i) {
    previousStep = currentStep;
    currentStep += increment;

    // Handle rotation of a formation. We will determine the rotation applied to
    // the leader robot and cause the others to rotate about it, maintaining
    // their realtive formation
    if(formationRotation) {
      /// @todo This can likely be optimized. For one, only one Configure call
      ///       should be necessary here. Also a lot of the group Cfgs here
      ///       could be made individual if using the leader, then using
      ///       Configure on that.

      // Advance the leader currentStep by the original increment (we will only use
      // data which is set in the leader body).
      leaderStep += originalIncrement;

      // Find the previousStep transformation of the leader robot's base.
      previousStep.ConfigureRobot();
      mathtool::Transformation initialTransform =
          previousStep.GetRobot(leaderRobotIndex)->GetMultiBody()->GetBase()->
          GetWorldTransformation();

      // Find the new transformation of the leader robot's base.
      leaderStep.ConfigureRobot();
      mathtool::Transformation finalTransform =
          leaderStep.GetRobot(leaderRobotIndex)->GetMultiBody()->GetBase()->
          GetWorldTransformation();

      // Find the relative transformation of the leader robot's base. This holds
      // the rotation to be applied to currentStep, which only increments
      // position in this case.
      mathtool::Transformation delta = -initialTransform * finalTransform;
      currentStep.RotateFormationAboutLeader(_robotIndexes, delta.rotation(),
          this->m_debug);
    }

    // Check collision if requested.
    if(_checkCollision) {
      ++cdCounter;
      const bool inBounds = currentStep.InBounds(env->GetBoundary());
      if(!inBounds or !vc->IsValid(currentStep, id)) {
        _col = currentStep;
        connected = false;
        break;
      }
    }

    // Save the resolution-level path if requested.
    if(_savePath)
      _lpOutput->m_path.push_back(currentStep);
  }

  // Set data in the LPOutput object.
  _lpOutput->m_edge.first.SetWeight(numSteps);
  _lpOutput->m_edge.second.SetWeight(numSteps);
  _lpOutput->SetIndividualEdges(_robotIndexes);
  _lpOutput->SetActiveRobots(_robotIndexes);

  if(connected)
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);

  // Track usage stats.
  stats->IncLPAttempts(id);
  stats->IncLPConnections(id, connected);
  stats->IncLPCollDetCalls(id, cdCounter);

  if(this->m_debug)
    std::cout << "\n\tLocal Plan is "
              << (connected ? "valid" : "invalid at " + _col.PrettyPrint())
              << std::endl;
  return connected;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedFunc(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  int cdCounter = 0;
  bool connected;
  if(m_binaryEvaluation)
    connected = IsConnectedSLBinary(_c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);
  else
    connected = IsConnectedSLSequential(_c1, _c2, _col, _lpOutput,
        cdCounter, _positionRes, _orientationRes, _checkCollision, _savePath);

  this->GetStatClass()->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return connected;
}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedSLSequential(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  auto robot = _c1.GetRobot();
  // If there is no robot pointer, this is assumed to be an interaction edge.
  /// @todo Why are we running a local planner on an interaction edge? That
  ///       shouldn't be necessary since we need neither intermediates nor CD
  ///       information in that case?
  if(!robot)
    return true;

  Environment* env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  const std::string id = this->GetNameAndLabel() + "::IsConnectedSLSequential";

  // Compute the number of resolution-level steps required to transition from
  // _c1 to _c2.
  int numSteps;
  CfgType increment(robot);
	if(_c1 == _c2) {
		numSteps = m_selfEdgeSteps;
		increment.FindIncrement(_c1, _c2, m_selfEdgeSteps);
	}
	else {
  	increment.FindIncrement(_c1, _c2, &numSteps, _positionRes, _orientationRes);
	}
  if(this->m_debug)
    std::cout << "\n\tComputed increment for " << numSteps << " steps: "
              << increment.PrettyPrint() << std::endl;

  // Step from _c1 by a distance increment, either numSteps - 1 times or until a
  // collision is detected (-1 because we the final step will land at _c2, which
  // we don't need to check).
  CfgType currentStep = _c1,
          previousStep;
  double distance = 0;
  for(int i = 1; i < numSteps; ++i) {
    // Update the current step.
    previousStep = currentStep;
    currentStep += increment;

    // Add the step distance to the total distance.
    distance += dm->Distance(previousStep, currentStep);

    // // Check for collisions.
    // if(_checkCollision) {
    //   _cdCounter++;
    //   if(this->m_debug)
    //     std::cout << "\n\t\tChecking step " << i << " at "
    //               << currentStep.PrettyPrint()
    //               << std::endl;

    //   const bool inBounds = currentStep.InBounds(env);
    //   if(!inBounds || !vc->IsValid(currentStep, id)) {
    //     _col = currentStep;

    //     if(this->m_debug)
    //       std::cout << "\n\t\t\tINVALID" << std::endl;
    //     return false;
    //   }
    //   else if(this->m_debug)
    //     std::cout << "\n\t\t\tOK" << std::endl;
    // }


    // Check for collisions. 
    // TODO: out of bounds and obst-space are treated the same.
    // This might be problematic.

    if(_checkCollision) {
      _cdCounter++;
      if(this->m_debug)
        std::cout << "\n\t\tChecking step " << i << " at "
                  << currentStep.PrettyPrint()
                  << std::endl;

      const bool inBounds = currentStep.InBounds(env);
      bool valid = false;

      //in case the validity is switched for toggle operations.
      const bool validity = vc->GetValidity();
      
      if (inBounds)
        valid = vc->IsValid(currentStep, id);
      
      if (!valid) {
        _col = currentStep;
        _col.SetLabel("VALID", valid and validity);
        return false;
      }
      
      if(this->m_debug)
          std::cout << "\n\t\t\t" << (valid ? "VALID" : "INVALID" ) << std::endl;
    }


    // Save the resolution-level path if requested.
    if(_savePath)
      _lpOutput->m_path.push_back(currentStep);
  }

  // The edge is valid. Add the distance to the final configuration.
  distance += dm->Distance(currentStep, _c2);

  auto& edge1 = _lpOutput->m_edge.first,
      & edge2 = _lpOutput->m_edge.second;

  edge1.SetWeight(edge1.GetWeight() + distance);
  edge2.SetWeight(edge2.GetWeight() + distance);
  edge1.SetTimeSteps(edge1.GetTimeSteps() + numSteps);
  edge2.SetTimeSteps(edge2.GetTimeSteps() + numSteps);

  return true;
}


template <typename MPTraits>
bool
StraightLine<MPTraits>::
IsConnectedSLBinary(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput, int& _cdCounter,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  // If there is no robot pointer, this is assumed to be an interaction edge.
  /// @todo Why are we running a local planner on an interaction edge? That
  ///       shouldn't be necessary since we need neither intermediates nor CD
  ///       information in that case?
  auto robot = _c1.GetRobot();
  if(!robot)
    return true;

  Environment* env = this->GetEnvironment();
  auto vc = this->GetValidityChecker(m_vcLabel);
  auto dm = this->GetDistanceMetric(m_dmLabel);
  const std::string id = this->GetNameAndLabel() + "::IsConnectedSLBinary";

  // Compute the number of resolution-level steps required to transition from
  // _c1 to _c2.
  int numSteps;
  CfgType increment(robot);
	if(_c1 == _c2) {
		numSteps = m_selfEdgeSteps;
		increment.FindIncrement(_c1, _c2, m_selfEdgeSteps);
	}
	else {
  	increment.FindIncrement(_c1, _c2, &numSteps, _positionRes, _orientationRes);
	}
  if(this->m_debug)
    std::cout << "\n\tComputed increment for " << numSteps << " steps: "
              << increment.PrettyPrint()
              << std::endl;

  // Create a queue of step intervals. Each one represents a set of steps from
  // (low step number) to (high step number), with the step numbers ranging from
  // 0 (source cfg) to numSteps (target cfg). We will check the midpoint of each
  // interval until a collision is found or all have been checked.
  std::queue<std::pair<int,int>> queue;

  // Only perform binary evaluation when the nodes are further apart than a
  // resolution step.
  if(numSteps > 1)
    queue.emplace(0, numSteps);

  while(!queue.empty()) {
    // Get the next interval to check.
    std::pair<int,int> interval = queue.front();
    queue.pop();

    // Extract the low and high step.
    int low = interval.first;
    int high = interval.second;

    // Compute the middle step and corresponding configuration.
    int mid = low + (high - low) / 2;
    CfgType midCfg = increment * mid + _c1;

    // // Check collision if requested.
    // if(_checkCollision) {
    //   _cdCounter++;
    //   if(this->m_debug)
    //     std::cout << "\n\t\tChecking step " << mid << " at "
    //               << midCfg.PrettyPrint()
    //               << std::endl;

    //   const bool inBounds = midCfg.InBounds(env);
    //   if(!inBounds or !vc->IsValid(midCfg, id)) {
    //     _col = midCfg;
    //     if(this->m_debug)
    //       std::cout << "\n\t\t\tINVALID" << std::endl;
    //     return false;
    //   }
    // }
    // else if(this->m_debug)
    //   std::cout << "\n\t\t\tOK" << std::endl;


    // Check for collisions.
    // TODO: out of bounds and obst-space are treated the same. 
    // This might be problematic.

    if(_checkCollision) {
      _cdCounter++;
      if(this->m_debug)
        std::cout << "\n\t\tChecking step " << mid << " at "
                  << midCfg.PrettyPrint()
                  << std::endl;

      const bool inBounds = midCfg.InBounds(env);
      bool valid = false;

      //in case the validity is switched for toggle operations.
      const bool validity = vc->GetValidity();
      

      if (inBounds)
        valid = vc->IsValid(midCfg, id);
      
      if (!valid) {
        _col = midCfg;
        _col.SetLabel("VALID", !validity);
        return false;
      }
      
      if(this->m_debug)
          std::cout << "\n\t\t\t" << (valid ? "VALID" : "INVALID" ) << std::endl;
    }

    // If there is at least one step between low and mid, add the interval from
    // low to mid to the queue.
    if(low + 1 != mid)
      queue.emplace(low, mid);
    // Same for the mid to high interval.
    if(mid + 1 != high)
      queue.emplace(mid, high);
  }

  // All steps have been validated. Compute the distance and path.
  double distance = 0;
  CfgType currentStep = _c1,
          previousStep;
  for(int n = 1; n < numSteps; ++n) {
    previousStep = currentStep;
    currentStep += increment;
    distance += dm->Distance(previousStep, currentStep);

    // Save the resolution-level path if requested.
    if(_savePath)
      _lpOutput->m_path.push_back(currentStep);
  }
  distance += dm->Distance(currentStep, _c2);

  auto& edge1 = _lpOutput->m_edge.first,
      & edge2 = _lpOutput->m_edge.second;

  edge1.SetWeight(edge1.GetWeight() + distance);
  edge2.SetWeight(edge2.GetWeight() + distance);
  edge1.SetTimeSteps(edge1.GetTimeSteps() + numSteps);
  edge2.SetTimeSteps(edge2.GetTimeSteps() + numSteps);

  return true;
}

/*----------------------------------------------------------------------------*/

#endif
