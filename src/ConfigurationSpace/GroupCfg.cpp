#include "GroupCfg.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "nonstd.h"


/*------------------------------- Construction -------------------------------*/

GroupCfg::
GroupCfg(GroupRoadmapType* const _groupMap, const bool _init)
    : m_groupMap(_groupMap) {

  // If no group map was given, this is a placeholder object. We can't do
  // anything with it since every meaningful operation requires a group map.
  if(!m_groupMap)
    return;

  // Set the VID list to all invalid.
  m_vids.resize(GetNumRobots(), INVALID_VID);

  // Initialize local configurations if requested.
  if(_init)
    InitializeLocalCfgs();
}


/*--------------------------------- Equality ---------------------------------*/

bool
GroupCfg::
operator==(const GroupCfg& _other) const noexcept {
  // If _other is from another map, these are not the same.
  if(m_groupMap != _other.m_groupMap)
    return false;

  // Else, compare VIDs if both are valid, or by-value other wise.
  for(size_t i = 0; i < m_vids.size(); ++i) {
    const VID thisVID  = m_vids[i],
              otherVID = _other.m_vids[i];

    if(thisVID != INVALID_VID and otherVID != INVALID_VID) {
      if(thisVID != otherVID)
        return false;
    }
    else if(GetRobotCfg(i) != _other.GetRobotCfg(i))
      return false;
  }

  return true;
}


bool
GroupCfg::
operator!=(const GroupCfg& _other) const noexcept {
  return !(*this == _other);
}

/*-------------------------------- Arithmetic --------------------------------*/

GroupCfg
GroupCfg::
operator+(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg += _other);
}


GroupCfg
GroupCfg::
operator-(const GroupCfg& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg -= _other);
}


GroupCfg
GroupCfg::
operator*(const double& _other) const {
  GroupCfg newCfg = *this;
  return (newCfg *= _other);
}


GroupCfg&
GroupCfg::
operator+=(const GroupCfg& _other) {
  // We must require the exact same group, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(m_groupMap->GetGroup() != _other.m_groupMap->GetGroup())
    throw RunTimeException(WHERE, "Cannot add GroupCfgs with different group "
                                  "roadmaps!");

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < GetNumRobots(); ++i)
    SetRobotCfg(i, GetRobotCfg(i) + _other.GetRobotCfg(i));

  return *this;
}


GroupCfg&
GroupCfg::
operator-=(const GroupCfg& _other) {
  // We must require the exact same group roadmap, which indicates everything
  // lines up between the two cfgs (namely the exact robots/order of the group).
  if(m_groupMap != _other.m_groupMap)
    throw RunTimeException(WHERE, "Cannot add GroupCfgs with different group "
                                  "roadmaps!");

  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < GetNumRobots(); ++i)
    SetRobotCfg(i, GetRobotCfg(i) - _other.GetRobotCfg(i));

  return *this;
}


GroupCfg&
GroupCfg::
operator*=(const double& _val) {
  // We will be using the local cfgs, as we don't want to require any cfgs that
  // use this operator to have to add cfgs to roadmaps.
  for(size_t i = 0; i < GetNumRobots(); ++i)
    SetRobotCfg(i, GetRobotCfg(i) * _val);

  return *this;
}


/*---------------------------------- Robots ----------------------------------*/

size_t
GroupCfg::
GetNumRobots() const noexcept {
  return m_groupMap ? m_groupMap->GetGroup()->Size() : 0;
}


const std::vector<Robot*>&
GroupCfg::
GetRobots() const noexcept {
  return m_groupMap->GetGroup()->GetRobots();
}


Robot*
GroupCfg::
GetRobot(const size_t _index) const {
  VerifyIndex(_index);

  Robot* const robot = m_groupMap->GetGroup()->GetRobot(_index);

  /// @todo Remove this after we are very sure things are working.
  if(!robot)
    throw RunTimeException(WHERE) << "Error! Robot pointer was null.";

  return robot;
}

/*---------------------------- Roadmap Accessors -----------------------------*/

GroupCfg::GroupRoadmapType*
GroupCfg::
GetGroupRoadmap() const noexcept {
  return m_groupMap;
}


GroupCfg
GroupCfg::
SetGroupRoadmap(GroupRoadmapType* const _newRoadmap) const {
  // Check that groups are compatible.
  if(m_groupMap->GetGroup() != _newRoadmap->GetGroup())
    throw RunTimeException(WHERE) << "Trying to change roadmaps on incompatible "
                                  << "groups!";

  // Create new cfg using _roadmap and initializing all entries locally to 0.
  GroupCfg newCfg(_newRoadmap);

  // Put all individual cfgs into group cfg so that all are local:
  for(size_t i = 0; i < GetNumRobots(); ++i)
    newCfg.SetRobotCfg(i, IndividualCfg(GetRobotCfg(i)));

  return newCfg;
}


GroupCfg::VID
GroupCfg::
GetVID(const size_t _index) const noexcept {
  VerifyIndex(_index);
  return m_vids[_index];
}

/*------------------------ Individual Configurations -------------------------*/

void
GroupCfg::
SetRobotCfg(Robot* const _robot, IndividualCfg&& _cfg) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  SetRobotCfg(index, std::move(_cfg));
}


void
GroupCfg::
SetRobotCfg(const size_t _index, IndividualCfg&& _cfg) {
  VerifyIndex(_index);

  // Allocate space for local cfgs if not already done.
  InitializeLocalCfgs();

  m_localCfgs[_index] = std::move(_cfg);
  m_vids[_index] = INVALID_VID;
}


void
GroupCfg::
SetRobotCfg(Robot* const _robot, const VID _vid) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  SetRobotCfg(index, _vid);
}


void
GroupCfg::
SetRobotCfg(const size_t _index, const VID _vid) {
  VerifyIndex(_index);

  m_vids[_index] = _vid;
}


void
GroupCfg::
ClearLocalCfgs() {
  m_localCfgs.clear();
}


GroupCfg::IndividualCfg&
GroupCfg::
GetRobotCfg(Robot* const _robot) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


GroupCfg::IndividualCfg&
GroupCfg::
GetRobotCfg(const size_t _index) {
  VerifyIndex(_index);

  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupMap->GetRoadmap(_index)->GetVertex(vid);
  else {
    InitializeLocalCfgs();
    return m_localCfgs[_index];
  }
}


const GroupCfg::IndividualCfg&
GroupCfg::
GetRobotCfg(Robot* const _robot) const {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  return GetRobotCfg(index);
}


const GroupCfg::IndividualCfg&
GroupCfg::
GetRobotCfg(const size_t _index) const {
  VerifyIndex(_index);

  // If we have a valid VID for this robot, fetch its configuration from its
  // individual roadmap.
  const VID vid = GetVID(_index);
  if(vid != INVALID_VID)
    return m_groupMap->GetRoadmap(_index)->GetVertex(vid);

  try {
    return m_localCfgs.at(_index);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Requested configuration for robot "
                                  << _index
                                  << ", but no roadmap or local cfg exists.";
  }
}

/*------------------------------ DOF Accessors -------------------------------*/

size_t
GroupCfg::
PosDOF(const size_t _index) const {
  return GetRobot(_index)->GetMultiBody()->PosDOF();
}


size_t
GroupCfg::
OriDOF(const size_t _index) const {
  return GetRobot(_index)->GetMultiBody()->OrientationDOF();
}


size_t
GroupCfg::
DOF(const size_t _index) const {
  return GetRobot(_index)->GetMultiBody()->DOF();
}


bool
GroupCfg::
IsNonholonomic() const noexcept {
  for(auto robot : GetRobots())
    if(robot->IsNonholonomic())
      return true;
  return false;
}


size_t
GroupCfg::
CompositeDOF() const {
  size_t dofSum = 0;
  for(auto robot : GetRobots())
    dofSum += robot->GetMultiBody()->DOF();
  return dofSum;
}


double
GroupCfg::
Magnitude() const {
  double result = 0;
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    const double m = GetRobotCfg(i).Magnitude();
    result += m * m;
  }
  return std::sqrt(result);
}


double
GroupCfg::
PositionMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    const double m = GetRobotCfg(i).PositionMagnitude();
    result += m * m;
  }
  return std::sqrt(result);
}


double
GroupCfg::
OrientationMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    const double m = GetRobotCfg(i).OrientationMagnitude();
    result += m * m;
  }
  return std::sqrt(result);
}

/*------------------------- Configuration Helpers ----------------------------*/

void
GroupCfg::
ConfigureRobot() const {
  for(size_t i = 0; i < GetNumRobots(); ++i)
    GetRobotCfg(i).ConfigureRobot();
}


bool
GroupCfg::
WithinResolution(const GroupCfg& _cfg, const double _posRes,
    const double _oriRes) const {
  for(size_t i = 0; i < GetNumRobots(); ++i)
    if(!GetRobotCfg(i).WithinResolution(_cfg.GetRobotCfg(i), _posRes, _oriRes))
      return false;

  return true;
}

/*------------------------------DOF Modifiers---------------------------------*/

void
GroupCfg::
RotateFormationAboutLeader(const Formation& _robotList,
    const mathtool::Orientation& _rotation, const bool _debug) {
  /// Note: Currently assumes all robots just have ONE body. The case of multi-
  /// bodied robots would need to be specially handled (right now it should just
  /// be split into multiple robots if a group is needed).

  /// @todo We can probably compute this without having to configure the models
  ///       (which cost a lot of transformations).
  ConfigureRobot(); // Configure all individual cfgs.

  const size_t leaderIndex = _robotList[0];

  // Get transformation of leader before rotation:
  const IndividualCfg& leaderCfg = GetRobotCfg(leaderIndex);

  // TODO update this to handle multiple bodies per robot.
  // Use the multibody's body 0, since we assume each MB just has a single body.
  mathtool::Transformation initialLeaderTransform = leaderCfg.GetMultiBody()->
                                           GetBody(0)->GetWorldTransformation();

  const mathtool::Transformation rotation(mathtool::Vector3d(0,0,0), _rotation);

  if(_debug)
    std::cout << "Rotating bodies " << _robotList << " with rotation = "
              << rotation << std::endl;

  // The transform to be applied to all parts (including the first one). We
  // move the part to its relative world position with A at the world origin,
  // then the rotation is applied, and we return the part to its relative
  // position from A.
  const mathtool::Transformation transform = initialLeaderTransform * rotation;

  ApplyTransformationForRobots(_robotList, transform, initialLeaderTransform);
}


void
GroupCfg::
ApplyTransformationForRobots(const Formation& _robotList,
    const mathtool::Transformation& _transform,
    const mathtool::Transformation& _relativeTransform) {
  //Compute each robot's needed transformation and set dofs in cfg.
  for (const size_t robotIndex : _robotList) {
    const IndividualCfg& robotCfg = GetRobotCfg(robotIndex);

    /// @todo Generalize this to handle robots with more than one body.
    if(robotCfg.GetMultiBody()->GetNumBodies() > 1)
      throw RunTimeException(WHERE) << "Multiple bodies not supported!";

    // Retrieve current position and rotation of robot:
    const mathtool::Transformation& initialRobotTransform =
                  robotCfg.GetMultiBody()->GetBody(0)->GetWorldTransformation();

    // From right to left: apply the inverse relative transform to the initial
    // robot transform (puts the robot into the desired frame). Then apply
    // the transform given.
    const mathtool::Transformation newTransformation =   _transform *
                                                       (-_relativeTransform) *
                                                         initialRobotTransform;

    // Extract the transformation. Note: This is assuming 6 DOFs!
    const std::vector<double>& transformed = newTransformation.GetCfg();

//    OverwriteDofsForRobots(transformed, _robotList);
    OverwriteDofsForRobots(transformed, {robotIndex});
  }
}


void
GroupCfg::
AddDofsForRobots(const std::vector<double>& _dofs, const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    if(IsLocalCfg(robotIndex)) {
      // We can simply modify the local values, since it's not a roadmap cfg yet
      IndividualCfg& cfg = GetRobotCfg(robotIndex);

      // Ensure this robot has the correct number of DOF.
      if(_dofs.size() != cfg.DOF())
        throw RunTimeException(WHERE) << "Tried to add " << _dofs.size()
                                      << "dofs to robot " << robotIndex
                                      << ", which has " << cfg.DOF() << " DOFs.";

      // Update the robot's cfg.
      for(unsigned int i = 0; i < _dofs.size(); ++i)
        cfg[i] += _dofs[i];
    }
    else {
      // Must copy the cfg since it is not local.
      IndividualCfg cfg = GetRobotCfg(robotIndex);

      // Ensure this robot has the correct number of DOF.
      if(_dofs.size() != cfg.DOF())
        throw RunTimeException(WHERE) << "Tried to add " << _dofs.size()
                                      << "dofs to robot " << robotIndex
                                      << ", which has " << cfg.DOF() << " DOFs.";

      // Update the robot's cfg.
      for(unsigned int i = 0; i < _dofs.size(); ++i)
        cfg[i] += _dofs[i];
      SetRobotCfg(robotIndex, std::move(cfg));
    }
  }
}


void
GroupCfg::
AddDofsForRobots(const mathtool::Vector3d& _dofs, const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg robotCfg = GetRobotCfg(robotIndex);
    for(size_t i = 0; i < robotCfg.PosDOF(); ++i)
      robotCfg[i] += _dofs[i];
    SetRobotCfg(robotIndex, std::move(robotCfg));
  }
}


void
GroupCfg::
OverwriteDofsForRobots(const std::vector<double>& _dofs,
    const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(GetRobot(robotIndex));
    newIndividualCfg.SetData(_dofs);
    SetRobotCfg(robotIndex, std::move(newIndividualCfg));
  }
}


void
GroupCfg::
OverwriteDofsForRobots(const mathtool::Vector3d& _dofs,
    const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg newIndividualCfg(GetRobot(robotIndex));
    newIndividualCfg.SetLinearPosition(_dofs);
    SetRobotCfg(robotIndex, std::move(newIndividualCfg));
  }
}


void
GroupCfg::
OverwriteDofsForRobots(const GroupCfg& _fromCfg, const Formation& _robots) {
  for(const size_t robotIndex : _robots) {
    IndividualCfg robotCfg = _fromCfg.GetRobotCfg(robotIndex);
    SetRobotCfg(robotIndex, std::move(robotCfg));
  }
}


void
GroupCfg::
OverwriteDofsForRobots(const GroupCfg& _fromCfg,
    const std::vector<Robot*>& _robots) {
  auto fromGroup = _fromCfg.GetGroupRoadmap()->GetGroup(),
       toGroup   = m_groupMap->GetGroup();

  for(Robot* const robot : _robots) {
    const size_t fromIndex = fromGroup->GetGroupIndex(robot),
                 toIndex   = toGroup->GetGroupIndex(robot);
    IndividualCfg robotCfg = _fromCfg.GetRobotCfg(fromIndex);
    SetRobotCfg(toIndex, std::move(robotCfg));
  }
}


void
GroupCfg::
SetData(const std::vector<double>& _dofs) {
  if(_dofs.size() != CompositeDOF())
    throw RunTimeException(WHERE) << "Tried to set " << _dofs.size()
                                  << " DOFs on a robot group with "
                                  << CompositeDOF() << " DOFs.";

  size_t compositeIndex = 0;
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    const size_t robotDof = DOF(i);
    IndividualCfg& robotCfg = GetRobotCfg(i);

    for(size_t i = 0; i < robotDof; ++i, ++compositeIndex)
      robotCfg[i] = _dofs[compositeIndex];
  }
}


void
GroupCfg::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal, const int _nTicks) {
  // Need positive number of ticks.
  if(_nTicks <= 0)
    throw RunTimeException(WHERE) << "Divide by 0";
  if(_start.m_groupMap != _goal.m_groupMap)
    throw RunTimeException(WHERE) << "Cannot use two different groups (or group "
                                  << "roadmaps) with this operation currently!";

  // For each robot in the group, find the increment for the individual cfg
  // given the number of ticks found.
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    IndividualCfg incr(GetRobot(i));
    incr.FindIncrement(_start.GetRobotCfg(i), _goal.GetRobotCfg(i), _nTicks);
    SetRobotCfg(i, std::move(incr));
  }
}


void
GroupCfg::
FindIncrement(const GroupCfg& _start, const GroupCfg& _goal, int* const _nTicks,
    const double _positionRes, const double _orientationRes) {
  const GroupCfg diff = _goal - _start;

  *_nTicks = std::max(1., std::ceil(std::max(
                      diff.PositionMagnitude() / _positionRes,
                      diff.OrientationMagnitude() / _orientationRes)));

  FindIncrement(_start, _goal, *_nTicks);
}


bool
GroupCfg::
InBounds(const Boundary* const _b) const noexcept {
  for(size_t i = 0; i < GetNumRobots(); ++i)
    if(!GetRobotCfg(i).InBounds(_b))
      return false;

  return true;
}


bool
GroupCfg::
InBounds(const Environment* const _env) const noexcept {
  return InBounds(_env->GetBoundary());
}


void
GroupCfg::
NormalizeOrientation(const Formation& _robots) noexcept {
  if(_robots.empty()) // Do all robots in this case.
    for(size_t i = 0; i < GetNumRobots(); ++i)
      GetRobotCfg(i).NormalizeOrientation();
  else
    for(size_t i : _robots)
      GetRobotCfg(i).NormalizeOrientation();
}

/*------------------------------ Output Helpers ------------------------------*/

std::string
GroupCfg::
PrettyPrint(const size_t _precision) const {
  std::ostringstream oss;
  oss.precision(_precision);
  oss << "{ ";
  for(size_t i = 0; i < GetNumRobots(); ++i) {
    const IndividualCfg& robotCfg = GetRobotCfg(i);
    if(IsLocalCfg(i))
      oss << "Local: ";
    oss << robotCfg.PrettyPrint(_precision) << ", ";
  }
  oss << " }";

  return oss.str();
}

/*----------------------------------------------------------------------------*/

bool
GroupCfg::
IsLocalCfg(const size_t _robotIndex) const noexcept {
  // Only true if there is local data (meaning INVALID_VID is present)
  return m_vids[_robotIndex] == INVALID_VID;
}


void
GroupCfg::
InitializeLocalCfgs() noexcept {
  // We will assume the local cfgs are initialized if the container size is
  // correct.
  const size_t numRobots = GetNumRobots();
  if(m_localCfgs.size() == numRobots)
    return;

  m_localCfgs.resize(numRobots);

  for(size_t i = 0; i < numRobots; ++i)
    SetRobotCfg(i, IndividualCfg(GetRobot(i)));
}


inline
void
GroupCfg::
VerifyIndex(const size_t _robotIndex) const noexcept {
  if(_robotIndex >= GetNumRobots())
    throw RunTimeException(WHERE) << "Requested data for robot " << _robotIndex
                                  << ", but the group has only " << GetNumRobots()
                                  << " robots.";
}

/*----------------------------------------------------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const GroupCfg& _groupCfg) {
  // Might not need to be hidden behind GROUP_MAP, but doing for consistency
#ifdef GROUP_MAP
  _os << "0 ";
#endif

  // Loop through all robots in the group and print each one's cfg in order.
  for(size_t i = 0; i < _groupCfg.GetNumRobots(); ++i)
    _os << _groupCfg.GetRobotCfg(i);

  return _os;
}

/*----------------------------------------------------------------------------*/
