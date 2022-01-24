#include "Cfg.h"

#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MetricUtils.h"
#include "nonstd.h"

using namespace std;
using mathtool::Vector3d;
using mathtool::Transformation;
using mathtool::EulerAngle;


/*-------------------------------- Construction ------------------------------*/

Cfg::
Cfg() : Cfg(inputRobot) { }


Cfg::
Cfg(Robot* const _robot) : m_robot(_robot) {
  if(m_robot) {
    m_dofs.resize(DOF(), 0);
    if(m_robot->IsNonholonomic())
      m_vel.resize(DOF(), 0);
  }
}


Cfg::
Cfg(const Vector3d& _v, Robot* const _robot) : m_robot(_robot) {
  if(m_robot) {
    m_dofs.resize(DOF(), 0);
    for(size_t i = 0; i < PosDOF(); ++i)
      m_dofs[i] = _v[i];
    if(m_robot->IsNonholonomic())
      m_vel.resize(DOF(), 0);
  }
}


Cfg::
Cfg(const Cfg& _other) :
    m_clearanceInfo(_other.m_clearanceInfo),
    m_witnessCfg(_other.m_witnessCfg),
    m_dofs(_other.m_dofs),
    m_vel(_other.m_vel),
    m_robot(_other.m_robot),
    m_labelMap(_other.m_labelMap),
    m_statMap(_other.m_statMap) { }


Cfg::
Cfg(Cfg&& _other) :
    m_clearanceInfo(std::move(_other.m_clearanceInfo)),
    m_witnessCfg(std::move(_other.m_witnessCfg)),
    m_dofs(std::move(_other.m_dofs)),
    m_vel(std::move(_other.m_vel)),
    m_robot(_other.m_robot),
    m_labelMap(std::move(_other.m_labelMap)),
    m_statMap(std::move(_other.m_statMap)) { }


Cfg::
~Cfg() = default;

/*------------------------------- Assignment ---------------------------------*/

Cfg&
Cfg::
operator=(const Cfg& _cfg) {
  if(this != &_cfg) {
    m_dofs.clear();
    m_dofs = _cfg.GetData();
    m_vel.clear();
    m_vel = _cfg.GetVelocity();
    m_labelMap = _cfg.m_labelMap;
    m_statMap = _cfg.m_statMap;
    m_robot = _cfg.m_robot;
    m_clearanceInfo = _cfg.m_clearanceInfo;
    m_witnessCfg = _cfg.m_witnessCfg;
  }
  return *this;
}


Cfg&
Cfg::
operator=(Cfg&& _cfg) {
  if(this != &_cfg) {
    m_dofs.clear();
    m_dofs = std::move(_cfg.m_dofs);
    m_vel.clear();
    m_vel = std::move(_cfg.m_vel);
    m_labelMap = std::move(_cfg.m_labelMap);
    m_statMap = std::move(_cfg.m_statMap);
    m_robot = _cfg.m_robot;
    m_clearanceInfo = std::move(_cfg.m_clearanceInfo);
    m_witnessCfg = std::move(_cfg.m_witnessCfg);
  }
  return *this;
}


Cfg&
Cfg::
operator+=(const Cfg& _cfg) {
  const size_t posDOF = PosDOF(), oriDOF = OriDOF(), dof = DOF();

  // Positional dofs
  for(size_t i = 0; i < posDOF; ++i)
    m_dofs[i] += _cfg[i];
  // Orientation dofs
  for(size_t i = posDOF; i < posDOF + oriDOF; ++i)
    m_dofs[i] = m_normalizer(m_dofs[i] + _cfg[i]);
  // Joint dofs
  for(size_t i = posDOF + oriDOF; i < dof; ++i)
    m_dofs[i] += _cfg[i];

  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] += _cfg.m_vel[i];

  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator-=(const Cfg& _cfg) {
  const size_t posDOF = PosDOF(), oriDOF = OriDOF(), dof = DOF();

  /// @TODO For optimal runtime, comment out this condition. Assembly planning
  ///       needs this case handled differently to make sure we get all dofs.
  if(!GetMultiBody()->IsComposite()) {
    // Positional dofs
    for(size_t i = 0; i < posDOF; ++i)
      m_dofs[i] -= _cfg[i];
    // Orientation dofs
    for(size_t i = posDOF; i < posDOF + oriDOF; ++i)
      m_dofs[i] = m_normalizer(m_dofs[i] - _cfg[i]);
    // Joint dofs
    for(size_t i = posDOF + oriDOF; i < dof; ++i)
      m_dofs[i] -= _cfg[i];
  }
  else {
    const MultiBody* const mb = GetMultiBody();
    for(size_t i = 0; i < dof; ++i) {
      if(mb->GetDOFType(i) == DofType::Rotational)
        m_dofs[i] = m_normalizer(m_dofs[i] - _cfg[i]);
      else
        m_dofs[i] -= _cfg[i];
    }
  }

  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] -= _cfg.m_vel[i];

  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator*=(const Cfg& _cfg) {
  const size_t posDOF = PosDOF(), oriDOF = OriDOF(), dof = DOF();

  // Positional dofs
  for(size_t i = 0; i < posDOF; ++i)
    m_dofs[i] *= _cfg[i];
  // Orientation dofs
  for(size_t i = posDOF; i < posDOF + oriDOF; ++i)
    m_dofs[i] = m_normalizer(m_dofs[i] * _cfg[i]);
  // Joint dofs
  for(size_t i = posDOF + oriDOF; i < dof; ++i)
    m_dofs[i] *= _cfg[i];

  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] *= _cfg.m_vel[i];

  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator/=(const Cfg& _cfg) {
  const size_t posDOF = PosDOF(), oriDOF = OriDOF(), dof = DOF();

  // Positional dofs
  for(size_t i = 0; i < posDOF; ++i)
    m_dofs[i] /= _cfg[i];
  // Orientation dofs
  for(size_t i = posDOF; i < posDOF + oriDOF; ++i)
    m_dofs[i] = m_normalizer(m_dofs[i] / _cfg[i]);
  // Joint dofs
  for(size_t i = posDOF + oriDOF; i < dof; ++i)
    m_dofs[i] /= _cfg[i];

  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] /= _cfg.m_vel[i];

  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator*=(const double _d) {
  const size_t posDOF = PosDOF(), oriDOF = OriDOF(), dof = DOF();

  // Positional dofs
  for(size_t i = 0; i < posDOF; ++i)
    m_dofs[i] *= _d;
  // Orientation dofs
  for(size_t i = posDOF; i < posDOF + oriDOF; ++i)
    m_dofs[i] = m_normalizer(m_dofs[i] * _d);
  // Joint dofs
  for(size_t i = posDOF + oriDOF; i < dof; ++i)
    m_dofs[i] *= _d;

  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] *= _d;

  m_witnessCfg.reset();
  return *this;
}


Cfg&
Cfg::
operator/=(const double _d) {
  const size_t posDOF = PosDOF(), oriDOF = OriDOF(), dof = DOF();

  // Positional dofs
  for(size_t i = 0; i < posDOF; ++i)
    m_dofs[i] /= _d;
  // Orientation dofs
  for(size_t i = posDOF; i < posDOF + oriDOF; ++i)
    m_dofs[i] = m_normalizer(m_dofs[i] / _d);
  // Joint dofs
  for(size_t i = posDOF + oriDOF; i < dof; ++i)
    m_dofs[i] /= _d;

  for(size_t i = 0; i < m_vel.size(); ++i)
    m_vel[i] /= _d;

  m_witnessCfg.reset();
  return *this;
}

/*------------------------------- Arithmetic ---------------------------------*/

Cfg
Cfg::
operator-() const {
  Cfg result = *this;
  return result *= -1;
}


Cfg
Cfg::
operator+(const Cfg& _cfg) const {
  Cfg result = *this;
  return result += _cfg;
}


Cfg
Cfg::
operator-(const Cfg& _cfg) const {
  Cfg result = *this;
  return result -= _cfg;
}


Cfg
Cfg::
operator*(const Cfg& _cfg) const {
  Cfg result = *this;
  return result *= _cfg;
}


Cfg
Cfg::
operator/(const Cfg& _cfg) const {
  Cfg result = *this;
  return result /= _cfg;
}


Cfg
Cfg::
operator*(const double _d) const {
  Cfg result = *this;
  return result *= _d;
}


Cfg
Cfg::
operator/(const double _d) const {
  Cfg result = *this;
  return result /= _d;
}

/*-------------------------------- Equality ----------------------------------*/

bool
Cfg::
operator==(const Cfg& _cfg) const {
  // First check for same robot pointer.
  if(m_robot != _cfg.m_robot)
    return false;

  static constexpr double tolerance = 100
                                    * std::numeric_limits<double>::epsilon();

  // Check the velocities.
  for(size_t i = 0; i < m_vel.size(); ++i)
    if(std::abs(m_vel[i] - _cfg.m_vel[i]) > tolerance)
      return false;

  // Check everything else (joints, position, and rotational dofs).
  return WithinResolution(_cfg, tolerance, tolerance);
}


bool
Cfg::
operator!=(const Cfg& _cfg) const {
  return !(*this == _cfg);
}

/*-------------------------------Comparison-----------------------------------*/

bool
Cfg::
operator<(const Cfg& _cfg) const {
  // Check DOFs.
  for(size_t i = 0; i < m_dofs.size(); ++i) {
    if(m_dofs[i] < _cfg.m_dofs[i])
      return true;
    else if(m_dofs[i] > _cfg.m_dofs[i])
      return false;
  }

  // Check velocities.
  for(size_t i = 0; i < m_vel.size(); ++i) {
    if(m_vel[i] < _cfg.m_vel[i])
      return true;
    else if(m_vel[i] > _cfg.m_vel[i])
      return false;
  }

  return false;
}

bool
Cfg::
WithinResolution(const Cfg& _cfg, const double _posRes,
                 const double _oriRes) const {
  ///@TODO comment this out if speed is important, the assembly planning stuff
  ///      needs to have composite bodies analyzed differently for now though.
  MultiBody* const mb = GetMultiBody();
  if(mb->IsComposite()) {
    for(unsigned int i = 0; i < DOF(); ++i) {
      switch(mb->GetDOFType(i)) {
        case DofType::Positional:
        case DofType::Joint:
          if(std::abs(m_dofs[i] - _cfg[i]) > _posRes)
            return false;
          break;
        case DofType::Rotational:
          if(std::abs(m_normalizer(m_dofs[i] - _cfg[i])) > _oriRes)
            return false;
          break;
      }
    }
    return true;
  }
  else {
    // Check all the DOFs and return on first failure.
    const size_t posDOF = PosDOF(), oriDOF = OriDOF(), dof = DOF();

    // Positional dofs
    for(size_t i = 0; i < posDOF; ++i)
      if(std::abs(m_dofs[i] - _cfg[i]) > _posRes)
        return false;

    // Orientation dofs
    for(size_t i = posDOF; i < posDOF + oriDOF; ++i)
      if(std::abs(m_normalizer(m_dofs[i] - _cfg[i])) > _oriRes)
        return false;

    // Joint dofs
    for(size_t i = posDOF + oriDOF; i < dof; ++i)
      if(std::abs(m_dofs[i] - _cfg[i]) > _posRes)
        return false;

    // If we're still here, the Cfgs are equal.
    return true;
  }
}

/*------------------------------- Robot Info ---------------------------------*/

Robot*
Cfg::
GetRobot() const noexcept {
  return m_robot;
}


void
Cfg::
SetRobot(Robot* const _r) noexcept {
  m_robot = _r;
}


MultiBody*
Cfg::
GetMultiBody() const noexcept {
  return m_robot->GetMultiBody();
}


size_t
Cfg::
DOF() const noexcept {
  return GetMultiBody()->DOF();
}


size_t
Cfg::
PosDOF() const noexcept {
  return GetMultiBody()->PosDOF();
}


size_t
Cfg::
OriDOF() const noexcept {
  return GetMultiBody()->OrientationDOF();
}


size_t
Cfg::
JointDOF() const noexcept {
  return GetMultiBody()->JointDOF();
}


bool
Cfg::
IsNonholonomic() const noexcept {
  return GetRobot()->IsNonholonomic();
}

/*----------------------------- DOF Accessors --------------------------------*/

double&
Cfg::
operator[](const size_t _dof) noexcept {
  m_witnessCfg.reset();
  return m_dofs[_dof];
}


double
Cfg::
operator[](const size_t _dof) const noexcept {
  return m_dofs[_dof];
}


const std::vector<double>&
Cfg::
GetData() const noexcept {
  return m_dofs;
}


double&
Cfg::
Velocity(const size_t _dof) noexcept {
  return m_vel[_dof];
}


double
Cfg::
Velocity(const size_t _dof) const noexcept {
  return m_vel[_dof];
}


const std::vector<double>&
Cfg::
GetVelocity() const noexcept {
  return m_vel;
}


void
Cfg::
SetData(const vector<double>& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != DOF())
    throw RunTimeException(WHERE) << "Tried to set data for " << _data.size()
                                  << " DOFs, but robot has " << DOF()
                                  << " DOFs!";

  m_dofs = _data;
  m_witnessCfg.reset();
}


void
Cfg::
SetData(vector<double>&& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != DOF())
    throw RunTimeException(WHERE) << "Tried to set data for " << _data.size()
                                  << " DOFs, but robot has " << DOF()
                                  << " DOFs!";

  m_dofs = std::move(_data);
  m_witnessCfg.reset();
}


void
Cfg::
SetJointData(const vector<double>& _data) {
  // Assert that we got the correct number of DOFs.
  if(_data.size() != JointDOF())
    throw RunTimeException(WHERE) << "Tried to set data for " << _data.size()
                                  << " joint DOFs, but robot has " << JointDOF()
                                  << " joint DOFs!";

  for(size_t i = 0, j = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Joint)
      m_dofs[i] = _data[j++];
  m_witnessCfg.reset();
}


Point3d
Cfg::
GetPoint() const noexcept {
  if(GetMultiBody()->GetBase()->GetMovementType() == Body::MovementType::Fixed)
    return GetMultiBody()->GetBase()->GetWorldTransformation().translation();
  else
    return Point3d(m_dofs[0], m_dofs[1], PosDOF() == 3 ? m_dofs[2] : 0);
}


vector<double>
Cfg::
GetPosition() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      ret.push_back(m_dofs[i]);
  return ret;
}


vector<double>
Cfg::
GetRotation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Rotational)
      ret.push_back(m_dofs[i]);
  return ret;
}


vector<double>
Cfg::
GetOrientation() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) != DofType::Positional)
      ret.push_back(m_dofs[i]);
  return ret;
}


vector<double>
Cfg::
GetJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i) {
    if(GetMultiBody()->GetDOFType(i) == DofType::Joint)
      ret.push_back(m_dofs[i]);
  }
  return ret;
}


vector<double>
Cfg::
GetNonJoints() const {
  vector<double> ret;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) != DofType::Joint)
      ret.push_back(m_dofs[i]);
  return ret;
}


double
Cfg::
Magnitude() const {
  double result = 0;
  for(size_t i = 0; i < DOF(); ++i)
    result += m_dofs[i] * m_dofs[i];
  return std::sqrt(result);
}


double
Cfg::
PositionMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) == DofType::Positional)
      result += m_dofs[i] * m_dofs[i];
  return std::sqrt(result);
}


double
Cfg::
OrientationMagnitude() const {
  double result = 0;
  for(size_t i = 0; i < DOF(); ++i)
    if(GetMultiBody()->GetDOFType(i) != DofType::Positional)
      result += m_dofs[i] * m_dofs[i];
  return std::sqrt(result);
}


Vector3d
Cfg::
GetLinearPosition() const {
  Vector3d out;
  for(size_t i = 0; i < PosDOF(); ++i)
    out[i] = m_dofs[i];
  return out;
}


Vector3d
Cfg::
GetAngularPosition() const {
  Vector3d vector;
  convertFromEulerAngle(vector, GetEulerAngle());
  return vector;
}


EulerAngle
Cfg::
GetEulerAngle() const {
  const size_t i = PosDOF();

  switch(OriDOF()) {
    case 1:
      return EulerAngle(m_dofs[i] * PI, 0, 0);
    case 3:
      return EulerAngle(m_dofs[i + 2] * PI,
                        m_dofs[i + 1] * PI,
                        m_dofs[i] * PI);
    default:
      return EulerAngle(0, 0, 0);
  }
}


Vector3d
Cfg::
GetLinearVelocity() const {
  if(m_vel.empty())
    return Vector3d();

  Vector3d out;
  for(size_t i = 0; i < PosDOF(); ++i)
    out[i] = m_vel[i];
  return out;
}


Vector3d
Cfg::
GetAngularVelocity() const {
  if(m_vel.empty())
    return Vector3d();

  const size_t i = PosDOF();
  switch(OriDOF()) {
    case 1:
      return Vector3d(0, 0, m_vel[i]);
    case 3:
      return Vector3d(m_vel[i], m_vel[i + 1], m_vel[i + 2]);
    default:
      return Vector3d();
  }
}


Transformation
Cfg::
GetBaseTransformation() const {
  return {GetLinearPosition(), GetEulerAngle()};
}


void
Cfg::
SetLinearPosition(const Vector3d& _v) {
  for(size_t i = 0; i < PosDOF(); ++i)
    m_dofs[i] = _v[i];
}


void
Cfg::
SetAngularPosition(const Vector3d& _v) {
  // Convert to euler angle.
  EulerAngle angle;
  convertFromEulerVector(angle, _v);
  SetEulerAngle(angle);
}


void
Cfg::
SetEulerAngle(const EulerAngle& _e) {
  const size_t i = PosDOF();
  switch(OriDOF()) {
    case 1:
      m_dofs[i] = _e.alpha() / PI;
      break;
    case 3:
      m_dofs[i + 2] = _e.alpha() / PI;
      m_dofs[i + 1] = _e.beta()  / PI;
      m_dofs[i]     = _e.gamma() / PI;
      break;
    default:;
  }
}


void
Cfg::
SetLinearVelocity(const Vector3d& _v) {
  if(m_vel.empty())
    return;

  for(size_t i = 0; i < PosDOF(); ++i)
    m_vel[i] = _v[i];
}


void
Cfg::
SetAngularVelocity(const Vector3d& _v) {
  if(m_vel.empty())
    return;

  const size_t i = PosDOF();
  switch(OriDOF()) {
    case 1:
      m_vel[i] = _v[2];
      return;
    case 3:
      m_vel[i] = _v[0];
      m_vel[i + 1] = _v[1];
      m_vel[i + 2] = _v[2];
      return;
    default:;
  }
}


void
Cfg::
SetBaseTransformation(const Transformation& _t) {
  EulerAngle e;
  convertFromMatrix(e, _t.rotation().matrix());

  SetLinearPosition(_t.translation());
  SetEulerAngle(e);
}

void
Cfg::
TransformCfg(const Transformation& _t) {
	auto current = GetBaseTransformation();
	auto newTrans = _t * current;
	SetBaseTransformation(newTrans);
}


/*------------------------- Labels and Stats ---------------------------------*/

bool
Cfg::
GetLabel(const std::string& _label) const {
  return m_labelMap.at(_label);
}


bool
Cfg::
IsLabel(const std::string& _label) const noexcept {
  return m_labelMap.count(_label);
}


void
Cfg::
SetLabel(const std::string& _label, const bool _value) noexcept {
  m_labelMap[_label] = _value;
}


double
Cfg::
GetStat(const std::string& _stat) const {
  if(!IsStat(_stat))
    return 0;
  return m_statMap.at(_stat);
}


bool
Cfg::
IsStat(const std::string& _stat) const noexcept {
  return m_statMap.count(_stat) > 0;
}


void
Cfg::
SetStat(const std::string& _stat, const double _value) noexcept {
  m_statMap[_stat] = _value;
}


void
Cfg::
IncrementStat(const std::string& _stat, const double _value) noexcept {
  m_statMap[_stat] += _value;
}

/*--------------------------- Generation Methods -----------------------------*/

void
Cfg::
Zero() noexcept {
  m_witnessCfg.reset();

  for(auto& v : m_dofs)
    v = 0;
  for(auto& v : m_vel)
    v = 0;
}


bool
Cfg::
InBounds(const Boundary* const _b) const noexcept {
  return GetRobot()->GetCSpace()->InBoundary(*this)
      and _b->InBoundary(*this);
}


bool
Cfg::
InBounds(const Environment* const _env) const noexcept {
  return InBounds(_env->GetBoundary());
}


void
Cfg::
GetRandomCfg(const Boundary* const _b) {
  // Zero the cfg and fetch both sampling spaces.
  Zero();
  auto cspace = GetRobot()->GetCSpace();
  auto vspace = GetRobot()->GetVSpace(); // Null for holonomic robots.

  const size_t dof = DOF();

  // Determine how many DOF and velocity values will be generated from _b.
  size_t numDof = std::min(_b->GetDimension(), dof),
         numVel = _b->GetDimension() > dof ? _b->GetDimension() - dof : 0;

  // If _b is a workspace boundary, use no more values than boundary size.
  if(_b->Type() == Boundary::Space::Workspace) {
    numDof = std::min(numDof, PosDOF());
    numVel = 0;
  }

  for(size_t tries = 1000; tries > 0; --tries) {
    // Generate a point in the boundary.
    auto sample = _b->GetRandomPoint();

    // Copy generated DOF values to m_dof.
    for(size_t i = 0; i < numDof; ++i)
      m_dofs[i] = sample[i];

    // For each DOF that was not generated inside the sampling boundary,
    // generate a value from the robot's c-space.
    for(size_t i = numDof; i < dof; ++i)
      m_dofs[i] = cspace->GetRange(i).Sample();

    if(vspace) {
      // Copy generated velocity values to m_vel.
      for(size_t j = 0; j < numVel; ++j)
        m_vel[j] = sample[j + numDof];

      // For each velocity that was not generated inside the sampling boundary,
      // generate a value from the robot's velocity space.
      for(size_t j = numVel; j < dof; ++j)
        m_vel[j] = vspace->GetRange(j).Sample();

      EnforceVelocityLimits();
    }

    // The result is valid if it satisfies all three boundaries.
    if(_b->InBoundary(*this)
        and cspace->InBoundary(m_dofs)
        and (!vspace or vspace->InBoundary(m_vel)))
      return;
  }

  // throw error message and some helpful statistics
  std::ostringstream oss;
  if(vspace)
    oss << *vspace;
  else
    oss << "none";
  throw RunTimeException(WHERE) << "GetRandomCfg could not generate a sample.\n"
                                << "\nRobot C-Space: " << *cspace
                                << "\nRobot V-Space: " << oss.str()
                                << "\nSampling boundary: " << *_b
                                << "\nRobot radius: "
                                << GetMultiBody()->GetBoundingSphereRadius();
}


void
Cfg::
GetRandomCfg(Environment* _env) {
  GetRandomCfg(_env->GetBoundary());
}


void
Cfg::
GetRandomVelocity() {
  // Do nothing if there is no velocity for this cfg.
  if(m_vel.empty())
    return;

  m_vel = GetRobot()->GetVSpace()->GetRandomPoint();
  EnforceVelocityLimits();
}


void
Cfg::
ConfigureRobot() const {
  GetMultiBody()->Configure(m_dofs);
}


void
Cfg::
IncrementTowardsGoal(const Cfg& _goal, const Cfg& _increment) {
  for(size_t i = 0; i < DOF(); ++i) {
    // Don't adjust DOFs that already have the exact goal value.
    if(m_dofs[i] == _goal[i])
      continue;

    // Adjust DOFs up to the goal value.
    switch(GetMultiBody()->GetDOFType(i)) {
      case DofType::Positional:
      case DofType::Joint:
        {
          const bool overshootsGoal = std::abs(_goal[i] - m_dofs[i]) <
                                      std::abs(_increment[i]);
          m_dofs[i] = overshootsGoal ? _goal[i] : m_dofs[i] += _increment[i];
        }
        break;
      case DofType::Rotational:
        {
          const double dist = m_normalizer(_goal[i] - m_dofs[i]);
          const bool overshootsGoal = std::abs(dist) < std::abs(_increment[i]);
          m_dofs[i] = overshootsGoal ? _goal[i] :
                                       m_normalizer(m_dofs[i] += _increment[i]);
        }
        break;
    }
  }

  m_witnessCfg.reset();
}


void
Cfg::
FindIncrement(const Cfg& _start, const Cfg& _goal, const int _nTicks) {
  // Need positive number of ticks.
  if(_nTicks <= 0)
    throw RunTimeException(WHERE) << "Divide by 0";

  // Compute the increment value for each DOF needed to go from _start to _goal
  // in _nTicks steps.
  for(size_t i = 0; i < DOF(); ++i) {
    switch(GetMultiBody()->GetDOFType(i)) {
      case DofType::Positional:
      case DofType::Joint:
        m_dofs[i] = (_goal[i] - _start[i]) / _nTicks;
        break;
      case DofType::Rotational:
        m_dofs[i] = m_normalizer(_goal[i] - _start[i]) / _nTicks;
        break;
    }
  }

  m_witnessCfg.reset();
}


void
Cfg::
FindIncrement(const Cfg& _start, const Cfg& _goal, int* _nTicks,
    double _positionRes, double _orientationRes) {
  const Cfg diff = _goal - _start;

  *_nTicks = std::max(1., std::ceil(std::max(
        diff.PositionMagnitude() / _positionRes,
        diff.OrientationMagnitude() / _orientationRes)));

  this->FindIncrement(_start, _goal, *_nTicks);
}


void
Cfg::
WeightedSum(const Cfg& _first, const Cfg& _second, const double _weight) {
  const double firstWeight = 1 - _weight;

  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] = _first[i] * firstWeight + _second[i] * _weight;
  for(size_t i = 0; i < m_vel.size(); ++i)
    m_dofs[i] = _first.m_vel[i] * firstWeight + _second.m_vel[i] * _weight;

  NormalizeOrientation();
  m_witnessCfg.reset();
}


void
Cfg::
GetPositionOrientationFrom2Cfg(const Cfg& _pos, const Cfg& _ori) {
  for(size_t i = 0; i < DOF(); ++i)
    m_dofs[i] = GetMultiBody()->GetDOFType(i) == DofType::Positional ? _pos[i] :
                                                                       _ori[i];

  NormalizeOrientation();
  m_witnessCfg.reset();
}

/*--------------------------- C-Space Directions -----------------------------*/

std::vector<double>
Cfg::
DirectionInLocalFrame(const Cfg& _target) const {
  // Compute direction from here to there.
  const Cfg dir = _target - *this;
  Cfg output = dir;

  // Get the world rotation of here.
  const QuaternionOrientation worldRotation(GetEulerAngle());

  // Compute the relative translation from here to there in the local frame.
  output.SetLinearPosition(-worldRotation * dir.GetLinearPosition());

  // If there are no orientation DOFs, then we are done (joint DOFs are always in
  // the local frame).
  const size_t ori = OriDOF();
  if(ori == 0)
    return output.GetData();

  // Compute the relative rotation from here to there in the local frame.
  Quaternion q;
  convertFromEulerVector(q, dir.GetAngularPosition());
  const QuaternionOrientation relativeWorldRotation(q);

  const QuaternionOrientation relativeLocalRotation = -worldRotation *
      relativeWorldRotation;

  Vector3d eulerVector;
  convertFromQuaternion(eulerVector, relativeLocalRotation.quaternion());

  //std::cout << "\nDiff EV:               " << dir.GetAngularPosition()
  //          << "\nDiff EA:               " << dir.GetEulerAngle()
  //          << "\nrelativeWorldRotation: " << relativeWorldRotation
  //          << "\nrelativeLocalRotation: " << relativeLocalRotation
  //          << "\neulerVector:           " << eulerVector
  //          << std::endl;

  const size_t pos = PosDOF();
  switch(ori)
  {
    case 1:
      output[pos] = eulerVector[2];
      break;
    case 3:
      output[pos]     = eulerVector[0];
      output[pos + 1] = eulerVector[1];
      output[pos + 2] = eulerVector[2];
      break;
    default:
      throw RunTimeException(WHERE) << "Computed relative orientation for Cfg "
                                    << "with " << ori << " orientation DOFs.";
  }

  return output.GetData();
}

/*----------------------------------- I/O ------------------------------------*/

Robot* Cfg::inputRobot = nullptr;


void
Cfg::
Read(istream& _is) {
  // Require the Cfg to already have a robot pointer for now.
  if(!m_robot)
    throw RunTimeException(WHERE) << "Cannot read in a Cfg without knowing the "
                                  << "robot. Use inputRobot member to specify "
                                  << "the default robot pointer.";

#ifdef VIZMO_MAP
  // If we are using the vizmo roadmap format, we need to read and discard the
  // robot index.
  size_t index;
  _is >> index;
#endif

  // Read one DOF first. If that fails, return and rely on checking _is.fail()
  // from the call site to determine that no more Cfg's are available.
  _is >> m_dofs[0];
  if(_is.fail())
    return;

  // Read remaining DOFs.
  for(size_t i = 1; i < DOF(); ++i) {
    _is >> m_dofs[i];
    if(_is.fail()) {
      // If we fail, print the DOFS we actually read with the error message.
      string dofs;
      for(size_t k = 0; k < i; ++k)
        dofs += to_string(m_dofs[i]) + " ";

      throw ParseException(WHERE) << "Failed reading values for all dofs. "
                                  << "Expected " << m_dofs.size()
                                  << ", but read " << i << ":\n\t" << dofs;
    }
  }

  // Read velocities, if any.
  for(size_t i = 0; i < m_vel.size(); ++i) {
    _is >> m_vel[i];
    if(_is.fail()) {
      // If we fail, print the DOFS we actually read with the error message.
      string err;
      for(size_t k = 0; k < i; ++k)
        err += to_string(m_vel[i]) + " ";

      throw ParseException(WHERE) << "Failed reading values for all velocities. "
                                  << "Expected " << m_vel.size()
                                  << ", but read " << i << ":\n\t" << err;
    }

  }

  m_witnessCfg.reset();
}


void
Cfg::
Write(ostream& _os) const {
#ifdef VIZMO_MAP
  #ifndef GROUP_MAP
    _os << "0 ";
  #endif
#endif
  // Write DOFs.
  _os << scientific << setprecision(16);
  for(auto i : m_dofs)
    _os << setw(25) << i << ' ';
#ifndef VIZMO_MAP
  for(auto i : m_vel)
    _os << setw(25) << i << ' ';
#endif

  // Unset scientific/precision options.
  _os.unsetf(ios_base::floatfield);
  if(_os.fail())
    throw RunTimeException(WHERE) << "Failed to write to file.";
}


std::string
Cfg::
PrettyPrint(const size_t _precision) const {
  std::ostringstream oss;
  oss.precision(_precision);

  if(IsNonholonomic())
    oss << "{";

  oss << "[";
  for(size_t i = 0; i < DOF() - 1; ++i)
    oss << m_dofs[i] << ", ";
  oss << m_dofs[DOF() - 1] << "]";

  if(IsNonholonomic()) {
    oss << ", <";
    for(size_t i = 0; i < DOF() - 1; ++i)
      oss << m_vel[i] << ", ";
    oss << m_vel[DOF() - 1] << ">}";
  }

  return oss.str();
}


istream&
operator>>(istream& _is, Cfg& _cfg) {
  _cfg.Read(_is);
  return _is;
}


ostream&
operator<<(ostream& _os, const Cfg& _cfg) {
  _cfg.Write(_os);
  return _os;
}

/*--------------------------------- Helpers ----------------------------------*/

void
Cfg::
EnableNormalization() const {
  m_normalizer = Normalize;
}


void
Cfg::
DisableNormalization() const {
  m_normalizer = Identity<double>;
}


void
Cfg::
NormalizeOrientation(const int _index) noexcept {
  if(_index == -1) {
    for(size_t i = 0; i < DOF(); ++i)
      if(GetMultiBody()->GetDOFType(i) == DofType::Rotational)
        m_dofs[i] = m_normalizer(m_dofs[i]);
  }
  else if(GetMultiBody()->GetDOFType(_index) == DofType::Rotational) {
    // orientation index
    m_dofs[_index] = m_normalizer(m_dofs[_index]);
  }
}


void
Cfg::
EnforceVelocityLimits() noexcept {
  if(m_vel.empty())
    return;

  // First push the velocity vector into the robot's velocity space.
  GetRobot()->GetVSpace()->PushInside(m_vel);

  // Ensure that the linear velocity is not greater than the max.
  auto linear = GetLinearVelocity();
  const double maxLinearVel = GetRobot()->GetMaxLinearVelocity();
  const double linearVel = linear.norm();

  if(linearVel > maxLinearVel)
    for(size_t i = 0; i < PosDOF(); ++i)
      m_vel[i] *= maxLinearVel / linearVel;

  // Ensure that the angular velocity is not greater than the max.
  auto angular = GetAngularVelocity();
  const double maxAngularVel = GetRobot()->GetMaxAngularVelocity();
  const double angularVel = angular.norm();

  if(angularVel > maxAngularVel)
    for(size_t i = PosDOF(); i < PosDOF() + OriDOF(); ++i)
      m_vel[i] *= maxAngularVel / angularVel;
}

/*----------------------------------------------------------------------------*/
