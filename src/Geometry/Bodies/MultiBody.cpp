#include "MultiBody.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <numeric>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Local Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/// Parse a string into a MultiBody::Type.
/// @param _tag The string to parse.
/// @param _where File location info for error reporting.
/// @return The MultiBody::Type described by _tag.
MultiBody::Type
GetMultiBodyTypeFromTag(std::string _tag, const std::string& _where) {
  // Downcase _tag to make the match case-insensitive.
  std::transform(_tag.begin(), _tag.end(), _tag.begin(), ::tolower);

  if(_tag == "passive")
    return MultiBody::Type::Passive;
  else if(_tag == "active")
    return MultiBody::Type::Active;
  else if(_tag == "internal")
    return MultiBody::Type::Internal;
  else {
    throw ParseException(_where, "Unknown MultiBody type '" + _tag + "'."
        " Options are: 'active', 'nonholonomic' 'passive', or 'internal'.");
    return MultiBody::Type::Passive;
  }
}


/// Print a MultiBody::Type to a string.
/// @param _b The type to print.
/// @return A string representation of _b.
std::string
GetTagFromMultiBodyType(const MultiBody::Type _b) {
  switch(_b) {
    case MultiBody::Type::Active:
      return "Active";
    case MultiBody::Type::Passive:
      return "Passive";
    case MultiBody::Type::Internal:
      return "Internal";
    default:
      return "Unknown Base Type";
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MultiBody ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

MultiBody::
MultiBody(const MultiBody::Type _type)
  : m_multiBodyType(_type)
{ }


MultiBody::
MultiBody(XMLNode& _node) {
  // Read the multibody type.
  const std::string type = _node.Read("type", true, "", "MultiBody type in "
      "{active, passive, internal}");
  m_multiBodyType = GetMultiBodyTypeFromTag(type, _node.Where());

  // Read the free bodies in the multibody node. Each body is either the Base
  // (just one) or a link (any number).
  for(auto& child : _node) {
    if(child.Name() == "Base") {
      // Make sure there is just one base.
      if(m_baseBody)
        throw ParseException(child.Where(), "Only one base is permitted.");

      m_baseIndex = AddBody(Body(this, child));
    }
    else if(child.Name() == "Link") {
      // A link node has body information and a single child node describing its
      // connection to its parent body.
      AddBody(Body(this, child));

      const bool oneChild = std::distance(child.begin(), child.end()) == 1;
      const bool isConnection = child.begin()->Name() == "Connection";
      if(!oneChild or !isConnection)
        throw ParseException(child.Where(), "Link nodes must have exactly one "
            "connection child node. Support for multiple connections is not "
            "yet implemented.");

      m_joints.emplace_back(new Connection(this, *child.begin()));
    }
  }
  SetBaseBody(m_baseIndex);

  for(auto& joint : m_joints)
    joint->SetBodies();

  // Make sure a base was provided.
  if(!m_baseBody)
    throw ParseException(_node.Where(), "MultiBody has no base.");

  SortJoints();
  FindMultiBodyInfo();
}


MultiBody::
MultiBody(const MultiBody& _other) {
  *this = _other;
}


MultiBody::
MultiBody(MultiBody&& _other) {
  *this = std::move(_other);
}


MultiBody::
~MultiBody() = default;


void
MultiBody::
InitializeDOFs(const Boundary* const _b) {
  m_dofInfo.clear();

  const Range<double> full(-1, 1);
  const DofType position = DofType::Positional,
                rotation = DofType::Rotational;

  // Set the base DOFs. Limit positional values to the boundary's extremes.
  if(m_baseType == Body::Type::Planar) {
    m_dofInfo.emplace_back("Base X Translation ", position, _b->GetRange(0));
    m_dofInfo.emplace_back("Base Y Translation ", position, _b->GetRange(1));

    if(m_baseMovement == Body::MovementType::Rotational)
      m_dofInfo.emplace_back("Base Rotation ", rotation, full);
  }
  else if(m_baseType == Body::Type::Volumetric) {
    m_dofInfo.emplace_back("Base X Translation ", position, _b->GetRange(0));
    m_dofInfo.emplace_back("Base Y Translation ", position, _b->GetRange(1));
    m_dofInfo.emplace_back("Base Z Translation ", position, _b->GetRange(2));

    if(m_baseMovement == Body::MovementType::Rotational) {
      m_dofInfo.emplace_back("Base X Rotation ", rotation, full);
      m_dofInfo.emplace_back("Base Y Rotation ", rotation, full);
      m_dofInfo.emplace_back("Base Z Rotation ", rotation, full);
    }
  }

  for(auto& joint : m_joints) {
    // Make a partial label for this joint out of the body indices.
    const std::string label(std::to_string(joint->GetPreviousBodyIndex()) + "-" +
                            std::to_string(joint->GetNextBodyIndex()));

    switch(joint->GetConnectionType()) {
      case Connection::JointType::Revolute:
        m_dofInfo.emplace_back("Revolute Joint " + label + " Angle",
                               DofType::Joint, joint->GetJointRange(0));
        break;
      case Connection::JointType::Spherical:
        m_dofInfo.emplace_back("Spherical Joint " + label + " Angle 0",
                               DofType::Joint, joint->GetJointRange(0));
        m_dofInfo.emplace_back("Spherical Joint " + label + " Angle 1",
                               DofType::Joint, joint->GetJointRange(1));
        break;
      case Connection::JointType::NonActuated:
        break;
    }
  }

  m_currentDofs.resize(DOF(), 0);
  Configure(m_currentDofs);
  FindMultiBodyInfo();
}

/*-------------------------------- Assignment --------------------------------*/

MultiBody&
MultiBody::
operator=(const MultiBody& _other) {
  if(this == &_other)
    return *this;

  m_multiBodyType = _other.m_multiBodyType;
  m_radius        = _other.m_radius;
  m_baseIndex     = _other.m_baseIndex;
  m_dofInfo       = _other.m_dofInfo;
  m_currentDofs   = _other.m_currentDofs;

  // Copy the bodies.
  for(const auto& body : _other.m_bodies)
    AddBody(Body(body));

  // Set the base body.
  SetBaseBody(m_baseIndex);

  // Copy the joints.
  for(const auto& joint : _other.m_joints) {
    m_joints.emplace_back(new Connection(*joint));
    m_joints.back()->SetBodies(this);
  }

  return *this;
}


MultiBody&
MultiBody::
operator=(MultiBody&& _other) {
  if(this == &_other)
    return *this;

  m_multiBodyType = std::move(_other.m_multiBodyType);
  m_radius        = std::move(_other.m_radius);
  m_baseIndex     = std::move(_other.m_baseIndex);
  m_dofInfo       = std::move(_other.m_dofInfo);
  m_currentDofs   = std::move(_other.m_currentDofs);

  // Move the bodies and set their multibody pointers.
  m_bodies = std::move(_other.m_bodies);
  _other.m_bodies.clear();
  for(auto& body : m_bodies)
    body.SetMultiBody(this);

  // Set the base body.
  SetBaseBody(m_baseIndex);

  // Move the joints and update their multibody pointers.
  m_joints = std::move(_other.m_joints);
  _other.m_joints.clear();
  for(auto& joint : m_joints)
    joint->SetBodies(this);

  return *this;
}

/*--------------------------- MultiBody Properties ---------------------------*/

MultiBody::Type
MultiBody::
GetType() const noexcept {
  return m_multiBodyType;
}


bool
MultiBody::
IsActive() const noexcept {
  return m_multiBodyType == Type::Active;
}


bool
MultiBody::
IsPassive() const noexcept {
  return m_multiBodyType != Type::Active;
}


bool
MultiBody::
IsInternal() const noexcept {
  return m_multiBodyType == Type::Internal;
}


bool
MultiBody::
IsComposite() const noexcept {
  return m_joints.empty() and m_bodies.size() > 1;
}


size_t
MultiBody::
DOF() const noexcept {
  return m_dofInfo.size();
}


size_t
MultiBody::
PosDOF() const noexcept {
  switch(m_baseType) {
    case Body::Type::Planar:
      return 2;
    case Body::Type::Volumetric:
      return 3;
    default:
      return 0;
  }
}


size_t
MultiBody::
OrientationDOF() const noexcept {
  if(m_baseMovement != Body::MovementType::Rotational)
    return 0;
  else
    return m_baseType == Body::Type::Volumetric ? 3 : 1;
}


size_t
MultiBody::
JointDOF() const noexcept {
  return DOF() - PosDOF() - OrientationDOF();
}


const std::vector<double>&
MultiBody::
GetCurrentDOFs() const noexcept {
  return m_currentDofs;
}


std::vector<double>
MultiBody::
GetCurrentCfg() noexcept {
  std::vector<double> output(DOF(), 0);

  // First get the DOFs for the base.
  const std::vector<double> base = GetBase()->GetWorldTransformation().GetCfg();

  // Define the boundaries of each DOF segment.
  auto pos = output.begin(),
       ori = output.begin() + PosDOF(),
       jnt = output.begin() + PosDOF() + OrientationDOF(),
       end = output.end();

  // Determine how many orientation values from 'base' will be unused.
  const size_t ignore = 3 - OrientationDOF();

  // Copy the required values into the output vector, depending on base movement
  // type. For the orientation, we will skip the unused values.
  std::copy(base.begin(), base.begin() + PosDOF(), pos);
  std::copy(base.begin() + 3 + ignore, base.end(), ori);

  // Make sure the orientation values are normalized.
  for(auto iter = ori; iter < jnt; ++iter)
    *iter = Normalize(*iter);

  // Make sure the joint transforms are current.
  UpdateLinks();

  // For each joint, copy its values.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get the connection object's DHParameters.
    const DHParameters& dh = joint->GetDHParameters();

    // Set the joint DOF values from the DH params.
    *jnt++ = dh.m_theta / PI;
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      *jnt++ = dh.m_alpha / PI;
  }

  // If jnt is not at the end now, we did something wrong.
  if(jnt != end)
    throw RunTimeException(WHERE) << "Computation error:"
                                  << "\n\tDOF: " << DOF()
                                  << "\n\tEnd distance: " << std::distance(pos, end)
                                  << "\n\tJnt distance: " << std::distance(pos, jnt)
                                  << std::endl;

  m_currentDofs = output;
  return output;
}

/*------------------------------ Body Accessors ------------------------------*/

size_t
MultiBody::
GetNumBodies() const noexcept {
  return m_bodies.size();
}


const std::vector<Body>&
MultiBody::
GetBodies() const noexcept {
  return m_bodies;
}


std::vector<const Body*>
MultiBody::
GetEndEffectors() const noexcept {
  std::vector<const Body*> endEffectors;

  for(const auto& body : m_bodies)
    if(body.ForwardConnectionCount() == 0)
      endEffectors.push_back(&body);

  return endEffectors;
}


Body*
MultiBody::
GetBody(const size_t _i) noexcept {
  return &m_bodies[_i];
}


const Body*
MultiBody::
GetBody(const size_t _i) const noexcept {
  return &m_bodies[_i];
}


size_t
MultiBody::
AddBody(Body&& _body) {
  /// @TODO Adjust our body tracking so that we don't have to add bodies in
  ///       index order.
  m_bodies.push_back(std::move(_body));
  auto& body = m_bodies.back();
  if(body.GetIndex() != m_bodies.size() - 1)
    throw ParseException(WHERE) << "Added body with index "
                                << body.GetIndex() << ", but it landed in slot "
                                << m_bodies.size() << ".";
  body.SetMultiBody(this);

  return body.GetIndex();
}


Body*
MultiBody::
GetBase() noexcept {
  return m_baseBody;
}


const Body*
MultiBody::
GetBase() const noexcept {
  return m_baseBody;
}


void
MultiBody::
SetBaseBody(const size_t _index) {
  if(_index >= m_bodies.size())
    throw RunTimeException(WHERE, "Cannot use index " + std::to_string(_index) +
        " for base body because there are only " +
        std::to_string(m_bodies.size()) + " parts in this multibody.");

  m_baseIndex    = _index;
  m_baseBody     = GetBody(_index);
  m_baseType     = m_baseBody->GetBodyType();
  m_baseMovement = m_baseBody->GetMovementType();
}


Body::Type
MultiBody::
GetBaseType() const noexcept {
  return m_baseType;
}


Body::MovementType
MultiBody::
GetBaseMovementType() const noexcept {
  return m_baseMovement;
}

/*--------------------------- Geometric Properties ---------------------------*/

const Vector3d&
MultiBody::
GetCenterOfMass() const noexcept {
  throw RunTimeException(WHERE) << "We have no correct implementation for the "
      "center of mass, and without a moment of inertia it isn't a well-formed "
      "concept. This cannot be computed once - it must depend on the robot's "
      "present configuration as well as a mass distribution across each body. If "
      "you think you want this, you are probably looking for the centroid or "
      "bounding box center instead (unless you're doing something "
      "physics-based).";
}


double
MultiBody::
GetBoundingSphereRadius() const noexcept {
  return m_radius;
}

/*------------------------------- Connections --------------------------------*/

const std::vector<std::unique_ptr<Connection>>&
MultiBody::
GetJoints() const noexcept {
  return m_joints;
}


Connection*
MultiBody::
GetJoint(const size_t _i) noexcept {
  return m_joints[_i].get();
}


const DofType&
MultiBody::
GetDOFType(const size_t _i) const noexcept {
  return m_dofInfo[_i].type;
}


const vector<DofInfo>&
MultiBody::
GetDofInfo() const noexcept {
  return m_dofInfo;
}


void
MultiBody::
UpdateJointLimits() noexcept {
  const size_t firstJointIndex = PosDOF() + OrientationDOF();

  auto joint = m_joints.begin(); // iter to unique_ptr
  for(size_t i = firstJointIndex; i < DOF(); ++i) {
    switch(joint->get()->GetConnectionType())
    {
      case Connection::JointType::Revolute:
        m_dofInfo[i].range = joint->get()->GetJointRange(0);
        break;
      case Connection::JointType::Spherical:
        m_dofInfo[i].range   = joint->get()->GetJointRange(0);
        m_dofInfo[++i].range = (++joint)->get()->GetJointRange(1);
        break;
      case Connection::JointType::NonActuated:
        break;
    }
    ++joint;
  }
}

/*------------------------- Configuration Methods ----------------------------*/

void
MultiBody::
Configure(const Cfg& _c) {
  Configure(_c.GetData());
}


void
MultiBody::
Configure(const vector<double>& _v) {
  // Do not reconfigure if we are already here.
  if(_v == m_currentDofs)
    return;

  size_t index = 0;

  std::copy(_v.begin(), _v.begin() + std::min(_v.size(), DOF()),
      m_currentDofs.begin());

  // Configure the base.
  if(m_baseType != Body::Type::Fixed) {
    m_baseBody->Configure(GenerateBaseTransformation(_v));
    index = PosDOF() + OrientationDOF();
  }

  // Configure the links.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Adjust the joint to reflect new configuration.
    auto& dh = joint->GetDHParameters();
    dh.m_theta = _v[index++] * PI;
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      dh.m_alpha = _v[index++] * PI;
  }

  // The base transform has been updated, now update the links.
  UpdateLinks();
}


void
MultiBody::
PushToNearestValidConfiguration() {
  // First get the actual configured Cfg.
  std::vector<double> cfg = GetCurrentCfg();

  // Check each value against the joint limits.
  for(size_t i = 0; i < DOF(); ++i) {
    const Range<double>& limit = m_dofInfo[i].range;
    // If the current cfg lies outside the limits, push it inside.
    if(!limit.Contains(cfg[i]))
      cfg[i] = limit.ClearancePoint(cfg[i]);
  }

  // Configure on the valid Cfg.
  Configure(cfg);
}

/*----------------------------------- I/O ------------------------------------*/

void
MultiBody::
Read(std::istream& _is, CountingStreamBuffer& _cbs) {
  // If not already marked active (as when we are parsing a robot), read the
  // type first.
  if(!IsActive()) {
    const std::string multibodyType = ReadFieldString(_is, _cbs,
        "Failed reading multibody type."
        " Options are: active, passive, internal, or surface.");
    m_multiBodyType = GetMultiBodyTypeFromTag(multibodyType, _cbs.Where());
  }

  // Read passive bodies differently as per the old file format.
  if(IsPassive()) {
    const size_t index = AddBody(Body(this));
    GetBody(index)->Read(_is, _cbs);
    SetBaseBody(index);

    FindMultiBodyInfo();
    return;
  }

  size_t bodyCount = ReadField<size_t>(_is, _cbs, "Failed reading body count.");

  m_baseIndex = -1;
  for(size_t i = 0; i < bodyCount && _is; ++i) {
    const size_t index = AddBody(Body(this, i));
    auto free = GetBody(index);
    free->Read(_is, _cbs);

    if(free->IsBase() && m_baseIndex == size_t(-1))
      m_baseIndex = index;
  }
  SetBaseBody(m_baseIndex);

  if(m_baseIndex == size_t(-1))
    throw ParseException(_cbs.Where(), "Active body has no base.");

  //get connection info
  std::string connectionTag = ReadFieldString(_is, _cbs,
      "Failed reading connections tag.");

  //check that there is any additional adjacency connections
    if(connectionTag == "ADJACENCIES") {
      size_t adjacencyCount = ReadField<size_t>(_is, _cbs,
          "Failed reading number of closing loop adjacency connections.");
      for(size_t m = 0; m < adjacencyCount && _is; ++m) {
        //read body indices
        int firstI = ReadField<int>(_is, _cbs, "Failed reading first closing loop index");
        int secondI = ReadField<int>(_is, _cbs, "Failed reading second closing loop index");
        m_joints.emplace_back(new Connection(this));
        m_joints.back()->SetAdjacentBodies(this, firstI, secondI);
      }
      connectionTag = ReadFieldString(_is, _cbs, "Failed reading connections tag.");
    }

  if(connectionTag != "CONNECTIONS")
    throw ParseException(_cbs.Where(),
        "Unknwon connections tag '" + connectionTag + "'."
        " Should read 'Connections'.");
  size_t connectionCount = ReadField<size_t>(_is, _cbs,
      "Failed reading number of connections.");

  for(size_t i = 0; i < connectionCount; ++i) {
    // add connection info to multibody connection map
    m_joints.emplace_back(new Connection(this));
    m_joints.back()->Read(_is, _cbs);
    m_joints.back()->SetBodies();
  }

  SortJoints();
  FindMultiBodyInfo();
}


void
MultiBody::
Write(std::ostream& _os) const {
  // Write type.
  _os << GetTagFromMultiBodyType(m_multiBodyType) << std::endl;

  // If this is a passive body, we write it differently to remain compatible
  // with the old file format.
  if(IsPassive()) {
    for(const auto& body : m_bodies)
      _os << body << std::endl;
    return;
  }

  // Write free body count and free bodies.
  _os << m_bodies.size() << std::endl;
  for(const auto& body : m_bodies)
    _os << body << std::endl;

  // Write connections.
  size_t numConnection = 0;
  for(const auto& body : m_bodies)
    numConnection += body.ForwardConnectionCount();

  _os << "Connections\n"
      << numConnection
      << std::endl;

  for(const auto& body : m_bodies)
    for(size_t j = 0; j < body.ForwardConnectionCount(); ++j)
      _os << body.GetForwardConnection(j);
}

/*---------------------------------- Helpers ---------------------------------*/

void
MultiBody::
SortJoints() {
  const static auto sorter =
      [](const std::unique_ptr<Connection>& _a,
         const std::unique_ptr<Connection>& _b)
      {
        if(_a->GetPreviousBodyIndex() != _b->GetPreviousBodyIndex())
          return _a->GetPreviousBodyIndex() < _b->GetPreviousBodyIndex();
        return _a->GetNextBodyIndex() < _b->GetNextBodyIndex();
      };
  std::sort(m_joints.begin(), m_joints.end(), sorter);
}


void
MultiBody::
UpdateLinks() {
  for(auto& body : m_bodies)
    if(!body.IsBase())
      body.MarkDirty();

  /// @TODO I think we should be able to remove this forced recomputation and
  ///       let it resolve lazily, need to test though.
  for(auto& body : m_bodies)
    if(body.ForwardConnectionCount() == 0)  // tree tips: leaves.
      body.GetWorldTransformation();
}


void
MultiBody::
FindMultiBodyInfo() {
  if(IsComposite())
    throw RunTimeException(WHERE) << "Composite bodies should be handled "
                                  << "through group cfgs.";

  // Roughly approximate the maximum bounding radius by assuming that all links
  // are chained sequentially end-to-end away from the base. This is a very
  // loose bound that we choose for its simplicity.
  m_radius = m_bodies[0].GetPolyhedron().GetMaxRadius();
  for(size_t i = 1; i < m_bodies.size(); ++i)
    m_radius += m_bodies[i].GetPolyhedron().GetMaxRadius() * 2.0;
}


Transformation
MultiBody::
GenerateBaseTransformation(const std::vector<double>& _v) const {
  const size_t pos = PosDOF(),
               ori = OrientationDOF();

  const Vector3d translation(_v[0], _v[1], pos == 3 ? _v[2] : 0.);

  EulerAngle rotation(0, 0, 0);
  if(ori == 1) {
    rotation.alpha() = _v[pos] * PI;     // about Z
  }
  else if(ori == 3) {
    rotation.gamma() = _v[pos]     * PI; // about X is the third Euler angle
    rotation.beta()  = _v[pos + 1] * PI; // about Y is the second Euler angle
    rotation.alpha() = _v[pos + 2] * PI; // about Z is the first Euler angle
  }

  return Transformation(std::move(translation), std::move(rotation));
}

/*----------------------------------------------------------------------------*/
