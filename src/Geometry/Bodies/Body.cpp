#include "Body.h"

#include "Connection.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "Utilities/Color.h"
#include "Utilities/XMLNode.h"

#include <CGAL/Quotient.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>

#include <algorithm>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Local Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/// Parse a body type from a textual label.
/// @param _tag The text label describing the body type.
/// @param _where Error information for undetected types.
/// @return The body type parsed from _tag.
Body::Type
GetBodyTypeFromTag(std::string _tag, const std::string& _where) {
  // Downcase the tag.
  std::transform(_tag.begin(), _tag.end(), _tag.begin(), ::tolower);

  if(_tag == "planar")
    return Body::Type::Planar;
  else if(_tag == "volumetric")
    return Body::Type::Volumetric;
  else if(_tag == "fixed")
    return Body::Type::Fixed;
  else if(_tag == "joint")
    return Body::Type::Joint;
  else
    throw ParseException(_where) << "Unknown body type '" << _tag << "'. "
                                 << "Options are: 'planar', 'volumetric', "
                                 << "'fixed', or 'joint'.";
}


/// Parse a movement type from a textual label.
/// @param _tag The text label describing the movement type.
/// @param _where Error information for undetected types.
/// @return The movement type parsed from _tag.
Body::MovementType
GetMovementTypeFromTag(std::string _tag, const std::string& _where) {
  // Downcase the tag.
  std::transform(_tag.begin(), _tag.end(), _tag.begin(), ::tolower);

  if(_tag == "rotational")
    return Body::MovementType::Rotational;
  else if(_tag == "translational")
    return Body::MovementType::Translational;
  else if(_tag == "fixed")
    return Body::MovementType::Fixed;
  else if(_tag == "joint")
    return Body::MovementType::Joint;
  else
    throw ParseException(_where) << "Unknown movement type '" << _tag << "'."
                                 << " Options are: 'rotational', "
                                 << "'translational', 'fixed'.";
}


std::string
GetTagFromBodyType(const Body::Type _b) {
  switch(_b) {
    case Body::Type::Planar:
      return "Planar";
    case Body::Type::Volumetric:
      return "Volumetric";
    case Body::Type::Fixed:
      return "Fixed";
    case Body::Type::Joint:
      return "Joint";
    default:
      throw ParseException(WHERE, "Unknown body type.");
  }
}


std::string
GetTagFromMovementType(const Body::MovementType _bm) {
  switch(_bm){
    case Body::MovementType::Rotational:
      return "Rotational";
    case Body::MovementType::Translational:
      return "Translational";
    case Body::MovementType::Fixed:
      return "Fixed";
    case Body::MovementType::Joint:
      return "Joint";
    default:
      throw ParseException(WHERE) << "Unknown movement type.";
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Body ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*---------------------------- Static Initializers ---------------------------*/

std::string Body::m_modelDataDir;

/*------------------------------ Construction --------------------------------*/

Body::
Body(MultiBody* const _owner, const size_t _index)
  : m_multibody(_owner),
    m_index(_index),
    m_label(std::to_string(m_index))
{ }


Body::
Body(MultiBody* const _owner, XMLNode& _node)
  : m_multibody(_owner) {

  // Read the index, defaulting to 0.
  m_index = _node.Read("index", false, size_t(0), size_t(0),
      numeric_limits<size_t>::max(), "Index of the body in the multibody.");

  // Read the label if one exists.
  m_label = _node.Read("label", false, std::to_string(m_index),
      "A semantic label for this part.");

  // Read the COM adjustment.
  const std::string adjust = _node.Read("comAdjustment", false, "none",
      "Specification of com adjustment");

  if(adjust == "com")
    m_comAdjust = GMSPolyhedron::COMAdjust::COM;
  else if(adjust == "surface")
    m_comAdjust = GMSPolyhedron::COMAdjust::Surface;
  else if(adjust == "none")
    m_comAdjust = GMSPolyhedron::COMAdjust::None;
  else
    throw ParseException(_node.Where(),
        "Invalid specification of com adjustment: '" + adjust +
        "'. Options are 'com', 'surface', or 'none'");

  // Read the color.
  const std::string color = _node.Read("color", false, "", "Color of the body.");
  if(!color.empty()) {
    std::istringstream buffer(color);
    buffer >> m_color;
  }

  // Parse optional texture file.
  m_textureFile = _node.Read("textureFile", false, "", "Name of the texture "
      "file.");

  // Read mass.
  m_mass = _node.Read("mass", false, 1.,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "Mass of the body.");

  // Read body type.
  std::string type = _node.Read("type", true, "",
      "Type of the body (volumetric, planar, fixed, or joint.");
  SetBodyType(GetBodyTypeFromTag(type, _node.Where()));
  const bool isFixed = GetBodyType() == Body::Type::Fixed,
             isJoint = GetBodyType() == Body::Type::Joint;

  // Read movement type.
  std::string movement = _node.Read("movement", !isFixed and !isJoint, "",
      "Type of the movement (rotational, or translational).");
  // If the body is fixed, we shouldn't receive a movement type.
  if((isFixed or isJoint) and !movement.empty()) {
    throw ParseException(_node.Where()) << "Fixed and joint bodies may not "
                                        << "specify a movement type ('"
                                        << movement << "' provided).";
  }
  if(isFixed)
    movement = "fixed";
  else if(isJoint)
    movement = "joint";
  SetMovementType(GetMovementTypeFromTag(movement, _node.Where()));

  // Read the transform for fixed bodies.
  if(isFixed) {
    const std::string transform = _node.Read("transform", true, "",
        "The transform for this body.");

    std::istringstream buffer(transform);
    buffer >> m_transform;
  }

  // Read geometry file.
  m_filename = _node.Read("filename", true, "", "File containing the geometry"
      " information for this body.");
  ReadGeometryFile(m_comAdjust);
}


Body::
Body(const Body& _other) {
  *this = _other;
}


Body::
Body(Body&& _other) = default;


Body::
~Body() = default;

/*-------------------------------- Assignment --------------------------------*/

Body&
Body::
operator=(const Body& _other) {
  if(this == &_other)
    return *this;

  m_multibody             = nullptr;
  m_index                 = _other.m_index;
  m_label                 = _other.m_label;
  m_filename              = _other.m_filename;
  m_bodyType              = _other.m_bodyType;
  m_movementType          = _other.m_movementType;
  m_polyhedron            = _other.m_polyhedron;
  m_boundingBox           = _other.m_boundingBox;
  m_transform             = _other.m_transform;
  m_transformCached       = _other.m_transformCached;
  m_worldPolyhedron       = _other.m_worldPolyhedron;
  m_worldPolyhedronCached = _other.m_worldPolyhedronCached;
  m_mass                  = _other.m_mass;
  m_moment                = _other.m_moment;
  m_comAdjust             = _other.m_comAdjust;

  m_forwardConnections.clear();
  m_backwardConnections.clear();
  m_adjacencyConnections.clear();
  m_transformFetcher = _other.m_transformFetcher;

  m_color         = _other.m_color;
  m_textureFile   = _other.m_textureFile;

  return *this;
}


Body&
Body::
operator=(Body&& _other) = default;

/*------------------------------- Validation ---------------------------------*/

void
Body::
Validate() const {
  using CGALPolyhedron = GMSPolyhedron::CGALPolyhedron;

  // First use CGAL to check valid, triangular, and closed.
  CGALPolyhedron mesh;
  try {
    mesh = m_polyhedron.CGAL();
  }
  catch(std::exception& _e) {
    throw ParseException(WHERE) << "Invalid polyhedron detected from "
                                << "file '" << m_filename << "'."
                                << "\n\tCould not build a CGAL model of this, "
                                << "which usually means that the normals are "
                                << "inconsistent or the file is corrupted."
                                << "\nThis is not ignorable! The CD algorithms "
                                << "yield undefined behavior on ill-formed "
                                << "polyhedrons."
                                << std::endl;
  }

  const bool valid      = mesh.is_valid(),
             triangular = mesh.is_pure_triangle(),
             closed     = mesh.is_closed();

  // Now use PQP to determine that the polygon is outward-facing.
  bool outward = true;
  {
    PQPSolid pqp;

    mathtool::Transformation t;

    // For each facet, make sure that the point just behind the center is inside.
    for(const auto& poly : m_polyhedron.GetPolygonList())
    {
      const Vector3d point = poly.FindCenter() - (1e-6 * poly.GetNormal());

      outward &= pqp.IsInsideObstacle(point, m_polyhedron, t);
      if(!outward)
        break;
    }
  }

  // The polyhedron is good if it is valid, triangular, closed, and
  // outward-facing.
  if(valid and triangular and closed) // and outward)
    return;

  // Something isn't good - report errors if requested.
  throw ParseException(WHERE) << "Invalid polyhedron detected from "
                              << "file '" << m_filename << "'."
                              << "\n\tnum vertices: " << mesh.size_of_vertices()
                              << "\n\tnum facets: " << mesh.size_of_facets()
                              << "\n\tvalid: " << valid
                              << "\n\ttriangular: " << triangular
                              << "\n\tclosed: " << closed
                              << "\n\toutward: " << outward
                              << "\nThis is not ignorable! The CD algorithms "
                              << "yield undefined behavior on ill-formed "
                              << "polyhedrons."
                              << std::endl;
}

/*---------------------------- MultiBody Accessors ---------------------------*/

MultiBody*
Body::
GetMultiBody() const noexcept {
  return m_multibody;
}


void
Body::
SetMultiBody(MultiBody* const _owner) noexcept {
  m_multibody = _owner;
}


size_t
Body::
GetIndex() const noexcept {
  return m_index;
}

/*----------------------------- Body Properties ------------------------------*/

const std::string&
Body::
Label() const noexcept {
  return m_label;
}


bool
Body::
IsBase() const noexcept {
  return m_bodyType != Body::Type::Joint;
}


void
Body::
SetBodyType(const Body::Type _type) noexcept {
  m_bodyType = _type;

  // Setup an expedited transform getter for fixed bodies to eliminate possible
  // branchings on the cached value. This prevents branch misses for all base
  // bodies.
  if(IsBase())
    m_transformFetcher = &Body::FetchBaseTransform;
  else
    m_transformFetcher = &Body::FetchLinkTransform;
}


Body::Type
Body::
GetBodyType() const noexcept {
  return m_bodyType;
}


void
Body::
SetMovementType(const MovementType _type) noexcept {
  m_movementType = _type;
}


Body::MovementType
Body::
GetMovementType() const noexcept {
  return m_movementType;
}

/*---------------------------- Physical Properties ---------------------------*/

double
Body::
GetMass() const {
  return m_mass;
}


const Matrix3x3&
Body::
GetMoment() const {
  return m_moment;
}

/*--------------------------- Geometric Properties ---------------------------*/

void
Body::
SetPolyhedron(GMSPolyhedron&& _poly) {
  _poly.UpdateCGALPoints();
  m_polyhedron = std::move(_poly);

  ComputeMomentOfInertia();
  ComputeBoundingBox();
  MarkDirty();
}


const GMSPolyhedron&
Body::
GetPolyhedron() const {
  return m_polyhedron;
}


const GMSPolyhedron&
Body::
GetWorldPolyhedron() const {
  if(!m_worldPolyhedronCached) {
    m_worldPolyhedron = GetWorldTransformation() * m_polyhedron;
    m_worldPolyhedronCached = true;
  }
  return m_worldPolyhedron;
}


const GMSPolyhedron&
Body::
GetBoundingBox() const {
  return m_boundingBox;
}


GMSPolyhedron
Body::
GetWorldBoundingBox() const {
  return GetWorldTransformation() * m_boundingBox;
}

/*---------------------------- Transform Functions ---------------------------*/

void
Body::
MarkDirty() {
  m_transformCached = false;
  m_worldPolyhedronCached = false;
}


void
Body::
Configure(const Transformation& _transformation) {
  m_transform = _transformation;
  MarkDirty();
}


const Transformation&
Body::
GetWorldTransformation() const {
  return (this->*m_transformFetcher)();
}

/*--------------------------- Connection Properties --------------------------*/

size_t
Body::
ForwardConnectionCount() const noexcept {
  return m_forwardConnections.size();
}


size_t
Body::
BackwardConnectionCount() const noexcept {
  return m_backwardConnections.size();
}


size_t
Body::
AdjacencyConnectionCount() const noexcept {
  return m_adjacencyConnections.size();
}


Connection&
Body::
GetForwardConnection(const size_t _index) const noexcept {
  try {
    return *m_forwardConnections.at(_index);
  }
  catch(std::exception&) {
    // Re-propgate out-of-range exception with better error info.
    throw RunTimeException(WHERE, "Cannot access forward connection " +
        ::to_string(_index) + ", number of forward connections = " +
        ::to_string(ForwardConnectionCount()) + ".");
  }
}


Connection&
Body::
GetBackwardConnection(const size_t _index) const noexcept {
  try {
    return *m_backwardConnections.at(_index);
  }
  catch(std::exception&) {
    // Re-propgate out-of-range exception with better error info.
    throw RunTimeException(WHERE, "Cannot access backward connection " +
        ::to_string(_index) + ", number of backward connections = " +
        ::to_string(BackwardConnectionCount()) + ".");
  }
}


Connection&
Body::
GetAdjacencyConnection(const size_t _index) const noexcept {
  try {
    return *m_adjacencyConnections.at(_index);
  }
  catch(std::exception&) {
    // Re-propgate out-of-range exception with better error info.
    throw RunTimeException(WHERE, "Cannot access adjacency connection " +
        ::to_string(_index) + ", number of adjacency connections = " +
        ::to_string(AdjacencyConnectionCount()) + ".");
  }
}


Connection*
Body::
GetConnectionTo(const Body* const _other) const noexcept {
  for(const auto c : m_forwardConnections)
    if(c->GetNextBody() == _other)
      return c;
  for(const auto c : m_backwardConnections)
    if(c->GetPreviousBody() == _other)
      return c;
  return nullptr;
}


bool
Body::
IsAdjacent(const Body* const _other) const {
  if(this == _other)
    return true;

  for(const auto& c : m_forwardConnections)
    if(c->GetNextBody() == _other)
      return true;
  for(const auto& c : m_backwardConnections)
    if(c->GetPreviousBody() == _other)
      return true;
  for(const auto& c : m_adjacencyConnections)
    if(c->GetNextBody() == _other || c->GetPreviousBody() == _other)
      return true;

  return false;
}


bool
Body::
SameParent(const Body* const _other) const {
  //1: for branched structures: check if there is a shared parent
  for(const auto& c1: m_backwardConnections) {
    for(const auto& c2: _other->m_backwardConnections) {
      if(c1->GetPreviousBodyIndex() == c2->GetPreviousBodyIndex())
        return true;
    }
  }
  return false;
}


void
Body::
LinkForward(Connection* const _c) {
  m_forwardConnections.push_back(_c);
  MarkDirty();
}


void
Body::
LinkBackward(Connection* const _c) {
  m_backwardConnections.push_back(_c);
  MarkDirty();
}


void
Body::
LinkAdjacency(Connection* const _c) {
  m_adjacencyConnections.push_back(_c);
  MarkDirty();
}


void
Body::
Unlink(Connection* const _c) {
  // Search for _c in the forward connections.
  auto forwardIter = std::find(m_forwardConnections.begin(),
                               m_forwardConnections.end(),
                               _c);

  // If we found it, remove the link and return.
  if(forwardIter != m_forwardConnections.end()) {
    m_forwardConnections.erase(forwardIter);
    return;
  }

  // Search for _c in the backward connections.
  auto backwardIter = std::find(m_backwardConnections.begin(),
                                m_backwardConnections.end(),
                                _c);

  // If we found it, remove the link and return.
  if(backwardIter != m_backwardConnections.end()) {
    m_backwardConnections.erase(backwardIter);
    return;
  }

  // Search for _c in the adjacency connections.
  auto adjacencyIter = std::find(m_adjacencyConnections.begin(),
                                m_adjacencyConnections.end(),
                                _c);

  // If we found it, remove the link and return.
  if(adjacencyIter != m_adjacencyConnections.end()) {
    m_adjacencyConnections.erase(adjacencyIter);
    return;
  }

  throw RunTimeException(WHERE, "Request to unlink a connection which was "
      "not previously linked.");
}

/*------------------------------------ I/O -----------------------------------*/

const std::string&
Body::
GetFileName() const {
  return m_filename;
}


std::string
Body::
GetFilePath() const {
  return m_modelDataDir == "/" || m_filename[0] == '/'
         ? m_filename
         : m_modelDataDir + m_filename;
}


void
Body::
ReadGeometryFile(const std::string& _filename) {
  m_filename = _filename;
  ReadGeometryFile();
}


void
Body::
ReadGeometryFile(GMSPolyhedron::COMAdjust _comAdjust) {
  std::string filename = GetFilePath();

  if(!FileExists(filename))
    throw ParseException(WHERE) << "File \'" << filename << "\' not found.";

  m_polyhedron.Read(filename, _comAdjust);

  ComputeMomentOfInertia();
  ComputeBoundingBox();
  MarkDirty();
  Validate();
}


void
Body::
Read(std::istream& _is, CountingStreamBuffer& _cbs) {
  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);

  // Read optional input between file name and joint/transform specification.

  // Read white space.
  char c;
  while(isspace(_is.peek()))
    _is.get(c);

  while(_is.peek() == '-') {
    // Read '-'.
    _is.get(c);

    // Read next option.
    _is >> c;

    // Parse optional com adjustment.
    if(c == 'a') {
      _is >> c; //read a(
      if(c != '(')
        throw ParseException(_cbs.Where(), "Invalid specification of com "
            "adjustment.");
      std::string adjust = ReadFieldString(_is, _cbs, "Invalid specification "
          "of com adjustment.");
      c = adjust.back();
      //read )
      if(c != ')')
        throw ParseException(_cbs.Where(), "Invalid specification of com "
            "adjustment.");
      adjust = adjust.substr(0, adjust.size() - 1);
      if(adjust == "COM")
        m_comAdjust = GMSPolyhedron::COMAdjust::COM;
      else if(adjust == "SURFACE")
        m_comAdjust = GMSPolyhedron::COMAdjust::Surface;
      else if (adjust == "NONE")
        m_comAdjust = GMSPolyhedron::COMAdjust::None;
      else
        throw ParseException(_cbs.Where(),
            "Invalid specification of com adjustment: '" + adjust +
            "'. Options are 'COM', 'Surface', or 'None'");
    }
    // Parse color.
    else if(c == 'c') {
      _is >> c; //read c(
      if(c != '(')
        throw ParseException(_cbs.Where(), "Invalid specification of color.");
      const Color4 color = ReadField<Color4>(_is, _cbs, "Invalid specification of color.");
      m_color = glutils::color(color[0], color[1], color[2], color[3]);
      _is >> c; //read )
      if(c != ')')
        throw ParseException(_cbs.Where(), "Invalid specification of color.");
    }
    // Parse optional texture file.
    else if(c == 't') {
      _is >> c; //read t(
      if(c != '(')
        throw ParseException(_cbs.Where(), "Invalid specification of texture.");
      m_textureFile = ReadFieldString(_is, _cbs,
          "Invalid specification of texture.", false);
      c = m_textureFile[m_textureFile.length() - 1];
      if(c == ')')
        m_textureFile = m_textureFile.substr(0, m_textureFile.length() - 1);
      else {
        _is >> c; //read )
        if(c != ')')
          throw ParseException(_cbs.Where(), "Invalid specification of texture.");
      }
    }
    // Put back - for possible -x translation.
    else {
      _is.putback(c);
      _is.putback('-');
      break;
    }

    while(isspace(_is.peek()))
      _is.get(c);
  }

  // Clear out whitespace and check if the next character is a letter.
  while(isspace(_is.peek()))
    _is.get(c);

  // Set the body type next.
  // If the next character is a letter, read the base type. Otherwise this is an
  // obstacle mb which does not specify 'fixed' for its body.
  if(isalpha(_is.peek())) {
    std::string bodyTag = ReadFieldString(_is, _cbs, "Failed reading base tag."
        " Options are: planar, volumetric, fixed, or joint.");
    SetBodyType(GetBodyTypeFromTag(bodyTag, _cbs.Where()));
  }
  else {
    SetBodyType(Body::Type::Fixed);
  }

  // Set the movement type and read the transform for fixed bodies.
  switch(m_bodyType) {
    case Body::Type::Volumetric:
    case Body::Type::Planar:
      // If base is volumetric or planar, we should parse the rotational type.
      {
        std::string baseMovementTag = ReadFieldString(_is, _cbs,
            "Failed reading rotation tag."
            " Options are: rotational or translational.");
        SetMovementType(GetMovementTypeFromTag(baseMovementTag, _cbs.Where()));
        break;
      }
    case Body::Type::Fixed:
      // If base if fixed, we should read a transformation.
      {
        m_transform = ReadField<Transformation>(_is, _cbs,
            "Failed reading fixed based transformation.");
        SetMovementType(Body::MovementType::Fixed);
        Configure(m_transform);
        break;
      }
    case Body::Type::Joint:
      // No additional parsing needed if base is a joint.
      SetMovementType(Body::MovementType::Joint);
      break;
  }

  // Read the mesh file.
  ReadGeometryFile(m_comAdjust);
}

/*----------------------------- Computation Helpers --------------------------*/

void
Body::
ComputeMomentOfInertia() const {
  const Vector3d centroid = m_polyhedron.GetCentroid();
  const std::vector<Vector3d>& vertices = m_polyhedron.GetVertexList();
  const double massPerVert = m_mass / vertices.size();
  auto& moment = const_cast<Matrix3x3&>(m_moment);

  moment = Matrix3x3();
  for(const auto& v : vertices) {
    const Vector3d r = v - centroid;
    moment[0][0] += massPerVert * (r[1] * r[1] + r[2] * r[2]);
    moment[0][1] += massPerVert * -r[0] * r[1];
    moment[0][2] += massPerVert * -r[0] * r[2];
    moment[1][0] += massPerVert * -r[1] * r[0];
    moment[1][1] += massPerVert * (r[0] * r[0] + r[2] * r[2]);
    moment[1][2] += massPerVert * -r[1] * r[2];
    moment[2][0] += massPerVert * -r[0] * r[2];
    moment[2][1] += massPerVert * -r[1] * r[2];
    moment[2][2] += massPerVert * (r[0] * r[0] + r[1] * r[1]);
  }
  moment = inverse(moment);
}


void
Body::
ComputeBoundingBox() const {
  auto& bbx = const_cast<GMSPolyhedron&>(m_boundingBox);
  bbx = GetPolyhedron().ComputeBoundingBox()->MakePolyhedron();
  bbx.Invert();
}


const Transformation&
Body::
FetchBaseTransform() const noexcept {
  return m_transform;
}


const Transformation&
Body::
FetchLinkTransform() const noexcept {
  if(m_transformCached)
    return m_transform;
  std::set<size_t> visited;
  return ComputeWorldTransformation(visited);
}


const Transformation&
Body::
ComputeWorldTransformation(std::set<size_t>& _visited) const {
  // If this link has already been visited, no need to do anything.
  if(_visited.find(m_index) != _visited.end())
    return m_transform;
  _visited.insert(m_index);

  // If the transform is already cached, return it.
  if(m_transformCached)
    return m_transform;
  m_transformCached = true;

  // If there are no backward connections (i.e. this is a base link), the
  // transform is already correct.
  if(m_backwardConnections.empty())
    return m_transform;

  // Compute the transform of this link from its backward connection.
  const Connection& back = *m_backwardConnections[0];
  auto& transform = const_cast<Transformation&>(m_transform);
  transform =
      back.GetPreviousBody()->ComputeWorldTransformation(_visited) *
      back.GetTransformationToDHFrame() *
      back.GetDHParameters().GetTransformation() *
      back.GetTransformationToBody2();

  return m_transform;
}

/*------------------------------- Display Stuff ------------------------------*/

const glutils::color&
Body::
GetColor() const {
  return m_color;
}


void
Body::
SetColor(const glutils::color& _m) {
  m_color = _m;
}


bool
Body::
IsTextureLoaded() const {
  return !m_textureFile.empty();
}


const string&
Body::
GetTexture() const {
  return m_textureFile;
}

/*-------------------------------- I/O Helpers -------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Body& _body) {
  const auto type = _body.GetBodyType();

  _os << _body.GetFileName() << " " << GetTagFromBodyType(type) << " ";

  switch(type) {
    case Body::Type::Planar:
    case Body::Type::Volumetric:
      _os << GetTagFromMovementType(_body.GetMovementType());
      break;
    case Body::Type::Fixed:
      _os << _body.GetWorldTransformation();
      break;
    case Body::Type::Joint:
      break;
  }

  return _os;
}

/*----------------------------------------------------------------------------*/
