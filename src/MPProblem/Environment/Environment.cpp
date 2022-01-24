#include "MPProblem/Environment/Environment.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/XMLNode.h"
#include "Workspace/WorkspaceDecomposition.h"
#include "Simulator/Conversions.h"

using namespace mathtool;


/*------------------------------- Construction -------------------------------*/

Terrain::
Terrain() = default;

Terrain::
Terrain(XMLNode& _node) {
	std::string color;
  for(auto& child : _node) {
		m_boundaries.push_back(std::move(Boundary::Factory(child)));
		color = child.Read("color", false, "blue", "Color of the Terrain");
		m_virtual = child.Read("virtual", false, false, "Does not invalidate other capabilites within.");
		m_wire = child.Read("wire", false, true, "Render type");
	}

  try {
    std::stringstream ss(color);
    ss >> this->m_color;
  }
  catch(const nonstd::exception&) {
    std::transform(color.begin(), color.end(), color.begin(), ::tolower);
    m_color = StringToColor(color);
  }
}


Terrain::
Terrain(const Terrain& _terrain) {
  m_color = _terrain.m_color;
	if(_terrain.m_boundary)
	  m_boundary = _terrain.m_boundary->Clone();
	for(auto& boundary : _terrain.m_boundaries){
		m_boundaries.push_back(std::move(boundary->Clone()));
	}
	m_virtual = _terrain.m_virtual;
  m_wire = _terrain.m_wire;
}

/*------------------------------- Accessors -------------------------------*/


const glutils::color&
Terrain::
Color() const noexcept {
  return m_color;
}


Boundary*
Terrain::
GetBoundary() const noexcept {
  return m_boundary.get();
}

const std::vector<std::unique_ptr<Boundary>>&
Terrain::
GetBoundaries() const noexcept {
	return m_boundaries;
}

double
Terrain::
GetPerimeter() {
	// Sum the perimeter of the individual boundaries 
  double total = 0;
	for(auto& boundary : m_boundaries){
		total += 2*boundary->GetRange(0).Length();
		total += 2*boundary->GetRange(1).Length();
	}

  // Subtract the overlap of the boundaries

  for(auto& boundary1: m_boundaries) {
    for(auto& boundary2: m_boundaries) {
      if(boundary1 == boundary2) {
        continue;
      }

      total -= 2 * Overlap(boundary1.get(), boundary2.get());
    }
  }

  return total;
}

bool
Terrain::
InTerrain(const Point3d _p) const noexcept {
	for(auto& boundary : m_boundaries){
		if(boundary->InBoundary(_p))
			return true;
	}
	return false;
}

bool
Terrain::
InTerrain(const Cfg _cfg) const noexcept {
	for(auto& boundary : m_boundaries){
		if(boundary->InBoundary(_cfg))
			return true;
	}
	return false;
}

bool
Terrain::
IsNeighbor(const Terrain& _terrain) {
  Axis type = Z;
  // iterate through the individual boundaries for both of the terrains
  for(auto& boundary1: m_boundaries){
    for(auto& boundary2: _terrain.m_boundaries){
      // if the boundaries are touching then return true
      if(IsTouching(boundary1.get(), boundary2.get(), type)){
          return true;
      }
    }
  }

  // if the boundaries aren't touching return false
  return false;
}

bool
Terrain::
IsVirtual() const noexcept {
	return m_virtual;
}

bool
Terrain::
IsWired() const noexcept {
  return m_wire;
}

/*------------------------------- Helpers -------------------------------*/

bool
Terrain::
IsTouching(Boundary* _b1, Boundary* _b2, Axis& _type) {
  /// @todo Check the implementation of this function

  // Get min/max values of boundaries in x direction
  double b1maxX = _b1->GetRange(0).max;
  double b1minX = _b1->GetRange(0).min;
  double b2maxX = _b2->GetRange(0).max;
  double b2minX = _b2->GetRange(0).min;

  // Get min/max values of boundaries in y direction
  double b1maxY = _b1->GetRange(1).max;
  double b1minY = _b1->GetRange(1).min;
  double b2maxY = _b2->GetRange(1).max;
  double b2minY = _b2->GetRange(1).min;

  // Determine if touching in x direction
  if((b1maxX == b2minX || b2maxX == b1minX)) {
     if(_b1->GetRange(1).Contains(b2maxY) ||
      			_b1->GetRange(1).Contains(b2minY) ||
      			_b2->GetRange(1).Contains(b1maxY) ||
      			_b2->GetRange(1).Contains(b1minY)) {
      _type = X;
      return true;
		}
  }
  // Determine if touching in y direction
  if(( b1maxY == b2minY || b2maxY == b1minY)) {
     if(_b1->GetRange(0).Contains(b2maxX) ||
      			 _b1->GetRange(0).Contains(b2minX) ||
      		 	 _b2->GetRange(0).Contains(b1maxX) ||
      			 _b2->GetRange(0).Contains(b1minX)) {
      _type = Y;
      return true;
		}
  }

  return false;
}

double
Terrain::
Overlap(Boundary* _b1, Boundary* _b2) {
  // Determine if boundaries touching on X or Y axis
	Axis type = Z;
  if(!IsTouching(_b1, _b2, type)) {
    std::cout << "ERROR: The two boundaries are not touching." << '\n';
    return 0;
  }

  double max1 = 0.0;
  double max2 = 0.0;
  double min1 = 0.0;
  double min2 = 0.0;

  switch(type) {
    case X:
      max1 = _b1->GetRange(1).max;
      min1 = _b1->GetRange(1).min;
      max2 = _b2->GetRange(1).max;
      min2 = _b2->GetRange(1).min;
    case Y:
      max1 = _b1->GetRange(0).max;
      min1 = _b1->GetRange(0).min;
      max2 = _b2->GetRange(0).max;
      min2 = _b2->GetRange(0).min;
    case Z:
      std::cout << "ERROR: The two boundaries are not touching." << '\n';
  }

  // this will calculate the overlap of the two borders
  return std::min(max1, max2) - std::max(min1, min2);
}

/*------------------------------- Construction -------------------------------*/

Environment::
Environment() = default;


Environment::
Environment(XMLNode& _node) {
  m_filename = GetPathName(_node.Filename())
             + _node.Read("filename", true, "", "env filename");

  // If the filename is an XML file we will read all of the environment
  // information from that file.
  const bool readXML = m_filename.substr(m_filename.rfind(".", std::string::npos))
                     == ".xml";

  if(readXML) {
    XMLNode envNode = XMLNode(m_filename, "Environment");
    // First read options from environment XML file.
    ReadXMLOptions(envNode);
    ReadXML(envNode);

    // Then read from problem XML node which may override some of the options from
    // the environment XML file.
    ReadXMLOptions(_node);
  }
  else {
    ReadXMLOptions(_node);
    Read(m_filename);
  }

  if(m_boundaryObstacle)
    CreateBoundaryObstacle();

}


Environment::
Environment(const Environment& _other) {
  *this = _other;
}


Environment::
Environment(Environment&& _other) = default;


Environment::
~Environment() = default;

/*-------------------------------- Assignment --------------------------------*/

Environment&
Environment::
operator=(const Environment& _other) {
  m_filename            = _other.m_filename;
  m_modelDataDir        = _other.m_modelDataDir;
  m_positionRes         = _other.m_positionRes;
  m_orientationRes      = _other.m_orientationRes;
  m_timeRes             = _other.m_timeRes;
  m_frictionCoefficient = _other.m_frictionCoefficient;
  m_gravity             = _other.m_gravity;
  m_terrains            = _other.m_terrains;

  // Copy the boundary.
  SetBoundary(_other.m_boundary->Clone());

  // Copy the obstacles.
  /// @note We deliberately duplicate the data to make sure the copies are
  ///       entirely independent. We may wish to revisit this decision at a
  ///       later time since it is probably safe to share data on static
  ///       obstacles. I am leaving it this way for now to minimize surprises.
  for(const auto& obstacle : _other.m_obstacles)
    m_obstacles.emplace_back(new MultiBody(*obstacle));

  return *this;
}


Environment&
Environment::
operator=(Environment&& _other) = default;

/*------------------------------------ I/O -----------------------------------*/

const std::string&
Environment::
GetEnvFileName() const noexcept {
  return m_filename;
}


void
Environment::
ReadXMLOptions(XMLNode& _node) {
  // Read the friction coefficient (to be used uniformly for now)
  m_frictionCoefficient = _node.Read("frictionCoefficient", false, 0., 0.,
      std::numeric_limits<double>::max(), "friction coefficient (uniform)");

  // Read in the gravity vector.
  {
    const std::string gravity = _node.Read("gravity", false, "0 0 0",
        "The gravity vector in this environment.");
    std::istringstream buffer(gravity);
    buffer >> m_gravity;
  }

  // If the position or orientation resolution is provided in the xml, overwrite
  // any previous value that could have been set in the env file.
  m_positionRes = _node.Read("positionRes", false, m_positionRes,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "Positional resolution of environment");
  m_orientationRes = _node.Read("orientationRes", false, m_orientationRes,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "Orientation resolution of environment");
  m_timeRes = _node.Read("timeRes", false, m_timeRes, .05, 10.,
      "Time resolution in seconds");

  m_boundaryObstacle = _node.Read("boundaryObstacle", false, m_boundaryObstacle,
      "Create a multibody obstacle for the boundary.");

  for(auto& child : _node) {
    if(child.Name() == "Terrain") {

      std::string capability = child.Read("capability", true, "", "The types of "
          "agents that can be within the terrain boundaries.");
      std::transform(capability.begin(), capability.end(), capability.begin(), ::tolower);

			Terrain terrain(child);

			m_terrains[capability].push_back(std::move(terrain));
      /*for(auto& grandChild : child) {
        if(grandChild.Name() == "Boundary") {

        }
      }*/
    }
  }

  // Read the initial camera transformation. Assume slightly behind the z = 0
  // plane if not provided.
  {
    const std::string transformation = _node.Read("cameraInit", false,
        "0 0 20 0 0 0", "The initial transformation for the camera.");
    std::istringstream buffer(transformation);
    buffer >> m_initialCameraTransform;
  }
}


void
Environment::
ReadXML(XMLNode& _node) {
  size_t sl = m_filename.rfind('/');
  m_modelDataDir = m_filename.substr(0, sl == std::string::npos ? 0 : sl) + "/";
  Body::m_modelDataDir = m_modelDataDir;

  m_obstacles.clear();

  // Read and construct boundary, bodies, and other objects in the environment.
  for(auto& child : _node) {
    if(child.Name() == "Boundary") {
      m_boundary = Boundary::Factory(child);
    }
    else if(child.Name() == "MultiBody") {
      m_obstacles.emplace_back(new MultiBody(child));

      /// @todo Add support for dynamic obstacles
      if(m_obstacles.back()->IsActive())
        throw ParseException(_node.Where(), "Dynamic obstacles are not yet "
            "supported.");
    }
  }
}


void
Environment::
Read(std::string _filename) {
  if(!FileExists(_filename))
    throw ParseException(_filename, "File does not exist");

  m_filename = _filename;
  size_t sl = m_filename.rfind('/');
  m_modelDataDir = m_filename.substr(0, sl == std::string::npos ? 0 : sl) + "/";
  Body::m_modelDataDir = m_modelDataDir;

  m_obstacles.clear();

  // open file
  CountingStreamBuffer cbs(_filename);
  std::istream ifs(&cbs);

  // read boundary
  std::string bndry = ReadFieldString(ifs, cbs, "Failed reading boundary tag.");
  if(bndry != "BOUNDARY")
    throw ParseException(cbs.Where(),
        "Unknown boundary tag '" + bndry + "'. Should read 'Boundary'.");
  std::string btype = ReadFieldString(ifs, cbs,
      "Failed reading boundary type. Options are: box or sphere.");
  InitializeBoundary(btype, cbs.Where());
  m_boundary->Read(ifs, cbs);

  // read resolutions
  std::string resolution;
  while((resolution = ReadFieldString(ifs, cbs, "Failed reading resolution tag."))
      != "MULTIBODIES") {
    if(resolution == "POSITIONRES")
      m_positionRes = ReadField<double>(ifs, cbs, "Failed reading Position "
          "resolution\n");
    else if(resolution == "ORIENTATION")
      m_orientationRes = ReadField<double>(ifs, cbs, "Failed reading "
          "Orientation resolution\n");
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    else if(resolution == "RDRES")
      m_rdRes = ReadField<double>(ifs, cbs, "Failed reading Reachable Distance "
          "resolution");
#endif
    else if(resolution == "TIMERES")
      m_timeRes = ReadField<double>(ifs, cbs, "Failed reading Time resolution\n");
    else
      throw ParseException(cbs.Where(), "Unknown resolution tag '" + resolution
          + "'");
  }

  size_t multibodyCount = ReadField<size_t>(ifs, cbs,
      "Failed reading number of multibodies.");

  //parse and construct each multibody
  for(size_t m = 0; m < multibodyCount && ifs; ++m) {
    m_obstacles.emplace_back(new MultiBody(MultiBody::Type::Passive));
    m_obstacles.back()->Read(ifs, cbs);

    /// @todo Add support for dynamic obstacles
    if(m_obstacles.back()->IsActive())
      throw ParseException(cbs.Where(), "Dynamic obstacles are not yet "
          "supported.");
  }
}


void
Environment::
Print(std::ostream& _os) const {
  _os << "Environment"
      << "\n\tpositionRes: " << m_positionRes
      << "\n\torientationRes: " << m_orientationRes
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
      << "\n\trdRes: " << m_rdRes
#endif
      << "\n\tboundary::" << *m_boundary
      << std::endl;
  for(const auto& obstacle : m_obstacles) {
    obstacle->Write(_os);
    _os << std::endl;
  }
}


void
Environment::
Write(std::ostream & _os) {
  _os << "Boundary ";
  m_boundary->Write(_os);
  _os << "\n\nObstacles\n"
      << m_obstacles.size()
      << "\n\n";
  for(const auto& body : m_obstacles) {
    body->Write(_os);
    _os << std::endl;
  }
}

/*-------------------------------- Resolutions -------------------------------*/

void
Environment::
ComputeResolution(const std::vector<std::unique_ptr<Robot>>& _robots) {
  if(m_positionRes > 0.)
    return; // Do not compute it.

  // The resolution needs to ensure that any motion of resolution length can't
  // completely skip a collision. I.e. if we have two thin objects moving past
  // each other, they must not be able to get all the way through each other by
  // taking steps of resolution length.
  //
  // There is no guaranteed way to compute this without considering all
  // possibile ways that two objects can collide. We will use a heuristic
  // resolution of 1% of the smallest bounding-box length (excluding dimensions
  // which aren't relevant in this environment).

  // We only need to care about dimensions that are relevant to the environment
  // boundary.
  const size_t dimensions = m_boundary->GetDimension();

  // Determine the minimum bbx span (i.e. the smallest bbx length for any object
  // in the environemnt).
  double minimumSpan = std::numeric_limits<double>::max();

  // Find the minimum span amongst the robots.
  for(const auto& robot : _robots) {
    const auto multibody = robot->GetMultiBody();
    for(const auto& body : multibody->GetBodies()) {
      const auto b = body.GetPolyhedron().ComputeBoundingBox();

      for(size_t i = 0; i < dimensions; ++i)
        minimumSpan = std::min(minimumSpan, b->GetRange(i).Length());
    }
  }

  // Find the minimum span amongst the obstacles.
  for(const auto& multibody : m_obstacles) {
    for(const auto& body : multibody->GetBodies()) {
      const auto b = body.GetPolyhedron().ComputeBoundingBox();

      for(size_t i = 0; i < dimensions; ++i)
        minimumSpan = std::min(minimumSpan, b->GetRange(i).Length());
    }
  }

  // Estimate a good resolution as 1% of the minimum bbx span.
  m_positionRes = minimumSpan * .01;

  /// @todo Add an automatic computation of the orientation resolution here.
  ///       This should be done so that rotating the robot base by one orientation
  ///       resolution makes a point on the bounding sphere move by one position
  ///       resolution. If possible we should also separate the resolutions for
  ///       joint angles and base orientations since this gets really, really
  ///       small with manipulators having several links (probably we'll need to
  ///       manage this in the local planner to make maximum use of the
  ///       available resolution).

  // This is a very important parameter - always report a notice when we compute
  // it automatically.
  std::cout << "Automatically computed position resolution as " << m_positionRes
            << std::endl;
}


double
Environment::
GetPositionRes() const noexcept {
  return m_positionRes;
}


void
Environment::
SetPositionRes(double _res) noexcept {
  m_positionRes = _res;
}


double
Environment::
GetOrientationRes() const noexcept {
  return m_orientationRes;
}


void
Environment::
SetOrientationRes(double _res) noexcept {
  m_orientationRes = _res;
}


double
Environment::
GetTimeRes() const noexcept {
  return m_timeRes;
}

/*----------------------------- Boundary Functions ---------------------------*/

Boundary*
Environment::
GetBoundary() const noexcept {
  return m_boundary.get();
}


void
Environment::
SetBoundary(std::unique_ptr<Boundary>&& _b) noexcept {
  m_boundary = std::move(_b);
}

/*---------------------------- Obstacle Functions ----------------------------*/

size_t
Environment::
NumObstacles() const noexcept {
  return m_obstacles.size();
}


MultiBody*
Environment::
GetObstacle(size_t _index) const {
  if(_index < 0 || _index >= m_obstacles.size())
    throw RunTimeException(WHERE) << "Cannot access obstacle '" << _index << "'.";
  return m_obstacles[_index].get();
}


MultiBody*
Environment::
GetRandomObstacle() const {
  if(m_obstacles.empty())
    throw RunTimeException(WHERE, "No static multibodies to select from.");

  size_t rIndex = LRand() % m_obstacles.size();
  return m_obstacles[rIndex].get();
}


size_t
Environment::
AddObstacle(const std::string& _dir, const std::string& _filename,
    const Transformation& _t) {
  const std::string filename = _dir.empty() ? _filename
                                            : _dir + '/' + _filename;

  // Make a multibody for this obstacle.
  std::unique_ptr<MultiBody> mb(new MultiBody(MultiBody::Type::Passive));

  // Make the obstacle geometry.
  Body body(mb.get());
  body.SetBodyType(Body::Type::Fixed);
  body.ReadGeometryFile(filename);
  body.Configure(_t);

  // Add the body to the multibody and finish initialization.
  const size_t index = mb->AddBody(std::move(body));
  mb->SetBaseBody(index);

  m_obstacles.push_back(std::move(mb));
  return m_obstacles.size() - 1;
}


void
Environment::
RemoveObstacle(const size_t _index) {
  const size_t count = m_obstacles.size();
  if(_index < count)
    m_obstacles.erase(m_obstacles.begin() + _index);
  else
    throw RunTimeException(WHERE) << "Cannot remove obstacle with index "
                                  << _index << ", only " << count
                                  << " obstacles in the environment.";
}


void
Environment::
RemoveObstacle(MultiBody* const _obst) {
  for(auto iter = m_obstacles.begin(); iter != m_obstacles.end(); ++iter) {
    if(iter->get() != _obst)
      continue;
    m_obstacles.erase(iter);
    return;
  }

  throw RunTimeException(WHERE) << "Cannot remove obstacle " << _obst
                                << ", does not match any obstacles in the "
                                << "environment.";
}


std::map<Vector3d, std::vector<size_t>>
Environment::
ComputeObstacleVertexMap() const {
  std::map<Vector3d, std::vector<size_t>> out;

  // Iterate through all the obstacles and add their points to the map.
  for(size_t i = 0; i < NumObstacles(); ++i) {
    MultiBody* const obst = GetObstacle(i);
    for(size_t j = 0; j < obst->GetNumBodies(); ++j) {
      const auto& obstaclePoly = obst->GetBody(j)->GetWorldPolyhedron();
      for(const auto& v : obstaclePoly.GetVertexList())
        out[v].push_back(i);
    }
  }

  return out;
}


bool
Environment::
UsingBoundaryObstacle() const noexcept {
  return m_boundaryObstacle;
}

/*-------------------------- Physical Properties -----------------------------*/

double
Environment::
GetFrictionCoefficient() const noexcept {
  return m_frictionCoefficient;
}


const Vector3d&
Environment::
GetGravity() const noexcept {
  return m_gravity;
}


/*-------------------------- Terrain Functions -----------------------------*/

const Environment::TerrainMap&
Environment::
GetTerrains() const noexcept {
  return m_terrains;
}

/*--------------------------- Simulation Functions ---------------------------*/

const mathtool::Transformation&
Environment::
GetInitialCameraTransformation() const noexcept {
  return m_initialCameraTransform;
}

/*------------------------------- Helpers ------------------------------------*/

void
Environment::
InitializeBoundary(std::string _type, const std::string _where) {
  std::transform(_type.begin(), _type.end(), _type.begin(), ::tolower);

  if(_type == "box")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingBox(3));
  else if(_type == "box2d")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingBox(2));
  else if(_type == "sphere")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingSphere(3));
  else if(_type == "sphere2d")
    m_boundary = std::unique_ptr<Boundary>(new WorkspaceBoundingSphere(2));
  else
    throw ParseException(_where, "Unknown boundary type '" + _type +
        "'. Options are: box, box2d, sphere, or sphere2d.");
}


void
Environment::
CreateBoundaryObstacle() {
  // Make a multibody for the boundary obstacle.
  std::unique_ptr<MultiBody> mb(new MultiBody(MultiBody::Type::Passive));

  // Make the boundary body.
  Body body(mb.get());
  body.SetBodyType(Body::Type::Fixed);
  GMSPolyhedron poly = m_boundary->MakePolyhedron();
  body.SetPolyhedron(std::move(poly));

  // Add the body to the multibody and finish initialization.
  const size_t index = mb->AddBody(std::move(body));
  mb->SetBaseBody(index);

  // Ensure that we don't double-add the boundary obstacle.
  const bool alreadyAdded = !m_obstacles.empty() and
    mb->GetBase()->GetPolyhedron() ==
    m_obstacles[0]->GetBase()->GetPolyhedron();
  if(alreadyAdded) {
    std::cout << "Boundary obstacle already exists." << std::endl;
    return;
  }
  else
    std::cout << "Created boundary obstacle." << std::endl;

  // Insert the boundary at obstacle index 0.
  m_obstacles.insert(m_obstacles.begin(), std::move(mb));
}

/*----------------------------------------------------------------------------*/


//IROS Hacks

bool
Environment::
IsolateTerrain(Cfg start, Cfg goal){
  if(start.GetRobot()->GetCapability() != goal.GetRobot()->GetCapability() or
      start.GetRobot()->GetCapability() == "")
    return false;
  for(auto& terrain : m_terrains[start.GetRobot()->GetCapability()]){
    if(terrain.GetBoundary()->InBoundary(start) and terrain.GetBoundary()->InBoundary(goal)){
      this->SetBoundary(std::move(terrain.GetBoundary()->Clone()));
      return true;
    }
  }
  return false;
}

bool
Environment::
SameTerrain(Cfg _start, Cfg _goal) {
	if(_start.GetRobot()->GetCapability() != _goal.GetRobot()->GetCapability() or
			_start.GetRobot()->GetCapability() == "")
		return false;
	for(auto & terrain : m_terrains[_start.GetRobot()->GetCapability()]) {
		if(terrain.InTerrain(_start) and terrain.InTerrain(_goal))
			return true;
	}
	return false;
}

void
Environment::
RestoreBoundary(){
  this->SetBoundary(std::move(m_originalBoundary));
}

void
Environment::
SaveBoundary(){
  m_originalBoundary = this->GetBoundary()->Clone();
}
