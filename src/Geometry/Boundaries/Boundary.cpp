#include "Geometry/Boundaries/Boundary.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/GMSPolyhedron.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Geometry/Boundaries/CSpaceBoundingSphere.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "Geometry/Boundaries/WorkspaceBoundingSphere.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/XMLNode.h"

#include "nonstd/numerics.h"


/*------------------------------- Construction -------------------------------*/

Boundary::
~Boundary() noexcept = default;


std::unique_ptr<Boundary>
Boundary::
Factory(XMLNode& _node) {
  // Read the boundary space, shape, and dimension.
  std::string space = _node.Read("space", false, "workspace",
      "The boundary space {cspace | workspace}.");
  std::string shape = _node.Read("shape", true, "",
      "The boundary shape {box | sphere}.");

  // Downcase space and shape.
  std::transform(space.begin(), space.end(), space.begin(), ::tolower);
  std::transform(shape.begin(), shape.end(), shape.begin(), ::tolower);

  if(space == "cspace") {
    if(shape == "box")
      return std::unique_ptr<CSpaceBoundingBox>(
          new CSpaceBoundingBox(_node)
      );
    else if(shape == "sphere")
      return std::unique_ptr<CSpaceBoundingSphere>(
          new CSpaceBoundingSphere(_node)
      );
  }
  else if(space == "workspace") {
    if(shape == "box")
      return std::unique_ptr<WorkspaceBoundingBox>(
          new WorkspaceBoundingBox(_node)
      );
    else if(shape == "sphere")
      return std::unique_ptr<WorkspaceBoundingSphere>(
          new WorkspaceBoundingSphere(_node)
      );
  }
  else
    throw ParseException(_node.Where()) << "Unrecognized space option '"
                                        << space << "'.";

  throw ParseException(_node.Where()) << "Unrecognized shape option '"
                                      << shape << "'.";
  return {nullptr};
}

/*--------------------------------- Scaling ----------------------------------*/

void
Boundary::
ScalePoint(std::vector<double>& _point) const noexcept {
  const size_t limit = std::min(GetDimension(), _point.size());

  for(size_t i = 0; i < limit; ++i) {
    const auto& r = GetRange(i);
    _point[i] = nonstd::rescale(_point[i], r.min, r.max, -1., 1.);
  }
}


void
Boundary::
UnscalePoint(std::vector<double>& _point) const noexcept {
  const size_t limit = std::min(GetDimension(), _point.size());

  for(size_t i = 0; i < limit; ++i) {
    const auto& r = GetRange(i);
    _point[i] = nonstd::rescale(_point[i], -1., 1., r.min, r.max);
  }
}

/*--------------------------- Containment Testing ----------------------------*/

bool
Boundary::
InBoundary(const Vector3d& _p) const {
  return InBoundary(std::vector<double>{_p[0], _p[1], _p[2]});
}

/*--------------------------- Containment Helpers ----------------------------*/

bool
Boundary::
InWorkspace(const Cfg& _cfg) const {
  auto multiBody = _cfg.GetMultiBody();

  // If the multiBody's center is more than a bounding radius away from the
  // wall, it is definitely inside.
  if(GetClearance(_cfg.GetPoint()) >= multiBody->GetBoundingSphereRadius())
    return true;

  // Robot is close to wall, have to perform strict check.
  _cfg.ConfigureRobot();

  // Check each part of the multibody for being inside of the boundary.
  for(size_t m = 0; m < multiBody->GetNumBodies(); ++m) {
    const auto body = multiBody->GetBody(m);

    // First check if the body's bounding box intersects the boundary.
    bool bbxGood = true;
    const GMSPolyhedron bbx = body->GetWorldBoundingBox();
    for(const auto& v : bbx.GetVertexList()) {
      if(!InBoundary(v)) {
        bbxGood = false;
        break;
      }
    }

    // If the bounding box is inside the boundary, the whole geometry must be.
    // Skip the fine-grain check.
    if(bbxGood)
      continue;

    // If the bounding box touches the boundary, we need to check each vertex in
    // the body geometry. Doing this one vertex at a time allows us to avoid
    // extraneously transforming the remaining vertices.
    const GMSPolyhedron& poly = body->GetPolyhedron();
    const Transformation& trans = body->GetWorldTransformation();
    for(const auto& v : poly.GetVertexList())
      if(!InBoundary(trans * v))
        return false;
  }

  // None of the multiBody parts touched the boundary, so we are inside.
  return true;
}


bool
Boundary::
InCSpace(const Cfg& _c) const {
  return InBoundary(_c.GetData());
}

/*----------------------- Clearance Testing ----------------------------------*/

double
Boundary::
GetClearance(const Vector3d& _p) const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return 0.;
}


Vector3d
Boundary::
GetClearancePoint(const Vector3d& _p) const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return Vector3d();
}

/*----------------------- Polyhedron Representations -------------------------*/

Boundary::CGALPolyhedron
Boundary::
CGAL() const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return CGALPolyhedron();
}


GMSPolyhedron
Boundary::
MakePolyhedron() const {
  throw RunTimeException(WHERE, "No base class implementation supported.");
  return GMSPolyhedron();
}

/*----------------------------------- I/O ------------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Boundary& _b) {
  /// @TODO Synchronize type printing with input parsing.
  _os << _b.Name() << " ";
  _b.Write(_os);
  return _os;
}


/// @TODO Move implementation from Environment.cpp to here.
//std::istream&
//operator>>(std::istream& _is, const Boundary& _b) {
//  return _is;
//}

/*----------------------------------------------------------------------------*/
