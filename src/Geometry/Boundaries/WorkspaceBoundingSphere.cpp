#include "WorkspaceBoundingSphere.h"

#include "Geometry/GMSPolyhedron.h"

#include "glutils/triangulated_model.h"


/*------------------------------ Construction --------------------------------*/

WorkspaceBoundingSphere::
WorkspaceBoundingSphere(const size_t _n, const double _radius)
  : AbstractBoundingSphere(_n, _radius) {
  if(_n != 2 and _n != 3)
    throw RunTimeException(WHERE, "Workspace boundaries must have 2 or 3 "
        "dimensions.");
}


WorkspaceBoundingSphere::
WorkspaceBoundingSphere(const std::vector<double>& _center, const double _radius)
  : AbstractBoundingSphere(_center, _radius) {
  if(_center.size() != 2 and _center.size() != 3)
    throw RunTimeException(WHERE, "Workspace boundaries must have 2 or 3 "
        "dimensions.");
}


WorkspaceBoundingSphere::
WorkspaceBoundingSphere(const Vector3d& _center, const double _radius)
  : AbstractBoundingSphere({_center[0], _center[1], _center[2]}, _radius) { }


WorkspaceBoundingSphere::
WorkspaceBoundingSphere(XMLNode& _node) : AbstractBoundingSphere(_node) { }


WorkspaceBoundingSphere::
~WorkspaceBoundingSphere() noexcept = default;


std::unique_ptr<Boundary>
WorkspaceBoundingSphere::
Clone() const {
  return std::unique_ptr<WorkspaceBoundingSphere>(
      new WorkspaceBoundingSphere(*this)
  );
}

/*---------------------------- Property Accessors ----------------------------*/

Boundary::Space
WorkspaceBoundingSphere::
Type() const noexcept {
  return Boundary::Space::Workspace;
}


std::string
WorkspaceBoundingSphere::
Name() const noexcept {
  return "WorkspaceBoundingSphere";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
WorkspaceBoundingSphere::
InBoundary(const Cfg& _cfg) const {
  return InWorkspace(_cfg);
}

/*------------------------ Polyhedron Representations ------------------------*/

GMSPolyhedron
WorkspaceBoundingSphere::
MakePolyhedron() const {
  // We must approximate a sphere - define the number of segments to use.
  const size_t segments = 16;

  // We will use a short cylinder to approximate a 2-d spherical boundary.
  // Define its height here.
  const double height = .02;

  glutils::triangulated_model t;
  const auto& center = GetCenter();

  switch(NSphere::GetDimension())
  {
    case 3:
      t = glutils::triangulated_model::make_sphere(GetRadius(), segments);
      t.translate({(float)center[0], (float)center[1], (float)center[2]});
      break;
    case 2:
      t = glutils::triangulated_model::make_cylinder(GetRadius(), height,
          segments);
      t.translate({(float)center[0], (float)center[1], 0.});
      break;
    default:
      throw RunTimeException(WHERE, "GMSPolyhedron doesn't make sense for "
        + std::to_string(NSphere::GetDimension()) + "-d boundaries.");
  }

  // Create the polyhedron with inverted normals so that being inside the
  // boundary is considered outside of obstacle space.
  GMSPolyhedron poly(std::move(t));
  poly.Invert();
  return poly;
}

/*----------------------------------------------------------------------------*/
