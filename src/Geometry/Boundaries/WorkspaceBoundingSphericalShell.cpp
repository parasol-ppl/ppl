#include "WorkspaceBoundingSphericalShell.h"

#include "Geometry/GMSPolyhedron.h"

#include "glutils/triangulated_model.h"


/*------------------------------ Construction --------------------------------*/

WorkspaceBoundingSphericalShell::
WorkspaceBoundingSphericalShell(const size_t _n, const double _outer,
    const double _inner)
  : AbstractBoundingSphericalShell(_n, _outer, _inner) {
  if(_n != 2 and _n != 3)
    throw RunTimeException(WHERE) << "Workspace boundaries must have 2 or 3 "
                                  << "dimensions.";
}


WorkspaceBoundingSphericalShell::
WorkspaceBoundingSphericalShell(const std::vector<double>& _center,
    const double _outer, const double _inner)
  : AbstractBoundingSphericalShell(_center, _outer, _inner) {
  if(_center.size() != 2 and _center.size() != 3)
    throw RunTimeException(WHERE) << "Workspace boundaries must have 2 or 3 "
                                  << "dimensions.";
}


WorkspaceBoundingSphericalShell::
WorkspaceBoundingSphericalShell(const Vector3d& _center, const double _outer,
    const double _inner)
  : AbstractBoundingSphericalShell({_center[0], _center[1], _center[2]}, _outer,
      _inner) { }


WorkspaceBoundingSphericalShell::
WorkspaceBoundingSphericalShell(XMLNode& _node)
  : AbstractBoundingSphericalShell(_node) { }


WorkspaceBoundingSphericalShell::
~WorkspaceBoundingSphericalShell() noexcept = default;


std::unique_ptr<Boundary>
WorkspaceBoundingSphericalShell::
Clone() const {
  return std::unique_ptr<WorkspaceBoundingSphericalShell>(
      new WorkspaceBoundingSphericalShell(*this)
  );
}

/*---------------------------- Property Accessors ----------------------------*/

Boundary::Space
WorkspaceBoundingSphericalShell::
Type() const noexcept {
  return Boundary::Space::Workspace;
}


std::string
WorkspaceBoundingSphericalShell::
Name() const noexcept {
  return "WorkspaceBoundingSphericalShell";
}

/*--------------------------- Containment Testing ----------------------------*/

bool
WorkspaceBoundingSphericalShell::
InBoundary(const Cfg& _cfg) const {
  return InWorkspace(_cfg);
}

/*------------------------ Polyhedron Representations ------------------------*/

GMSPolyhedron
WorkspaceBoundingSphericalShell::
MakePolyhedron() const {
  // We must approximate a sphere - define the number of segments to use.
  const size_t segments = 16;

  // We will use a short cylinder to approximate a 2-d spherical boundary.
  // Define its height here.
  const double height = .02;

  using glutils::triangulated_model;
  triangulated_model t;
  const auto& center = GetCenter();

  switch(NSphericalShell::GetDimension())
  {
    case 3:
      t = triangulated_model::make_sphere(GetOuterRadius(), segments);
      t.add_model(
          triangulated_model::make_sphere(GetInnerRadius(), segments).reverse());
      t.translate({(float)center[0], (float)center[1], (float)center[2]});
      break;
    case 2:
      t = triangulated_model::make_cylinder(GetOuterRadius(), height, segments);
      t.add_model(
          triangulated_model::make_cylinder(GetInnerRadius(), height, segments).reverse());
      t.translate({(float)center[0], (float)center[1], 0.});
      break;
    default:
      throw RunTimeException(WHERE) << "GMSPolyhedron doesn't make sense for "
                                    << NSphericalShell::GetDimension()
                                    << "-d boundaries.";
  }

  // Create the polyhedron with inverted normals so that being inside the
  // boundary is considered outside of obstacle space.
  GMSPolyhedron poly(std::move(t));
  poly.Invert();
  return poly;
}

/*----------------------------------------------------------------------------*/
