#include "TetrahedralBoundary.h"

#include "Geometry/GMSPolyhedron.h"
#include "Simulator/Conversions.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"

#include <algorithm>


/*------------------------------- Construction -------------------------------*/

TetrahedralBoundary::
TetrahedralBoundary(const std::array<Point3d, 4>& _pts, const bool _check) :
    m_points(_pts) {
  if(_check)
    OrderPoints();
  m_normals = ComputeNormals();
  m_bbx = ComputeBBX();
  m_volume = ComputeVolume();
}


TetrahedralBoundary::
TetrahedralBoundary(const std::vector<Point3d>& _pts, const bool _check) {
  // Ensure the input vector contains exactly four points.
  if(_pts.size() != 4)
    throw RunTimeException(WHERE) << "Can't build tetrahedron with "
                                  << _pts.size() << " points.";

  std::copy(_pts.begin(), _pts.end(), m_points.begin());
  if(_check)
    OrderPoints();
  m_normals = ComputeNormals();
  m_bbx = ComputeBBX();
  m_volume = ComputeVolume();
}


TetrahedralBoundary::
TetrahedralBoundary(XMLNode& _node) {
  throw NotImplementedException(WHERE);
}


TetrahedralBoundary::
~TetrahedralBoundary() noexcept = default;


std::unique_ptr<Boundary>
TetrahedralBoundary::
Clone() const {
  return std::unique_ptr<TetrahedralBoundary>(new TetrahedralBoundary(*this));
}

/*---------------------------- Property Accessors ----------------------------*/

Boundary::Space
TetrahedralBoundary::
Type() const noexcept {
  return Boundary::Space::Workspace;
}


std::string
TetrahedralBoundary::
Name() const noexcept {
  return "Tetrahedral";
}


size_t
TetrahedralBoundary::
GetDimension() const noexcept {
  return 3;
}


double
TetrahedralBoundary::
GetMaxDist(const double _r1, const double _r2) const {
  auto edges = ComputeEdges();

  std::array<double, 6> edgeLengths;
  for(size_t i = 0; i < 6; ++i)
    edgeLengths[i] = edges[i].norm();

  return *std::max_element(edgeLengths.begin(), edgeLengths.end());
}


const Range<double>&
TetrahedralBoundary::
GetRange(const size_t _i) const {
  return m_bbx.GetRange(_i);
}


const std::vector<double>&
TetrahedralBoundary::
GetCenter() const noexcept {
  throw RunTimeException(WHERE) << "Impl is wrong, need to compute the barycenter.";
  return m_bbx.GetCenter();
}


double
TetrahedralBoundary::
GetVolume() const noexcept {
  return m_volume;
}

/*--------------------------------- Sampling ---------------------------------*/

std::vector<double>
TetrahedralBoundary::
GetRandomPoint() const {
  // From:
  //   C. Rocchini and P. Cignoni, "Generating Random Points in a Tetrahedron,"
  //       Journal of Graphics Tools, 2001.

  // Pick a point in unit cube.
  double s = DRand();
  double t = DRand();
  double u = DRand();

  // Cut cube in half with plane s + t = 1.
  if(s + t > 1) {
    s = 1 - s;
    t = 1 - t;
  }

  // Cut cube with planes t + u = 1 and s + t + u = 1.
  if(s + t + u > 1) {
    if(t + u > 1) {
      double ttmp = 1 - u;
      double utmp = 1 - s - t;
      std::swap(t, ttmp);
      std::swap(u, utmp);
    }
    else {
      double stmp = 1 - t - u;
      double utmp = s + t + u - 1;
      std::swap(s, stmp);
      std::swap(u, utmp);
    }
  }

  // Determine random point in tetrahedron.
  auto pt = m_points[0] + s * (m_points[1] - m_points[0])
                        + t * (m_points[2] - m_points[0])
                        + u * (m_points[3] - m_points[0]);

  return std::vector<double>{pt[0], pt[1], pt[2]};
}


void
TetrahedralBoundary::
PushInside(std::vector<double>& _sample) const noexcept {
  if(_sample.size() != 3)
    throw RunTimeException(WHERE) << "Only three dimensional points are "
                                  << "supported.";

  const Vector3d input(_sample[0], _sample[1], _sample[2]);

  // If _sample is already in the boundary, there is nothing to do.
  if(InBoundary(input))
    return;

  const Vector3d output = GetClearancePoint(input);
  _sample = {output[0], output[1], output[2]};
}

/*--------------------------- Containment Testing ----------------------------*/

bool
TetrahedralBoundary::
InBoundary(const Vector3d& _p) const {
  // Check dot-product with normals touching point 0.
  const Vector3d p = _p - m_points[0];
  for(size_t i = 0; i < 3; ++i)
    if(p * m_normals[i] > 0)
      return false;

  // Check dot-product with last normal.
  if((_p - m_points[1]) * m_normals[3] > 0)
    return false;

  return true;
}


bool
TetrahedralBoundary::
InBoundary(const std::vector<double>& _v) const {
  if(_v.size() != 3)
    throw RunTimeException(WHERE) << "Does not make sense for vectors of "
                                  << "dimension " << _v.size() << " != 3.";

  return InBoundary(Vector3d(_v[0], _v[1], _v[2]));
}


bool
TetrahedralBoundary::
InBoundary(const Cfg& _c) const {
  return InWorkspace(_c);
}

/*---------------------------- Clearance Testing -----------------------------*/

double
TetrahedralBoundary::
GetClearance(const Vector3d& _p) const {
  throw NotImplementedException(WHERE);
}


Vector3d
TetrahedralBoundary::
GetClearancePoint(const Vector3d& _p) const {
  // If _p is inside, the clearance point will be _p projected to the nearest
  // plane.
  // Otherwise, there are possibilities:
  // 1. _p is nearest to a vertex. This happens when it is 'in front' of exactly
  //    three facet planes. The near point is the shared vertex.
  // 2. _p is nearest to an edge. This happens when it is 'in front' of exactly
  //    two facet planes. The near point is the nearest point on the edge.
  // 3. _p is nearest to a facet. This happens when it is 'in front' of exactly
  //    one facet plane. The near point is the nearest point on the triangle.
  throw NotImplementedException(WHERE);

  return Vector3d();
}

/*--------------------------------- Modifiers --------------------------------*/

void
TetrahedralBoundary::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), size_t(3));

  for(size_t i = 0; i < maxIndex; ++i) {
    const double offset = _c[i] - GetCenter()[i];
    for(size_t j = 0; j < 4; ++j)
      m_points[j][i] += offset;
  }

  m_bbx.SetCenter(_c);
}


void
TetrahedralBoundary::
Translate(const Vector3d& _v) {
  m_bbx.Translate({_v[0], _v[1], _v[2]});
  for(auto& p : m_points)
    p += _v;
}


void
TetrahedralBoundary::
Translate(const std::vector<double>& _t) {
  m_bbx.Translate(_t);

  const size_t maxIndex = std::min(size_t(3), _t.size());
  Vector3d v;
  for(size_t i = 0; i < maxIndex; ++i)
    v[i] = _t[i];

  for(auto& p : m_points)
    p += v;
}


void
TetrahedralBoundary::
ResetBoundary(const std::vector<std::pair<double, double>>& _bbx,
    const double _margin) {
  throw NotImplementedException(WHERE) << "Does not make sense for this object.";
}

/*------------------------------------ I/O -----------------------------------*/

void
TetrahedralBoundary::
Read(std::istream& _is, CountingStreamBuffer& _cbs) {
  throw NotImplementedException(WHERE);
}


void
TetrahedralBoundary::
Write(std::ostream& _os) const {
  _os << *this;
}

/*------------------------------- CGAL Representation ------------------------*/

Boundary::CGALPolyhedron
TetrahedralBoundary::
CGAL() const {
  // Define builder object.
  struct builder : public CGAL::Modifier_base<CGALPolyhedron::HalfedgeDS> {

    const std::array<Point3d, 4>& m_points;

    /// The first three points must form an outward-facing facet.
    builder(const std::array<Point3d, 4>& _points) : m_points(_points) {}

    void operator()(CGALPolyhedron::HalfedgeDS& _h) {
      using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
      CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

      b.begin_surface(4, 4, 12);

      for(const auto& point : m_points)
        b.add_vertex(Point(point[0], point[1], point[2]));

      // Face 1
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(2);
      b.end_facet();

      // Face 2
      b.begin_facet();
      b.add_vertex_to_facet(1);
      b.add_vertex_to_facet(3);
      b.add_vertex_to_facet(2);
      b.end_facet();

      // Face 3
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(3);
      b.add_vertex_to_facet(1);
      b.end_facet();

      // Face 4
      b.begin_facet();
      b.add_vertex_to_facet(0);
      b.add_vertex_to_facet(2);
      b.add_vertex_to_facet(3);
      b.end_facet();

      b.end_surface();
    }
  };

  CGALPolyhedron cp;
  builder b(m_points);
  cp.delegate(b);

  if(!cp.is_valid())
    throw RunTimeException(WHERE) << "TetrahedralBoundary:: Invalid CGAL "
                                  << "polyhedron created!";
  return cp;
}


GMSPolyhedron
TetrahedralBoundary::
MakePolyhedron() const {
  GMSPolyhedron poly;

  // Add vertices.
  auto& v = poly.GetVertexList();
  v = {m_points[0],
       m_points[1],
       m_points[2],
       m_points[3]};

  // Add facets.
  auto& f = poly.GetPolygonList();
  f.emplace_back(0, 1, 2, v);
  f.emplace_back(0, 2, 3, v);
  f.emplace_back(0, 3, 1, v);
  f.emplace_back(1, 3, 2, v);

  poly.Invert();
  return poly;
}

/*-------------------------------- Helpers -----------------------------------*/

void
TetrahedralBoundary::
OrderPoints() noexcept {
  // Get the points.
  const Vector3d& base = m_points[0];
  const Vector3d& a    = m_points[1];
  const Vector3d& b    = m_points[2];
  const Vector3d& c    = m_points[3];

  // Find a,b,c relative to base.
  const Vector3d edgeA = a - base;
  const Vector3d edgeB = b - base;
  const Vector3d edgeC = c - base;

  // Points are OK if the norm of (base, a, b) faces away from edgeC.
  // Otherwise, swap points a and b to fix the ordering.
  const Vector3d norm  = edgeA % edgeB;
  const bool normFacesAway = norm * edgeC < 0;
  if(!normFacesAway)
    std::swap(m_points[1], m_points[2]);
}


std::array<Vector3d, 6>
TetrahedralBoundary::
ComputeEdges() const {
  std::array<Vector3d, 6> edges = {m_points[1] - m_points[0],
                                   m_points[2] - m_points[0],
                                   m_points[3] - m_points[0],
                                   m_points[2] - m_points[1],
                                   m_points[3] - m_points[1],
                                   m_points[3] - m_points[2]};
  return edges;
}


std::array<Vector3d, 4>
TetrahedralBoundary::
ComputeNormals() const {
  auto edges = ComputeEdges();
  std::array<Vector3d, 4> normals = {(edges[0] % edges[1]).normalize(),
                                     (edges[1] % edges[2]).normalize(),
                                     (edges[2] % edges[0]).normalize(),
                                     (edges[4] % edges[3]).normalize()};
  return normals;
}


NBox
TetrahedralBoundary::
ComputeBBX() const {
  NBox bbx(3);

  for(size_t i = 0; i < bbx.GetDimension(); ++i) {
    Range<double> r(std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::lowest());

    for(size_t j = 0; j < 4; ++j)
      r.ExpandToInclude(m_points[j][i]);

    bbx.SetRange(i, r);
  }

  return bbx;
}


double
TetrahedralBoundary::
ComputeVolume() const {
  const mathtool::Vector3d v1 = m_points[2] - m_points[0],
                           v2 = m_points[1] - m_points[0],
                           v3 = m_points[3] - m_points[0],
                           n  = v1 % v2,
                           h  = v3.proj(n);
  const double baseArea = n.norm() / 2.,
               height   = h.norm();
  return baseArea * height / 3.;
}

/*---------------------------------- I/O -------------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const TetrahedralBoundary& _b) {
  _os << "[";
  bool first = true;
  for(const auto& point : _b.m_points) {
    _os << (first ? "" : " ; ") << point;
    first = false;
  }
  return _os << "]";
}

/*----------------------------------------------------------------------------*/
