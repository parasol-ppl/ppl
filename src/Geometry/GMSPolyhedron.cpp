#include "GMSPolyhedron.h"

#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/RapidCollisionDetection.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/PQPCollisionDetection.h"
#include "Utilities/IOUtils.h"
#include "Utilities/MPUtils.h"

#include "PQP.h"
#include "RAPID.H"

#include "MovieBYULoader.h"
#include "ModelFactory.h"
#include "ObjLoader.h"

#include "glutils/triangulated_model.h"

#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <algorithm>
#include <fstream>
#include <limits>

using namespace std;


/*------------------------------ Construction --------------------------------*/

GMSPolyhedron::
GMSPolyhedron() = default;


GMSPolyhedron::
GMSPolyhedron(const GMSPolyhedron& _p) :
    m_vertexList(_p.m_vertexList),
    m_cgalPoints(_p.m_cgalPoints),
    m_area(_p.m_area),
    m_maxRadius(_p.m_maxRadius),
    m_minRadius(_p.m_minRadius),
    m_insidePoint(_p.m_insidePoint)
{
  // Need to manually copy the polygons so that they refer to this polyhedron's
  // point list and not _p's.
  m_polygonList.reserve(_p.m_polygonList.size());
  for(const auto& p : _p.m_polygonList)
    m_polygonList.emplace_back(p[0], p[1], p[2], m_vertexList);
}


GMSPolyhedron::
GMSPolyhedron(GMSPolyhedron&& _p) :
    m_vertexList(std::move(_p.m_vertexList)),
    m_cgalPoints(std::move(_p.m_cgalPoints)),
    m_area(_p.m_area),
    m_maxRadius(_p.m_maxRadius),
    m_minRadius(_p.m_minRadius),
    m_rapidModel(std::move(_p.m_rapidModel)),
    m_pqpModel(std::move(_p.m_pqpModel)),
    m_insidePoint(std::move(_p.m_insidePoint))
{
  // Need to manually copy the polygons so that they refer to this polyhedron's
  // point list and not _p's.
  m_polygonList.reserve(_p.m_polygonList.size());
  for(const auto& p : _p.m_polygonList)
    m_polygonList.emplace_back(p[0], p[1], p[2], m_vertexList);
}


GMSPolyhedron::
GMSPolyhedron(glutils::triangulated_model&& _t) {
  // Copy vertices.
  m_vertexList.reserve(_t.num_points());
  for(auto iter = _t.points_begin(); iter != _t.points_end(); ++iter) {
    const glutils::vector3f& v = *iter;
    m_vertexList.emplace_back(v[0], v[1], v[2]);
  }

  // Copy facets.
  m_polygonList.reserve(_t.num_facets());
  for(auto iter = _t.facets_begin(); iter != _t.facets_end(); ++iter) {
    const glutils::triangle_facet& f = *iter;
    m_polygonList.emplace_back(f[0], f[1], f[2], m_vertexList);
  }

  OrderFacets();
  ComputeSurfaceArea();
  ComputeRadii();
  ComputeInsidePoint();
}


GMSPolyhedron::
~GMSPolyhedron() = default;

/*------------------------------- Assignment ---------------------------------*/

GMSPolyhedron&
GMSPolyhedron::
operator=(const GMSPolyhedron& _p) {
  if(this == &_p)
    return *this;

  m_vertexList = _p.m_vertexList;
  m_cgalPoints = _p.m_cgalPoints;
  m_area = _p.m_area;
  m_maxRadius = _p.m_maxRadius;
  m_minRadius = _p.m_minRadius;
  m_centroid = _p.m_centroid;
  m_centroidCached = _p.m_centroidCached;
  m_insidePoint = _p.m_insidePoint;

  // Need to manually copy the polygons so that they refer to this polyhedron's
  // point list and not _p's.
  m_polygonList.clear();
  m_polygonList.reserve(_p.m_polygonList.size());
  for(const auto& p : _p.m_polygonList)
    m_polygonList.emplace_back(p[0], p[1], p[2], m_vertexList);

  return *this;
}


GMSPolyhedron&
GMSPolyhedron::
operator=(GMSPolyhedron&& _p) {
  if(this == &_p)
    return *this;

  m_vertexList = std::move(_p.m_vertexList);
  m_cgalPoints = std::move(_p.m_cgalPoints);
  m_area = _p.m_area;
  m_maxRadius = _p.m_maxRadius;
  m_minRadius = _p.m_minRadius;
  m_centroid = std::move(_p.m_centroid);
  m_centroidCached = _p.m_centroidCached;
  m_rapidModel = std::move(_p.m_rapidModel);
  m_pqpModel = std::move(_p.m_pqpModel);
  m_insidePoint = std::move(_p.m_insidePoint);

  // Need to manually copy the polygons so that they refer to this polyhedron's
  // point list and not _p's.
  m_polygonList.clear();
  m_polygonList.reserve(_p.m_polygonList.size());
  for(const auto& p : _p.m_polygonList)
    m_polygonList.emplace_back(p[0], p[1], p[2], m_vertexList);

  return *this;
}

/*----------------------------- Transformation -------------------------------*/

GMSPolyhedron&
GMSPolyhedron::
operator*=(const Transformation& _t) {
  // Update the vertices.
  for(auto& point : m_vertexList)
    point = _t * point;
  m_insidePoint = _t * m_insidePoint;

  // Update the face normals.
  for(auto& facet : m_polygonList)
    facet.GetNormal() = _t.rotation() * facet.GetNormal();

  // Update the CGAL points (if any).
  if(!m_cgalPoints.empty()) {
    // Create a CGAL representation of the world transform.
    const auto& r = _t.rotation().matrix();
    const auto& t = _t.translation();
    CGAL::Aff_transformation_3<GMSPolyhedron::CGALKernel>
        cgalTransform(r[0][0], r[0][1], r[0][2], t[0],
                      r[1][0], r[1][1], r[1][2], t[1],
                      r[2][0], r[2][1], r[2][2], t[2]);

    for(auto& point : m_cgalPoints)
      point = cgalTransform(point);
  }

  // Trigger rebuild of CD models.
  m_rapidModel.release();
  m_pqpModel.release();
  m_centroidCached = false;

  return *this;
}


void
GMSPolyhedron::
Invert() {
  for(auto& facet : m_polygonList)
    facet.Reverse();

  // Trigger rebuild of CD models to get the normals facing the right way.
  m_rapidModel.release();
  m_pqpModel.release();
  ComputeInsidePoint();
}


void
GMSPolyhedron::
Scale(double _scalingFactor) {
  //to make sure that the centroid is not moved, center the model before scaling
  //and bring it back to its center afterwards

  double scaleVec[3][3] = {
    {_scalingFactor, 0.0 , 0.0},
    {0.0, _scalingFactor, 0.0},
    {0.0, 0.0, _scalingFactor}};

  Vector3d toCenter = -(GetCentroid());

  Matrix3x3 scaleM(scaleVec);

  double unitOrientation[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}};

  Matrix3x3 unitM(unitOrientation);

  const Transformation center(toCenter, Orientation(unitM)),
                       scale(Vector3d(0,0,0), Orientation(scaleM)),
                       recenter(-toCenter, Orientation(unitM)),
                       t = center * scale * recenter;

  *(this) *= t;
}


GMSPolyhedron
operator*(const Transformation& _t, const GMSPolyhedron& _poly) {
  GMSPolyhedron poly = _poly;
  return poly *= _t;
}

/*-------------------------------- Equality ----------------------------------*/

bool
GMSPolyhedron::
operator==(const GMSPolyhedron& _p) const {
  return m_area == _p.m_area
      && m_maxRadius == _p.m_maxRadius
      && m_minRadius == _p.m_minRadius
      && m_polygonList == _p.m_polygonList
      && m_vertexList == _p.m_vertexList;
}


bool
GMSPolyhedron::
operator!=(const GMSPolyhedron& _p) const {
  return !(*this == _p);
}

/*------------------------------ I/O Functions -------------------------------*/

Vector3d
GMSPolyhedron::
Read(const std::string& _fileName, COMAdjust _comAdjust) {
  if(!FileExists(_fileName))
    throw ParseException(WHERE, "Geometry file '" + _fileName + "' not found.");

  // Get file extension.
  string ext;
  size_t pos = _fileName.rfind(".");
  if(pos != string::npos)
    ext = _fileName.substr(pos + 1);

  if(ext != "g" && ext != "obj")
    throw ParseException(WHERE, _fileName + " has an unrecognized format '" +
        ext + "'. Recognized formats are BYU(.g) and OBJ(.obj).");

  unique_ptr<IModel> imodel(CreateModelLoader(_fileName, false));
  if(!imodel)
    throw ParseException(WHERE, "Cannot read model '" + _fileName + "'.");

  Vector3d com = LoadFromIModel(imodel.get(), _comAdjust);

  return com;
}


Vector3d
GMSPolyhedron::
LoadFromIModel(IModel* _imodel, COMAdjust _comAdjust) {
  // Add vertices to the polyhedron and compute center of mass.
  Vector3d com;
  for(const auto& v : _imodel->GetVertices()) {
    m_vertexList.push_back(v);
    com += v;
  }
  com /= m_vertexList.size();

  // Repeat for CGAL points.
  CGALPoint ccom;
  for(const auto& c : _imodel->GetCGALVertices()) {
    m_cgalPoints.push_back(c);
    ccom[0] += c[0];
    ccom[1] += c[1];
    ccom[2] += c[2];
  }
  ccom[0] /= double(m_cgalPoints.size());
  ccom[1] /= double(m_cgalPoints.size());
  ccom[2] /= double(m_cgalPoints.size());

  // Apply COMAdjust to vertices.
  /// @TODO Remove this and force everything to be bounding-box centered.
  for(size_t i = 0; i < m_vertexList.size(); ++i) {
    auto& v = m_vertexList[i];
    auto& c = m_cgalPoints[i];
    switch(_comAdjust) {
      case COMAdjust::COM:     // Move COM to xyz origin.
        v -= com;
        c[0] -= ccom[0];
        c[1] -= ccom[1];
        c[2] -= ccom[2];
        break;
      case COMAdjust::Surface: // Move COM to xz origin.
        v[0] -= com[0];
        v[2] -= com[2];
        c[0] -= ccom[0];
        c[2] -= ccom[2];
        break;
      case COMAdjust::None:    // Do nothing.
      default:
        break;
    }
  }

  // Add triangles to the polyhedron.
  for(auto& t : _imodel->GetTriP())
    m_polygonList.emplace_back(t[0], t[1], t[2], m_vertexList);

  // Compute the surface area and radii.
  OrderFacets();
  ComputeSurfaceArea();
  ComputeRadii();
  ComputeInsidePoint();

  return com;
}


void
GMSPolyhedron::
WriteBYU(ostream& _os) const {
  size_t numTri = m_polygonList.size();
  _os << "1 " << m_vertexList.size() << " " << numTri << " " << numTri * 3 << endl
      << "1 " << numTri << endl;
  for(const auto& v : m_vertexList)
    _os << v << endl;
  for(const auto& p : m_polygonList) {
    for(auto i = p.begin(); (i + 1) != p.end(); ++i)
      _os << *i + 1 << " ";
    _os << "-" << (*--p.end()) + 1 << endl;
  }
}

void
GMSPolyhedron::
WriteObj(ostream& _os) const {
 _os << "#Number of vertices: " << m_vertexList.size() << endl;
 _os << "#Number of triangles: " << m_polygonList.size() << endl;
 for(const auto& v : m_vertexList)
   _os << "v " << v << std::endl;
 for(const auto& p: m_polygonList)
   _os << "vn " << p.GetNormal() << std::endl;

 size_t count = 1;
 for(const auto& p: m_polygonList) {
   _os << "f";
   for(const auto index : p)
     _os << " " << index + 1 << "//" << count;
   _os << std::endl;

   ++count;
 }
}

/*-------------------------------- Accessors ---------------------------------*/

vector<Vector3d>&
GMSPolyhedron::
GetVertexList() noexcept {
  return m_vertexList;
}


vector<GMSPolygon>&
GMSPolyhedron::
GetPolygonList() noexcept {
  return m_polygonList;
}


const vector<Vector3d>&
GMSPolyhedron::
GetVertexList() const noexcept {
  return m_vertexList;
}


const vector<GMSPolygon>&
GMSPolyhedron::
GetPolygonList() const noexcept {
  return m_polygonList;
}


const Vector3d&
GMSPolyhedron::
GetCentroid() const {
  if(!m_centroidCached)
    ComputeCentroid();
  return m_centroid;
}


double
GMSPolyhedron::
GetSurfaceArea() const noexcept {
  return m_area;
}


double
GMSPolyhedron::
GetMaxRadius() const noexcept {
  return m_maxRadius;
}


double
GMSPolyhedron::
GetMinRadius() const noexcept {
  return m_minRadius;
}

/*--------------------------- Geometry Functions -----------------------------*/

Point3d
GMSPolyhedron::
GetRandPtOnSurface() const {
  // Chose a polygon on the surface. Half of the time, choose by fraction of
  // total surface area. Otherwise, choose randomly.
  size_t index;
  if(DRand() < 0.5) {
    // Choose by fractional area.
    do {
      index = 0;
      double prob = DRand(), cummProb = 0;
      for(const auto& tri : m_polygonList) {
	cummProb += tri.GetArea() / m_area;
	if(prob <= cummProb)
	  break;
        ++index;
      }
    } while(!m_polygonList[index].IsTriangle());
  }
  else {
    // Choose randomly.
    do {
      index = LRand() % m_polygonList.size();
    } while(!m_polygonList[index].IsTriangle());
  }

  // Now that a polygon index has been selected, get a random point on it using
  // barycentric coordinates.
  double u = DRand(), v = DRand();
  if(u + v >= 1) {
    u = 1 - u;
    v = 1 - v;
  }
  const GMSPolygon& tri = m_polygonList[index];
  const Vector3d& p0 = tri.GetPoint(0);
  const Vector3d& p1 = tri.GetPoint(1);
  const Vector3d& p2 = tri.GetPoint(2);
  const Vector3d AB = p1 - p0;
  const Vector3d AC = p2 - p0;
  return p0 + (u * AB) + (v * AC);
}


void
GMSPolyhedron::
ComputeSurfaceArea() {
  m_area = 0;
  for(const auto& p : m_polygonList)
    m_area += p.GetArea();
}


void
GMSPolyhedron::
ComputeRadii() {
  m_maxRadius = 0;
  m_minRadius = numeric_limits<double>::infinity();

  for(const auto& v : m_vertexList) {
    const double distance = v.norm();
    m_maxRadius = std::max(m_maxRadius, distance);
    m_minRadius = std::min(m_minRadius, distance);
  }
}


void
GMSPolyhedron::
ComputeInsidePoint() {
  // Compute a point just beneath the first facet.
  const auto& facet = GetPolygonList()[0];
  m_insidePoint = facet.FindCenter() - 1e-6 * facet.GetNormal();
}


std::unique_ptr<WorkspaceBoundingBox>
GMSPolyhedron::
ComputeBoundingBox() const {
  // Initialize point-ranges in each dimension on the first vertex.
  const auto& vo = m_vertexList[0];
  Range<double> x(vo[0], vo[0]),
                y(vo[1], vo[1]),
                z(vo[2], vo[2]);

  // Find Extreme values.
  for(const auto& v : m_vertexList) {
    x.ExpandToInclude(v[0]);
    y.ExpandToInclude(v[1]);
    z.ExpandToInclude(v[2]);
  }

  // Make workspace bounding box.
  std::unique_ptr<WorkspaceBoundingBox> bbx(new WorkspaceBoundingBox(3));
  bbx->SetRange(0, x);
  bbx->SetRange(1, y);
  bbx->SetRange(2, z);

#if 0
  // Assert that all poly points are inside.
  for(const auto& v : m_vertexList)
    if(!bbx->InBoundary(v))
      throw RunTimeException(WHERE) << "Computed bbx " << *bbx << " does not "
                                    << "contain polyhedron point " << v << ".";
#endif

  return bbx;
}


GMSPolyhedron::CGALPolyhedron
GMSPolyhedron::
CGAL() const {
  // Define builder object.
  struct builder : public CGAL::Modifier_base<CGALPolyhedron::HalfedgeDS> {

    const GMSPolyhedron& m_poly;

    builder(const GMSPolyhedron& _p) : m_poly(_p) {}

    void operator()(CGALPolyhedron::HalfedgeDS& _h) {
      using Point = CGALPolyhedron::HalfedgeDS::Vertex::Point;
      CGAL::Polyhedron_incremental_builder_3<CGALPolyhedron::HalfedgeDS> b(_h);

      size_t num_vertices = m_poly.m_vertexList.size();
      size_t num_facets   = m_poly.m_polygonList.size();
      size_t num_edges    = num_facets * 3;

      b.begin_surface(num_vertices, num_facets, num_edges);

      // Add vertices.
      for(const auto& p : m_poly.m_cgalPoints)
        b.add_vertex(p);

      // Add facets.
      for(const auto& facet : m_poly.m_polygonList) {
        b.begin_facet();
        for(const auto& index : facet)
          b.add_vertex_to_facet(index);
        b.end_facet();
      }

      b.end_surface();
    }
  };

  // Delegate builder object.
  CGALPolyhedron cp;
  builder b(*this);
  cp.delegate(b);
  if(!cp.is_valid())
    throw RunTimeException(WHERE) << "Invalid CGAL polyhedron created!";
  return cp;
}


GMSPolyhedron
GMSPolyhedron::
ComputeConvexHull() const {
  // Compute convex hull of non-collinear points with CGAL.
  CGALPolyhedron poly;
  CGAL::convex_hull_3(m_cgalPoints.begin(), m_cgalPoints.end(), poly);

  GMSPolyhedron convexHull;

  // Add the convex hull points computed by CGAL to our output object.
  for(auto vit = poly.points_begin(); vit != poly.points_end(); ++vit) {
    convexHull.m_vertexList.emplace_back(
        to_double((*vit)[0]), to_double((*vit)[1]), to_double((*vit)[2]));
    convexHull.m_cgalPoints.emplace_back(*vit);
  }

  // Add the convex hull facets computed by CGAL to our output object.
  for(CGALPolyhedron::Facet_iterator fit = poly.facets_begin();
      fit != poly.facets_end(); ++fit) {
    // Each facet is described by a set of half-edges. Get the vertex/point
    // indexes in each facet.
    CGALPolyhedron::Halfedge_around_facet_circulator he = fit->facet_begin();
    vector<int> indexes;
    do {
      indexes.push_back(std::distance(poly.vertices_begin(), he->vertex()));
    } while(++he != fit->facet_begin());

    // Ensure we got a valid polygon.
    if(indexes.size() < 3)
      throw RunTimeException(WHERE) << "Polygons can't have less than 3 vertices."
                                    << std::endl;

    // Add this polygon to the convex hull.
    convexHull.m_polygonList.emplace_back(GMSPolygon(indexes[0], indexes[1],
          indexes[2], convexHull.m_vertexList));
  }

  convexHull.OrderFacets();
  convexHull.ComputeSurfaceArea();
  convexHull.ComputeRadii();
  convexHull.ComputeInsidePoint();

  return convexHull;
}

/*-------------------------- Initialization Helpers --------------------------*/

void
GMSPolyhedron::
ComputeCentroid() const {
  auto& centroid = const_cast<Vector3d&>(m_centroid);
  centroid(0, 0, 0);
  for(const auto& v : m_vertexList)
    centroid += v;
  centroid /= m_vertexList.size();
  m_centroidCached = true;
}


void
GMSPolyhedron::
OrderFacets() {
  std::sort(m_polygonList.begin(), m_polygonList.end(),
      std::greater<GMSPolygon>());
}


void
GMSPolyhedron::
UpdateCGALPoints() {
  // If the sizes are the same, it's probably safe to assume the CGAL points are
  // already correct (usually this function is called for polys where the
  // vertexList has been set explicitly and the CGAL points haven't been set at
  // all).
  if(m_vertexList.size() == m_cgalPoints.size())
    return;

  m_cgalPoints.clear();
  for(const auto& v : m_vertexList)
    m_cgalPoints.emplace_back(v[0], v[1], v[2]);
}

/*------------------------ Collision Detection Helpers -----------------------*/

RAPID_model*
GMSPolyhedron::
GetRapidModel() const noexcept {
  if(!m_rapidModel)
    m_rapidModel.reset(Rapid::Build(*this));
  return m_rapidModel.get();
}


PQP_Model*
GMSPolyhedron::
GetPQPModel() const noexcept {
  if(!m_pqpModel)
    m_pqpModel.reset(PQP::Build(*this));
  return m_pqpModel.get();
}


const Vector3d&
GMSPolyhedron::
GetInsidePoint() const noexcept {
  return m_insidePoint;
}

/*------------------------------- Common Shapes ------------------------------*/

GMSPolyhedron
GMSPolyhedron::
MakeBox(const Range<double>& _x, const Range<double>& _y,
    const Range<double>& _z) {
  // Make output polyhedron.
  GMSPolyhedron bbx;
  auto& verts = bbx.m_vertexList;
  auto& polys = bbx.m_polygonList;

  // Add vertices.
  verts.reserve(8);
  verts.emplace_back(_x.min, _y.min, _z.min);
  verts.emplace_back(_x.min, _y.min, _z.max);
  verts.emplace_back(_x.min, _y.max, _z.min);
  verts.emplace_back(_x.min, _y.max, _z.max);
  verts.emplace_back(_x.max, _y.min, _z.min);
  verts.emplace_back(_x.max, _y.min, _z.max);
  verts.emplace_back(_x.max, _y.max, _z.min);
  verts.emplace_back(_x.max, _y.max, _z.max);

  // Add polygons.
  polys.reserve(12);
  polys.emplace_back(0, 1, 3, verts);
  polys.emplace_back(0, 3, 2, verts);
  polys.emplace_back(4, 0, 2, verts);
  polys.emplace_back(4, 2, 6, verts);
  polys.emplace_back(5, 4, 6, verts);
  polys.emplace_back(5, 6, 7, verts);
  polys.emplace_back(1, 5, 7, verts);
  polys.emplace_back(1, 7, 3, verts);
  polys.emplace_back(3, 7, 6, verts);
  polys.emplace_back(3, 6, 2, verts);
  polys.emplace_back(0, 4, 5, verts);
  polys.emplace_back(0, 5, 1, verts);

  bbx.OrderFacets();
  bbx.ComputeSurfaceArea();
  bbx.ComputeRadii();
  bbx.ComputeInsidePoint();

  return bbx;
}

/*----------------------------------------------------------------------------*/
