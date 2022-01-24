#include "PQPCollisionDetection.h"

#include "CDInfo.h"
#include "Geometry/GMSPolyhedron.h"
#include "Utilities/PMPLExceptions.h"

#include "PQP.h"

#include <limits>
#include <set>
#include <utility>

using namespace mathtool;


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PQP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

PQP::
PQP() : CollisionDetectionMethod("PQP") { }


PQP::
~PQP() = default;

/*---------------------------- Model Construction ----------------------------*/

PQP_Model*
PQP::
Build(const GMSPolyhedron& _polyhedron) {
  const auto& facets = _polyhedron.GetPolygonList();
  PQP_Model* pqpModel(new PQP_Model);
  pqpModel->BeginModel();
  for(size_t q = 0; q < facets.size(); q++) {
    double point[3][3];
    for(int i = 0; i < 3; i++) {
      const Vector3d& tmp = facets[q].GetPoint(i);
      for(int j = 0; j < 3; j++)
        point[i][j] = tmp[j];
    }
    pqpModel->AddTri(point[0], point[1], point[2], q);
  }
  pqpModel->EndModel();

  return pqpModel;
}

/*------------------------------- CD Interface -------------------------------*/

bool
PQP::
IsInCollision(
    const GMSPolyhedron& _polyhedron1,
    const mathtool::Transformation& _transformation1,
    const GMSPolyhedron& _polyhedron2,
    const mathtool::Transformation& _transformation2,
    CDInfo& _cdInfo) {
  auto model1 = _polyhedron1.GetPQPModel();
  auto model2 = _polyhedron2.GetPQPModel();
  /// @TODO See if we can modify PQP_Distance to take const double arrays
  ///       instead of just double arrays so we don't have to copy.
  Transformation t1 = _transformation1,
                 t2 = _transformation2;

  if(_cdInfo.m_retAllInfo) {
    PQP_DistanceResult result;
    if(PQP_Distance(&result,
          t1.rotation().matrix(), t1.translation(), model1,
          t2.rotation().matrix(), t2.translation(), model2, 0.0, 0.0))
      throw RunTimeException(WHERE) << "PQP out of memory.";

    _cdInfo.m_minDist = result.Distance();
    /// @TODO Update to use polyhedrons instead of bodies?
    //_cdInfo.m_clearanceMap.SetClearance(_polyhedron1, _polyhedron2,
    //    result.Distance());

    _cdInfo.m_robotPoint  = t1 * Vector3d(result.P1());
    _cdInfo.m_objectPoint = t2 * Vector3d(result.P2());

    const bool inCollision = result.Distance() <= 0.0;

    if(inCollision) {
      // Now do a collision check to get all colliding triangle pairs.
      PQP_CollideResult result;
      if(PQP_Collide(&result,
            t1.rotation().matrix(), t1.translation(), model1,
            t2.rotation().matrix(), t2.translation(), model2,
            PQP_ALL_CONTACTS))
        throw RunTimeException(WHERE) << "PQP out of memory.";

      for(int i = 0; i < result.NumPairs(); ++i)
        _cdInfo.m_trianglePairs.emplace_back(result.Id1(i), result.Id2(i));
    }

    return inCollision;
  }
  else {

    PQP_CollideResult result;
    if(PQP_Collide(&result,
          t1.rotation().matrix(), t1.translation(), model1,
          t2.rotation().matrix(), t2.translation(), model2,
          PQP_FIRST_CONTACT))
      throw RunTimeException(WHERE) << "PQP out of memory.";

    if(result.Colliding())
      _cdInfo.m_trianglePairs.emplace_back(result.Id1(0), result.Id2(0));

    return result.Colliding();
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PQPSolid ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

PQPSolid::
PQPSolid() : PQP() {
  m_name = "PQP_SOLID";
}


PQPSolid::
~PQPSolid() = default;

/*------------------------------- CD Interface -------------------------------*/

bool
PQPSolid::
IsInCollision(
    const GMSPolyhedron& _polyhedron1,
    const mathtool::Transformation& _transformation1,
    const GMSPolyhedron& _polyhedron2,
    const mathtool::Transformation& _transformation2,
    CDInfo& _cdInfo) {
  bool collision = PQP::IsInCollision(_polyhedron1, _transformation1,
      _polyhedron2, _transformation2, _cdInfo);
  if(!collision)
  {
    const auto point1 = _transformation1 * _polyhedron1.GetInsidePoint();
    const auto point2 = _transformation2 * _polyhedron2.GetInsidePoint();

    collision = IsInsideObstacle(point1, _polyhedron2, _transformation2)
             or IsInsideObstacle(point2, _polyhedron1, _transformation1);
  }
  return collision;
}


bool
PQPSolid::
IsInsideObstacle(const Vector3d& _point, const GMSPolyhedron& _polyhedron,
    const mathtool::Transformation& _transformation) {
  // Set up a pseudo-ray for a ray-shooting test.
  static PQP_Model* ray = BuildPseudoRay();
  static PQP_REAL rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  PQP_REAL translation[3] = {_point[0], _point[1], _point[2]};

  /// @TODO See if we can modify PQP_Collide to take const double arrays
  ///       instead of just double arrays so we don't have to copy.
  Transformation t = _transformation;

  // Perform ray-shooting collision test.
  PQP_CollideResult result;
  PQP_Collide(&result, rotation, translation, ray,
      t.rotation().matrix(), t.translation(), _polyhedron.GetPQPModel());

  // Sort collisions by relative X-value.
  static const double tolerance = 10 * std::numeric_limits<double>::epsilon();
  static const Vector3d r(10e6, 0, 0); // Vector-representation of the ray.
  const auto& vertices = _polyhedron.GetVertexList();
  const auto& polygons = _polyhedron.GetPolygonList();

  enum TransitionType {Entering = 0, Exiting = 1};
  typedef std::pair<double, TransitionType> Transition;

  // We will store the processed collisions in a set to sort them and remove
  // duplicate transitions of the same type and x-value. Duplicate removal guards
  // against the case where the psuedo-ray passes through an edge or vertex that
  // is shared by multiple triangles with the same facing.
  auto compare = [](const Transition& _t1, const Transition& _t2) -> bool {
    if(abs(_t1.first - _t2.first) > tolerance)
      return _t1.first < _t2.first;
    else
      return _t1.second < _t2.second;
  };
  std::set<Transition, decltype(compare)> collisions(compare);

  // Process each collision.
  collisions.clear();
  for(int i = 0; i < result.NumPairs(); ++i) {
    const auto& triangle = polygons[result.Id2(i)];
    const auto& v = _transformation * vertices[triangle[0]];
    const auto& n = _transformation.rotation() * triangle.GetNormal();

    // Skip collisions against triangles whose normals are perpendicular to the
    // ray: these are scrapes and don't affect inside/outside-ness.
    if(abs(n[0]) < tolerance)
      continue;

    // The collision occurs at some fraction of r. This fraction is the ratio of
    // |pt to the triangle plane| over |r's projection along n|.
    const double alpha = ((v - _point) * n) / (r * n);

    // We are exiting the triangle if the normal has positive x value and
    // entering it otherwise (zero x values handled above).
    collisions.emplace(alpha * r[0], TransitionType(n[0] > 0));
  }

  // Check the ordered collisions to see what happened. Skip over any points
  // where we enter and exit at the same point.
  /// @todo This can still fail. I.e., what if we collide with two entering and
  ///       one leaving triangle, as if we shot straight past the tip of a
  ///       tetrahedron with the seem exactly aligned with the starting point?
  ///       We need to check *all* colocated points, not just a pair. On
  ///       detection of a scrape (colocated and different type) we need to
  ///       erase all of the colocated collisions.
  while(!collisions.empty()) {
    if(collisions.size() == 1)
      return collisions.begin()->second;

    auto one = collisions.begin(),
         two = ++collisions.begin();

    const bool colocated = abs(one->first - two->first) < tolerance,
               sameType  = one->second == two->second;

    if(colocated and !sameType)
      collisions.erase(one, ++two);
    else
      return one->second;
  }

  // If we're still here, there are no valid collisions.
  return false;
}

/*--------------------------------- Helpers ----------------------------------*/

PQP_Model*
PQPSolid::
BuildPseudoRay() const {
  PQP_Model* ray{new PQP_Model()};

  PQP_REAL zero[3] = {0, 0, 0};
  PQP_REAL x[3] = {1e10, 0, 0};

  ray->BeginModel();
  ray->AddTri(zero, zero, x, 0);
  ray->EndModel();

  return ray;
}

/*----------------------------------------------------------------------------*/
