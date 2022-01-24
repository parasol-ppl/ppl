#include "SpheresCollisionDetection.h"

#include "Geometry/Bodies/Body.h"


/*----------------------------- BoundingSpheres ------------------------------*/

BoundingSpheres::
BoundingSpheres() : CollisionDetectionMethod("BoundingSpheres") { }


bool
BoundingSpheres::
IsInCollision(
    const GMSPolyhedron& _polyhedron1,
    const mathtool::Transformation& _transformation1,
    const GMSPolyhedron& _polyhedron2,
    const mathtool::Transformation& _transformation2,
    CDInfo& _cdInfo) {
  const Vector3d& center1 = _transformation1 * _polyhedron1.GetCentroid();
  const Vector3d& center2 = _transformation2 * _polyhedron2.GetCentroid();

  const double distance = (center1 - center2).norm();

  const double radius1 = _polyhedron1.GetMaxRadius();
  const double radius2 = _polyhedron2.GetMaxRadius();

  return distance <= radius1 + radius2;
}

/*------------------------------- InsideSpheres ------------------------------*/

InsideSpheres::
InsideSpheres() : CollisionDetectionMethod("InsideSpheres") { }


bool
InsideSpheres::
IsInCollision(
    const GMSPolyhedron& _polyhedron1,
    const mathtool::Transformation& _transformation1,
    const GMSPolyhedron& _polyhedron2,
    const mathtool::Transformation& _transformation2,
    CDInfo& _cdInfo) {
  const Vector3d& center1 = _transformation1 * _polyhedron1.GetCentroid();
  const Vector3d& center2 = _transformation2 * _polyhedron2.GetCentroid();

  const double distance = (center1 - center2).norm();

  const double radius1 = _polyhedron1.GetMinRadius();
  const double radius2 = _polyhedron2.GetMinRadius();

  return distance <= radius1 + radius2;
}

/*----------------------------------------------------------------------------*/
