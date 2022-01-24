#ifndef PMPL_SPHERES_COLLISION_DETECTION_H_
#define PMPL_SPHERES_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Collision detection using bounding spheres. This implies that collisions
/// will be unsure, but no collision is certain.
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class BoundingSpheres : public CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    BoundingSpheres();

    ///@}
    ///@name CollisionDetectionMethod Overrides
    ///@{

    virtual bool IsInCollision(
        const GMSPolyhedron& _polyhedron1, const mathtool::Transformation& _t1,
        const GMSPolyhedron& _polyhedron2, const mathtool::Transformation& _t2,
        CDInfo& _cdInfo) override;

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// Collision detection using inscribed spheres only. This implies that
/// collisions will be sure, but no collision is uncertain.
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class InsideSpheres : public CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    InsideSpheres();

    ///@}
    ///@name CollisionDetectionMethod Overrides
    ///@{

    virtual bool IsInCollision(
        const GMSPolyhedron& _polyhedron1, const mathtool::Transformation& _t1,
        const GMSPolyhedron& _polyhedron2, const mathtool::Transformation& _t2,
        CDInfo& _cdInfo) override;

    ///@}

};

#endif
