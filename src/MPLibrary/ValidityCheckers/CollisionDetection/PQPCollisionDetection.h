#ifndef PMPL_PQP_COLLISION_DETECTION_H_
#define PMPL_PQP_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"

class GMSPolyhedron;
class PQP_Model;


////////////////////////////////////////////////////////////////////////////////
/// Compute collision information with Proximity Query Package (PQP) package.
///
/// PQP has the option to compute clearance and penetration information through
/// distance queries. To enable this pass in @c CDInfo with @c m_retAllInfo set
/// to true.
///
/// Reference:
///   Eric Larsen and Stefan Gottschalk and Ming C. Lin and Dinesh Manocha.
///   "Fast Proximity Queries with Swept Sphere Volumes". ICRA 2000.
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class PQP : public CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    PQP();

    virtual ~PQP();

    ///@}
    ///@name Model Construction
    ///@{

    /// Build a PQP model for a GMSPolyhedron.
    /// @param _polyhedron The polyhedron to model.
    /// @return A PQP model of _model.
    static PQP_Model* Build(const GMSPolyhedron& _polyhedron);

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
/// PQPSolid is an extended PQP which also checks if one mesh lies entirely
/// within another.
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class PQPSolid : public PQP {

  public:

    ///@name Construction
    ///@{

    PQPSolid();

    virtual ~PQPSolid();

    ///@}
    ///@name CollisionDetectorMethod Overrides
    ///@{

    virtual bool IsInCollision(
        const GMSPolyhedron& _polyhedron1, const mathtool::Transformation& _t1,
        const GMSPolyhedron& _polyhedron2, const mathtool::Transformation& _t2,
        CDInfo& _cdInfo) override;

    /// Shoot a pseudo-ray outward from a reference point to determine if it
    /// lies within a given body.
    /// @param _pt The reference point of interest.
    /// @param _body The body to check against.
    /// @return True if _pt is inside _body.
    virtual bool IsInsideObstacle(const mathtool::Vector3d& _point,
        const GMSPolyhedron& _polyhedron,
        const mathtool::Transformation& _transformation) override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Build a zero-area triangle with base at the origin and extending to
    /// (+inf, 0, 0) for the ray-shooting test in IsInsideObstacle.
    /// @return A PQP model of a triangular pseudo-ray.
    PQP_Model* BuildPseudoRay() const;

    ///@}

};

#endif
