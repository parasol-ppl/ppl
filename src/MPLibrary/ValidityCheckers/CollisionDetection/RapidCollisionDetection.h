#ifndef PMPL_RAPID_COLLISION_DETECTION_H_
#define PMPL_RAPID_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"

#include <mutex>

class GMSPolyhedron;
class RAPID_model;


////////////////////////////////////////////////////////////////////////////////
/// RAPID checks for polygon collisions between two meshes.
///
/// RAPID is fast, but it cannot determine any auxiliary information. The
/// contacts (colliding triangle IDs) are stored in the CDInfo object.
///
/// @todo There is no reason we can't support IsInsideObstacle with RAPID. This
///       isn't really a part of PQP either - it's auxilliary code we created
///       with a ray-shooting test. We can do the same thing here with an added
///       'RapidSolid' class, or by toggling the IsInsideObstacle check with a
///       flag both here and in PQP.
///
/// Reference:
///   Stefan Gottschalk and Ming C. Lin and Dinesh Manocha. "OBBTree: A
///   Hierarchical Structure for Rapid Interference Detection". SIGGRAPH 1996.
///
/// @warning RAPID is not a thread-safe library; it uses static data as part of
///          its internal implementation. Each call to any RAPID instance will
///          be serialized with a mutex (so it is a bad choice for
///          multi-threaded use like simulations with multiple planning agents).
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class Rapid : public CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    Rapid();

    virtual ~Rapid();

    ///@}
    ///@name Model Construction
    ///@{

    /// Build a RAPID model for a GMSPolyhedron.
    /// @param _polyhedron The polyhedron to model.
    /// @return A RAPID model of _model.
    static RAPID_model* Build(const GMSPolyhedron& _polyhedron);

    ///@}
    ///@name CollisionDetectionMethod Overrides
    ///@{

    virtual bool IsInCollision(
        const GMSPolyhedron& _polyhedron1, const mathtool::Transformation& _t1,
        const GMSPolyhedron& _polyhedron2, const mathtool::Transformation& _t2,
        CDInfo& _cdInfo) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    static std::mutex s_lock; ///< Serializing lock.

    ///@}

};

#endif
