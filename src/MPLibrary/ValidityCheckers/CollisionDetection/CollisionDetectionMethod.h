#ifndef PMPL_COLLISION_DETECTION_METHOD_H_
#define PMPL_COLLISION_DETECTION_METHOD_H_

#include "Transformation.h"
#include "Vector.h"

#include <iostream>
#include <string>

class CDInfo;
class GMSPolyhedron;


////////////////////////////////////////////////////////////////////////////////
/// Base abstraction for geometric collision detection methods.
///
/// @ingroup CollisionDetection
////////////////////////////////////////////////////////////////////////////////
class CollisionDetectionMethod {

  public:

    ///@name Construction
    ///@{

    /// @param _name The method name.
    CollisionDetectionMethod(const std::string& _name = "CD_USER1");

    virtual ~CollisionDetectionMethod();

    ///@}
    ///@name Accessors
    ///@{

    /// @return Name of CD Method
    const std::string& GetName() const;

    /// Print information to an output stream.
    /// @param _os The output stream.
    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name CD Interface
    ///@{

    /// Check if two polyhedrons are in collision.
    /// @param _polyhedron1     The first polyhedron.
    /// @param _transformation1 Transformation for the first polyhedron.
    /// @param _polyhedron2     The second polyhedron.
    /// @param _transformation2 Transformation for the second polyhedron.
    /// @param _cdInfo      Output information from the collision computation.
    /// @return True if the polyhedrons are considered to be in collision. Some
    ///         method check only intersection (i.e. Rapid, PQP) while others
    ///         also check if one polyhedron is inside the other (i.e. PQPSolid).
    virtual bool IsInCollision(
        const GMSPolyhedron& _polyhedron1,
        const mathtool::Transformation& _transformation1,
        const GMSPolyhedron& _polyhedron2,
        const mathtool::Transformation& _transformation2,
        CDInfo& _cdInfo);
    ///@example CollisionDetection_UseCase.cpp
    /// This is an example of how to use the collision detection methods.

    /// Check if a point is inside of a polyhedron.
    /// @param _point          The point to check.
    /// @param _polyhedron     The polyhedron to check against.
    /// @param _transformation Transformation for the polyhedron.
    /// @return Is the point inside _polyhedron?
    virtual bool IsInsideObstacle(const mathtool::Vector3d& _point,
        const GMSPolyhedron& _polyhedron,
        const mathtool::Transformation& _transformation);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::string m_name; ///< Name of the CD method.

    ///@}

};

#endif
