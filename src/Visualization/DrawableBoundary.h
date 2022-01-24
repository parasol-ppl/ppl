#ifndef PMPL_DRAWABLE_BOUNDARY_H_
#define PMPL_DRAWABLE_BOUNDARY_H_

#include "DrawablePolyhedron.h"

#include "Geometry/GMSPolyhedron.h"

class Boundary;


////////////////////////////////////////////////////////////////////////////////
/// Drawable representation of a pmpl boundary. Does not get updated if the
/// boundary is adjusted.
////////////////////////////////////////////////////////////////////////////////
class DrawableBoundary : public DrawablePolyhedron {

  public:

    ///@name Constructor
    ///@{

    /// Build a new drawable boundary.
    /// @param _boundary Boundary to be drawn
    /// @param _color The color to render the boundary.
    /// @param _wire Draw the boundary in wire frame?
    DrawableBoundary(const Boundary* const _boundary,
        const glutils::color& _color, const bool _wire = false);

    ///@}

  protected:

    ///@name DrawalbePolyhedron Overrides
    ///@{

    void build() override;

    ///@}
    ///@name Internal State
    ///@{

    GMSPolyhedron m_polyhedron; ///< The boundary polyhedron.

    ///@}

};

#endif
