#ifndef PMPL_DRAWABLE_POLYHEDRON_H_
#define PMPL_DRAWABLE_POLYHEDRON_H_

#include "glutils/drawable_display_list.h"

#include <set>

class GMSPolyhedron;


////////////////////////////////////////////////////////////////////////////////
/// A visualization of a polyhedron.
////////////////////////////////////////////////////////////////////////////////
class DrawablePolyhedron : public glutils::drawable_display_list {

  public:

    ///@name Constructor
    ///@{

    /// Construct a polyhedron visualization.
    /// @param _polyhedron The polyhedron to visualize.
    /// @param _color The color to render in.
    /// @param _wire Render a wireframe instead of the full model?
    DrawablePolyhedron(const GMSPolyhedron& _polyhedron,
        const glutils::color& _color, const bool _wire = false);

    ///@}
    ///@name Rendering Options
    ///@{

    /// Toggle between wire and full-frame rendering.
    void ToggleRenderMode();

    ///@}

  protected:

    ///@name drawable_display_list overrides
    ///@{

    virtual void build() override;
    virtual void build_select() override;
    virtual void build_selected() override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// The build commands for the polyhedron model.
    void build_polyhedron();

    /// The build commands for the wireframe model.
    void build_wireframe();

    /// Builds a map showing the lines of adjacent polygons and whether they
    /// need to be drawn.
    /// @return The adjacency map of the current polyhedron.
    std::set<std::pair<size_t, size_t>> BuildAdjacencyMap();

    ///@}
    ///@name Internal State
    ///@{

    const GMSPolyhedron& m_polyhedron; ///< The polyhedron to visualize.
    glutils::color m_color;            ///< The rendering color.
    bool m_wire{false};                ///< Render a wireframe only?

    ///@}

};

#endif
