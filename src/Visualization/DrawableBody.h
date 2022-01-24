#ifndef DRAWABLE_BODY_H_
#define DRAWABLE_BODY_H_

#include "DrawablePolyhedron.h"

class Body;
class DrawableMultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Drawable representation of a pmpl Body.
////////////////////////////////////////////////////////////////////////////////
class DrawableBody final : public  DrawablePolyhedron {

  ///@name Internal State
  ///@{

  DrawableMultiBody* const m_parent; ///< The owning Drawable object.
  Body* const m_body;                ///< The Body to draw.

  ///@}

  public:

    ///@name Construction
    ///@{
    /// @param _parent the owning Drawable object
    /// @param _body the body to draw
    /// @param _wired Draw the boundary in wire frame?
    DrawableBody(DrawableMultiBody* const _parent, Body* const _body, bool _wired = false);

    virtual ~DrawableBody() = default;

    ///@}
    ///@name Body Support
    ///@{

    Body* GetBody() const noexcept;

    ///@}
    ///@name drawable Overrides
    ///@{
    /// When this object is selected/unselected, propogate the event to the
    /// parent object.

    virtual void select() noexcept override;
    virtual void deselect() noexcept override;

    virtual void highlight() noexcept override;
    virtual void unhighlight() noexcept override;

    ///@}
};

#endif
