#ifndef DRAWABLE_MULTIBODY_H_
#define DRAWABLE_MULTIBODY_H_

#include <cstddef>
#include <string>
#include <vector>

#include "glutils/drawable.h"

#include "DrawableBody.h"

class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Instructions for drawing a pmpl MultiBody in the Simulator.
////////////////////////////////////////////////////////////////////////////////
class DrawableMultiBody final : public glutils::drawable {

  ///@name Internal State
  ///@{

  MultiBody* const m_multibody;       ///< The corresponding PMPL multibody.
  std::vector<DrawableBody> m_bodies; ///< Drawables for each sub-body.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a drawable representation of a PMPL multibody.
    /// @param[in] _m The PMPL multibody to draw.
    DrawableMultiBody(MultiBody* const _m);

    virtual void initialize() override;

    /// Since this is a container, uninit requests must be passed on to the
    /// child bodies.
    virtual void uninitialize() override;

    virtual ~DrawableMultiBody() = default;

    ///@}
    ///@name MultiBody Support
    ///@{

    /// Get the PMPL multibody.
    MultiBody* GetMultiBody() const noexcept;

    /// Get the number of bodies in this drawable.
    size_t GetNumBodies() const noexcept;

    /// Get a child drawable body.
    /// @param[in] _i The index of the drawable body to get.
    /// @return A pointer to the drawable.
    DrawableBody* GetDrawableBody(const size_t _i) noexcept;

    /// Push a transform for a given body.
    /// @param[in] _i The index of the body to update.
    /// @param[in] _t The transform to push.
    void PushTransform(const size_t _i, const glutils::transform& _t);

    /// Update the transform queue for all bodies.
    void UpdateTransform();

    ///@}
    ///@name Drawable Overrides
    ///@{

    virtual void draw() override;
    virtual void draw_select() override;
    virtual void draw_selected() override;
    virtual void draw_highlighted() override;

    virtual void select() noexcept override;
    virtual void deselect() noexcept override;

    virtual void highlight() noexcept override;
    virtual void unhighlight() noexcept override;

    ///@}

};

#endif
