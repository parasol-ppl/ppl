#ifndef PMPL_DRAWABLE_WORKSPACE_SKELETON_H_
#define PMPL_DRAWABLE_WORKSPACE_SKELETON_H_

#include "Workspace/WorkspaceSkeleton.h"

#include "glutils/drawable_display_list.h"


////////////////////////////////////////////////////////////////////////////////
/// Renderable representation of a workspace skeleton. This is a static
/// rendering that will not change with the skeleton.
////////////////////////////////////////////////////////////////////////////////
class DrawableWorkspaceSkeleton : public glutils::drawable_display_list  {

  public:

    ///@name Constructor
    ///@{

    /// Construct a skeleton rendering.
    /// @param _skeleton The workspace skeleton to render.
    /// @param _color The color to draw the cfgs and edges.
    DrawableWorkspaceSkeleton(WorkspaceSkeleton* const _skeleton,
        const glutils::color& _color);

    ///@}

  protected:

    ///@name drawable_display_list overrides
    ///@{

    virtual void build() override;

    /// Presently these objects cannot be selected or highlighted.
    virtual void build_select() override {}

    ///@}

  private:

    ///@name Internal State
    ///@{

    WorkspaceSkeleton* const m_skeleton;    /// The workspace skeleton to render.
    const glutils::color m_color;           /// The color to draw the cfgs and edges.

    ///@}

};

#endif
