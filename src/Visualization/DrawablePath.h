#ifndef DRAWABLE_PATH_H_
#define DRAWABLE_PATH_H_

#include <cstddef>
#include <string>
#include <vector>

#include "glutils/color.h"
#include "glutils/drawable_display_list.h"

class Cfg;
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Instructions for drawing a pmpl Path in the Simulator.
////////////////////////////////////////////////////////////////////////////////
class DrawablePath final : public glutils::drawable_display_list {

  ///@name Internal State
  ///@{

  std::vector<Cfg> m_cfgs;  ///< The path Cfgs.
  glutils::color m_color;   ///< The rendering color.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a drawable representation of a PMPL multibody.
    /// @param _cfgs The path Cfgs.
    /// @param _color The drawing color.
    DrawablePath(const std::vector<Cfg>& _cfgs, glutils::color _color);

    virtual ~DrawablePath();

    ///@}
    ///@name drawable_display_list Overrides
    ///@{
    /// Paths cannot be selected/highlighted.

    virtual void build() override;

    virtual void build_select() override {}

    ///@}

};

#endif
