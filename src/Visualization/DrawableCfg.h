#ifndef DRAWABLE_CFG_H_
#define DRAWABLE_CFG_H_

#include "ConfigurationSpace/Cfg.h"
#include "glutils/drawable_display_list.h"

class DrawableMultiBody;

////////////////////////////////////////////////////////////////////////////////
/// A Visual Representation of a configuration
////////////////////////////////////////////////////////////////////////////////
class DrawableCfg : public glutils::drawable_display_list {
  public:

    ///@name Construction
    ///@{

    /// Builds member variable data for a renderable cfg
    ///@param _data DOF data of the cfg. (this is to reduce duplicate data)
    ///@param _body The drawable multibody begin drawn (stored in
    ///             DrawableRoadmap)

    DrawableCfg(const std::vector<double>& _data, DrawableMultiBody* _body);

    ///@}
    ///@name Accessors
    ///@{

    /// Gets the reference point of the base.
    ///@return Returns a 3D vector of the positional dofs
    glutils::vector3f GetPoint();

    ///@}
  protected:

    ///@name drawable_call_list overrides
    ///@{

    virtual void build() override;
    virtual void build_select() override;

    ///@}
  private:
    std::vector<double> m_data;             ///< DOF data of the cfg to be rendered (temporary)
    glutils::vector3f m_point;              ///< Reference point for the base
    DrawableMultiBody* m_drawableMultiBody; ///< The drawable multibody that is rendered
};

#endif
