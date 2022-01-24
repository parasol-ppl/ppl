#include "DrawablePath.h"

#include "ConfigurationSpace/Cfg.h"
#include "Simulator/Conversions.h"


/*------------------------------- Construction -------------------------------*/

DrawablePath::
DrawablePath(const std::vector<Cfg>& _cfgs, glutils::color _color)
  : m_cfgs(_cfgs), m_color(_color) { }


DrawablePath::
~DrawablePath() = default;

/*--------------------------- Drawable Overrides -----------------------------*/

void
DrawablePath::
build() {
  glDisable(GL_LIGHTING);
  glColor4fv(m_color);
  glLineWidth(1.);

  // For each pair of Cfgs in the path, draw a line between their points. Do
  // this first so that Cfgs always occlude the lines rather than the other way
  // around.
  glBegin(GL_LINE_STRIP);
  for(const auto& cfg : m_cfgs) {
    const Point3d p = cfg.GetPoint();
    glVertex3f(float(p[0]), float(p[1]), float(p[2]));
  }
  glEnd();

  // Now that the drawing is complete, we can release the cfgs as they are no
  // longer needed.
  m_cfgs.clear();
  glEnable(GL_LIGHTING);
}

/*----------------------------------------------------------------------------*/
