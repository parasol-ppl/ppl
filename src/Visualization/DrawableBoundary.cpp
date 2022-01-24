#include "DrawableBoundary.h"

#include "Geometry/Boundaries/Boundary.h"

#include "nonstd/io.h"

#include <fstream>


DrawableBoundary::
DrawableBoundary(const Boundary* const _boundary, const glutils::color& _color,
    const bool _wire) :
  DrawablePolyhedron(m_polyhedron, _color, _wire)
{
  // Set the transformation so that the boundary is centered.
  auto b = _boundary->Clone();
  const auto& c = b->GetCenter();
  this->push_transform(glutils::transform{
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    (float)c[0], (float)c[1], (float)(c.size() > 2 ? c[2] : 0.), 1
  });

  // Copy the centered boundary as the polyhedron model.
  const std::vector<double> zero(c.size(), 0);
  b->SetCenter(zero);
  m_polyhedron = b->MakePolyhedron();
}


void
DrawableBoundary::
build() {
  const bool transparent = m_color[3] != 1.0;

  if(!this->m_wire)
    glDisable(GL_CULL_FACE);
  if(transparent) {
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
  }

  DrawablePolyhedron::build();

  // Build normals
#if 0
  if(!this->m_wire) {
    glColor4fv(glutils::color::red);
    glLineWidth(4);
    glBegin(GL_LINES);
    for(const auto& polygon : m_polyhedron.GetPolygonList()) {
      const auto center = polygon.FindCenter(),
                 normal = polygon.GetNormal();
      glVertex3dv(static_cast<const GLdouble*>(center));
      glVertex3dv(static_cast<const GLdouble*>(center + normal));
    }
    glEnd();
  }
#endif

  if(!this->m_wire)
    glEnable(GL_CULL_FACE);
  if(transparent) {
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }
}
