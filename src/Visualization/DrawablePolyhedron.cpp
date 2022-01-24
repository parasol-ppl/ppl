#include "DrawablePolyhedron.h"

#include "Geometry/GMSPolyhedron.h"


/*------------------------------- Construction -------------------------------*/

DrawablePolyhedron::
DrawablePolyhedron(const GMSPolyhedron& _polyhedron,
    const glutils::color& _color, const bool _wire)
  : glutils::drawable_display_list(),
    m_polyhedron(_polyhedron),
    m_color(_color),
    m_wire(_wire)
{ }

/*----------------------------------------------------------------------------*/

void
DrawablePolyhedron::
ToggleRenderMode() {
  m_wire = !m_wire;
  this->uninitialize();
}

/*----------------------------------------------------------------------------*/

void
DrawablePolyhedron::
build() {
  glColor4fv(m_color);

  if(m_wire) {
    glLineWidth(1);
    build_wireframe();
  }
  else {
    build_polyhedron();
  }
}


void
DrawablePolyhedron::
build_select() {
  if(m_wire) {
    glLineWidth(4);
    build_wireframe();
  }
  else {
    build_polyhedron();
  }
}


void
DrawablePolyhedron::
build_selected() {
  glColor4fv(glutils::color::yellow);
  glLineWidth(4);

  build_wireframe();
}

/*--------------------------------- Helpers ----------------------------------*/

void
DrawablePolyhedron::
build_polyhedron() {
  glBegin(GL_TRIANGLES);
  for(const auto& polygon : m_polyhedron.GetPolygonList()) {
    for(size_t i = 0; i < 3; ++i) {
      glNormal3dv(static_cast<const GLdouble*>(polygon.GetNormal()));
      glVertex3dv(static_cast<const GLdouble*>(polygon.GetPoint(i)));
    }
  }
  glEnd();
}


void
DrawablePolyhedron::
build_wireframe() {
  // If lighting is on, then turn it off temporarily.
  const bool lighting = glIsEnabled(GL_LIGHTING);
  if(lighting)
    glDisable(GL_LIGHTING);

  const auto triangleList = BuildAdjacencyMap();

  const auto& points = m_polyhedron.GetVertexList();
  glBegin(GL_LINES);
  for(auto& elem : triangleList) {
    glVertex3dv(static_cast<const GLdouble*>(points[elem.first]));
    glVertex3dv(static_cast<const GLdouble*>(points[elem.second]));
  }
  glEnd();

  if(lighting)
    glEnable(GL_LIGHTING);
}


std::set<std::pair<size_t, size_t>>
DrawablePolyhedron::
BuildAdjacencyMap() {
  std::set<std::pair<size_t, size_t>> triangleList;
  const auto& triangles = m_polyhedron.GetPolygonList();

  for(auto triangle = triangles.begin(); triangle != triangles.end(); ++triangle) {
    const auto& n1 = triangle->GetNormal();
    for(auto iter = triangle + 1; iter < triangles.end(); ++iter) {
      // Skip triangle pairs with identical normals since these represent a
      // continuous surface.
      const auto& n2 = iter->GetNormal();
      if(n1 == n2)
        continue;

      auto commonEdge = triangle->CommonEdge(*iter);
      if(commonEdge.first != -1 and commonEdge.second != -1)
        triangleList.emplace((size_t)commonEdge.first,
                             (size_t)commonEdge.second);
    }
  }

  return triangleList;
}

/*----------------------------------------------------------------------------*/
