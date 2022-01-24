#include "DrawableWorkspaceSkeleton.h"

#include "Simulator/Conversions.h"
#include "Simulator/Simulation.h"


/*------------------------------- Construction -------------------------------*/

DrawableWorkspaceSkeleton::
DrawableWorkspaceSkeleton(WorkspaceSkeleton* _skeleton,
    const glutils::color& _color)
  : m_skeleton(_skeleton), m_color(_color)
{ }

/*--------------------------- Drawable Overrides -----------------------------*/

void
DrawableWorkspaceSkeleton::
build() {
  glDisable(GL_LIGHTING);
  glColor4fv(m_color);

  auto& graph = m_skeleton->GetGraph();

  // Draw each vertex.
  glPointSize(6);
  glBegin(GL_POINTS);
  for(auto iter = graph.begin(); iter != graph.end(); ++iter) {
    const Point3d& point = iter->property();
    glVertex3dv(point);
  }
  glEnd();

  // Draw each edge.
  glLineWidth(1.);
  glPointSize(3);
  for(auto iter = graph.edges_begin(); iter != graph.edges_end(); ++iter) {
    const std::vector<Point3d>& path = iter->property();
    glBegin(GL_LINE_STRIP);
    for(const auto& point : path)
      glVertex3dv(point);
    glEnd();
    glBegin(GL_POINTS);
    for(const auto& point : path)
      glVertex3dv(point);
    glEnd();
  }

  glEnable(GL_LIGHTING);
}

/*----------------------------------------------------------------------------*/
