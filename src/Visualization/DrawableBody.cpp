#include "DrawableBody.h"

#include "glutils/color.h"

#include "Geometry/Bodies/Body.h"
#include "Visualization/DrawableMultiBody.h"


/*------------------------------ Construction --------------------------------*/

DrawableBody::
DrawableBody(DrawableMultiBody* const _parent, Body* const _body, bool _wired)
  : DrawablePolyhedron(_body->GetPolyhedron(), _body->GetColor(), _wired),
    m_parent(_parent), m_body(_body) { }

/*------------------------------- Body Support -------------------------------*/

Body*
DrawableBody::
GetBody() const noexcept {
  return m_body;
}

/*--------------------------- drawable Overrides -----------------------------*/

void
DrawableBody::
select() noexcept {
  m_parent->select();
}


void
DrawableBody::
deselect() noexcept {
  m_parent->deselect();
}


void
DrawableBody::
highlight() noexcept {
  m_parent->highlight();
}


void
DrawableBody::
unhighlight() noexcept {
  m_parent->unhighlight();
}

/*----------------------------------------------------------------------------*/
