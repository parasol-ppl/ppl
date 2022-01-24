#include "DrawableCfg.h"
#include "DrawableMultiBody.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Simulator/Conversions.h"

DrawableCfg::
DrawableCfg(const std::vector<double>& _data, DrawableMultiBody* _body)
  : glutils::drawable_display_list(),
    m_data(_data),
    m_drawableMultiBody(_body)
{ }


glutils::vector3f
DrawableCfg::
GetPoint() {
  return m_point;
}


void
DrawableCfg::
build() {
  auto mb = m_drawableMultiBody->GetMultiBody();
  mb->Configure(m_data);
  m_data.clear();

  m_point = ToGLUtils(mb->GetBase()->GetWorldTransformation().translation());

  for(size_t i = 0; i < m_drawableMultiBody->GetNumBodies(); ++i) {
    m_drawableMultiBody->PushTransform(i,
        ToGLUtils(mb->GetBody(i)->GetWorldTransformation()));
  }

  m_drawableMultiBody->UpdateTransform();

  m_drawableMultiBody->render();
}

void
DrawableCfg::
build_select() {}
