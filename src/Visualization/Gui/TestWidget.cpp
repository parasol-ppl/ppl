#include "TestWidget.h"

#include "EditWidgets.h"
#include "SliderTextWidget.h"

#include "sandbox/gui/main_window.h"

#include "Transformation.h"

using namespace mathtool;


/*------------------------------- Construction -------------------------------*/

TestWidget::
TestWidget(main_window* const _parent) : QWidget(_parent), m_main(_parent) {
  // Create a title element.
  QLabel* const title = new QLabel(this);
  title->setText("Test Widget");

  // Create a transformation edit widget.
  Transformation t(Vector3d(4, 5, 6), EulerAngle(PI, 0, -PI));
  m_transformEditor = new EditTransformationWidget(this, "Test", t);

  // Create a button to show a transform export.
  QPushButton* showIt = new QPushButton("Show Transformation", this);

  // Create the layout.
  QVBoxLayout* const layout = new QVBoxLayout(this);
  layout->addWidget(title);
  layout->addWidget(m_transformEditor);
  layout->addWidget(showIt);
  setLayout(layout);

  // Connect signals and slots.
  connect(showIt, SIGNAL(clicked()), this, SLOT(ShowTransformationOutput()));
}

/*----------------------------------------------------------------------------*/

void
TestWidget::
ShowTransformationOutput() {
  const Transformation t = m_transformEditor->GetValue();

  std::stringstream ss;
  ss << t;
  m_main->show_alert(ss.str().c_str());
}

/*----------------------------------------------------------------------------*/
