#include "EditConnectionDialog.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Simulator/Simulation.h"
#include "Visualization/DrawableBody.h"
#include "Visualization/DrawableMultiBody.h"
#include "Visualization/Gui/EditWidgets.h"

#include "sandbox/gui/main_window.h"

#include "Transformation.h"

using namespace mathtool;


/*------------------------------- Construction -------------------------------*/

EditConnectionDialog::
EditConnectionDialog(main_window* const _parent, DrawableMultiBody* const _mb,
    const size_t _index)
  : QDialog(_parent), m_main(_parent), m_drawable(_mb),
    m_connection(m_drawable->GetMultiBody()->GetJoint(_index))
{
  /// @TODO Figure out how to make this take up the whole dialog space.
  setWindowTitle("Edit Connection");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400);
  setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  // Create a display area which will hold all the subcomponents (needed for
  // scroll area support).
  QWidget* displayArea = new QWidget(this);
  displayArea->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  // Create widgets to edit the joint limits.
  std::string type;
  switch(m_connection->GetConnectionType())
  {
    case Connection::JointType::Spherical:
      type = "Spherical";
    case Connection::JointType::Revolute:
      if(type.empty())
        type = "Revolute";
      m_limitsEditor = new EditJointLimitsWidget(this, type, *m_connection);
    default:
      break;
  }

  // Create Widgets to edit the transforms.
  m_transform1Editor = new EditTransformationWidget(this, "Parent -> Actuation",
      m_connection->GetTransformationToDHFrame());
  m_transform2Editor = new EditTransformationWidget(this, "Actuation -> Link",
      m_connection->GetTransformationToBody2());

  // Create a widget to edit the DH params.
  m_dhParamsEditor = new EditDHParametersWidget(this, "Actuation",
      m_connection->GetDHParameters());

  // Create buttons.
  QDialogButtonBox* okCancel = new QDialogButtonBox(this);
  okCancel->setOrientation(Qt::Horizontal);
  okCancel->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

  // Create a layout for the display area and add the subcomponents.
  QVBoxLayout* displayLayout = new QVBoxLayout(this);
  displayLayout->setSizeConstraint(QLayout::SetMaximumSize);
  if(m_limitsEditor)
    displayLayout->addWidget(m_limitsEditor);
  displayLayout->addWidget(m_transform1Editor);
  displayLayout->addWidget(m_dhParamsEditor);
  displayLayout->addWidget(m_transform2Editor);
  displayLayout->addWidget(okCancel);
  displayArea->setLayout(displayLayout);

  // Create a scroll area to allow displayArea to be scrolled up and down.
  QScrollArea* scroll = new QScrollArea(this);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scroll->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  scroll->setWidget(displayArea);

  // Create a layout for the dialog and add the scroll area to it.
  QVBoxLayout* scrollLayout = new QVBoxLayout(this);
  scrollLayout->setSizeConstraint(QLayout::SetMaximumSize);
  scrollLayout->addWidget(scroll);
  setLayout(scrollLayout);

  // Connect the ok/cancel buttons.
  connect(okCancel, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancel, SIGNAL(rejected()), this, SLOT(reject()));

  // Connect the edit tools.
  connect(m_transform1Editor, SIGNAL(ValueChanged()),
          this,               SLOT(UpdateTransformationToDHFrame()));
  connect(m_transform2Editor, SIGNAL(ValueChanged()),
          this,               SLOT(UpdateTransformationToBody2()));
  connect(m_dhParamsEditor,   SIGNAL(ValueChanged()),
          this,               SLOT(UpdateDHParameters()));
  if(m_limitsEditor)
    connect(m_limitsEditor,   SIGNAL(ValueChanged()),
            this,             SLOT(UpdateJointLimits()));
}

/*--------------------------------- Helpers ----------------------------------*/

void
EditConnectionDialog::
UpdateTransformationToDHFrame() {
  UpdateTransformation(m_transform1Editor,
                       m_connection->GetTransformationToDHFrame());
}


void
EditConnectionDialog::
UpdateTransformationToBody2() {
  UpdateTransformation(m_transform2Editor,
                       m_connection->GetTransformationToBody2());
}


void
EditConnectionDialog::
UpdateDHParameters() {
  m_connection->GetDHParameters() = m_dhParamsEditor->GetValue();
  auto mb = m_drawable->GetMultiBody();
  mb->Configure(mb->GetCurrentDOFs());
}


void
EditConnectionDialog::
UpdateTransformation(EditTransformationWidget* const _w, Transformation& _t) {
  _t = _w->GetValue();
  auto mb = m_drawable->GetMultiBody();
  mb->Configure(mb->GetCurrentDOFs());
}


void
EditConnectionDialog::
UpdateJointLimits() {
  const std::vector<Range<double>> values = m_limitsEditor->GetValue();

  // Update the connection.
  m_connection->SetJointRange(0, values[0]);
  if(values.size() > 1)
    m_connection->SetJointRange(1, values[1]);

  // Update the multibody to make sure the configuration is within limits.
  auto mb = m_drawable->GetMultiBody();
  mb->UpdateJointLimits();
  mb->PushToNearestValidConfiguration();
}

/*----------------------------------------------------------------------------*/
