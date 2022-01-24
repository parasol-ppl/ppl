#include "EditBodyDialog.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Utilities/RuntimeUtils.h"
#include "Visualization/DrawableBody.h"

#include "sandbox/gui/main_window.h"


/*------------------------------- Construction -------------------------------*/

EditBodyDialog::
EditBodyDialog(main_window* const _parent, DrawableBody* const _b)
  : QDialog(_parent), m_main(_parent), m_drawable(_b),
    m_body(_b->GetBody())
{
  setWindowTitle("Edit Body");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400);

  // Create buttons for the supported actions.
  QPushButton* editMeshButton    = new QPushButton("Edit Mesh", this),
             * editTypeButton    = new QPushButton("Edit Type", this),
             * editColorButton   = new QPushButton("Edit Color", this),
             * editPhysicsButton = new QPushButton("Edit Physics", this);

  // Create buttons for OK and Cancel.
  QDialogButtonBox* okCancel = new QDialogButtonBox(this);
  okCancel->setOrientation(Qt::Horizontal);
  okCancel->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

  // Create a layout for the dialog and add the subcomponents.
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addWidget(editMeshButton);
  layout->addWidget(editTypeButton);
  layout->addWidget(editColorButton);
  layout->addWidget(editPhysicsButton);
  layout->addWidget(okCancel);
  setLayout(layout);

  // Connect the buttons to their appropriate functions.
  connect(editMeshButton,    SIGNAL(clicked()), this, SLOT(EditMesh()));
  connect(editTypeButton,    SIGNAL(clicked()), this, SLOT(EditType()));
  connect(editColorButton,   SIGNAL(clicked()), this, SLOT(EditColor()));
  connect(editPhysicsButton, SIGNAL(clicked()), this, SLOT(EditPhysics()));

  // Connect the ok/cancel buttons.
  connect(okCancel, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancel, SIGNAL(rejected()), this, SLOT(reject()));
}

/*--------------------------------- Helpers ----------------------------------*/

void
EditBodyDialog::
EditMesh() {
  m_main->show_alert("Not yet implemented: " + WHERE);
}


void
EditBodyDialog::
EditType() {
  m_main->show_alert("Not yet implemented: " + WHERE);
}


void
EditBodyDialog::
EditColor() {
  m_main->show_alert("Not yet implemented: " + WHERE);
}


void
EditBodyDialog::
EditPhysics() {
  m_main->show_alert("Not yet implemented: " + WHERE);
}

/*----------------------------------------------------------------------------*/
