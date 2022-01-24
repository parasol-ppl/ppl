#include "EditMultiBodyDialog.h"

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Visualization/DrawableBody.h"
#include "Visualization/DrawableMultiBody.h"
#include "Visualization/Gui/EditBodyDialog.h"
#include "Visualization/Gui/EditConnectionDialog.h"

#include "sandbox/gui/main_window.h"

#include "nonstd/io.h"


/*------------------------------- Construction -------------------------------*/

EditMultiBodyDialog::
EditMultiBodyDialog(main_window* const _parent, DrawableMultiBody* const _mb)
  : QDialog(_parent), m_main(_parent), m_drawable(_mb),
    m_multibody(_mb->GetMultiBody())
{
  setWindowTitle("Edit MultiBody");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400);
  setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  // Create a list of bodies to edit.
  m_bodyList = new QListWidget(this);
  for(size_t i = 0; i < m_multibody->GetNumBodies(); ++i) {
    std::stringstream ss;
    ss << i << " (label '" << m_multibody->GetBody(i)->Label() << "')";

    m_bodyList->addItem(ss.str().c_str());
  }

  // Create a list of connections to edit.
  m_connectionList = new QListWidget(this);
  const auto& joints = m_multibody->GetJoints();
  for(size_t i = 0; i < joints.size(); ++i) {
    const Connection* connection = joints[i].get();

    std::stringstream ss;
    ss << i << " (bodies " << connection->GetPreviousBodyIndex() << ":"
       << connection->GetNextBodyIndex()
       << ")";

    m_connectionList->addItem(ss.str().c_str());
  }

  // Create buttons for editing bodies and connections.
  QPushButton* editBodyButton       = new QPushButton("Edit Body", this),
             * editConnectionButton = new QPushButton("Edit Connection", this),
             * exportButton         = new QPushButton("Export", this);

  // Create buttons for OK and Cancel.
  QDialogButtonBox* okCancel = new QDialogButtonBox(this);
  okCancel->setOrientation(Qt::Horizontal);
  okCancel->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

  // Create a display area which will hold all the subcomponents (needed for
  // scroll area support).
  QWidget* displayArea = new QWidget(this);
  displayArea->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  // Create a layout for the display area and add the subcomponents.
  QVBoxLayout* displayLayout = new QVBoxLayout(this);
  displayLayout->setSizeConstraint(QLayout::SetMaximumSize);
  displayLayout->addWidget(m_bodyList);
  displayLayout->addWidget(editBodyButton);
  displayLayout->addWidget(m_connectionList);
  displayLayout->addWidget(editConnectionButton);
  displayLayout->addWidget(exportButton);
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

  // Connect the buttons to their appropriate functions.
  connect(editBodyButton, SIGNAL(clicked()),
          this,           SLOT(EditBody()));
  connect(editConnectionButton, SIGNAL(clicked()),
          this,                 SLOT(EditConnection()));
  connect(exportButton, SIGNAL(clicked()),
          this,         SLOT(Export()));

  // Connect the ok/cancel buttons.
  connect(okCancel, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancel, SIGNAL(rejected()), this, SLOT(reject()));
}

/*--------------------------------- Helpers ----------------------------------*/

void
EditMultiBodyDialog::
EditBody() {
  // Show an error if more or less than one item is selected.
  if(m_bodyList->selectedItems().size() != 1) {
    m_main->show_alert("Please select a single body to edit");
    return;
  }

  // Get the current item row.
  size_t row = m_bodyList->row(m_bodyList->currentItem());

  // Launch an edit body dialog for this body.
  DrawableBody* drawable = m_drawable->GetDrawableBody(row);
  m_main->show_dialog(new EditBodyDialog(m_main, drawable));
}


void
EditMultiBodyDialog::
EditConnection() {
  // Show an error if more or less than one item is selected.
  if(m_connectionList->selectedItems().size() != 1) {
    m_main->show_alert("Please select a single connection to edit");
    return;
  }

  // Get the current item row.
  size_t row = m_connectionList->row(m_connectionList->currentItem());

  // Launch an edit body dialog for this connection.
  m_main->show_dialog(new EditConnectionDialog(m_main, m_drawable, row));
}


#ifdef DEBUG_BULLET_PROBLEMS
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "Simulator/Conversions.h"
#endif

void
EditMultiBodyDialog::
Export() {
  m_multibody->Write(std::cout);
  std::cout << "\nBody transforms:";
  for(size_t i = 0; i < m_multibody->GetNumBodies(); ++i)
    std::cout << "\n\tBody " << i << ": "
              << m_multibody->GetBody(i)->GetWorldTransformation();

  std::cout << "\nCfg: " << m_multibody->GetCurrentCfg()
            << std::endl;

#ifdef DEBUG_BULLET_PROBLEMS
  btMultiBody* const b = m_multibody->m_bullet;

  std::cout << "\nBullet transforms:"
            << "\n\tBody 0: " << ToPMPL(b->getBaseWorldTransform());
  for(size_t i = 0; i < (size_t)b->getNumLinks(); ++i)
    std::cout << "\n\tBody " << i + 1 << ": "
              << ToPMPL(b->getLink(i).m_cachedWorldTransform);

  std::cout << "\nJoint Cfg:";
  for(size_t i = 0; i < (size_t)b->getNumLinks(); ++i)
    std::cout << " " << b->getLink(i).m_jointPos[0] / PI;
  std::cout << std::endl;
#endif
}

/*----------------------------------------------------------------------------*/
