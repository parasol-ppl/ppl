#include "EditCfgDialog.h"

#include "SliderTextWidget.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Visualization/DrawableMultiBody.h"

#include "sandbox/gui/main_window.h"



/*------------------------------- Construction -------------------------------*/

EditCfgDialog::
EditCfgDialog(main_window* const _parent, DrawableMultiBody* const _mb)
  : QDialog(_parent), m_main(_parent), m_drawable(_mb)
{
  setWindowTitle("Edit Cfg");
  setAttribute(Qt::WA_DeleteOnClose, true);
  setMinimumWidth(400);
  setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  // Create a display area which will hold all the subcomponents (needed for
  // scroll area support).
  QWidget* displayArea = new QWidget(this);
  displayArea->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  // Create a layout for the display area.
  QVBoxLayout* displayLayout = new QVBoxLayout(this);
  displayArea->setLayout(displayLayout);

  // Create a slider for each dof.
  auto mb = m_drawable->GetMultiBody();
  const auto& dofInfo = mb->GetDofInfo();
  m_originalCfg = mb->GetCurrentCfg();

  for(size_t i = 0; i < mb->DOF(); ++i) {
    const auto& info = dofInfo[i];
    SliderTextWidget* slider = new SliderTextWidget(this, info.name, info.range);
    slider->SetValue(m_originalCfg[i]);

    // Add to the layout and connect the signal.
    displayLayout->addWidget(slider);
    connect(slider, SIGNAL(ValueChanged(double)),
            this,     SLOT(UpdateCfg()));

    m_sliders.push_back(slider);
  }

  // Create buttons for OK and Cancel.
  QDialogButtonBox* okCancel = new QDialogButtonBox(this);
  okCancel->setOrientation(Qt::Horizontal);
  okCancel->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);
  displayLayout->addWidget(okCancel);

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
}


EditCfgDialog::
~EditCfgDialog() = default;

/*--------------------------------- Helpers ----------------------------------*/

void
EditCfgDialog::
UpdateCfg() {
  auto mb = m_drawable->GetMultiBody();
  std::vector<double> cfg(mb->DOF(), 0);

  for(size_t i = 0; i < cfg.size(); ++i)
    cfg[i] = m_sliders[i]->GetValue();

  mb->Configure(cfg);
}

/*---------------------------- QDialog Overrides -----------------------------*/

void
EditCfgDialog::
reject() {
  m_drawable->GetMultiBody()->Configure(m_originalCfg);
  QDialog::reject();
}

/*----------------------------------------------------------------------------*/
