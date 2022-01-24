#include "Setup.h"

#include "Simulator/Simulation.h"
#include "Visualization/DrawableMultiBody.h"
#include "Visualization/Gui/EditCfgDialog.h"
#include "Visualization/Gui/EditMultiBodyDialog.h"
#include "Visualization/Gui/TestWidget.h"

#include "sandbox/base_visualization.h"
#include "sandbox/gui/main_window.h"


/*------------------------ Local Helper Declarations -------------------------*/

DrawableMultiBody* GetSelectedMultiBody(const std::string& _label);

void LaunchEditMultiBodyDialog(main_window* const _main);

void LaunchEditCfgDialog(main_window* const _main);

/*----------------------------------------------------------------------------*/

void
SetupMainWindow(main_window* const _main) {
  // Add a test widget in the F2 slot for GUI development.
  TestWidget* testWidget = new TestWidget(_main);
  _main->add_window(testWidget);

  // Add a keymap to trigger our GL gui components.
  main_window::key_map keyMap(
      [_main](QKeyEvent* const _e)
      {
        switch(_e->key())
        {
          case Qt::Key_E:
            LaunchEditMultiBodyDialog(_main);
            break;
          case Qt::Key_C:
            LaunchEditCfgDialog(_main);
            break;
        }
      }
  );
  _main->add_key_mapping("sim-gui", std::move(keyMap));
}

/*--------------------------------- Helpers ----------------------------------*/

DrawableMultiBody*
GetSelectedMultiBody(const std::string& _label) {
  // Locate the currently selected multibody and ensure there is only
  // one.
  DrawableMultiBody* selected = nullptr;
  auto all = Simulation::Get()->selected_drawables();
  for(auto* d : all) {
    DrawableMultiBody* test = dynamic_cast<DrawableMultiBody*>(d);
    if(test) {
      if(selected) {
        std::cerr << "Cannot start " << _label
                  << " dialog because multiple multibodies are selected."
                  << std::endl;
        return nullptr;
      }
      selected = test;
    }
  }

  // If no multibody was selected, refuse to start the dialog.
  if(!selected)
    std::cerr << "Cannot start " << _label << " dialog because no multibodies "
              << "are selected."
              << std::endl;
  return selected;
}


void
LaunchEditMultiBodyDialog(main_window* const _main) {
  DrawableMultiBody* selected = GetSelectedMultiBody("EditMultiBody");

  // Start the dialog.
  EditMultiBodyDialog* d = new EditMultiBodyDialog(_main, selected);
  _main->show_dialog(d);
}


void
LaunchEditCfgDialog(main_window* const _main) {
  DrawableMultiBody* selected = GetSelectedMultiBody("EditCfg");

  // Start the dialog.
  EditCfgDialog* d = new EditCfgDialog(_main, selected);
  _main->show_dialog(d);
}

/*----------------------------------------------------------------------------*/
