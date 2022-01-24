//#include "MPLibrary/PMPL.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/MPTask.h"
#include "Simulator/Simulation.h"
#include "Visualization/Gui/Setup.h"

#include "sandbox/gui/main_window.h"
#include "sandbox/gui/gl_widget.h"


int
main(int _argc, char** _argv) {
  try {
    const std::string flag = std::string(_argv[1]);
    if(_argc < 3 || (flag != "-f" and flag != "-e" and flag != "-h" and flag != "-c"))
      throw ParseException(WHERE, "Incorrect usage. Usage: {-f|-e|-h|-c} options.xml");

    // Make problem object.
    std::shared_ptr<MPProblem> problem(new MPProblem(_argv[2]));

    // Position the robot by sampling from the first task and set colors.
    /// @TODO Decide on a way to declare the starting configuration either
    ///       explicitly or from a specific task. For now we will assume that
    ///       the first task is a query and its start boundary is a single point.
    if(!problem->GetRobotGroups().empty()) {
      for(const auto& group : problem->GetRobotGroups()) {
        //TODO needs to be updated to track which robots have been given a
        //starting position and which ones have not when considering multiple
        //grouptasks
        auto groupTask = problem->GetTasks(group.get()).front();
        for(auto it = groupTask->begin(); it != groupTask->end(); it++){
          Robot* const r = it->GetRobot();
          if(r->IsVirtual())
            continue;

          // Position the robot at zero, or at the task center if one exists.
          std::vector<double> dofs(r->GetMultiBody()->DOF(), 0);
          if(it->GetStartConstraint())
            dofs = it->GetStartConstraint()->
              GetBoundary()->GetCenter();
          r->GetMultiBody()->Configure(dofs);

					// Store robot's initial position
					Cfg initial(r);
					initial.SetData(dofs);
					problem->SetInitialCfg(r,initial);

        }
      }
    }
    else {
      for(const auto& robot : problem->GetRobots()) {
        Robot* const r = robot.get();
        if(r->IsVirtual())
          continue;

        // Position the robot at zero, or at the task center if one exists.
        std::vector<double> dofs(r->GetMultiBody()->DOF(), 0);
        if(problem->GetTasks(r).front()->GetStartConstraint())
          dofs = problem->GetTasks(r).front()->GetStartConstraint()->
                 GetBoundary()->GetCenter();
        r->GetMultiBody()->Configure(dofs);
				// Store robot's initial position
				Cfg initial(r);
				initial.SetData(dofs);
				problem->SetInitialCfg(r,initial);
      }
    }

    // Make simulation object.
    const bool editMode = flag == "-e";
		const bool hidden = flag == "-h";
    Simulation::Create(problem.get(), editMode);
    Simulation* simulation = Simulation::Get();

		simulation->start();
    // Make visualizer object.
    QApplication app(_argc, _argv);
    main_window window;

    // Set up the extra gui elements if we are in edit mode.
    if(editMode)
      SetupMainWindow(&window);

    // Load the simulation into the visualizer and start it.
    window.visualization(simulation);
    if(!hidden)
			window.show();
    window.gl()->start();
    app.exec();

    // Clean up the simulation and problem when we are done.
    simulation->Uninitialize();

    delete simulation;

    return 0;
  }
  catch(const std::runtime_error& _e) {
    // Write exceptions to cout so that we still get them when piping stdout.
    std::cout << std::endl << _e.what() << std::endl;
    return 1;
  }
}
