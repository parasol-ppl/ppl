#include "Simulation.h"

#include <iostream>
#include <fstream>

#include "BulletEngine.h"
#include "Conversions.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MetricUtils.h"
#include "Visualization/DrawableMultiBody.h"
#include "Visualization/DrawablePath.h"
#include "Visualization/DrawableRoadmap.h"
#include "Visualization/DrawableBoundary.h"
#include "Visualization/DrawableWorkspaceSkeleton.h"

#include "nonstd/io.h"
#include "nonstd/numerics.h"
#include "glutils/camera.h"
#include "sandbox/gui/gl_widget.h"
#include "sandbox/gui/main_window.h"
#include "glutils/selector.h"


/*---------------------------- Construction ----------------------------------*/

/// Create the singleton.
static std::unique_ptr<Simulation> s_singleton;


Simulation::
Simulation(MPProblem* _problem, const bool _edit)
  : m_problem(_problem), m_editMode(_edit), m_stats(new StatClass()) {
  // If we are in edit mode, then the backlog is annoying rather than helpful.
  // Disable it in that case.
  if(m_editMode)
    SetBacklog(1);
}


void
Simulation::
Create(MPProblem* _problem, const bool _edit) {
  if(s_singleton.get())
    throw RunTimeException(WHERE, "THERE CAN ONLY BE ONE!");
  s_singleton.reset(new Simulation(_problem, _edit));
}


Simulation::
~Simulation() {
  reset();

  // Print the stats on our way out so that we still get output in the event of
  // an uncaught exception.
  PrintStatFile();

  // Release the singleton so that it points to nothing.
  s_singleton.release();
}

/*-------------------------------- Accessors ---------------------------------*/

Simulation*
Simulation::
Get() noexcept {
  return s_singleton.get();
}


size_t
Simulation::
NearestNumSteps(const double _dt) noexcept {
  const double timeRes  = s_singleton->m_problem->GetEnvironment()->GetTimeRes(),
               multiple = _dt / timeRes,
               threshold = timeRes * .01;

  // If _dt is approximately an integer number of time steps, return that
  // integer.
  if(nonstd::approx(multiple, std::round(multiple), threshold))
    return std::lround(multiple);

  // Otherwise, return the next largest number of steps.
  return size_t(std::ceil(multiple));
}


size_t
Simulation::
GetTimestamp() noexcept {
  return s_singleton->m_timestep;
}

/*-------------------------- Simulation Interface ----------------------------*/

void
Simulation::
Initialize() {
  // If the engine object is not null, we are already initialized.
  if(m_engine)
    return;

  // Require a non-null problem to initialize.
  if(!m_problem)
    throw RunTimeException(WHERE) << "Cannot initialize with a null problem!";

  // Require a non-null main window to initialize.
  //if(!main_window::get())
  //  throw RunTimeException(WHERE) << "Cannot initialize without a main window!";

  // Create a bullet engine.
  m_engine = new BulletEngine(m_problem);

  // Add the problem objects to the simulation.
  AddBBX();
  AddRobots();
  AddObstacles();
  AddTerrain();

  // Set the camera position.
  const auto& t = m_problem->GetEnvironment()->GetInitialCameraTransformation();
  const glutils::vector3f pos = ToGLUtils(t.translation()),
                          at  = ToGLUtils(t * Vector3d(0, 0, -1)),
                          up  = ToGLUtils(t.rotation() * Vector3d(0, 1, 0));

  if(main_window::get())
		main_window::get()->gl()->camera()->position(pos, at, up);
}


void
Simulation::
Uninitialize() {
  // If the engine object is null, we are already uninitialized.
  if(!m_engine)
    return;

  // Delete the bullet engine.
  delete m_engine;
  m_engine = nullptr;

  // Delete the drawable objects.
  auto drawables = this->m_drawables;
  for(auto d : drawables)
    this->remove_drawable(d);

  // Remove bullet model pointers from problem objects.
  for(size_t i = 0; i < m_problem->NumRobots(); ++i)
    m_problem->GetRobot(i)->SetSimulationModel(nullptr);
}


void
Simulation::
SimulationStep() {
  // Push the current transforms for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    ++m_timestep;

    // If we have already pre-computed the maximum number of steps allowed,
    // don't do anything.
    if(m_backloggedSteps >= m_backlogMax)
      return;
    ++m_backloggedSteps;

    // Enqueue the current position of each mobile object in the scene.
    for(size_t i = 0; i < this->m_drawables.size(); ++i) {
      auto d = static_cast<DrawableMultiBody*>(this->m_drawables[i]);
      for(size_t j = 0; j < d->GetNumBodies(); ++j)
        d->PushTransform(j, m_engine->GetObjectTransform(d->GetMultiBody(), j));
    }
  }

  const double timestep = m_problem->GetEnvironment()->GetTimeRes();

  // Set each Robot's multibody to its current simulated state.
  for(size_t i = 0; i < m_problem->NumRobots(); ++i)
    m_problem->GetRobot(i)->SynchronizeModels();

  // Step each Robot's agent to set the forces for the next step.
  /// @note Any agents which will plan concurrently with the main simulation
  ///       loop must first copy the current problem state in order to get
  ///       accurate positions for the other dynamic objects.
  for(size_t i = 0; i < m_problem->NumRobots(); ++i)
    m_problem->GetRobot(i)->Step(timestep);

  // Step the simulation.
  m_engine->Step(timestep);
}


void
Simulation::
EditStep() {
  // Push the current transforms for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    // If we have already pre-computed the maximum number of steps allowed,
    // don't do anything.
    if(m_backloggedSteps >= m_backlogMax)
      return;
    ++m_backloggedSteps;

    // Enqueue the current position of each mobile object in the scene.
    for(size_t i = 0; i < this->m_drawables.size(); ++i) {
      auto d  = static_cast<DrawableMultiBody*>(this->m_drawables[i]);
      auto mb = d->GetMultiBody();
      for(size_t j = 0; j < d->GetNumBodies(); ++j)
        d->PushTransform(j, ToGLUtils(mb->GetBody(j)->GetWorldTransformation()));
    }
  }
}

/*-------------------------- Rendering Interface -----------------------------*/

void
Simulation::
render() {
  // Update the transform for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    // If there are queued frames, update objects to the next queued positions.
    if(m_backloggedSteps > 0)
    {
      --m_backloggedSteps;

      for(auto d : m_drawables)
        static_cast<DrawableMultiBody*>(d)->UpdateTransform();
    }

    for(auto d : m_boundaries.get_all())
      d->update_transform();

    for(auto d : m_removedDrawables) {
      d->uninitialize();
      delete d;
    }
    m_removedDrawables.clear();
  }

  for(auto d : m_paths.get_all())
    d->render();

  // Rrrrrender.
  base_visualization::render();

  for(auto d : m_roadmaps.get_all())
    d->render();

  for(auto d : m_boundaries.get_all())
    d->render();

  for(auto d : m_skeletons.get_all())
    d->render();
}


void
Simulation::
start() {
  // The simulation is already started if we are running.
  if(m_running)
    return;
  m_running = true;

  Initialize();

  // Create a worker object to step the physics simulation.
  auto physicsWorkFunction = [this]() {
    while(this->m_running)
      this->SimulationStep();
  };

  // Create a worker object to step the edit simulation.
  auto editWorkFunction = [this]() {
    while(this->m_running)
      this->EditStep();
  };

  // Start the appropriate stepping function in a worker thread.
  m_worker = m_editMode ? std::thread(editWorkFunction)
                        : std::thread(physicsWorkFunction);

  // Ensure the thread was created successfully.
  if(!m_worker.joinable())
    throw RunTimeException(WHERE, "Could not create worker thread.");
}


void
Simulation::
reset() {
  // The simulation is already reset if we are not running.
  if(!m_running)
    return;
  m_running = false;

  // Wait for the worker thread to stop before uninitializing.
  if(m_worker.joinable())
    m_worker.join();

  Uninitialize();
}


void
Simulation::
SetBacklog(const size_t _max) {
  m_backlogMax = _max;
}

/*------------------------- Additional Visualization -------------------------*/

size_t
Simulation::
AddPath(const std::vector<Cfg>& _path, const glutils::color& _c) {
  std::lock_guard<std::mutex> lock(m_guard);

  // Assert the path is not empty.
  if(_path.empty())
    throw RunTimeException(WHERE) << "Cannot draw an empty path.";

  return m_paths.add(new DrawablePath(_path, _c));
}


void
Simulation::
RemovePath(const size_t _id) {
  std::lock_guard<std::mutex> lock(m_guard);

  // Remove this path from the collection and release it.
  DrawablePath* path = m_paths.take(_id);
  m_removedDrawables.push_back(path);
}


size_t
Simulation::
AddRoadmap(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph,
    const glutils::color& _color) {
  std::lock_guard<std::mutex> lock(m_guard);

  if(!_graph)
    throw RunTimeException(WHERE) << "Cannot draw a NULL graph.";

  return m_roadmaps.add(new DrawableRoadmap(_graph, _color));
}


void
Simulation::
RemoveRoadmap(const size_t _id) {
  std::lock_guard<std::mutex> lock(m_guard);

  // Remove this path from the collection and release it.
  DrawableRoadmap* roadmap = m_roadmaps.take(_id);
  m_removedDrawables.push_back(roadmap);
}


size_t
Simulation::
AddBoundary(const Boundary* const _boundary, const glutils::color& _color,
    const bool _wired) {
  std::lock_guard<std::mutex> lock(m_guard);

  if(!_boundary)
    throw RunTimeException(WHERE) << "Cannot draw a null boundary.";

  if(_boundary->Type() == Boundary::CSpace)
    throw RunTimeException(WHERE) << "Cannot draw a cspace boundary.";

  auto draw = new DrawableBoundary(_boundary, _color, _wired);
  m_selector->add_drawable(draw);
  m_highlighter->add_drawable(draw);

  return m_boundaries.add(draw);
}


void
Simulation::
RemoveBoundary(const size_t _id) {
  std::lock_guard<std::mutex> lock(m_guard);

  // Remove this path from the collection and release it.
  DrawableBoundary* boundary = m_boundaries.take(_id);

  m_selector->remove_drawable(boundary);
  m_highlighter->remove_drawable(boundary);

  m_removedDrawables.push_back(boundary);
}


void
Simulation::
TransformBoundary(const size_t _id, const glutils::transform& _t)
{
  std::lock_guard<std::mutex> lock(m_guard);
  DrawableBoundary* boundary = m_boundaries[_id];
  boundary->push_transform(_t);
}


size_t
Simulation::
AddWorkspaceSkeleton(WorkspaceSkeleton* const _skeleton,
    const glutils::color& _c) {
  std::lock_guard<std::mutex> lock(m_guard);

  if(!_skeleton)
    throw RunTimeException(WHERE) << "Cannot draw a null skeleton.";

  auto draw = new DrawableWorkspaceSkeleton(_skeleton, _c);

  return m_skeletons.add(draw);
}

/*--------------------------------- Editing ----------------------------------*/

void
Simulation::
RebuildMultiBody(DrawableMultiBody* const _d) {
  std::lock_guard<std::mutex> lock(m_guard);

  _d->uninitialize();
  if(!m_editMode)
    m_engine->RebuildObject(_d->GetMultiBody());
}

/*--------------------------- Performance Measurement ------------------------*/

StatClass*
Simulation::
GetStatClass() noexcept {
  return s_singleton->m_stats.get();
}


void
Simulation::
PrintStatFile(const std::string& _basename) {
  // Determine the base and full filenames.
  const std::string basename = _basename.empty() ? m_problem->GetBaseFilename()
                                                 : _basename,
                    fullname = m_problem->GetPath(basename + "-sim.stat");

  // If the base name is empty, then we did not request a simulation stat file.
  if(basename.empty())
    return;

  // Print the stats to file.
  std::ofstream osStat(fullname);
  m_stats->PrintAllStats(osStat);
}

/*----------------------------------- Helpers --------------------------------*/

void
Simulation::
AddBBX() {
  /// @TODO We are rendering the boundary but not including it in the physics
  ///       simulation. Ideally it should be an immobile object in the
  ///       simulation like any other obstacle.
  AddBoundary(m_problem->GetEnvironment()->GetBoundary(), glutils::color::black);

#if 0
  const auto& boundary = m_problem->GetEnvironment()->GetBoundary();

  // Get bounding box ranges
  auto r1 = boundary->GetRange(0);
  auto r2 = boundary->GetRange(1);
  auto r3 = boundary->GetRange(2);

  // Compute half edge lengths of bounding box
  const btScalar thickness = 1.;
  const btScalar x = r1.Center();
  const btScalar y = r2.min - thickness / 2;
  const btScalar z = r3.Center();

  // Make the transform.
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(btVector3(0, -y, 0));

  // Create a btCollisionShape using half lengths of the bounding box
  btBoxShape* box = new btBoxShape(btVector3(x, thickness, z));
  if(!m_editMode)
    m_engine->AddMultiBody(box, trans, 0.);
#endif
}


void
Simulation::
AddObstacles() {
  Environment* env = m_problem->GetEnvironment();

  for(size_t i = 0; i < env->NumObstacles(); ++i) {
    MultiBody* const multiBody = env->GetObstacle(i);

    if(!m_editMode)
      m_engine->AddMultiBody(multiBody);

    this->add_drawable(new DrawableMultiBody(multiBody));
  }
}


void
Simulation::
AddRobots() {
  for(size_t i = 0; i < m_problem->NumRobots(); ++i) {
    auto robot = m_problem->GetRobot(i);

    // Do not add virtual robots to the simulation.
    if(robot->IsVirtual())
      continue;

    if(!m_editMode) {
      auto bulletModel = m_engine->AddRobot(robot);
      robot->SetSimulationModel(bulletModel);
    }

    auto multiBody = robot->GetMultiBody();
    this->add_drawable(new DrawableMultiBody(multiBody));
  }
}


void
Simulation::
AddTerrain() {
  auto env = m_problem->GetEnvironment();
  for(const auto& elem : env->GetTerrains()) {
    for(const auto& terrain : elem.second) {
      //AddBoundary(terrain.GetBoundary(), terrain.Color(), terrain.IsWired());
      for(auto& boundary : terrain.GetBoundaries()){
      	AddBoundary(boundary.get(), terrain.Color(), terrain.IsWired());
			}
    }
  }
}

/*----------------------------------------------------------------------------*/
