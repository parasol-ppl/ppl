#ifndef PMPL_SIMULATION_H_
#define PMPL_SIMULATION_H_

#include "sandbox/base_visualization.h"
#include "nonstd/collection.h"
#include "Utilities/PMPLExceptions.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/RoadmapGraph.h"

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>

class BulletEngine;
class Cfg;
class DrawableBoundary;
class DrawableMultiBody;
class DrawablePath;
class DrawableRoadmap;
class DrawableWorkspaceSkeleton;
class MPProblem;
class MultiBody;
class StatClass;
class WorkspaceSkeleton;


////////////////////////////////////////////////////////////////////////////////
/// Simulate an MPProblem using the bullet physics engine. Rendering is
/// performed by the base_visualization parent class.
///
/// @note This is a singleton object; there can only be one instance.
////////////////////////////////////////////////////////////////////////////////
class Simulation : public base_visualization {

  private:

    ///@name Construction
    ///@{

    /// Create a simulation of an MPProblem.
    /// @param _problem The MPProblem to simulate.
    /// @param _edit Start in edit mode instead of the physical simulator?
    Simulation(MPProblem* _problem, const bool _edit);

  public:

    /// Create the singleton.
    /// @param _problem The MPProblem to simulate.
    /// @param _edit Start in edit mode instead of the physical simulator?
    static void Create(MPProblem* _problem,
        const bool _edit = false);

    virtual ~Simulation();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the singleton.
    static Simulation* Get() noexcept;

    /// Get the nearest number of timesteps needed to represent a continuous
    /// time interval.
    /// @param _dt The time interval.
    /// @return The number of steps needed to represent _dt.
    static size_t NearestNumSteps(const double _dt) noexcept;

    /// Get the current timestamp.
    /// @return The number of steps taken so far.
    static size_t GetTimestamp() noexcept;

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Set up the simulation.
    void Initialize();

    /// Tear down the simulation.
    void Uninitialize();

    /// Advance the simulation one timestep.
    void SimulationStep();

    /// Update the drawable transforms to match those in the MPProblem.
    void EditStep();

    ///@}
    ///@name Rendering Interface
    ///@{

    virtual void render() override;
    virtual void start() override;
    virtual void reset() override;

    /// Set the maximum number of frames that the simulator can pre-compute.
    /// @param _max The maximum number of backlogged frames to compute.
    void SetBacklog(const size_t _max);

    ///@}
    ///@name Additional Visualization
    ///@{

    /// Add a path graphic to the scene as a sequence of connected line
    /// segments.
    /// @param _path The path to render.
    /// @param _c The line color.
    /// @return The ID of the path.
    size_t AddPath(const std::vector<Cfg>& _path, const glutils::color& _c);

    /// Remove a path graphic from the scene.
    /// @param _id The path ID.
    void RemovePath(const size_t _id);

    /// Add a roadmap graphic to the scene.
    /// @param _graph the graph to rendered
    /// @param _c The line color.
    /// @return The ID of the path.
    size_t AddRoadmap(RoadmapGraph<Cfg, DefaultWeight<Cfg>>* _graph,
        const glutils::color& _c);

    /// Remove a roadmap graphic from the scene.
    /// @param _id The path ID.
    void RemoveRoadmap(const size_t _id);

    /// Add a boundary graphic to the scene.
    /// @param _boundary The boundary to be rendered
    /// @param _c        The rendering color.
    /// @param _wired    Render in wireframe if true, solid if false.
    /// @return The ID of the boundary
    size_t AddBoundary(const Boundary* const _boundary, const glutils::color& _c,
        const bool _wired = true);

    /// Remove a boundary graphic from the scene.
    /// @param _id The boundary ID.
    void RemoveBoundary(const size_t _id);

    /// Update a boundary graphic.
    /// @param _id The boundary ID.
    /// @param _t  The transformation to apply.
    void TransformBoundary(const size_t _id, const glutils::transform& _t);

    /// Add a workspace skeleton rendering to to the scene.
    /// @param _skeleton The workspace skeleton.
    /// @param _c The rendering color.
    /// @return The skeleton rendering's ID.
    size_t AddWorkspaceSkeleton(WorkspaceSkeleton* const _skeleton,
        const glutils::color& _c);

    ///@}
    ///@name Editing
    ///@{

    /// Rebuild the physics engine's model for a given multibody.
    /// @param _m The drawable multibody to rebuild.
    /// @WARNING The simulation will lock while you do this!
    void RebuildMultiBody(DrawableMultiBody* const _m);

    ///@}
    ///@name Performance Measurement
    ///@{

    /// Get the simulator's stat class for time profiling.
    static StatClass* GetStatClass() noexcept;

    /// Print the simulator's stats to an output file.
    /// @param _basename The base name for the output file, or empty to use the
    ///                  MPProblem base name.
    void PrintStatFile(const std::string& _basename = "");

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Create a btCollisionShape for the environment bounding box.
    void AddBBX();

    /// Add all environment obstacles to the bullet world.
    void AddObstacles();

    /// Add all robots in the problem to the bullet world.
    void AddRobots();

    /// Ad all terrain in the problem to the bullet world.
    void AddTerrain();

    ///@}
    ///@name Internal State
    ///@{

    MPProblem* m_problem; 			 					 ///< The simulated MPProblem.
    BulletEngine* m_engine{nullptr};       ///< The physics engine.

    mutable std::mutex m_guard;            ///< Lock for updating transforms.
    std::thread m_worker;                  ///< Thread for stepping simulation.
    std::atomic<bool> m_running{false};    ///< Is the simulation running?
    volatile size_t m_backloggedSteps{0};  ///< Number of precomputed steps.
    size_t m_backlogMax{1};                ///< Max number of precomputed steps.
    std::atomic<size_t> m_timestep{0};     ///< Total steps taken.

    const bool m_editMode{false};             ///< Are we in edit mode?

    std::unique_ptr<StatClass> m_stats;       ///< StatClass for time profiling.

    nonstd::collection<DrawablePath> m_paths; ///< Paths we are drawing.
    nonstd::collection<DrawableRoadmap> m_roadmaps; ///< Roadmaps to be drawn.
    nonstd::collection<DrawableBoundary> m_boundaries; ///< Boundaries to be drawn.
    nonstd::collection<DrawableWorkspaceSkeleton> m_skeletons; ///< Skeletons to be drawn.

    std::vector<glutils::drawable*> m_removedDrawables; ///< Drawables to remove.

    ///@}
    ///@name Deleted Functions
    ///@{
    /// Move/copy are not supported.

    Simulation(const Simulation&) = delete;
    Simulation(Simulation&&)      = delete;

    Simulation& operator=(const Simulation&) = delete;
    Simulation& operator=(Simulation&&)      = delete;

    ///@}

};

#endif
