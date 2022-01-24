#ifndef INTERACTION_INFORMATION_H_
#define INTERACTION_INFORMATION_H_

#include <memory>
#include <string>
#include <vector>

#include "MPTask.h"
#include "MPProblem.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPLibrary/MPBaseObject.h"
#include "Utilities/XMLNode.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"

////////////////////////////////////////////////////////////////////////////////
/// This represents a Handoff Template, which stores the tasks required for
/// robots to perform a handoff.
////////////////////////////////////////////////////////////////////////////////
//template <typename MPTraits>
class InteractionInformation {

  public:
    ///@name Construction
    ///@{

    /// Parse tasks in Handoff template
    /// @param _problem MPProblem for Handoff
    /// @param _node XMLNode to parse
    InteractionInformation(MPProblem* _problem, XMLNode& _node);

    ///@}

    ///@name Accessors
    ///@{

    /// Get label for handoff template
    std::string GetLabel() const;

    /// Get max attempts for placing template in real environment
    size_t GetMaxAttempts() const;

    /// Get MPProblem for handoff
    MPProblem* GetMPProblem() const;

    /// Get interaction tasks for handoff
    std::vector<std::shared_ptr<MPTask>>& GetInteractionTasks();

    /// Get all tasks of the given type
    /// @param _s Type of tasks desired
		std::vector<std::shared_ptr<MPTask>>& GetTypeTasks(const std::string& _s);

    /// Get interaction weight
    double GetInteractionWeight() const;

    /// Adds an addition location to place an IT
    /// @param _location Location to add to handoff locations
    void AddTemplateLocation(Cfg _location);

    /// Gets the set of locations to place ITs
    std::vector<Cfg>& GetTemplateLocations();

    /// Gets the final position of robots at each of the IT locations
    std::vector<Cfg> GetInteractionPositions();

    /// Gets the final position of robot pairs at each of the IT locations. 
    /// First is the robot handing off. Second is robot receving.
    std::vector<std::pair<Cfg,Cfg>> GetInteractionPositionPairs();

    /// Gets the paths of robots at each of the IT locations
    std::vector<std::vector<Cfg>> GetInteractionPaths();

    /// Get the final position of robot of input capability.
    std::vector<Cfg> GetInteractionPosition(std::string _capability);

    /// Get the paths of robot of input capability.
    std::vector<std::vector<Cfg>> GetInteractionPath(std::string _capability);

    /// Get if paths are saved
    /// @return True if paths are saved
    bool SavedPaths();

    /// Gets the environment to plan the interaction in.
    Environment* GetInteractionEnvironment();

    ///@}

  protected:

    MPProblem* m_problem{nullptr}; ///< The handoff template problem.

    ///The set of tasks that must be performed to handoff.
    std::vector<std::shared_ptr<MPTask>> m_tasks;

    /// The list of tasks stored by type
		std::unordered_map<std::string,std::vector<std::shared_ptr<MPTask>>> m_taskType;

    /// The handoff label
    std::string m_label;

    /// The number of attempts to try and place the template in the environment.
    size_t m_maxAttempts;

    /// The weight of the edge between interaction cfgs
    double m_interactionWeight{0};

    /// The locations for manually placed handoffs
    std::vector<Cfg> m_handoffLocations;

    /// Environment to plan the interaction in
    std::unique_ptr<Environment> m_interactionEnvironment;

    /// Indicates if the interaction template should save the entire paths of the
    /// interaction or just the final configurations.
    bool m_savePaths;
};
#endif
