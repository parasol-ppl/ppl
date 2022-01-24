#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ConfigurationSpace/Cfg.h"
//class Cfg;
class Decomposition;
class DynamicObstacle;
class InteractionInformation;
class Environment;
class MPTask;
class GroupTask;
class MultiBody;
class Robot;
class RobotGroup;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Representation of a motion planning problem, including an environment,
/// tasks, and robots.
////////////////////////////////////////////////////////////////////////////////
#ifdef _PARALLEL
class MPProblem : public stapl::p_object
#else
class MPProblem 
#endif
{

  public:

    ///@name Construction
    ///@{

    /// Instantiate an empty MPProblem.
    MPProblem();

    /// Instantiate an MPProblem from an XML file.
    /// @param _filename The name of the XML file.
    explicit MPProblem(const std::string& _filename);

    MPProblem(const MPProblem& _other); ///< Copy.
    MPProblem(MPProblem&& _other) = delete;

    virtual ~MPProblem();

    ///@}
    ///@name Assignment
    ///@{

    MPProblem& operator=(const MPProblem& _other); ///< Copy.
    MPProblem& operator=(MPProblem&& _other) = delete;

    ///@}
    ///@name XML File Parsing
    ///@{

    /// Get the XML filename from which this object was parsed.
    const std::string& GetXMLFilename() const;

    /// Read an XML file.
    /// @param _filename The XML file name.
    void ReadXMLFile(const std::string& _filename);

    ///@}
    ///@name Environment Accessors
    ///@{

    /// Get the environment object.
    Environment* GetEnvironment();

    /// Set the environment object.
    /// @param _e Environment to set to
    void SetEnvironment(std::unique_ptr<Environment>&& _e);

    ///@}
    ///@name Robot Accessors
    ///@{

    /// Get the number of robots in our problem.
    size_t NumRobots() const noexcept;

    /// Get a specific robot by index.
    /// @param _index Index of robot to get
    Robot* GetRobot(const size_t _index) const noexcept;

    /// Get a specific robot by label.
    /// @param _label Label of robot to get
    Robot* GetRobot(const std::string& _label) const noexcept;

    /// Get all robots in this problem.
    const std::vector<std::unique_ptr<Robot>>& GetRobots() const noexcept;

    /// Get all robots of a specified type.
    /// @param _type Type of robots to retrieve
    /// @return List of robots of type _type
    const std::vector<Robot*> GetRobotsOfType(std::string _type) const noexcept;

    /// Group versions:
    /// Get the number of robot groups in our problem.
    size_t NumRobotGroups() const noexcept;

    /// Get a specific robot group by index.
    /// @param _index Index of robot group to retrieve
    RobotGroup* GetRobotGroup(const size_t _index) const noexcept;

    /// Get a specific robot group by group's label.
    /// @param _label Label of robot group to retrieve
    RobotGroup* GetRobotGroup(const std::string& _label) const noexcept;

    /// Get all robot groups in this problem.
    /// @return List of robot groups
    const std::vector<std::unique_ptr<RobotGroup>>& GetRobotGroups() const noexcept;

    /// Get initial configuration of a robot
    /// @param _r Robot whose configuration to retrieve
		Cfg GetInitialCfg(Robot* _r);

    /// Get initial configuration of a robot
    /// @param _r Robot whose configuration to retrieve
    /// @param _cfg Configuration to set for _r
		void SetInitialCfg(Robot* _r, Cfg _cfg);

    ///@}
    ///@name Task Accessors
    ///@{

    /// Get task by label
    /// @param _label Label of task to retrieve
    MPTask* GetTask(std::string _label);

    /// Get the unfinished tasks currently assigned to a given robot.
    /// @param _robot The robot to retrieve tasks for.
    /// @return The set of tasks currently assigned to _robot.
    std::vector<std::shared_ptr<MPTask>> GetTasks(Robot* const _robot) const
        noexcept;

    /// Group overload
    /// Get the unfinished tasks currently assigned to a group of robots.
    /// @param _group The robot group to retrieve tasks for.
    /// @return The set of group tasks currently assigned to _group.
    std::vector<std::shared_ptr<GroupTask>> GetTasks(RobotGroup* const _group)
        const noexcept;

    /// Add a task to the problem. The assigned robot will be taken from the
    /// task object.
    /// @param _task The new task.
    void AddTask(std::unique_ptr<MPTask>&& _task);

    /// Reassign a task to another robot.
    /// @param _task The task to reassign.
    /// @param _newOwner The new robot assigned to _task.
    void ReassignTask(MPTask* const _task, Robot* const _newOwner);

    /// Assign task decomposition to a robot
    /// @param _coordinator Robot to assign task to
    /// @param _decomp The decomposition task to assign
		void AddDecomposition(Robot* _coordinator, std::unique_ptr<Decomposition>&& _decomp);

    /// Get set of decomposition tasks for given robot
    /// @param _coordinator Robot to return task decompositions for
		const std::vector<std::unique_ptr<Decomposition>>& GetDecompositions(Robot* _coordinator);

    /// Get map of robots to decompositions
		const std::unordered_map<Robot*,std::vector<std::unique_ptr<Decomposition>>>& 
														GetDecompositions();

    ///@}
    ///@name Dynamic Obstacle Accessors
    ///@{

    /// Get all of the dynamic obstacles in this problem.
    const std::vector<DynamicObstacle>& GetDynamicObstacles() const noexcept;

    /// Add dynamic obstacle to this problem
    /// @param Dynamic obstacle to add
    void AddDynamicObstacle(DynamicObstacle&& _obstacle);

    /// Remove all of the dynamic obstacles in this problem.
    void ClearDynamicObstacles();

    ///@}
    ///@name Debugging
    ///@{

    /// Print the environment, robot, and task information.
    /// @param _os Stream to print to
    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name File Path Accessors
    ///@{

    /// Get the base filename for output files.
    const std::string& GetBaseFilename() const;

    /// Set the base filename for output files.
    /// @param _s New base filename
    void SetBaseFilename(const std::string& _s);

    /// Get the base path for input files to a file name.
    /// @param _filename The filename to modify.
    /// @return The base path + filename, or just the path if no name is given.
    std::string GetPath(const std::string& _filename = "");

    /// Set the base path for input files.
    /// @param _fileName File path to set
    void SetPath(const std::string& _filename);

    ///@}

    ///@name Handoff Template Accessors
    ///@{

    /// Return the list of handoff templates defined in the problem
    /// @return vector of handoff templates
    std::vector<std::unique_ptr<InteractionInformation>>&
        GetInteractionInformations();

    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    /// Helper for parsing XML nodes of various types.
    /// @param _node The child node to be parsed.
    void ParseChild(XMLNode& _node);

    /// Create a pseudo-point robot.
    void MakePointRobot();

    ///@}
    ///@name Core Properties
    ///@{

    std::unique_ptr<Environment> m_environment;    ///< The planning environment.

    std::vector<std::unique_ptr<Robot>> m_robots;  ///< The robots in our problem.
    std::vector<std::unique_ptr<RobotGroup>> m_robotGroups; ///< Robot groups.
    std::unique_ptr<Robot> m_pointRobot;           ///< A pseudo point-robot.
  
    /// Map of robot type to set of robots.
    std::unordered_map<std::string,std::vector<Robot*>> m_robotCapabilityMap;

		std::unordered_map<Robot*,Cfg> m_initialCfgs;  ///< Map of robot initial locations.

    /// The dynamic obstacles in our problem.
    std::vector<DynamicObstacle> m_dynamicObstacles;

    /// All handoff templates for a problem.
    std::vector<std::unique_ptr<InteractionInformation>> m_interactionInformations;

    /// Map the tasks assigned to each robot.
    std::unordered_map<Robot*, std::list<std::shared_ptr<MPTask>>> m_taskMap;
    /// Map the group tasks assigned to each robot group.
    std::unordered_map<RobotGroup*, std::list<std::shared_ptr<GroupTask>>>
        m_groupTaskMap;

    /// Map task labels to tasks.
    std::unordered_map<std::string, MPTask*> m_taskLabelMap;
    /// Map robots to task decompositions
		std::unordered_map<Robot*,std::vector<std::unique_ptr<Decomposition>>> m_taskDecompositions;

    ///@}
    ///@name Files
    ///@{

    std::string m_xmlFilename;   ///< The XML file name.
    std::string m_baseFilename;  ///< The base name for output files.
    std::string m_filePath;      ///< The relative path for the problem XML.

    ///@}

};

#endif
