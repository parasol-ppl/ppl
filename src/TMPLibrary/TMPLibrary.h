#ifndef PMPL_TMP_LIBRARY_H_
#define PMPL_TMP_LIBRARY_H_

#include "MPLibrary/PMPL.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#ifndef PPL_TEST_TRAITS_H_
  #include "Traits/CfgTraits.h"
#else
  #include "Traits/TestTraits.h"
#endif

#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"
#include "Utilities/TMPMethodSet.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <atomic>
#include <unordered_map>

//TMPLibrary TODOs::
//Save the ITs in a single location so they only need to be constructed once

class Decomposition;
class Plan;
class PoIPlacementMethod;
class StateGraph;
class TaskAllocatorMethod;
class TaskDecomposerMethod;
class TaskEvaluatorMethod;
class TMPStrategyMethod;
class TMPTools;
class Coordinator;
class Agent;
//template<typename TMPMethod>TMPMethodSet;

////////////////////////////////////////////////////////////////////////////////
/// A collection of TMP planning algorithms that can operate on a specific
/// MPProblem comprised of MPTasks.
////////////////////////////////////////////////////////////////////////////////

class TMPLibrary {
  public:
    ///@name Local Types
    ///@{

    ///@}
    /// Solver represents an input set to MPLibraryType. It includes an
    /// MPStrategy label, seed, base file name, and vizmo debug option.
    struct Solver {
      std::string label;         ///< The XML label for the strategy to use.
      std::string baseFilename;  ///< The base name for output files.
      bool vizmoDebug;           ///< Save vizmo debug info?
    };

    ///@}
    ///@name Method Set Types
    ///@{

    typedef TMPMethodSet<TMPStrategyMethod>        TMPStrategyMethodSet;
    typedef TMPMethodSet<PoIPlacementMethod>       PoIPlacementMethodSet;
    typedef TMPMethodSet<TaskEvaluatorMethod>      TaskEvaluatorMethodSet;
    typedef TMPMethodSet<TaskDecomposerMethod>     TaskDecomposerMethodSet;
    typedef TMPMethodSet<TaskAllocatorMethod>      TaskAllocatorMethodSet;
    typedef TMPMethodSet<StateGraph>               StateGraphSet;

    ///@}
    ///@name Method Pointer Types
    ///@{

    typedef typename TMPStrategyMethodSet::TMPMethodPointer     TMPStrategyMethodPointer;
    typedef typename PoIPlacementMethodSet::TMPMethodPointer    PoIPlacementMethodPointer;
    typedef typename TaskEvaluatorMethodSet::TMPMethodPointer   TaskEvaluatorMethodPointer;
    typedef typename TaskDecomposerMethodSet::TMPMethodPointer  TaskDecomposerMethodPointer;
    typedef typename TaskAllocatorMethodSet::TMPMethodPointer   TaskAllocatorMethodPointer;
    typedef typename StateGraphSet::TMPMethodPointer            StateGraphPointer;

    ///@}
    ///@name Construction
    ///@{

    TMPLibrary();

    TMPLibrary(const std::string& _filename);

    ~TMPLibrary();

    ///@}
    ///@name Configuration
    ///@{

    /// Read an XML file to set the algorithms and parameters in this instance.
    /// @param _filename The XML file name.
    void ReadXMLFile(const std::string& _filename);

    ///@}
    ///@name TMPStrategyMethod Accessors
    ///@{

    /// Get a TMPStrategyMethod method
    TMPStrategyMethodPointer GetTMPStrategy(const std::string& _l);

    /// Add a TMPStrategyMethod pointer
    /// @param TMPStrategyMethodPointer
    void AddTMPStrategy(TMPStrategyMethodPointer _sm, const std::string& _l);

    ///@}
    ///@name PoIPlacementMethod Accessors
    ///@{

    /// Get a Point-of-Interest placement method
    PoIPlacementMethodPointer GetPoIPlacementMethod(const std::string& _l);

    /// Add a Point-of-Interest placement method pointer
    /// @param PoIPlacementMethodPointer
    void AddPoIPlacementMethod(PoIPlacementMethodPointer _pm, const std::string& _l);

    ///@}
    ///@name TaskEvaluator Accessors
    ///@{

    /// Get a Task Evaluator
    TaskEvaluatorMethodPointer GetTaskEvaluator(const std::string& _l);

    /// Add a TaskEvalutorMethod pointer
    /// @param TaskEvaluatorMethodPointer
    void AddTaskEvaluator(TaskEvaluatorMethodPointer _te, const std::string& _l);

    ///@}
    ///@name Task Decomposition Accessors
    ///@{

    /// Get a TaskDecomposition
    TaskDecomposerMethodPointer GetTaskDecomposer(const std::string& _l);

    /// Add a TaskDecomposition pointer
    /// @param TaskDecomposerMethodPointer
    void AddTaskDecomposer(TaskDecomposerMethodPointer _td, const std::string& _l);

    ///@}
    ///@name Task ALlocator Accessors
    ///@{

    /// Get a TaskAllocator
    TaskAllocatorMethodPointer GetTaskAllocator(const std::string& _l);

    /// Add a TaskAlloctor pointer
    /// @param TaskAllocatorMethodPointer
    void AddTaskAllocator(TaskAllocatorMethodPointer _ta, const std::string& _l);

    ///@}
    ///@name TMPTool Accessors
    ///@{

    /// Get the TMP tool container
    TMPTools* GetTMPTools() {
      return m_tmpTools;
    }

    ///@}
    ///@name Input Accessors
    ///@{

    MPLibrary* GetMPLibrary() const noexcept;
    void SetMPLibrary(MPLibrary* _l) noexcept;
    MPProblem* GetMPProblem() const noexcept;
    void SetMPProblem(MPProblem* const _problem) noexcept;

    std::vector<std::shared_ptr<MPTask>>& GetTasks() noexcept;
    void AddTask(MPTask* const _task) noexcept;
    void AddTask(std::shared_ptr<MPTask> const _task) noexcept;
    void ClearTasks();

    std::vector<std::shared_ptr<GroupTask>> GetGroupTasks() const noexcept;
    void AddGroupTask(GroupTask* const _task) noexcept;
    void AddGroupTask(std::shared_ptr<GroupTask> const _task) noexcept;
    void ClearGroupTasks();

    const std::string& GetBaseFilename() const noexcept;
    void SetBaseFilename(const std::string& _s) noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    Plan* GetPlan();

    void SetPlan(Plan* _plan);

    StateGraphPointer GetStateGraph(const std::string& _l);

    void AddStateGraph(StateGraphPointer _sg, const std::string& _l);

    ///@}
    ///@name Execution
    ///@{

    void Solve(MPProblem* _problem, Decomposition* _decomp, Plan* _plan,
                Coordinator* _coordinator, std::vector<Robot*> _team);

    void InitializeMPProblem(MPProblem* _problem);

    ///@}
    ///@name Debug
    ///@{

    void Print(std::ostream&) const;

    ///@}

  private:
    ///@name Execution Helpers
    ///@{

    /// Execute a solver with the current problem, task, and solution.
    /// @param _s The solver to execute.
    void RunSolver(const Solver& _s);

    ///@}
    ///@name Construction Helpers
    ///@{

    /// Initialize all algorithms in each method set.
    void Initialize();

    /// Clear temporary states and variables.
    void Uninitialize();

    /// Helper for parsing XML nodes.
    /// @param _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Inputs
    ///@{

    MPLibrary*                              m_library;    ///< The underlying MPLibrary
    MPProblem*                              m_problem;    ///< The current MPProblem
    std::vector<std::shared_ptr<MPTask>>    m_tasks;      ///< Current set of tasks
    std::vector<std::shared_ptr<GroupTask>> m_groupTasks; ///< Current set of group tasks
    std::vector<Solver>                     m_solvers;    ///< Set of inputs to execute
    TMPTools*                               m_tmpTools;   ///< TMPTools container

    ///@}
    ///@name TMPMethod Sets
    ///@{
    /// Method sets hold and offer access to the tmp planning objects of the
    /// corresponding type.
    TMPStrategyMethodSet*     m_tmpStrategies;
    PoIPlacementMethodSet*    m_poiPlacementMethods;
    TaskEvaluatorMethodSet*   m_taskEvaluators;
    TaskDecomposerMethodSet*  m_taskDecomposers;
    TaskAllocatorMethodSet*   m_taskAllocators;
    StateGraphSet*            m_stateGraphs;

    ///@}
    ///@name Solution
    ///@{

    Plan* m_plan;

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
