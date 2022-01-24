#ifndef PPL_SEMANTIC_TASK_H_
#define PPL_SEMANTIC_TASK_H_

#include "MPProblem/GroupTask.h"
#include "MPProblem/MPTask.h"

#include <unordered_map>
#include <unordered_set>

class Decomposition;
class MPProblem;

class SemanticTask {

	public:
	
		///@name Local Types
		///@{

		enum SubtaskRelation {
			XOR, /// Any subtask can be complete to satisfy this task.
			AND /// All subtasks must be complete to satisfy this task.
		};

		enum DependencyType {
			Completion, /// Task A must be complete to start task B.
			Initiation, /// Task A must be started to start task B.
			Synchronous, /// Task A and task B must be start simultaneously.
			Asynchronous, /// TODO::Might be the same as none?
			None
		};

		typedef std::unordered_map<DependencyType, std::unordered_set<SemanticTask*>,
															 std::hash<int>> DependencyMap;

		///@}
		///@name Construction
		///@{

		SemanticTask();

    /// @param _problem Owning MPProblem.
    /// @param _node XML node containing task info.
    /// @param _decomp Owning decomposition.
		SemanticTask(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp);

    /// @param _label Label for referencing task.
    /// @param _parent Parent task in decomposition tree.
    /// @param _decomp Owning decomposition.
    /// @param _relation Relationship between subtasks.
    /// @param _decomposable Flag indicating if this task can be further decomposed.
    /// @param _fixedAssign Flag indicating if this task has a fixed assignment.
    /// @param _motionTask Corresponding motion task for completing this task.
		SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
								 SubtaskRelation _relation, bool _decomposable, bool _fixedAssign, 
								 std::shared_ptr<MPTask> _motionTask = nullptr);

    /// @param _label Label for referencing task.
    /// @param _parent Parent task in decomposition tree.
    /// @param _decomp Owning decomposition.
    /// @param _relation Relationship between subtasks.
    /// @param _decomposable Flag indicating if this task can be further decomposed.
    /// @param _fixedAssign Flag indicating if this task has a fixed assignment.
    /// @param _groupMotionTask Corresponding group motion task for completing this
    ///                         task.
		SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
								 SubtaskRelation _relation, bool _decomposable, bool _fixedAssign,
								 std::shared_ptr<GroupTask> _groupMotionTask);

    /// @param _parent Parent task in decomposition tree.
    /// @param _decomp Owning decomposition.
    /// @param _motionTask Corresponding motion task for completing this task.
    /// @param _decomposable Flag indicating if this task can be further decomposed.
		SemanticTask(SemanticTask* _parent, Decomposition* _decomp, 
                 std::shared_ptr<MPTask> _motionTask, bool _decomposable=true);

    /// @param _parent Parent task in decomposition tree.
    /// @param _decomp Owning decomposition.
    /// @param _groupMotionTask Corresponding group motion task for completing this
    ///                         task.
    /// @param _decomposable Flag indicating if this task can be further decomposed.
		SemanticTask(SemanticTask* _parent, Decomposition* _decomp,
                 std::shared_ptr<GroupTask> _groupMotionTask, bool _decomposable=true);

    /// @param _label Label for referencing task.
    /// @param _parent Parent task in decomposition tree.
    /// @param _decomp Owning decomposition.
    /// @param _motionTask Corresponding motion task for completing this task.
    /// @param _decomposable Flag indicating if this task can be further decomposed.
		SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
								 std::shared_ptr<MPTask> _motionTask = nullptr, bool _decomposable=true);

    /// @param _label Label for referencing task.
    /// @param _parent Parent task in decomposition tree.
    /// @param _decomp Owning decomposition.
    /// @param _groupMotionTask Corresponding group motion task for completing this
    ///                         task.
    /// @param _decomposable Flag indicating if this task can be further decomposed.
		SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
								 std::shared_ptr<GroupTask> _groupMotionTask, bool _decomposable=true);

		~SemanticTask();

		///@}
		///@name Accessors
		///@{

    /// Return referencing label for this task.
		std::string GetLabel() const;

		/// Sets the dependencies of all the semantic tasks below this in the hierarchy,
		std::vector<SemanticTask*> SetDependencies();

		/// Indicates if this task can be decomposed.
		bool IsDecomposable();

    /// Set the motion task corresponding to this task.
    /// @param _motion New corresponding motion task.
		void SetMotionTask(std::shared_ptr<MPTask> _motion);

    /// Return the corresponding motion task.
		std::shared_ptr<MPTask> GetMotionTask();

    /// Set the group motion task corresponding to this task.
    /// @param _motion New corresponding group motion task.
		void SetGroupMotionTask(std::shared_ptr<GroupTask> _motion);

    /// Return the corresponding group motion task.
		std::shared_ptr<GroupTask> GetGroupMotionTask();

    /// Return the parent of this task in the decomposition tree.
		SemanticTask* GetParent();

    /// Set the parent of this task within the decomposition tree.
    /// @param _parent Parent task.
		void SetParent(SemanticTask* _parent);

    /// Add new dependency on another task.
    /// @param _task The task this one is now dependent on.
    /// @param _type The type of dependenc relation between this and _task.
		std::unordered_set<SemanticTask*> AddDependency(SemanticTask* _task, DependencyType _type);

    /// Return the map of all dependencies this task has.
		DependencyMap& GetDependencies();

    /// Return flag for fixed assignment status.
		bool IsFixedAssignment();

    /// Add an additional subtask.
    /// _task The new subtask.
		void AddSubtask(SemanticTask* _task);

    /// Return the set of all subtasks.
		std::vector<SemanticTask*> GetSubtasks();

    /// Return the relationship between this task's subtasks.
		SubtaskRelation GetSubtaskRelation();

		///@}

	private:
		///@name Helper Functions
		///@{

    /// Read the dependencies from other tasks in the XML node.
    /// @param _problem Owning MPProblem.
    /// @param _node XML node to parse.
    /// @param _decomp Owning decomposition.
		void ParseDependency(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp);

		///@}
		///@name Internal State
		///@{

		/// Label distinguishing this semantic task
		std::string m_label;

		/// Parent SemanticTask that includes this task in its decomposition
		SemanticTask* m_parent{nullptr};

		/// If this SemanticTask is a simple task, this holds the corresponding motion task
		std::shared_ptr<MPTask>	m_motionTask;

		/// If this SemanticTask is a simple task, this holds the corresponding motion task
		std::shared_ptr<GroupTask>	m_groupMotionTask;

		/// Indicates if the assignment of the simple task is fixed
		bool m_fixedAssignment{false};

		/// Indicates if the task can be decomposed into subtasks
		bool m_decomposable{true};

		/// Set of subtasks that make up this task's decomposition
		std::vector<SemanticTask*> m_subtasks;

		/// Relationship between the subtasks indicating if they're alternatives or all required
		SubtaskRelation m_subtasksRelation;

		/// Keeps track of all the semantic tasks for each dependency type
		DependencyMap	m_dependencyMap;
		
		///@}
};

#endif
