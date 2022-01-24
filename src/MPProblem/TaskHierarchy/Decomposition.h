#ifndef PPL_DECOMPOSITION_H_
#define PPL_DECOMPOSITION_H_

#include "SemanticTask.h"
#include "SubtaskFlow.h"

class MPProblem;

class Decomposition {

	public:

		///@name Construction
		///@{

		Decomposition();

    /// @param _mainTask The root task in the decomposition tree.
		Decomposition(std::shared_ptr<SemanticTask> _mainTask);
	
    /// @param _node The xml node containing the decomposition 
    ///        information.
    /// @param _problem The parent MPProblem that this decomposition
    ///        belongs to.
		Decomposition(XMLNode& _node, MPProblem* _problem);

		~Decomposition();

		///@}
		///@name Accessors
		///@{
		
    /// Return the label for the decomposition.
		const std::string GetLabel() const;

    /// Return the robot responsible for the decomposition.
		Robot* GetCoordinator() const;

    /// Set the coordinator responsible for the decomposition.
    /// @param _robot The new coordinator responsible for the
    ///        decomposition.
		void SetCoordinator(Robot* _robot);
	
    /// Return the root task in the decomposition tree.
		SemanticTask* GetRootTask();
    
    /// Set the root task for the decomposition tree.
    /// @param _task The new root task in the decomp tree.
		void SetRootTask(SemanticTask* _task);

    /// Add a new task to the decomposition.
    /// @param _task New task to add to decomposition.
		void AddTask(std::shared_ptr<SemanticTask> _task);

    /// Return the named task from the decomposition.
    /// @param _label Label of desired task.
		SemanticTask* GetTask(std::string _label);

    /// Return the set of tasks with corresponding motion tasks.
		std::vector<SemanticTask*>& GetMotionTasks();

    /// Return the set of tasks with corresponding group motion
    /// tasks.
		std::vector<SemanticTask*>& GetGroupMotionTasks();

    /// Add a semantic task to the set of tasks with motion tasks.
    /// @param _task Task with corresponding motion task.
		void AddMotionTask(SemanticTask* _task);

    /// Add a semantic task to the set of tasks with group motion 
    /// tasks.
    /// @param _task Task with corresponding group motion task.
		void AddGroupMotionTask(SemanticTask* _task);

    /// Return the map of all semantic tasks in the decomposition.
		const std::unordered_map<std::string,std::shared_ptr<
                               SemanticTask>>& GetTaskMap() const;

    /// Marks the decomposition as (in)complete.
    /// @param _complete Flag indicating complete status.
    void SetComplete(bool _complete=true);

    /// Checks if the decompsition has been marked complete.
    bool IsComplete();

		///@}
		
	private:
		///@name Helper Functions
		///@{

    /// Parse an individual task node within the decomposition/
    /// @param _node XML node for the task.
    /// @param _problem Owning MPProblem.
		void ParseTask(XMLNode& _node, MPProblem* _problem);

		///@}
		///@name Internal State
		///@{

    /// Map of label to task containing all semantic tasks within
    /// the decomposition.
		std::unordered_map<std::string,std::shared_ptr<
                        SemanticTask>> m_taskMap;

		/// Keeps track of the tasks with motion tasks.
		std::vector<SemanticTask*> m_motionTasks;

		/// Keeps track of the tasks with group motion tasks.
		std::vector<SemanticTask*> m_groupMotionTasks;

		///< Root node in the decomposition tree.
		SemanticTask*	m_rootTask;

    /// Label for referencing the decomposition.
		std::string m_label;

    /// Robot responsible for the decomposition.
		Robot* m_coordinator;

    /// Flag indiciating if decomposition has been completed.
    bool m_complete{false};

    /// Subtask flow containing dependency flows in tree.
    std::unique_ptr<SubtaskFlow> m_subtaskFlow;

		///@}
};
#endif
