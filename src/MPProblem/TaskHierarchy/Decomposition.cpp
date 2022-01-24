#include "Decomposition.h"

#include "MPProblem/MPProblem.h"
#include "Utilities/XMLNode.h"

/*---------------------------- Construction ---------------------------*/
Decomposition::
Decomposition() {}

Decomposition::
Decomposition(std::shared_ptr<SemanticTask> _mainTask) {
	m_rootTask = _mainTask.get();
}
	
Decomposition::
Decomposition(XMLNode& _node, MPProblem* _problem) {
	
	std::string mainTask = _node.Read("taskLabel", true, "",
							"Label for the highest level semantic task.");

	m_label = _node.Read("label", true, "", 
							"Unique label for the decomposition");

	std::string coordinator = _node.Read("coordinator", true, "",
							"Indicates which robot is responsible for coordinating this decompostion.");

	m_coordinator = _problem->GetRobot(coordinator);

	// create all the semantic tasks
	for(auto& child : _node) {
		if(child.Name() == "SemanticTask") {
			auto task = std::shared_ptr<SemanticTask>(new SemanticTask(_problem,child,this));
			if(m_taskMap[task->GetLabel()])
				throw RunTimeException(WHERE, "SemanticTask labels must be unique within a decomposition.");
			m_taskMap[task->GetLabel()] = task;
			if(task->GetMotionTask())
				m_motionTasks.push_back(task.get());
		}
	}

	// set the highest level task
	m_rootTask = m_taskMap[mainTask].get();

  // Compute subtask flow
  m_subtaskFlow = std::unique_ptr<SubtaskFlow>(new SubtaskFlow(m_rootTask));
}

Decomposition::
~Decomposition() {}

	
/*---------------------------- Accessors ---------------------------*/
const std::string
Decomposition::
GetLabel() const {
	return m_label;
}

Robot*
Decomposition::
GetCoordinator() const {
	return m_coordinator;
}

void
Decomposition::
SetCoordinator(Robot* _robot) {
	m_coordinator = _robot;
}

SemanticTask*
Decomposition::
GetRootTask() {
	return m_rootTask;
}

void
Decomposition::
SetRootTask(SemanticTask* _task) {
	m_rootTask = _task;
}

void 
Decomposition::
AddTask(std::shared_ptr<SemanticTask> _task) {
	m_taskMap[_task->GetLabel()] = _task;
	if(_task->GetMotionTask())
		m_motionTasks.push_back(_task.get());
	if(_task->GetGroupMotionTask())
		m_groupMotionTasks.push_back(_task.get());
}

SemanticTask* 
Decomposition::
GetTask(std::string _label) {
	auto iter = m_taskMap.find(_label);
		if(iter == m_taskMap.end())
			return nullptr;
			/*throw RunTimeException(WHERE) << "Tried to access non-existent semantic task ("
																		<< _label
																		<< ") in decomposition." 
																		<< std::endl;
			*/
	return m_taskMap[_label].get();
}

std::vector<SemanticTask*>&
Decomposition::
GetMotionTasks() {
	return m_motionTasks;
}

std::vector<SemanticTask*>&
Decomposition::
GetGroupMotionTasks() {
	return m_groupMotionTasks;
}

void
Decomposition::
AddMotionTask(SemanticTask* _task) {
	m_motionTasks.push_back(_task);
}

void
Decomposition::
AddGroupMotionTask(SemanticTask* _task) {
	m_groupMotionTasks.push_back(_task);
}
		
const std::unordered_map<std::string,std::shared_ptr<SemanticTask>>&
Decomposition::
GetTaskMap() const {
	return m_taskMap;
}

/*--------------------------------- Helper Functions ------------------*/

void
Decomposition::
ParseTask(XMLNode& _node, MPProblem* _problem) {
	std::string label = _node.Read("label", true, "", "Label for semantic task.");
	auto task = m_taskMap[label];

	std::string parentLabel = _node.Read("parent", false, "", "Label for task's parent.");
	auto parent = m_taskMap[parentLabel];
	if(parent) {
		task->SetParent(parent.get());
		parent->AddSubtask(task.get());
	}

	for(auto child : _node) {
		if(child.Name() == "Dependency") {
			std::string depTaskLabel = child.Read("task", true, "",
													"Label for semantic task.");
			auto depTask = m_taskMap[depTaskLabel];

			std::string dependencyType = child.Read("type", true, "",
													"Type of dependency");
			std::transform(dependencyType.begin(), dependencyType.end(), dependencyType.begin(), ::tolower);

			SemanticTask::DependencyType type = SemanticTask::DependencyType::None;

			if(dependencyType == "initiation")
				type = SemanticTask::DependencyType::Initiation;
			else if(dependencyType == "completion")
				type = SemanticTask::DependencyType::Completion;
			else if(dependencyType == "asynchronous")
				type = SemanticTask::DependencyType::Asynchronous;
			else if(dependencyType == "synchronous")
				type = SemanticTask::DependencyType::Synchronous;
			else 
				throw RunTimeException(WHERE, "Unknown dependency type: " + dependencyType);
			task->AddDependency(depTask.get(),type);
		}
		//else if(child.Name() == "Task") {
		//	std::shared_ptr<MPTask> simpleTask = std::shared_ptr<MPTask>(new MPTask(_problem,grandchild));
		//	task->SetMotionTask(simpleTask);
		//	AddSimpleTask(task.get());
		//}
	}
}
    
void
Decomposition::
SetComplete(bool _complete) {
  m_complete = _complete;
}
    
bool
Decomposition::
IsComplete() {
  return m_complete;
}
