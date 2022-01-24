#include "SemanticTask.h"

#include "Decomposition.h"
#include "MPProblem/MPProblem.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

/*---------------------------- Construction -----------------------*/

SemanticTask::
SemanticTask() {}

SemanticTask::
SemanticTask(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp) {

	m_label = _node.Read("label", true, "", "Unique label to identify this task.");

	std::string relation = _node.Read("subtaskRelation",false,"",
										"Indicates if this task's subtasks are alternatives or both required.");

	std::transform(relation.begin(), relation.end(), relation.begin(), ::tolower);

	if(relation == "and")
		m_subtasksRelation = SubtaskRelation::AND;
	else if(relation == "xor" or relation == "or")
		m_subtasksRelation = SubtaskRelation::XOR;
	

	m_decomposable = _node.Read("decomposable", false, true, 
				"Indicates if a task is decomposable into subtasks.");

	m_fixedAssignment = _node.Read("fixedAllocation", false, false,
				"Indicates if the simple tasks within this have fixed allocations.");

	std::string parentLabel = _node.Read("parent", false, "", "Label for task's parent.");
	SemanticTask* parent = (parentLabel=="") ? nullptr : _decomp->GetTask(parentLabel);
	if(parent) {
		this->SetParent(parent);
		parent->AddSubtask(this);
	}

	for(auto& child : _node) {
		if(child.Name() == "Task") {
			m_motionTask = std::shared_ptr<MPTask>(new MPTask(_problem,child));
			_decomp->AddMotionTask(this);
		}
		if(child.Name() == "GroupTask") {
			m_groupMotionTask = std::shared_ptr<GroupTask>(new GroupTask(_problem,child));;
			_decomp->AddGroupMotionTask(this);
		}
		else if(child.Name() == "Dependency")
			ParseDependency(_problem, _node, _decomp);
	}

}
		
SemanticTask::
SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
								SubtaskRelation _relation, bool _decomposable, bool _fixed,
								std::shared_ptr<MPTask> _motionTask) :
								m_label(_label),
								m_parent(_parent),
								m_subtasksRelation(_relation) {

	m_decomposable = _decomposable;
	m_fixedAssignment = _fixed;
	m_motionTask = _motionTask;

	if(m_parent) {
		m_parent->AddSubtask(this);
	}
	if(_decomp and _motionTask.get()) {
		_decomp->AddMotionTask(this);
	}
}

SemanticTask::
SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
								SubtaskRelation _relation, bool _decomposable, bool _fixed,
								std::shared_ptr<GroupTask> _groupMotionTask) :
								m_label(_label),
								m_parent(_parent),
								m_subtasksRelation(_relation) {

	m_decomposable = _decomposable;
	m_fixedAssignment = _fixed;
	m_groupMotionTask = _groupMotionTask;

	if(m_parent) {
		m_parent->AddSubtask(this);
	}
	if(_decomp and _groupMotionTask.get()) {
		_decomp->AddGroupMotionTask(this);
	}
}

SemanticTask::
SemanticTask(SemanticTask* _parent, Decomposition* _decomp,
							std::shared_ptr<MPTask> _motionTask, bool _decomposable) :
							m_parent(_parent),
							m_motionTask(_motionTask),
							m_decomposable(_decomposable) {

	if(m_parent) {
		m_parent->AddSubtask(this);
		m_label = _parent->GetLabel() + "_" + std::to_string(_parent->GetSubtasks().size());
	}
	if(_decomp) {
		_decomp->AddMotionTask(this);
	}
}

SemanticTask::
SemanticTask(SemanticTask* _parent, Decomposition* _decomp,
							std::shared_ptr<GroupTask> _groupMotionTask, bool _decomposable) :
							m_parent(_parent),
							m_groupMotionTask(_groupMotionTask), 
						 	m_decomposable(_decomposable) {

	if(m_parent) {
		m_parent->AddSubtask(this);
		m_label = _parent->GetLabel() + "_" + std::to_string(_parent->GetSubtasks().size());
	}
	if(_decomp) {
		_decomp->AddGroupMotionTask(this);
	}
}

SemanticTask::
SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
							std::shared_ptr<MPTask> _motionTask, bool _decomposable) :
							m_label(_label),
							m_parent(_parent),
							m_motionTask(_motionTask),
						 	m_decomposable(_decomposable) {

	if(m_parent) {
		m_parent->AddSubtask(this);
		if(_label == "")
			m_label = _parent->GetLabel() + "_" + std::to_string(_parent->GetSubtasks().size());
	}
	if(_decomp) {
		_decomp->AddMotionTask(this);
	}
}

SemanticTask::
SemanticTask(std::string _label, SemanticTask* _parent, Decomposition* _decomp,
						 std::shared_ptr<GroupTask> _groupMotionTask, bool _decomposable) :
						 m_label(_label), m_parent(_parent), m_groupMotionTask(_groupMotionTask),
						 m_decomposable(_decomposable) {

	if(m_parent) {
		m_parent->AddSubtask(this);
		if(_label == "")
			m_label = _parent->GetLabel() + "_" + std::to_string(_parent->GetSubtasks().size());
	}
	if(_decomp) {
		_decomp->AddGroupMotionTask(this);
	}
}

SemanticTask::
~SemanticTask() {}
	
void
SemanticTask::
ParseDependency(MPProblem* _problem, XMLNode& _node, Decomposition* _decomp) {

	for(auto& child : _node) {
		if(child.Name() == "Dependency") {
			std::string depTaskLabel = child.Read("task", true, "",
													"Label for semantic task.");
			auto depTask = _decomp->GetTask(depTaskLabel);

			std::string dependencyType = child.Read("type", true, "",
													"Type of dependency");
			std::transform(dependencyType.begin(), dependencyType.end(), 
                     dependencyType.begin(), ::tolower);

			SemanticTask::DependencyType type = SemanticTask::DependencyType::None;

			if(dependencyType == "initiation")
				type = SemanticTask::DependencyType::Initiation;
			else if(dependencyType == "completion")
				type = SemanticTask::DependencyType::Completion;
			else if(dependencyType == "asynchronous")
				type = SemanticTask::DependencyType::Asynchronous;
			else if(dependencyType == "synchronous") {
				type = SemanticTask::DependencyType::Synchronous;
				depTask->AddDependency(this,type);
			}
			else 
				throw RunTimeException(WHERE, "Unknown dependency type: " + dependencyType);
			this->AddDependency(depTask,type);

		}
	}
}

/*---------------------------- Accessors -----------------------*/
	
std::string 
SemanticTask::
GetLabel() const {
	return m_label;
}

std::vector<SemanticTask*>
SemanticTask::
SetDependencies() {

	std::unordered_map<SemanticTask*,
							std::vector<SemanticTask*>> simpleTaskDecompositions;

	std::vector<SemanticTask*> simpleTasks;

	// Check if this is a motion task and, if so, add it to the set
  // of motion tasks for itself
	if(m_motionTask or m_groupMotionTask) {
		return {this};
	}

	// Collect all of the simple tasks composing the subtasks
	for(auto subtask : m_subtasks) {
		simpleTaskDecompositions[subtask] = subtask->SetDependencies();
		for(auto simple : simpleTaskDecompositions[subtask]){
			simpleTasks.push_back(simple);
		}
	}

	for(auto taskDecomp : simpleTaskDecompositions) {
		auto task = taskDecomp.first;
		for(auto dependencyTasks : task->GetDependencies()) {

			// iterate through dependent semantic tasks
			// make sure they all have THIS task as their parent
			// add dependencies from their set of simple tasks

			for(auto depTask : dependencyTasks.second) {
				if(depTask->GetParent() != this) {
					throw RunTimeException(WHERE,
						"Decomposition input has a listed dependency between non-sibling tasks.");
				}
				
				for(auto simple1 : simpleTaskDecompositions[task]) {
					for(auto simple2 : simpleTaskDecompositions[depTask]) {
						simple1->AddDependency(simple2,dependencyTasks.first);
					}
				}
			}
		}
	}

	return simpleTasks;
}

bool 
SemanticTask::
IsDecomposable() {
	return m_decomposable;
}

void
SemanticTask::
SetMotionTask(std::shared_ptr<MPTask> _motion) {
	m_motionTask = _motion;
}

std::shared_ptr<MPTask> 
SemanticTask::
GetMotionTask() {
	return m_motionTask;
}

void
SemanticTask::
SetGroupMotionTask(std::shared_ptr<GroupTask> _groupMotion) {
	m_groupMotionTask = _groupMotion;
}

std::shared_ptr<GroupTask>
SemanticTask::
GetGroupMotionTask() {
	return m_groupMotionTask;
}

SemanticTask*
SemanticTask::
GetParent() {
	return m_parent;
}

void
SemanticTask::
SetParent(SemanticTask* _parent) {
	m_parent = _parent;
}

std::unordered_set<SemanticTask*>
SemanticTask::
AddDependency(SemanticTask* _task, DependencyType _type) {
	m_dependencyMap[_type].insert(_task);

	// Return dependencies in case the caller needs to build dependency chains.
	return m_dependencyMap[_type];
}

std::unordered_map<SemanticTask::DependencyType,std::unordered_set<SemanticTask*>,std::hash<int>>&
SemanticTask::
GetDependencies() {
	return m_dependencyMap;
}

bool 
SemanticTask::
IsFixedAssignment() {
	return m_fixedAssignment;
}

void 
SemanticTask::
AddSubtask(SemanticTask* _task) {
	m_subtasks.push_back(_task);
}

std::vector<SemanticTask*>
SemanticTask::
GetSubtasks() {
	return m_subtasks;
}
		
SemanticTask::SubtaskRelation
SemanticTask::
GetSubtaskRelation() {
	return m_subtasksRelation;
}

/*		
void 
SemanticTask::
SetCompletionFunction(TBDFunction _func) {
	m_completionFunction = _func;
}

TBDFunction 
SemanticTask::
GetCompletionFunction() {
	return m_completionFunction;
}
*/
