#include "InteractionInformation.h"
#include "MPProblem/Environment/Environment.h"

/*------------------------------ Construction --------------------------------*/

InteractionInformation::
InteractionInformation(MPProblem* _problem, XMLNode& _node) : m_problem(_problem){
  // Parse the tasks within the Handoff Template

  m_label = _node.Read("label", true, "", "Label for the handoff template.");

  m_maxAttempts = _node.Read("maxAttempts", true, 0, 0, MAX_INT,
      "The number of attempts made to place the template in the real environment");

  m_interactionWeight = _node.Read("interactionWeight", false, 0, -9999, 9999,
      "The weight of the connecting edge between interaction robot configurations");

  m_savePaths = _node.Read("savePaths", false, false,
      "Indicates if the handoff requires explicit paths");

  for(auto& child : _node) {
    if(child.Name() == "Receiving") {
      m_tasks.emplace_back(new MPTask(m_problem, *child.begin()));
			m_taskType["receiving"].push_back(m_tasks.back());
    }
		else if(child.Name() == "Delivering"){
      m_tasks.emplace_back(new MPTask(m_problem, *child.begin()));
			m_taskType["delivering"].push_back(m_tasks.back());
		}
		else if(child.Name() == "Auxiliary"){
			throw RunTimeException(WHERE, "Auxiliary interactions are not yet supported.");
      m_tasks.emplace_back(new MPTask(m_problem, *child.begin()));
			m_taskType["auxiliary"].push_back(m_tasks.back());
		}
    else if(child.Name() == "Location"){
      const std::string pointString = child.Read("point", false, "",
          "The center point of the handoff template");
      if(!m_tasks[0])
        throw RunTimeException(WHERE, "Must declare the handoff task components before the locations");

      Cfg point(m_tasks[0]->GetRobot());
#ifdef VIZMO_MAP
      std::istringstream pointStream("0 " + pointString);
#else
      std::istringstream pointStream(pointString);
#endif
      point.Read(pointStream);

      m_handoffLocations.push_back(point);

    }
    else if(child.Name() == "InteractionEnvironment"){
      if(!m_interactionEnvironment){
        m_interactionEnvironment = std::unique_ptr<Environment>(new Environment(child));
      }
    }
  }
}


/*------------------------------ Accessors --------------------------------*/

std::string
InteractionInformation::
GetLabel() const {
  return m_label;
}

size_t
InteractionInformation::
GetMaxAttempts() const {
  return m_maxAttempts;
}

MPProblem*
InteractionInformation::
GetMPProblem() const{
  return m_problem;
}

std::vector<std::shared_ptr<MPTask>>&
InteractionInformation::
GetInteractionTasks(){
  return m_tasks;
}

std::vector<std::shared_ptr<MPTask>>&
InteractionInformation::
GetTypeTasks(const std::string& _s){
	if(_s != "receiving" and _s != "delivering" and _s != "auxiliary")
		throw RunTimeException(WHERE, _s + " is not a recognized interaction task type.");
  return m_taskType[_s];
}

double
InteractionInformation::
GetInteractionWeight() const{
  return m_interactionWeight;
}

std::vector<Cfg>&
InteractionInformation::
GetTemplateLocations(){
  return m_handoffLocations;
}

void
InteractionInformation::
AddTemplateLocation(Cfg _location){
  m_handoffLocations.push_back(_location);
}

bool
InteractionInformation::
SavedPaths(){
  return m_savePaths;
}

Environment*
InteractionInformation::
GetInteractionEnvironment(){
  return m_interactionEnvironment.get();
}
