#ifndef PMPL_TRP_TOOL_H_
#define PMPL_TRP_TOOL_H_

#include "ConfigurationSpace/Path.h"
#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/MPLibrary.h"
#include "MPProblem/TRPGoalMap.h"
#include "MPProblem/Robot/Robot.h"


////////////////////////////////////////////////////////////////////////////////
/// Evaluate a roadmap to find the optimal assignment of goals to a team of
/// robots. This solves the Multiple Depot, Multiple Traveling Salesman Problem
/// hereby refered to as the Traveling Robots Problem.
///
/// This implements the transformation of the TRP into a Atypical Traveling
/// Salesman Problem as described in the paper :
/// @todo insert paper name and authors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TRPTool : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPLibrary              MPLibrary;
    typedef typename MPTraits::Path                   Path;

    ///@}
    ///@name Local Types
    ///@{

    typedef TRPGoalMap<MPTraits>                      GoalMapType;
    typedef typename GoalMapType::vertex_descriptor   vertex_descriptor;
    typedef std::vector<std::vector<Path>>            PathSets;

    ///@}
    ///@name Construction
    ///@{

    TRPTool();

    TRPTool(XMLNode& _node);

    virtual ~TRPTool() = default;

    ///@}
    ///@name Problem Interface
    ///@{

    /// Provide problem input information
    /// @param _robot Main robot that holds the set of tasks to be performed by
    ///               the group.
    /// @param _workers The group of robots that will be performing the set of
    ///                 tasks.
    void Initialize(Robot* _robot, std::vector<Robot*> _workers);

    /// Perform TRP search on the problem
    /// @return Set of paths with each index corresponding to index of robot in
    /// _worker input in Initialize.
    PathSets Search();

    ///@}
    ///@name Setters
    ///@{

    /// Set the MPLibrary object to use
    /// @param _library The MPLibrary to use.
    void SetMPLibrary(MPLibrary* _library);

    ///@}

  private:

    ///@name helpers
    ///@{

    /// Adds the depots (robot locations) to the goal map.
    void AddDepots();

    /// Extracts the path set from the output of the LKH library.
    /// @param _pathVertices The indices of the paths taken by each robot in
    /// the solution.
    PathSets Extract(std::vector<std::vector<size_t>> _pathVertices);

    ///@}
    ///@name internal state
    ///@{

    MPLibrary* m_library{nullptr};
    std::string m_queryMethod;
    Robot* m_robot{nullptr};
    std::vector<Robot*> m_workers;
    GoalMapType m_goalMap;
    vector<vertex_descriptor> m_ATSPPath;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TRPTool<MPTraits>::
TRPTool() {
  this->SetName("TRPTool");
}


template <typename MPTraits>
TRPTool<MPTraits>::
TRPTool(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("TRPTool");

  m_queryMethod = _node.Read("queryLabel", true, "",
      "TRP Query Method for Goal Map");
}


/*------------------------------ Problem Interface -----------------------------*/

template <typename MPTraits>
void
TRPTool<MPTraits>::
Initialize(Robot* _robot, std::vector<Robot*> _workers) {
  m_robot = _robot;
  m_workers = _workers;
  m_goalMap = GoalMapType(_robot, m_queryMethod, m_library);
  AddDepots();
}


template <typename MPTraits>
typename TRPTool<MPTraits>::PathSets
TRPTool<MPTraits>::
Search(){
  auto lkh = m_library->GetMPTools()->GetLKHSearch("lkhSearch");
  std::vector<std::vector<size_t>> pathVertices = lkh->SearchTRP(&m_goalMap);
  return Extract(pathVertices);
}

/*--------------------------------- Setters -----------------------------------*/

template <typename MPTraits>
void
TRPTool<MPTraits>::
SetMPLibrary(MPLibrary* _library){
  m_library = _library;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
TRPTool<MPTraits>::
AddDepots(){
  m_goalMap.AddDepots(m_workers);
}


template <typename MPTraits>
typename TRPTool<MPTraits>::PathSets
TRPTool<MPTraits>::
Extract(std::vector<std::vector<size_t>> _pathVertices){
  //each index is the set of paths for the corresponding worker
  TRPTool<MPTraits>::PathSets pathSets;
  //vertex set is the goal map vertex descriptor representation of the path
  //for each worker as found by the LKH Library and transformation to/from ATSP
  for(auto vertexSet : _pathVertices){
    std::vector<typename MPTraits::Path> workerSet;
    for(size_t i = 0; i < vertexSet.size()-1; i++){
      workerSet.push_back(m_goalMap.GetPath(vertexSet[i], vertexSet[i+1]));
    }
    pathSets.push_back(workerSet);
  }
  return pathSets;
}

#endif
