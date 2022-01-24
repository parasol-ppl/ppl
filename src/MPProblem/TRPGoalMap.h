#ifndef PMPL_TRP_GOAL_MAP_H_
#define PMPL_TRP_GOAL_MAP_H_

#include "ConfigurationSpace/Path.h"
#include "Geometry/Boundaries/Boundary.h"
#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/BulletModel.h"
#include "Utilities/PMPLExceptions.h"

#include <containers/sequential/graph/graph.h>
#include "Vector.h"

#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// @todo What is this?
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TRPGoalMap : public stapl::sequential::graph<stapl::DIRECTED,
                          stapl::NONMULTIEDGES, std::shared_ptr<Constraint>,
                          PathType<MPTraits>>{

  public:

    ///@name Local Types
    ///@{

    typedef stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
        std::shared_ptr<Constraint>, PathType<MPTraits>>    GoalGraph;

    typedef typename GoalGraph::vertex_iterator                 iterator;
    typedef typename GoalGraph::const_vertex_iterator           const_iterator;

    typedef typename GoalGraph::edge_iterator                   edge_iterator;
    typedef typename GoalGraph::const_edge_iterator             const_edge_iterator;

    typedef typename GoalGraph::adj_edge_iterator               adj_edge_iterator;
    typedef typename GoalGraph::const_adj_edge_iterator         const_adj_edge_iterator;

    typedef typename GoalGraph::vertex_descriptor               vertex_descriptor;
    typedef typename GoalGraph::edge_descriptor                 edge_descriptor;

    typedef typename MPTraits::RoadmapType                      RoadmapType;
    typedef typename RoadmapType::VID                           VID;
    typedef typename MPTraits::Path                             Path;

    typedef typename MPTraits::MPLibrary                        MPLibrary;

    ///@}
    ///@name Construction
    ///@{

    TRPGoalMap();

    /// Construct TRPGoalMap
    /// @param _robot Robot coordinator assigned to all goals
    /// @param _queryMethod Query method for finding shortest paths
    /// @param _library Library fir calculating paths
    TRPGoalMap(Robot* _robot, std::string _queryMethod, MPLibrary* _library);

    virtual ~TRPGoalMap();

    ///@}
    ///@name Vertex Accessors
    ///@{

    /// Get the number of vertices contained in the goal map.
    /// @return Number of vertices.
    const size_t GetNumVertices() const noexcept;

    /// Get the set of descriptors for the goal vertices.
    /// @return Set of descriptors for the goal vertices.
    std::vector<size_t>* GetGoalDescriptors();

    /// Get the set of descriptors for the depot vertices (used in TRP).
    /// @return Set of descriptors for the depot vertices.
    std::vector<size_t>* GetDepotDescriptors();

    ///@}
    ///@name Path Accessors
    ///@{

    /// Get a particular path.
    /// @param _i1 Origin vertex descriptor.
    /// @param _i2 Destination vertex descriptor.
    /// @return Path with given origin and destination.
    const Path& GetPath(const size_t _i1, const size_t _i2) const
        noexcept;

    ///@}
    ///@name Problem Customization
    ///@{

    /// Add the depots needed for TRP->ATSP
    /// @param _workers Set of robots to use for depot locations.
    void AddDepots(std::vector<Robot*> _workers);

    ///@}
    ///@name Modifiers

    /// Write the goal graph to an output stream.
    void Print(std::ostream& _os) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Find the shortest paths between the goals
    void UpdateGoalMap();

    ///@}
    ///@name Internal State
    ///@{

    bool m_debug{true};

    std::string m_queryMethodLabel; ///< denotes the query method utilized to find
                                    ///< the shortest path between goals

    MPLibrary* m_library{nullptr};  ///< used to calculate paths between goals
    Robot* m_robot{nullptr};        ///< should be coordinator asigned all the goals
    //vector<VID> m_goals;            ///< stores the goals for the roadmap
    vector<Path> m_intergoalPaths;  ///< stores shortest paths between goals
    vector<edge_descriptor> m_pathDescriptors; ///<descruptor for edges on the path

    vector<std::shared_ptr<Constraint>> m_goalConstraints; ///< constraints on the goal
    vector<std::shared_ptr<Constraint>> m_depotConstraints; ///< depot constraints

    vector<vertex_descriptor> m_goalDescriptors; ///< Descriptors for the goals
    vector<vertex_descriptor> m_depotDescriptors; ///< Descriptors for the depots

    /// Descriptors from the edges to the goal paths
    vector<edge_descriptor> m_depotGoalPathDescriptors; 
    vector<Path> m_depotGoalPaths; ///< Paths from the depots to the goals

    /// Descriptors from the goal paths to the edges
    vector<edge_descriptor> m_goalDepotPathDescriptors;
    vector<Path> m_goalDepotPaths; ///< Paths from the goals to the depots

    vector<edge_descriptor> m_depotPathDescriptors; ///< Descriptors from the depots
    vector<Path> m_depotPaths; ///< Paths for the depots

    ///@}
};


/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
TRPGoalMap<MPTraits>::
TRPGoalMap() = default;

template <typename MPTraits>
TRPGoalMap<MPTraits>::
TRPGoalMap(Robot* _robot, std::string _queryMethod, MPLibrary* _library){
  m_robot = _robot;
  m_queryMethodLabel = _queryMethod;
  m_library = _library;
  UpdateGoalMap();
}

template <typename MPTraits>
TRPGoalMap<MPTraits>::
~TRPGoalMap() = default;

/*----------------------------- Accessors ------------------------------------*/

template <typename MPTraits>
const size_t
TRPGoalMap<MPTraits>::
GetNumVertices() const noexcept {
  return m_depotDescriptors.size() + m_goalDescriptors.size();
}

template <typename MPTraits>
std::vector<size_t>*
TRPGoalMap<MPTraits>::
GetGoalDescriptors() {
  return &m_goalDescriptors;
}

template <typename MPTraits>
std::vector<size_t>*
TRPGoalMap<MPTraits>::
GetDepotDescriptors() {
  return &m_depotDescriptors;
}

template <typename MPTraits>
const typename TRPGoalMap<MPTraits>::Path&
TRPGoalMap<MPTraits>::
GetPath(const size_t _i1, const size_t _i2) const noexcept {
  const_adj_edge_iterator edge;
  const_iterator vert;
  edge_descriptor ed(_i1, _i2);
  this->find_edge(ed, vert, edge);
  return edge->property();

}

/*----------------------- Problem Customization ------------------------------*/

template <typename MPTraits>
void
TRPGoalMap<MPTraits>::
AddDepots(std::vector<Robot*> _workers){
  if(m_debug)
    std::cout << "adding depots for " << _workers.size() << " workers" << endl;
  for(auto robot : _workers){
    std::unique_ptr<CSpaceConstraint> constraint(
          new CSpaceConstraint(robot,robot->GetSimulationModel()->GetState()));
    constraint->SetRobot(nullptr);
    m_depotConstraints.push_back(constraint->Clone());
  }
  for(auto& depot : m_depotConstraints){
    m_depotDescriptors.push_back(this->add_vertex(depot->Clone()));
  }
  //debug purposes
  if(m_debug){
    for(auto depot : m_depotDescriptors){
      std::cout << "Depot Descriptor: " << depot << endl;
    }
  }

  for(size_t i = 0; i < m_depotConstraints.size(); i++){
    for(size_t j = 0 ; j < m_depotConstraints.size(); j++){
      if(i == j) continue;
      MPTask* task(new MPTask(m_robot));
      task->SetStartConstraint(m_depotConstraints[i]->Clone());
      task->AddGoalConstraint(m_depotConstraints[j]->Clone());
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_depotPaths.push_back(*m_library->GetPath());
      m_depotPathDescriptors.push_back(this->add_edge(
            m_depotDescriptors[i], m_depotDescriptors[j],*m_library->GetPath()));
      if(m_debug){
        std::cout << "Path for: " << m_depotDescriptors[i] << ", "<<  m_depotDescriptors[j]
                  << " has length: " << m_depotPaths.back().Length() << std::endl;
      }
    }
  }

  for(size_t i = 0; i < m_depotConstraints.size(); i++){
    for(size_t j = 0 ; j < m_goalConstraints.size(); j++){
      MPTask* task(new MPTask(m_robot));
      task->SetStartConstraint(std::move(m_depotConstraints[i]->Clone()));
      task->AddGoalConstraint(std::move(m_goalConstraints[j]->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_depotGoalPaths.push_back(*m_library->GetPath());
      m_depotGoalPathDescriptors.push_back(this->add_edge(
            m_depotDescriptors[i], m_goalDescriptors[j],*m_library->GetPath()));
      //debug purposes
      if(m_debug){
        std::cout << "Depot Descriptor: " << m_depotDescriptors[i] << std::endl;
        std::cout << "Goal Descriptor: " << m_goalDescriptors[j] << std::endl;
        std::cout << "Distance: " << m_depotGoalPaths.back().Length() << std::endl;
      }




      task = new MPTask(m_robot);
      task->SetStartConstraint(std::move(m_goalConstraints[j]->Clone()));
      task->AddGoalConstraint(std::move(m_depotConstraints[i]->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_goalDepotPaths.push_back(*m_library->GetPath());
      m_goalDepotPathDescriptors.push_back(this->add_edge(
            m_goalDescriptors[j], m_depotDescriptors[i],*m_library->GetPath()));
      //debug purposes
      if(m_debug){
        std::cout << "Goal Descriptor: " << m_goalDescriptors[j] << std::endl;
        std::cout << "Depot Descriptor: " << m_depotDescriptors[i] << std::endl;
        std::cout << "Distance: " << m_goalDepotPaths.back().Length() << std::endl;
      }
    }
  }
}




/*------------------------------ Helpers -------------------------------------*/


template <typename MPTraits>
void
TRPGoalMap<MPTraits>::
UpdateGoalMap() {
  if(m_debug)
    std::cout << "Updating goal map" << std::endl;
  for(auto task : m_library->GetMPProblem()->GetTasks(m_robot)){
    const auto& constraints = task->GetGoalConstraints();
    if(constraints.size() > 1)
      throw RunTimeException(WHERE, "Multi-stage task not supported.");
    m_goalConstraints.push_back(task->GetGoalConstraints().front()->Clone());
  }
  for(auto& goal : m_goalConstraints){
    m_goalDescriptors.push_back(this->add_vertex(goal->Clone()));
  }
  for(size_t i = 0; i < m_goalConstraints.size(); i++){
    for(size_t j = 0 ; j < m_goalConstraints.size(); j++){
      if(i == j) continue;
      MPTask* task(new MPTask(m_robot));
      task->SetStartConstraint(std::move(m_goalConstraints[i]->Clone()));
      task->AddGoalConstraint(std::move(m_goalConstraints[j]->Clone()));
      m_library->Solve(m_library->GetMPProblem(),task,
          m_library->GetMPSolution(), m_queryMethodLabel, LRand(), "");
      m_intergoalPaths.push_back(*m_library->GetPath());
      m_pathDescriptors.push_back(this->add_edge(
            m_goalDescriptors[i], m_goalDescriptors[j],*m_library->GetPath()));
    }
  }


}


/*----------------------------------------------------------------------------*/



#endif
