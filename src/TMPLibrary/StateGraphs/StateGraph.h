#ifndef PMPL_STATE_GRAPH_H_
#define PMPL_STATE_GRAPH_H_

#include "TMPLibrary/TMPBaseObject.h"

#include "ConfigurationSpace/RoadmapGraph.h"

#include <iostream>

class StateGraph : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    StateGraph();

    StateGraph(XMLNode& _node);

    virtual ~StateGraph() = default;

    ///@}
    ///@name Initialization
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{
    /// Access RoadmapGraph.
    virtual RoadmapGraph<Cfg,DefaultWeight<Cfg>>* GetGraph();

    /// Copies the state graph into the coordinator solution object.
    virtual void LoadStateGraph();

    ///@}

  protected:

    ///@name Helpers
    ///@{
    ///Construct graph
    virtual void ConstructGraph();

    ///@}
    ///@name member variables
    ///@{

    RoadmapGraph<Cfg,DefaultWeight<Cfg>>* m_graph{nullptr};

    std::string m_pmLabel{""};

    ///@}

};

/*----------------------------------------------------------------------------*/

#endif
