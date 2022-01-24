#ifndef CLEARANCE_MAP_H_
#define CLEARANCE_MAP_H_

#include <queue>
#include <unordered_map>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "MPLibrary/MPBaseObject.h"
#include "WorkspaceSkeleton.h"

#include <containers/sequential/graph/directed_preds_graph.h>

#include "Vector.h"

using namespace mathtool;
using namespace std;

class WorkspaceSkeleton;
class Cfg;

////////////////////////////////////////////////////////////////////////////////
/// Filtration function.
////////////////////////////////////////////////////////////////////////////////
struct Filtration{
  double m_min;   ///< Minimum clearance. 
  Filtration(double _a){m_min = _a;}  ///< Set the minimum clearance to the given clearance value.

  /// Compare the given clearance value to the minimum clearance.
  /// @param _i the clearance to be compared to the minimum clearance.
  /// @return true if the given clearance is smaller, false otherwise. 
  bool operator()(double _i){ return _i < m_min; } 
};

////////////////////////////////////////////////////////////////////////////////
/// Clearance map of the workspace skeleton.
/// 
/// To use clearance map, put it before workspace skeleton. Direct()
/// and SetMPlibrary before construct().
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ClearanceMap : public MPBaseObject<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    /// Graph type is a directed multiedge graph of points and paths.
    typedef stapl::sequential::directed_preds_graph<
        stapl::MULTIEDGES, Point3d, vector<Point3d>>   GraphType;
    typedef GraphType::vertex_descriptor               VD;
    typedef GraphType::edge_descriptor                 ED;
    typedef typename GraphType::vertex_iterator        vertex_iterator;
    typedef typename GraphType::adj_edge_iterator      adj_edge_iterator;

    typedef typename MPTraits::CfgType                 CfgType;

    typedef function<bool(double)>                     FilterFunction;

    ///@}

  
    /// @name Construction
    /// @{
    ClearanceMap() = default;
    void Construct(WorkspaceSkeleton* _s);
    /// @}

    /// Annotate the clearance map in terms of filter function.
    /// The graph in skeleton won't be touched.
    /// @param _f Filter functor.
    /// @return A reference to the annotated clearance map.
    map<ED, double>& GetAnnotatedClearanceMap(FilterFunction&& _f);
    /// Compute the graph in terms of filter funtion
    /// then replace the old graph with the annotated one.
    /// @param _f Filter functor.
    /// @return A reference to the annotated workspace skeleton.
    WorkspaceSkeleton* GetAnnotatedSkeleton(FilterFunction&& _f);

    /// Get the clerance map
    map<ED, double>& GetClearanceMap(){ return m_clearanceMap; }


  private:

    ///@name Helpers
    ///@{

    /// Compute the minimum clearance along an edge.
    /// @param _ed The edge descriptor of graph.
    void ComputeMinClearance(const ED& _ed);

    /// Compute the clearance information of a single point.
    /// Check the algorithm in DRRRT.
    /// @param _p single point.
    double GetClearanceInfo(const Point3d& _p);

    /// Remove 2-nodes after deletion of edge.
    /// Check the algorithm in DRRRT.
    void Remove2Nodes();
    ///@}
    ///@name Internal State
    ///@{

    //////////////////////////////////////////////////
    /// We have to define comparator for ED
    /// since stapl will complain in map.find function.
    /////////////////////////////////////////////////
    struct EdgeDescriptorComp{

      /// Comparing two edge descriptors. 
      /// @param _a the first edge descriptor.
      /// @param _b the second edge descriptor.
      /// @return True if the first is larger than or equal to 
      /// the second. False otherwise.
      bool operator()(const ED& _a, const ED& _b) const{
        if(_a.source() > _b.source()) return true;
	      else if(_a.source() == _b.source()){
	        if(_a.target() > _b.target()){
	          return true;
	        }
	      }
	      return false;
      }
    };

    map<ED, double, EdgeDescriptorComp> m_clearanceMap;    ///< The map of edges and their corresponding clearance.
    WorkspaceSkeleton* m_skeleton{nullptr};                ///< Original skeleton.

    ///@}
};



/*----------------------------------------------------------------------------*/

template<class MPTraits>
void
ClearanceMap<MPTraits>::
Construct(WorkspaceSkeleton* _s){
  m_skeleton = _s;
  for(auto eit = m_skeleton->GetGraph().edges_begin();
		  eit != m_skeleton->GetGraph().edges_end(); eit++){
    ComputeMinClearance(eit->descriptor());
  }
}

template<class MPTraits>
void
ClearanceMap<MPTraits>::
ComputeMinClearance(const ED& _ed){
  vertex_iterator vit;
  adj_edge_iterator eit;
  m_skeleton->GetGraph().find_edge(_ed, vit, eit);

  const vector<Point3d>& vp = eit->property();

  // Compute the minimum clearance of points along the edge
  double min = numeric_limits<double>::max();
  for(const auto &n : vp){
    double c = GetClearanceInfo(n);
    if(c < min) min = c;
  }

  if(m_clearanceMap.find(_ed) == m_clearanceMap.end()){
    m_clearanceMap.insert(make_pair(_ed, min));
  }

}


template<class MPTraits>
WorkspaceSkeleton*
ClearanceMap<MPTraits>::
GetAnnotatedSkeleton(FilterFunction&& _f){

  auto graph = m_skeleton->GetGraph();

  for(auto mit = m_clearanceMap.begin();
		  mit != m_clearanceMap.end(); ++mit){
    if(_f(mit->second)){
      m_clearanceMap[mit->first] = numeric_limits<double>::infinity();
      graph.delete_edge(mit->first);
    }
  }
  m_skeleton->SetGraph(graph);
  Remove2Nodes();

  return m_skeleton;

}

template<class MPTraits>
map<typename ClearanceMap<MPTraits>::ED, double>&
ClearanceMap<MPTraits>::
GetAnnotatedClearanceMap(FilterFunction&& _f){
  for(auto mit = m_clearanceMap.begin();
		  mit != m_clearanceMap.end(); ++mit){
    if(_f(mit->second)){
      m_clearanceMap[mit->first] = numeric_limits<double>::infinity();
    }
  }
  return m_clearanceMap;
}

template<class MPTraits>
double
ClearanceMap<MPTraits>::
GetClearanceInfo(const Point3d& _p){
  //auto boundary = this->GetEnvironment()->GetBoundary();
  auto vc = this->GetValidityChecker("pqp_solid");
  auto pointRobot = this->GetMPProblem()->GetRobot("point");

  CfgType cfg(_p, pointRobot);
  CDInfo cdInfo;
  cdInfo.ResetVars();
  cdInfo.m_retAllInfo = true;
  vc->IsValid(cfg, cdInfo, "Obtain clearance map");

  // Check against boundary
  /*
  const double boundaryClearance = boundary->GetClearance(_p);
  if(boundaryClearance < cdInfo.m_minDist){
    cdInfo.m_objectPoint = boundary->GetClearancePoint(_p);
    cdInfo.m_minDist = boundaryClearance;
  }
  */
  return cdInfo.m_minDist;
}

template<class MPTraits>
void
ClearanceMap<MPTraits>::
Remove2Nodes() {
  bool removed;
  auto graph = m_skeleton->GetGraph();
  do {
    removed = false;
    for(auto vit = graph.begin(); vit != graph.end(); ++vit) {
      //detect 2-node
      if(vit->predecessors().size() == 1 && vit->size() == 1) {
        //get in and out edge
        auto pred = graph.find_vertex(vit->predecessors()[0]);
        ED in, out;
        vector<Vector3d> inPath;
        vector<Vector3d> outPath;
        for(auto eit : *pred) {
          if(eit.target() == vit->descriptor()){
            in = eit.descriptor();
            inPath = eit.property();
          }
        }
        out = (*vit->begin()).descriptor();
        outPath = (*vit->begin()).property();

        //Merge edge
        vector<Vector3d> mergedPath;
        for(size_t i = 0; i < inPath.size(); i++){
          mergedPath.push_back(inPath[i]);
        }
        for(size_t i = 1; i < outPath.size(); i++){
          mergedPath.push_back(outPath[i]);
        }
        ED neweid = graph.add_edge(in.source(), out.target(), mergedPath);

        //Delete old edges and delete old vertex
        graph.delete_edge(in);
        graph.delete_edge(out);
        cout << "deleted 2-node ID: " << vit->descriptor() << endl;
        graph.delete_vertex(vit->descriptor());
        --vit;
        removed = true;
      }
    }
  } while(removed);
  m_skeleton->SetGraph(graph);
}


/*----------------------------------------------------------------------------*/

#endif
