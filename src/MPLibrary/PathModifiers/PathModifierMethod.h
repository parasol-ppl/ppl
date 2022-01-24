#ifndef PATH_MODIFIER_METHOD_H_
#define PATH_MODIFIER_METHOD_H_

#include "Utilities/MPUtils.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
/// @brief Base algorithm abstraction for \ref PathModifiers.
///
/// PathModifierMethod has one main method, @c Modify, which takes an input path
/// and produces a valid output path.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PathModifierMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    PathModifierMethod() = default;

    PathModifierMethod(XMLNode& _node);

    virtual ~PathModifierMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name PathModifier Interface
    ///@{

    /// Modifies the input path to a new valid path
    /// @param _path A path of configurations within a resolution
    ///        distance of each other
    /// @param _newPath An empty vector to place the resulting modified path
    ///
    /// @usage
    /// @code
    /// PathModifierPointer pm = this->GetPathModifier(m_pmLabel);
    /// vector<CfgType> inputPath, outputPath;
    /// pm->Modify(inputPath, outputPath);
    /// @endcode
    virtual void Modify(vector<CfgType>& _path,
        vector<CfgType>& _newPath);
    ///@example PathModifiers_UseCase.cpp
    /// This is an example of how to use the path modifier methods.

    virtual void Modify(RoadmapType* _graph, vector<CfgType>& _path,
                        vector<CfgType>& _newPath);

  protected:

    /// Modifies the input path to a new valid path
    /// @param _path A path of configurations within a resolution
    ///        distance of each other
    /// @param _newPath An empty vector to place the resulting modified path
    /// @return Success/failed modification
    virtual bool ModifyImpl(RoadmapType* _graph, vector<CfgType>& _path,
        vector<CfgType>& _newPath) = 0;

    /// Appends local plan to path
    /// @param _path Path to append local plan to
    /// @param _lpOutput Local plan output
    /// @param _end End Cfg of local plan
    void AddToPath(vector<CfgType>& _path, LPOutput<MPTraits>* _lpOutput,
        CfgType& _end);

    /// Extract path VIDs in roadmap from path
    /// @param _path Path to extract VIDs from
    /// @param _graph RoadmapGraph containing path nodes
    /// @return Path VIDs
    vector<VID> GetPathVIDs(vector<CfgType>& _path, RoadmapType* _graph);

    /// @TODO Figure out what this does and document it.
    void RemoveBranches(const string& _dmLabel, vector<CfgType>& _path,
        vector<CfgType>& _newPath);

  private:

    /// Write path to file
    /// @param _path Path
    void OutputPath(vector<CfgType>& _path);

    ///@}
    ///@name Internal State
    ///@{

    string m_pathFile; ///< Where to write the smoothed path.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
PathModifierMethod<MPTraits>::
PathModifierMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  m_pathFile = _node.Read("pathFile", false, "", "Smoothed path filename");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
PathModifierMethod<MPTraits>::
Print(ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tpath file: \"" << m_pathFile << "\"" << endl;
}

/*--------------------------- PathModifier Interface -------------------------*/

template <typename MPTraits>
void
PathModifierMethod<MPTraits>::
Modify(vector<CfgType>& _path, vector<CfgType>& _newPath) {
  ModifyImpl(this->GetRoadmap(), _path, _newPath);
}

template <typename MPTraits>
void
PathModifierMethod<MPTraits>::
Modify(RoadmapType* _graph, vector<CfgType>& _path, vector<CfgType>& _newPath) {
  ModifyImpl(_graph, _path, _newPath);
}


template <typename MPTraits>
void
PathModifierMethod<MPTraits>::
AddToPath(vector<CfgType>& _path, LPOutput<MPTraits>* _lpOutput, CfgType& _end) {
  if(!_lpOutput->m_path.empty())
    _path.insert(_path.end(), _lpOutput->m_path.begin(),
        _lpOutput->m_path.end());
  _path.push_back(_end);
}


template <typename MPTraits>
vector<typename MPTraits::RoadmapType::VID>
PathModifierMethod<MPTraits>::
GetPathVIDs(vector<CfgType>& _path, RoadmapType* _graph) {
  vector<VID> pathVIDs;
  for(auto&  cfg : _path) {
    VID v = _graph->GetVID(cfg);
    if(v != INVALID_VID)
      pathVIDs.push_back(v);
  }
  return pathVIDs;
}


template <typename MPTraits>
void
PathModifierMethod<MPTraits>::
RemoveBranches(const string& _dmLabel, vector<CfgType>& _path,
    vector<CfgType>& _newPath) {
  _newPath.clear();

  Environment* env = this->GetEnvironment();
  auto dm = this->GetDistanceMetric(_dmLabel);

  //RemoveBranches Algorithm
  //_path = {q_1, q_2, ..., q_m}
  //for i = 1 -> m
  //  _newPath = _newPath + {q_i}
  //  j <- m
  //  while(d(q_i, q_j) > resolution
  //    j <- j - 1
  //  i <- j
  //return _newPath

  double res = min(env->GetPositionRes(), env->GetOrientationRes());

  for(auto cit = _path.begin(); cit != _path.end(); ++cit) {
    _newPath.push_back(*cit);

    auto rcit = _path.rbegin();
    while(dm->Distance(*cit, *rcit) > res)
      rcit++;

    //when q_i != q_j,
    //push q_j onto the new path to avoid skipping it in the loop
    if(cit != rcit.base()-1)
      _newPath.push_back(*rcit);

    cit = rcit.base()-1;
  }
  //the loop doesn't push the goal of the path, be sure to do it
  _newPath.push_back(_path.back());
}

/*----------------------------------------------------------------------------*/

#endif
