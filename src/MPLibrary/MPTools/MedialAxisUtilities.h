#ifndef PMPL_MEDIAL_AXIS_UTILITY_H_
#define PMPL_MEDIAL_AXIS_UTILITY_H_

#include <ctgmath>
#include <deque>

#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"
//#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Return structure for clearance information.
////////////////////////////////////////////////////////////////////////////////
struct ClearanceStats {
  double m_avg{0};
  double m_min{0};
  double m_max{0};
  double m_var{0};
  vector<double> m_clearanceAlongPath;
  double m_pathLength{0};
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// A ray in configuration space for approximate methods.
////////////////////////////////////////////////////////////////////////////////
template<class CfgType>
struct Ray {
  CfgType m_incr;
  CfgType m_tick;

  Ray(const CfgType& _i, const CfgType& _t) : m_incr(_i), m_tick(_t) {}
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Computes clearance and penetration of configurations with respect to
/// obstacles.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ClearanceUtility : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType        CfgType;
    typedef typename MPTraits::WeightType     WeightType;
    typedef typename MPTraits::RoadmapType    RoadmapType;
    typedef typename RoadmapType::VID         VID;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a ClearanceUtility object
    /// @param _vcLabel The validity checker to use.
    /// @param _dmLabel The distance metric to use.
    /// @param _exactClearance Use exact clearance or not.
    /// @param _exactPenetration Use exact penetration or not.
    /// @param _clearanceRays The number of clearance rays to use.
    /// @param _penetrationRays The number of penetration rays to use.
    /// @param _approxStepSize The approximate step size to use.
    /// @param _approxResolution The approximate resolution to use.
    /// @param _useBBX Use BBX or not.
    /// @param _positional Use only positional DOFs.
    /// @param _debug Set to debug mode or not.
    ClearanceUtility(
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        double _approxStepSize = MAX_DBL, double _approxResolution = MAX_DBL,
        bool _useBBX = true, bool _positional = true, bool _debug = false);

    /// Construct a ClearanceUtility object from an XML node
    /// @param _node The XML node to use
    ClearanceUtility(XMLNode& _node);

    virtual ~ClearanceUtility() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    /// Print the internal state of this object
    /// @param _os The std::ostream to print to.
    virtual void Print(ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the label of the distance metric
    const string& GetDistanceMetricLabel() const {return m_dmLabel;}

    /// Get the label of the validity checker
    const string& GetValidityCheckerLabel() const {return m_vcLabel;}

    /// Set the label of the validity checker
    /// @param _s The label of the validity checker to use.
    void SetValidityCheckerLabel(const string& _s) {m_vcLabel = _s;}

    /// Get whether exact clearance is used or not
    bool GetExactClearance() const {return m_exactClearance;}

    /// Get the position resolution
    double GetPositionResolution() const {return m_rayTickResolution;}

    /// Get the orientation resolution
    double GetOrientationResolution() const {return m_orientationResolution;}

    /// Get whether the object is initialized or not
    bool IsInitialized() const {return m_initialized;}

    ///@}
    ///@name Clearance Functions
    ///@{

    /// Calculate clearance information for the medial axis computation.
    bool CollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo,
        const bool& _useOppValidityWitness = true);

    /// Calculate clearance information exactly by taking the validity
    ///        checker results against obstacles to the bounding box.
    bool ExactCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo,
        const bool _useOppValidityWitness = true);

    /// Calculate the approximate clearance using a series of rays. The
    ///        specified number of rays are pushed outward until they change in
    ///        validity. The shortest ray is then considered the best candidate.
    bool ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg,
        const Boundary* const _b, CDInfo& _cdInfo,
        const bool& _useOppValidityWitness = true);

    /// Helpers for ApproxCollisionInfo (should maybe be protected):
    /// Find the witness of an initial sample of any validity. A witness here is
    /// defined as the nearest point of obstacle boundary.
    ///@TODO Make the user have the option to have same or opposite validity
    /// for witnesses.
    bool FindApproximateWitness(const std::size_t& _numRays,
        std::vector<Ray<CfgType> >& _rays, const CfgType& _sampledCfg,
        const bool& _initValidity, const Boundary* const _b,
        std::pair<size_t, CfgType>& _candidate,
        const bool& _useOppValidityWitness = true);

    /// Generates the rays for the witness finding.
    void MakeRays(const CfgType& _sampledCfg, const std::size_t& _numRays,
        std::vector<Ray<CfgType> >& _rays);

    /// A final sanity check to be performed on the witness candidate
    bool ValidateCandidate(const std::pair<size_t, CfgType>& _cand,
        const std::vector<Ray<CfgType> >& _rays,
        const Boundary* const _b, const bool& _useOppValidityWitness = true);

    /// Calculate roadmap clearance statistics including averages, mins,
    ///        and maxes for clearance across roadmaps, paths, and edges.
    ClearanceStats RoadmapClearance();

    /// Calculate path clearance statistics including averages, mins,
    ///        and maxes for clearance across the path
    /// @param _path The path to use.
    ClearanceStats PathClearance(vector<VID>& _path);

    /// Calculate path clearance statistics including averages, mins,
    ///        and maxes for clearance across the path
    /// @param _path The path to use.
    ClearanceStats PathClearance(vector<Cfg>& _path);

    /// Calculate path clearance statistics including averages, mins,
    ///        and maxes for clearance across the edge
    /// @param _c1 The source vertex.
    /// @param _c2 The target vertex.
    /// @param _weight The weight of the edge.
    vector<double> EdgeClearance(const CfgType& _c1, const CfgType& _c2,
        const WeightType& _weight);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    string m_vcLabel{"cd4"};        ///< Validity checker method label.
    string m_dmLabel{"euclidean"};  ///< Distance metric method label.

    bool m_useBBX{true};            ///< Use bounding box as obstacle?

    bool m_positionalDofsOnly{true};        ///< Use only positional dofs?

    bool m_exactClearance{false};   ///< Use exact clearance calculations?
    bool m_exactPenetration{false}; ///< Use exact penetration calculations?

    size_t m_clearanceRays{10};  ///< Number of rays for approximate clearance.
    size_t m_penetrationRays{10};///< Number of rays for approximate penetration.

    // These replaced two similar members from the ClearanceUtility
    // class (m_approxStepSize and m_approxResolution):
    /// The resolution for rayTicks
    double m_rayTickResolution;//no default values as these are required in node
    /// The orientation resolution
    double m_orientationResolution;

    //These two are linearly related based on the resolution for ray ticking.
    // The max ray magnitude is just how far out a ray will be attempted.
    /// How far out a ray will be attempted
    double m_maxRayMagnitude;
    /// The maximum number of ray iterations
    size_t m_maxRayIterations;

    //These are just for getting rayTickResolution and orientationResolution.
    // They get multiplied into their respective values from the environment,
    // and should only really be used there. See Initialize() for more info.
    double m_orientationResFactor{0.}; ///< The orientation resolution factor
    double m_positionalResFactor{0.}; ///< The position resoluiton factor
    double m_maSearchResolutionFactor{1}; ///< The search resolution factor

    /// A boolean to determine whether initialize has been called on the
    /// utility or not yet. This is important, as there are MPLibrary things
    /// we need that are not available at construction time:
    bool m_initialized{false};

    ///@}

    /// Adjusts a witness from a nearby witness point to a point of guaranteed
    /// validity, based on _initValidity and _useOppValidityWitness.
    bool AdjustWitnessToEnsureValidity(
                                const CfgType& _cfg, const bool _initValidity,
                                CfgType& _witnessCfg, const Boundary* const _b,
                                const bool _useOppValidityWitness = true);

    /// GetNearestVertexWitness will do this for us, including boundary witness
    /// information, if BBX is active as an obstacle.
    const bool GetNearestVertexWitness(CfgType& _cfg, CDInfo& _cdInfo,
                                       const Boundary* const _b);
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Utilities
/// Tool for pushing configurations to the medial axis.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MedialAxisUtility : public ClearanceUtility<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a MedialAxisUtility object
    /// @param _vcLabel The validity checker to use.
    /// @param _dmLabel The distance metric to use.
    /// @param _exactClearance Use exact clearance or not.
    /// @param _exactPenetration Use exact penetration or not.
    /// @param _clearanceRays The number of clearance rays to use.
    /// @param _penetrationRays The number of penetration rays to use.
    /// @param _useBBX Use BBX or not.
    /// @param _positional Use only positional DOFs.
    /// @param _debug Set to debug mode or not.
    /// @param _epsilon The epsilon value to use.
    /// @param _historyLength The history length to use.
    MedialAxisUtility(
        string _vcLabel = "", string _dmLabel = "",
        bool _exactClearance = false, bool _exactPenetration = false,
        size_t _clearanceRays = 10, size_t _penetrationRays = 10,
        bool _useBBX = true, bool _positional = true, bool _debug = false,
        double _epsilon = 0.1, size_t _historyLength = 5);

    /// Construct a MedialAxisUtility object from an XML node
    /// @param _node The XML node to use.
    MedialAxisUtility(XMLNode& _node);

    virtual ~MedialAxisUtility() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    /// Print the internal state of this object
    /// @param _os The std::ostream to print to.
    virtual void Print(ostream& _os) const override;

    /// Set to print debug statements or not
    /// @param _debug Print debug statements or not.
    void SetDebug(const bool _debug) { this->m_debug = _debug; }

    ///@}
    ///@name Property Accessors
    ///@{

    /// Get the value of epsilon
    double GetEpsilon() const {return m_epsilon;}

    ///@}
    ///@name Medial Axis Functions
    ///@{

    /// Push a configuration to the medial axis.
    /// @param _cfg The configuration to push.
    /// @param _b The boundary to use.
    bool PushToMedialAxis(CfgType& _cfg, const Boundary* const _b);

    /// Push a configuration that is known to be inside an obstacle
    /// towards the free-space medial axis. It will be pushed until it is
    /// outside the obstacle.
    /// @param _cfg The configuration to push.
    /// @param _b The boundary to use.
    bool PushFromInsideObstacle(CfgType& _cfg, const Boundary* const _b);


    /// Push a configuration to the medial axis by stepping away from the
    /// nearest obstacle at the resolution until a second witness (invalid cfg)
    /// is found and then the midpoint of those two cfgs will be the MA cfg.
    /// @param _cfg The configuration to push.
    /// @param _b The boundary to use.
    bool PushCfgToMedialAxisMidpointRule(CfgType& _cfg, const Boundary* const _b);
    ///@}

  private:

    ///@name Internal State
    ///@{

    // m_epsilon is used only externally right now (not used in pushing to MA)
    double m_epsilon{.1};
    size_t m_historyLength{5};

    ///@}

    //Does a fuzzy equals using the minimum epsilon of _tolerance,
    // for _v1 == _v2.
    bool FuzzyVectorEquality(mathtool::Vector3d _v1, mathtool::Vector3d _v2,
        const double _tolerance = 10.*std::numeric_limits<double>::epsilon());
};

/*---------------------------- Clearance Utility -----------------------------*/

template<class MPTraits>
ClearanceUtility<MPTraits>::
ClearanceUtility(
    string _vcLabel, string _dmLabel, bool _exactClearance,
    bool _exactPenetration, size_t _clearanceRays, size_t _penetrationRays,
    double _approxStepSize, double _approxResolution, bool _useBBX,
    bool _positional, bool _debug):
    m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
    m_useBBX(_useBBX), m_positionalDofsOnly(_positional),
    m_exactClearance(_exactClearance), m_exactPenetration(_exactPenetration),
    m_clearanceRays(_clearanceRays), m_penetrationRays(_penetrationRays)
    {
  this->SetName("ClearanceUtility");
  this->m_debug = _debug;
  this->m_rayTickResolution = 0.;
  this->m_orientationResolution = 0.;
  this->m_maxRayMagnitude = DBL_MAX;//So that Initialize() will update this.
  this->m_maxRayIterations = 0;
}


template<class MPTraits>
ClearanceUtility<MPTraits>::
ClearanceUtility(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("ClearanceUtility");

  m_vcLabel = _node.Read("vcLabel", false, m_vcLabel, "Validity Test Method");
  m_dmLabel = _node.Read("dmLabel", false, m_dmLabel, "Distance metric");

  string clearanceType = _node.Read("clearanceType", true, "",
      "Clearance Computation (exact or approx)");
  m_exactClearance = clearanceType.compare("exact") == 0;

  string penetrationType = _node.Read("penetrationType", true, "",
      "Penetration Computation (exact or approx)");
  m_exactPenetration = penetrationType.compare("exact") == 0;

  //Update the orientation and positional resolutions based on input factor.
  //if one is approx, ensure we initialize correctly:
  bool useApprox = !m_exactClearance || !m_exactPenetration;
  m_positionalResFactor = _node.Read("rayTickPosResolutionFactor", useApprox,
          1., 0., MAX_DBL, "Resolution factor for ray ticking, gets multiplied "
              "into the environment/CD resolution");

  m_orientationResFactor = _node.Read("orientationResolutionFactor", useApprox,
      1., 0., MAX_DBL, "Resolution factor for orientation, gets multiplied "
          "into the environment/CD resolution");


  //I cannot access env vars like orientation/position res and, so we must do
  // that in Initialize(). If we are doing approx but don't Initialize(),
  // then we SHOULD have problems, which is why I'm setting these to 0.
  m_rayTickResolution = 0.;
  m_orientationResolution = 0.;
  this->m_maxRayIterations = 0;

  //get the max distance rays can go to find a witness:
  m_maxRayMagnitude = _node.Read(
        "maxRayTickMagnitude", useApprox, 100., 0., DBL_MAX,
        "Total magnitude that rays will search, determines iteration limits");


  // If using approximate calculations, require number of rays to be defined.
  this->m_clearanceRays =
      _node.Read("clearanceRays", !m_exactClearance,
      m_clearanceRays, size_t(1), size_t(1000), "Number of clearance Rays");

  // simply have clearance rays defined as a multiple of penetration:
  double penetrationRayFactor =
      _node.Read("penetrationRayFactor", !m_exactPenetration,
      2., 0., 1000., "Number of penetration Rays as multiple of clearance rays");
  this->m_penetrationRays = (size_t)(m_clearanceRays * penetrationRayFactor);

  m_useBBX = _node.Read("useBBX", false, m_useBBX, "Use the Bounding Box as an "
      "obstacle");

  ///@TODO: change the name of the XML variable to "positionalDofsOnly" as well.
  m_positionalDofsOnly = _node.Read("positional", false, m_positionalDofsOnly,
      "Use only positional DOFs, ignoring all others");

  // @TODO: change to a required attribute.
  m_maSearchResolutionFactor = _node.Read("maSearchResFactor", false,
                                m_maSearchResolutionFactor, .0000001, 10e10,
                                "Use only positional DOFs, ignoring all others");
}


template<class MPTraits>
void
ClearanceUtility<MPTraits>::
Print(ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl
      << "\tdmLabel = " << m_dmLabel << endl
      << "\tpositional = " << m_positionalDofsOnly << endl
      << "\tuseBBX = " << m_useBBX << endl
      << "\tclearance = " << (m_exactClearance ? "exact" :
         "approx, " + to_string(m_clearanceRays) + " rays") << endl
      << "\tpenetration = " << (m_exactPenetration ? "exact" :
         "approx, " + to_string(m_penetrationRays) + " rays") << endl;
  if(!m_exactClearance || !m_exactPenetration)
    _os << "\trayTickResolution = " << m_rayTickResolution << endl
        << "\torientationResolution = " << m_orientationResolution << endl;
}


//NOTE: Calling initialize is absolutely necessary if doing approximate
// clearance or approximate penetrations
template <class MPTraits>
void
ClearanceUtility<MPTraits>::
Initialize() {
  if(m_maxRayMagnitude == DBL_MAX) {
    m_maxRayMagnitude = this->GetEnvironment()->GetBoundary()->GetMaxDist();
  }

  //Note that calling initialize on a MedialAxisUtility object will call this.
  m_rayTickResolution =
        m_positionalResFactor*this->GetEnvironment()->GetPositionRes();

  m_orientationResolution =
        m_orientationResFactor*this->GetEnvironment()->GetOrientationRes();

  m_maxRayIterations = (size_t)(m_maxRayMagnitude/m_rayTickResolution);

  if(this->m_debug) {
    if(m_exactClearance)
      std::cout << "Using exact clearance" << std::endl;
    else
      std::cout << "Using approximate clearance" << std::endl;

    if(m_exactPenetration)
        std::cout << "Using exact penetration" << std::endl;
      else
        std::cout << "Using approximate penetration" << std::endl;
  }
  m_initialized = true;
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
CollisionInfo(CfgType& _cfg, CfgType& _clrCfg, const Boundary* const _b,
    CDInfo& _cdInfo, const bool& _useOppValidityWitness) {
  if(m_exactClearance)
    return ExactCollisionInfo(_cfg, _clrCfg, _b, _cdInfo, _useOppValidityWitness);
  else
    return ApproxCollisionInfo(_cfg, _clrCfg, _b, _cdInfo, _useOppValidityWitness);
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
ExactCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, const Boundary* const _b,
                   CDInfo& _cdInfo, const bool _useOppValidityWitness) {
  if(_cfg.m_witnessCfg.get() != 0) {
    _cdInfo = _cfg.m_clearanceInfo;
    _clrCfg = CfgType(*_cfg.m_witnessCfg);
    return true;
  }

  const string callee = this->GetNameAndLabel() + "::ExactCollisionInfo";

  if(!_cfg.InBounds(_b)) {
    if(this->m_debug)
      std::cout << callee << "Returning false due to _cfg being OOB initially."
                << std::endl;
  }

  if(this->m_debug) {
    VDClearAll();
    VDAddTempCfg(_cfg, false);
  }

  const bool initValidity = GetNearestVertexWitness(_cfg, _cdInfo, _b);

  auto robot = _cfg.GetRobot();
  //Use the vector constructor (positional dofs set to vector, all others are 0)
  // so this creates a witness that is translated from the sample by the vector
  // from the sample cfg's closest vertex to the witness vertex.
  _clrCfg = _cfg +
            CfgType((_cdInfo.m_objectPoint - _cdInfo.m_robotPoint), robot);

  if(_clrCfg == _cfg) {
    if(this->m_debug)
      std::cout << callee << ": Returning false from _clrCfg == _cfg."
                << std::endl;
    return false;
  }

  if(!AdjustWitnessToEnsureValidity(_cfg, initValidity, _clrCfg,
                                    _b, _useOppValidityWitness)) {
    if(this->m_debug)
      std::cout << callee << "Returning false from witness adjustment failing"
                << std::endl;
    return false;
  }

  if(this->m_debug)
    VDAddTempCfg(_clrCfg, true);

  //Final data to set for successful witness found:
  _cfg.m_witnessCfg = shared_ptr<Cfg>(new CfgType(_clrCfg));
  _cfg.m_clearanceInfo = _cdInfo;

  return true;
}

template<class MPTraits>
const bool
ClearanceUtility<MPTraits>::
GetNearestVertexWitness(CfgType& _cfg, CDInfo& _cdInfo,
                        const Boundary* const _b) {
  //This function places the nearest vertex of the environment and the nearest
  // robot vertex into _cdInfo.
  //Returns the validity of _cfg provided.

  // Setup Validity Checker
  auto vcm = this->GetValidityChecker(m_vcLabel);

  _cdInfo.ResetVars(true);//true so we get all data back

  const string callee = this->GetNameAndLabel() + "::GetNearestVertexWitness";

  // Checking the validity here populates the _cdInfo object.
  const bool initValidity = vcm->IsValid(_cfg, _cdInfo, callee); // Initial Validity

  //If initially invalid, then we are doing penetration. Return a negative
  // distance in order to indicate this.
  if(!initValidity)
    _cdInfo.m_minDist = -_cdInfo.m_minDist;

  // If not using the bbx, we have the closest obstacle point already.

  if(this->m_useBBX) {
    // CfgType is now know as good, get BBX and ROBOT info
    auto multiBody = _cfg.GetMultiBody();

    // Find closest point between robot and bbx, set if less than min dist
    // from obstacles
    for(size_t m = 0; m < multiBody->GetNumBodies(); ++m) {
      const GMSPolyhedron& poly = multiBody->GetBody(m)->GetWorldPolyhedron();
      for(size_t j = 0; j < poly.GetVertexList().size(); ++j) {
        const double clr = _b->GetClearance(poly.GetVertexList()[j]);
        if(clr < _cdInfo.m_minDist) {
          _cdInfo.m_robotPoint = poly.GetVertexList()[j];
          _cdInfo.m_objectPoint = _b->GetClearancePoint(poly.GetVertexList()[j]);
          _cdInfo.m_minDist = clr;
          _cdInfo.m_nearestObstIndex = -1;
        }
      }
    }
  }

  return initValidity;
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
AdjustWitnessToEnsureValidity(const CfgType& _cfg, const bool _initValidity,
                              CfgType& _witnessCfg, const Boundary* const _b,
                              const bool _useOppValidityWitness) {
  //_cfg is the sample configuration of _initValidity. _witnessCfg is its
  // initial witness, which is just the translated _cfg from its nearest vertex
  // to the nearest obstacle vertex.
  Environment* env = this->GetEnvironment();
  // Setup Validity Checker
  auto vcm = this->GetValidityChecker(m_vcLabel);

  //See if the current _witnessCfg is a passing witness.
  const string callee = this->GetNameAndLabel()+"::AdjustWitnessToEnsureValidity";
  bool currValidity = vcm->IsValid(_witnessCfg, callee);
  if(this->m_useBBX)
    currValidity = currValidity && _witnessCfg.InBounds(_b);

  const bool witnessIsSameValidity = (_initValidity == currValidity);
  bool passed = _useOppValidityWitness ? !witnessIsSameValidity :
                                          witnessIsSameValidity;

  if(!passed) {
    auto robot = _cfg.GetRobot();
    //Get the increment:
    CfgType incr(robot);
    int nTicks; //This is used as the maximum number of adjustments possible.
    incr.FindIncrement(_cfg, _witnessCfg, &nTicks,
                       env->GetPositionRes(), env->GetOrientationRes());
    nTicks = max(nTicks,100); // Minimum allowable ticks bound is 100 for now.
    int count = 0;
    while(!passed) {
      //All of the cases boil down to this condition wrt which direction to move
      // (think about it in terms of the _useOppValidityWitness value, if it's
      // set to true, then the only condition that could be in here is if they
      // have the same validity, otherwise it's false and they are opposite).
      if(witnessIsSameValidity)
        _witnessCfg += incr;//Push out until validity changes.
      else
        _witnessCfg -= incr;//Pull cfg back until validity changes.

      //Note: a potential optimization is to only do a validity check against
      // the obstacle originally colliding with the witness, as that shouldn't
      // change when adjusting. It would only help certain cases though.
      currValidity = vcm->IsValid(_witnessCfg, callee);
      const bool inBounds = _witnessCfg.InBounds(_b);
      if(this->m_useBBX)
        currValidity = currValidity && inBounds;
      passed = _useOppValidityWitness ? (_initValidity != currValidity) :
                                        (_initValidity == currValidity);

      //If we go out of bounds while adjusting a witness, call it a failure.
      // (Note that an acceptable OOB witness would have already been caught)
      // Checking not passed here is very important!
      if(!inBounds && !passed) {
        if(this->m_debug)
          cout << callee << ": Fell out of BBX after " << count << " steps, "
               "returning false" << endl;
        return false;
      }
      if(++count >= nTicks) {
        if(this->m_debug)
          std::cout << callee << ": Tried more than nTicks = " << nTicks <<
                    " when adjusting witness, returning false" << std::endl;
        return false;
      }
    }
  }

  return passed;
}

template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
ApproxCollisionInfo(CfgType& _cfg, CfgType& _clrCfg, const Boundary* const _b,
                    CDInfo& _cdInfo, const bool& _useOppValidityWitness) {
  const string fName = "ApproxCollisionInfo: ";

  // Check computation cache
  if(_cfg.m_witnessCfg.get() != 0) {
    _cdInfo = _cfg.m_clearanceInfo;
    _clrCfg = CfgType(*_cfg.m_witnessCfg);
    return true;
  }

  // If in BBX, check validity to get _cdInfo, return false if not valid
  if(!_cfg.InBounds(_b)) {
    if(this->m_debug)
      std::cout << fName + "returning false from not being in bounds initially"
                << std::endl;
    return false;
  }

  // Initialization
  string callee = this->GetNameAndLabel() + "::ApproxCollisionInfo";
  auto dm  = this->GetDistanceMetric(m_dmLabel);
  auto vcm = this->GetValidityChecker(m_vcLabel);
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  // Compute initial validity state
  const bool initValidity = vcm->IsValid(_cfg, _cdInfo, callee);
  if(this->m_debug) {
    std::cout << "initial cfg = " << std::endl << _cfg << std::endl;
    std::cout << "\tinitValidity = " << initValidity << std::endl;
  }

  // Make a container for our rays. FindApproximateWitness will generate them
  // when given an empty vector.
  size_t numRays;
  if(initValidity)
    numRays = m_clearanceRays;
  else
    numRays = m_penetrationRays;
  std::vector<Ray<CfgType>> rays;
  std::pair<size_t, CfgType> cand;

  if(!FindApproximateWitness(
        numRays, rays, _cfg, initValidity, _b, cand, _useOppValidityWitness)) {
    if(this->m_debug)
      std::cout << fName + "Returning false from not finding any witness "
                "candidate" << std::endl;
    return false;
  }

  if(this->m_debug) {
    cout << "found candidate:\n";
    cout << "\t" << cand.first << ": " << cand.second;

    CDInfo tmpInfo;
    bool currValidity = vcm->IsValid(cand.second, tmpInfo, callee);
    if(m_useBBX)
      currValidity = (currValidity && cand.second.InBounds(_b));
    cout << " (currValidity = " << currValidity << ")";
    cout << endl;
    cout << "DEBUG:: setting info, returning" << std::endl;
  }

  //Set the final info to return. Note that the witness here needs
  // not be ticked either direction, as FindApproximateWitness handles all of
  // the witness validity logic.
  _clrCfg = cand.second;

  _cdInfo.m_minDist = (initValidity ? 1.0 : -1.0) * dm->Distance(_clrCfg, _cfg);
  _cfg.m_clearanceInfo = _cdInfo;
  _cfg.m_witnessCfg = shared_ptr<Cfg>(new CfgType(_clrCfg));
  return true;
}


template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
FindApproximateWitness(const std::size_t& _numRays,
            std::vector<Ray<CfgType> >& _rays, const CfgType& _sampledCfg,
            const bool& _initValidity, const Boundary* const _b,
            std::pair<size_t, CfgType>& _candidate,
            const bool& _useOppValidityWitness) {
  if(this->m_debug)
    cout << "FindApproximateWitness: numRays = " << _numRays << endl;
  auto vcm = this->GetValidityChecker(m_vcLabel);

  string callee = this->GetNameAndLabel() + "::FindApproximateWitness()";

  //Assume that the directions have been populated already if _rays isn't empty.
  if(_rays.empty()) {
    //Reserve the number of rays that we need for faster push_back later
    _rays.reserve(_numRays);
    MakeRays(_sampledCfg, _numRays, _rays);
  }

  // Step out along each direction to determine the candidate
  bool candidateFound = false;
  size_t iterations = 0;
  CfgType candidateCfg;

  if(this->m_debug)
    cout << "DEBUG:: stepping out along each direction to find candidates";

  //This loop goes until it has just one of its rays go from free->not free
  //or not free->free or iterates n times. Within it, it loops through all
  //rays each time, moving outward along these and checking that condition.
  //m_maxRayIterations is calculated by maxRayMagnitude/rayTickResolution
  while(!candidateFound && iterations++ < m_maxRayIterations) {
    if(this->m_debug)
      cout << "\n\t" << iterations;

    for(auto rit = _rays.begin(); rit != _rays.end();) {
      //step out
      rit->m_tick += rit->m_incr;

      //determine new state
      CDInfo tmpInfo;

      bool currValidity{false};//will get overwritten
      const bool inBounds = rit->m_tick.InBounds(_b);

      // If doing the opposite validity witness, then it's the case that if
      // we are out of bounds and initially invalid, this ray can never lead
      // to a successful witness of opposite validity, so remove the ray.
      //Note: this is not taking into account whether or not m_useBBX is true.
      // If doing the same-validity witness, then we still will count the tick
      // before going out of bounds as the nearest witness, since it still must
      // correspond to an obstacle's boundary being crossed.
      if(!inBounds && !_initValidity && _useOppValidityWitness) {
        if(this->m_debug)
          cout << "Ray is out of bounds at: " << rit->m_tick << endl;

        // only one ray
        if(_rays.size() == 1) {
          if(this->m_debug)
            cout << "Last valid ray just failed, Returning false" << endl;
          return false;
        }
        else {
          //Swap the ray leading to an invalid OOB cfg to the back of the vector
          // and then pop from back of vector:
          if(rit != _rays.end() - 1) { //Don't swap if it's already the last ray
            auto eit = _rays.end() - 1;
            swap(*rit, *eit);
          }

          //Remove the ray that cannot lead to a valid witness.
          _rays.pop_back();
          continue; //move to next ray (note: won't advance the iterator)
        }
      }
      //If we are doing the same validity witness, then the same triggers for a
      // witness still apply, except that the condition above should also
      // trigger a witness instead of disqualifying a ray.
      else if (!inBounds && !_initValidity && !_useOppValidityWitness) {
        // Force a change in validity, since we have found the edge of the
        // obstacle we are in and it happens to border the BBX boundary.
        currValidity = !_initValidity;
      }
      //So now we know that we have a ray tick that is either OOB and was an
      // initially valid sample, or we are in bounds and initial state is N/A.
      else {
        currValidity = vcm->IsValid(rit->m_tick, tmpInfo, callee);
        if(m_useBBX) // Only consider bounds here if using it as obstacle
          currValidity = inBounds && currValidity;
      }

      if(this->m_debug)
        cout << " (currValidity for direction " << distance(_rays.begin(), rit)
             << " = " << currValidity << ")";

      //if validity state has changed, save the candidate and stop searching:
      //Note that OOB is of the same invalid status here as in collision.
      if(currValidity != _initValidity) {
        candidateFound = true; // So the enveloping while loop will exit

        candidateCfg = rit->m_tick;
        //tick back the current cfg only if using the same validity witness:
        if(!_useOppValidityWitness)
          candidateCfg -= rit->m_incr;

        //Note: this is pushing back the index of the candidate ray (what
        // distance() returns) and the actual location that we have found. The
        // location's validity is determined by the type of witness desired.
        _candidate = make_pair(distance(_rays.begin(), rit), candidateCfg);

        //Quit the loop since the witness has been found.
        break;
      }
      //Must increment the iterator here due to the flow of the for loop:
      ++rit;
    }
  }//end while (!stateChangedFlag && iterations < max)

  //Check that we actually found a candidate
  if(!candidateFound) {
    if(this->m_debug)
      std::cout << callee + "Returning false after not finding a witness "
                "candidate" << std::endl;
    return false;
  }

  if(this->m_debug) {
    cout << "\nDEBUG:: done stepping out along rays. Candidate = " << std::endl;
    cout << "\tray " << _candidate.first << ": " << _candidate.second;

    CDInfo tmpInfo;
    bool currValidity = vcm->IsValid(_candidate.second, tmpInfo, callee);
    if(m_useBBX)
      currValidity = (currValidity && _candidate.second.InBounds(_b));
    cout << " (currValidity = " << currValidity << ")" << endl;
  }

  // Ensure the candidate is valid:
  // Note: this is something we could remove for optimization
  if(!ValidateCandidate(_candidate, _rays, _b, _useOppValidityWitness))
    return false;

  // This check should be impossible to trigger, but it's a good thing to ensure
  if(candidateCfg == _sampledCfg) {
    if(this->m_debug)
      std::cout << callee + "returning false from _clrCfg == _cfg" << std::endl;
    return false;
  }

  CDInfo tmpInfo;
  bool currValidity = vcm->IsValid(candidateCfg, tmpInfo, callee);
  if(this->m_useBBX)
    currValidity = currValidity && candidateCfg.InBounds(_b);

  //Final check to make sure the witness is of correct validity.
  bool passed = _useOppValidityWitness ? (_initValidity != currValidity) :
                                         (_initValidity == currValidity);
  if(!passed) {
    if(this->m_debug)
      std::cout << callee + "returning false from candidate being of wrong "
                "validity" << std::endl;
    return false;
  }

  //If we make it here, we have a successful witness.
  return true;
}


template<class MPTraits>
void
ClearanceUtility<MPTraits>::
MakeRays(const CfgType& _sampledCfg, const std::size_t& _numRays,
         std::vector<Ray<CfgType> >& _rays) {
  auto dm  = this->GetDistanceMetric(m_dmLabel);
  string callee = this->GetNameAndLabel() + "::MakeRays()";

  //For 2d:
  //initial ray starts at rand angle, then all others are uniformly distributed:
  double angleRad = 2.*PI*DRand();//Note 2d case only currently

  for(size_t i = 0; i < _numRays; ++i) {
    CfgType tmpDirection(this->GetTask()->GetRobot());

    ///@TODO expand to 3D, and then to N dimensions for uniform distribution:
    if(_sampledCfg.DOF() == 2) {
      // This evenly divides up the rest of the angles all the way around,
      // starting from the random initial angle.
      angleRad += 2. * PI * (1. / _numRays);//note this happens numRays times
      std::vector<double> dofData = {cos(angleRad), sin(angleRad)};
      tmpDirection.SetData(dofData); // There's likely a better way to do this
    }
    else {
      //The non-uniform way to get each ray:
      tmpDirection.GetRandomRay(m_rayTickResolution, dm, false);
    }

    if(this->m_debug)
      cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;

    if(m_positionalDofsOnly) { // Use only positional dofs
      if(tmpDirection.PosDOF() == 0)
        throw RunTimeException(WHERE, "Attempting to only use positional DOFs "
                           "for a fixed-base robot, this is invalid behavior!");
      double factor = 0.0;
      for(size_t j = 0; j < tmpDirection.DOF(); ++j) {
        if(j < tmpDirection.PosDOF())
          factor += tmpDirection[j] * tmpDirection[j];
        else
          tmpDirection[j] = 0.0;
      }
      tmpDirection *= m_rayTickResolution / sqrt(factor);
    }
    if(this->m_debug) {
      cout << "DEBUG:: tmpDirection " << i << " is " << tmpDirection << endl;
      cout << "DEBUG:: \tdistance(_sampledCfg, _sampledCfg+tmpDirection) = "
        << dm->Distance(_sampledCfg, _sampledCfg + tmpDirection) << endl;
    }

    _rays.push_back(Ray<CfgType>(tmpDirection, _sampledCfg));
  }// end for (_numRays)

  if(this->m_debug) {
    cout << "DEBUG:: rays initialized\n";
    cout << "DEBUG:: \ttick are:\n\t\t";
    for(auto&  ray : _rays)
      cout << ray.m_tick << "\n\t\t";
    cout << endl;
    cout << "DEBUG:: \tincr are:\n\t\t";
    for(auto&  ray : _rays)
      cout << ray.m_incr << "\n\t\t";
    cout << endl;
  }
  return;
}

template<class MPTraits>
bool
ClearanceUtility<MPTraits>::
ValidateCandidate(const std::pair<size_t, CfgType>& _cand,
                  const std::vector<Ray<CfgType> >& _rays,
                  const Boundary* const _b, const bool& _useOppValidityWitness){
  string callee = this->GetNameAndLabel() + "::ValidateCandidate()";
  auto vcm = this->GetValidityChecker(m_vcLabel);

  // This is checking whether if we have an "incorrect" candidate, meaning
  // that if a witness was found, then the candidate should have an initial cfg
  // position that is valid, and then ticking forward once  should be invalid.
  if(this->m_debug) {
    cout << "DEBUG:: Verifying candidate is valid\n";
    cout << _cand.first << std::endl;
  }
  CDInfo tmpInfo;

  //Note: the "low" and "high" names are due to this before being a step of a
  // binary search step for refining the witness location. The binary search has
  // been removed, but the names remain.
  //lowCfg is the one before the witness (for opposite-validity witnesses), so
  // it must get ticked back unless we're doing same-validity witness.
  CfgType lowCfg, highCfg;
  if(_useOppValidityWitness)
    lowCfg = _rays[_cand.first].m_incr * -1.0 + _cand.second;
  else
    lowCfg = _cand.second;


  bool lowValidity = vcm->IsValid(lowCfg, tmpInfo, callee);
  if(m_useBBX)
    lowValidity = (lowValidity && lowCfg.InBounds(_b));

  //highCfg is the witness (for opposite-validity witnesses) and is one tick
  // past the witness for same-validity witnesses.
  if(_useOppValidityWitness)
    highCfg = _cand.second;
  else
    highCfg = _rays[_cand.first].m_incr + _cand.second;

  bool highValidity = vcm->IsValid(highCfg, tmpInfo, callee);
  if(m_useBBX)
    highValidity = (highValidity && highCfg.InBounds(_b));

  if(this->m_debug)
    std::cout << "lowCfg =" << std::endl << lowCfg << std::endl
              << " (lowValidity = " << lowValidity << ")" << std::endl
              << "highCfg =" << std::endl << highCfg << std::endl
              << " (highValidity = " << highValidity << ")" << endl;

  //Do the validity check, this actually needs no condition on the flag, since
  // the two ticks should be of opposite validity no matter the flag. We just
  // have to handle the case if we have an OOB witness and invalid initial
  // validity. The check conditioned on the flag is outside ValidateCandidate().
  if(lowValidity == highValidity && !highCfg.InBounds(_b) && !lowValidity) {
    if(this->m_debug)
      std::cout << "Returning false from invalid witness candidate"
                << std::endl;
    return false;
  }
  return true;
}


template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::
RoadmapClearance() {
  auto g = this->GetRoadmap();

  //TODO handle case of singleton nodes to be part of roadmap clearance
  //computation.
  if(g->get_num_edges() == 0)
    return ClearanceStats();

  vector<double> clearanceVec;

  //loop over graph edges and calculate clearance
  for(auto it = g->edges_begin(); it != g->edges_end(); ++it) {
    vector<double> currentClearance =
      EdgeClearance(g->GetVertex((*it).source()),
          g->GetVertex((*it).target()), (*it).property());
    //Save this value for variance computation later
    clearanceVec.insert(clearanceVec.end(),
			currentClearance.begin(),
			currentClearance.end());
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) /
    clearanceVec.size();
  stats.m_clearanceAlongPath = clearanceVec;
  double varSum = 0;
  for(auto&  i : clearanceVec)
    varSum += sqr(i - stats.m_avg);
  stats.m_var = varSum / clearanceVec.size();

  return stats;
}


template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::
PathClearance(vector<VID>& _path) {
  if(_path.empty())
    return ClearanceStats();

  auto g = this->GetRoadmap();
  auto dm = this->GetDistanceMetric(m_dmLabel);

  typedef typename RoadmapType::EI EI;
  typedef typename RoadmapType::VI VI;
  typedef typename RoadmapType::EID EID;

  vector<double> clearanceVec;
  double pathLength = 0;

  for(auto vit = _path.begin(); (vit+1) != _path.end(); ++vit) {
    EI ei;
    VI vi;
    EID ed(*vit, *(vit + 1));
    g->find_edge(ed, vi, ei);
    CfgType& s = g->GetVertex((*ei).source()), t = g->GetVertex((*ei).target());
    pathLength += dm->Distance(s, t);
    vector<double> currentClearance = EdgeClearance(s, t, (*ei).property());
    clearanceVec.insert(clearanceVec.end(),
			currentClearance.begin(),
			currentClearance.end());
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_pathLength = pathLength;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) /
    clearanceVec.size();
  stats.m_clearanceAlongPath = clearanceVec;

  double varSum = 0;
  for(auto&  i : clearanceVec) {
    varSum += sqr(i - stats.m_avg);
  }
  stats.m_var = varSum / clearanceVec.size();

  return stats;
}

template<class MPTraits>
ClearanceStats
ClearanceUtility<MPTraits>::
PathClearance(vector<Cfg>& _path) {
  if(_path.empty())
    return ClearanceStats();

  auto g = this->GetRoadmap();
  auto dm = this->GetDistanceMetric(m_dmLabel);

  typedef typename RoadmapType::EI EI;
  typedef typename RoadmapType::VI VI;
  typedef typename RoadmapType::EID EID;
  typedef typename RoadmapType::const_vertex_iterator CVI;

  double pathLength = 0;
  vector<double> clearanceVec;

  for(auto cit = _path.begin(); cit + 1 != _path.end(); ++cit) {
    pathLength += dm->Distance(*cit, *(cit + 1));

    WeightType weight;
    bool sourceFound = false;
    CVI si;
    for(si = g->begin(); si != g->end(); ++si)
      if(si->property() == *cit) {
        sourceFound = true;
        break;
      }
    if(sourceFound) {
      bool targetFound = false;
      CVI ti;
      for(ti = g->begin(); ti != g->end(); ++ti)
        if(ti->property() == *(cit + 1)) {
          targetFound = true;
          break;
        }
      if(targetFound) {
        VI vi;
        EI ei;
        EID ed(VID(si->descriptor()), VID(ti->descriptor()));
        if(g->find_edge(ed, vi, ei))
          weight = (*ei).property();
      }
    }
    vector<double> currentClearance = EdgeClearance(*cit, *(cit + 1), weight);
    clearanceVec.insert(clearanceVec.end(),
			currentClearance.begin(),
			currentClearance.end());
  }

  //min, max, avg, variance of graph edge variances
  ClearanceStats stats;
  stats.m_pathLength = pathLength;
  stats.m_min = *min_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_max = *max_element(clearanceVec.begin(), clearanceVec.end());
  stats.m_avg = accumulate(clearanceVec.begin(), clearanceVec.end(), 0.0) /
    clearanceVec.size();
  stats.m_clearanceAlongPath = clearanceVec;
  double varSum = 0;
  for(auto&  i : clearanceVec)
    varSum += sqr(i - stats.m_avg);
  stats.m_var = varSum / clearanceVec.size();

  return stats;
}


template<class MPTraits>
vector<double>
ClearanceUtility<MPTraits>::
EdgeClearance(const CfgType& _c1, const CfgType& _c2,
    const WeightType& _weight) {
  //This function calculates the edge clearance
  //for each intermediate cfg and return a vector
  //of their clearances.

  //TODO: if the vector of clearance values for an edge
  //is already available, return it here.

  Environment* env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();

  // Find the LP to use.
  const auto& intermediates = _weight.GetIntermediates();
  const std::string lpLabel = !intermediates.size()
                            and !_weight.GetLPLabel().empty()
                            ? _weight.GetLPLabel()
                            : "sl";

  std::vector<CfgType> waypoints = intermediates;
  waypoints.insert(waypoints.begin(), _c1);
  waypoints.push_back(_c2);

  // Reconstruct the path between the waypoint nodes.
  auto lp = this->GetLocalPlanner(lpLabel);
  std::vector<CfgType> reconEdge = lp->BlindPath(waypoints,
      m_rayTickResolution, m_orientationResolution);

  // Compute the clearance for each configuration.
  std::vector<double> clearance;
  clearance.reserve(reconEdge.size());
  for(auto it = reconEdge.begin(); it != reconEdge.end(); ++it) {
    CDInfo collInfo;
    CfgType clrCfg(robot);
    //Decide which collision info function to use
    CollisionInfo(*it, clrCfg, env->GetBoundary(), collInfo);
    clearance.push_back(collInfo.m_minDist);
  }
  return clearance;
}

/*--------------------------- Medial Axis Utility ----------------------------*/

template<class MPTraits>
MedialAxisUtility<MPTraits>::
MedialAxisUtility(
    string _vcLabel, string _dmLabel,
    bool _exactClearance, bool _exactPenetration,
    size_t _clearanceRays, size_t _penetrationRays,
    bool _useBBX, bool _positional, bool _debug,
    double _epsilon, size_t _historyLength) :
    ClearanceUtility<MPTraits>(_vcLabel, _dmLabel,
      _exactClearance, _exactPenetration, _clearanceRays, _penetrationRays,
      _useBBX, _positional, _debug),
    m_epsilon(_epsilon), m_historyLength(_historyLength) {
  this->SetName("MedialAxisUtility");
}


template<class MPTraits>
MedialAxisUtility<MPTraits>::
MedialAxisUtility(XMLNode& _node) : ClearanceUtility<MPTraits>(_node) {
  this->SetName("MedialAxisUtility");

  //These are now unused in MedialAxisUtilities.
//  m_epsilon = _node.Read("epsilon", false, m_epsilon, 0.0, 1.0,
//      "Epsilon-Close to the MA (fraction of the resolution)");
//  m_historyLength = _node.Read("historyLength", false, m_historyLength,
//      size_t(3), size_t(100), "History Length");

  // Note that all necessary initialization is done in ClearanceUtility
}


template<class MPTraits>
void
MedialAxisUtility<MPTraits>::
Print(ostream& _os) const {
  ClearanceUtility<MPTraits>::Print(_os);
  _os << "\tepsilon::" << m_epsilon << endl
      << "\thistory length::" << m_historyLength << endl;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
PushToMedialAxis(CfgType& _cfg, const Boundary* const _b) {
  // Initialization
  string callee = this->GetNameAndLabel() + "::PushToMedialAxis";
  if(this->m_debug)
    cout << endl << callee << endl << "Being Pushed: " << _cfg;

  auto vcm = this->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars(true);//sets m_retAllInfo to true.

  // If invalid, push to the outside of the obstacle
  bool inCollision = !(vcm->IsValid(_cfg, tmpInfo, callee));

  if(this->m_debug)
    cout << "In-Collision: " << inCollision << endl;
  if(inCollision) {
    if(!PushFromInsideObstacle(_cfg, _b)) {
      if(this->m_debug)
        std::cout << callee << "Returning false from not being able to push "
            "from obstacle" << std::endl;
      return false;
    }
  }
  bool pushed = PushCfgToMedialAxisMidpointRule(_cfg, _b);

  if(!pushed) {
    if(this->m_debug)
      std::cout << callee << "Returning false from not being able to push to MA"
                << std::endl;
    return false;
  }

  if(this->m_debug)
    cout << "Successfully got MA sample: " << _cfg << endl << callee << "::END"
         << endl << endl;

  return true;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
PushFromInsideObstacle(CfgType& _cfg, const Boundary* const _b) {
  string callee = this->GetNameAndLabel() + "::PushFromInsideObstacle";
  if(this->m_debug)
    cout << callee << endl << " CfgType: " << _cfg << endl;

  CfgType transCfg = _cfg;
  CDInfo tmpInfo;
  tmpInfo.ResetVars(true);//true so it returns all info.

  // Determine direction to move. Note that passing true to CollisionInfo means
  // all witnesses will be of opposite validity of the sample (_cfg).
  // This is what we need for pushing from the obstacle.
  if(!this->CollisionInfo(_cfg, transCfg, _b, tmpInfo, true)) {
    if(this->m_debug)
      std::cout << callee + " Returning false from failing to get a witness"
                << std::endl;
    return false;
  }

  // _cfg cannot equal transCfg here, as CollisionInfo would have returned false
  _cfg = transCfg;

  if(this->m_debug) {
    cout << "Clearance Cfg: " << transCfg << endl;
    cout << "FINAL CfgType: " << _cfg << endl << callee << "::END " << endl;
  }

  return true;
}

template<class MPTraits>
inline bool
MedialAxisUtility<MPTraits>::
FuzzyVectorEquality(mathtool::Vector3d _v1, mathtool::Vector3d _v2,
                    double _tolerance) {
  return (_v1 - _v2).norm() <= _tolerance;
}


template<class MPTraits>
bool
MedialAxisUtility<MPTraits>::
PushCfgToMedialAxisMidpointRule(CfgType& _cfg, const Boundary* const _b) {
  //This function is using the premise of taking the midpoint of
  // 2 cfgs in C-Space and calling it the MA. Basically the witness point and
  // the normal to push the cfg along are found as usual, but this normal is
  // simply pushed along until another collision is found, then the midpoint
  // between the two is called the MA cfg.
  //This method means that the witness only needs to be found at the beginning,
  // then the cfg is pushed (checking collisions still) along that line. Only
  // doing the witness/ray shooting a single time is a HUGE benefit.
  //When using Exact clearance/witness finding, the iterations stop when the
  // witness changes. We can only do this in exact since the approximate witness
  // finding is extremely noise-prone and slow.
  string callee = this->GetNameAndLabel() + "::PushCfgToMedialAxisMidpointRule";
  if(this->m_debug)
    cout << callee << endl << "Cfg: " << _cfg << endl;

  auto vcm = this->GetValidityChecker(this->m_vcLabel);

  CDInfo tmpInfo;
  tmpInfo.ResetVars(true);//true so it returns all info.

  // Should already be in free space
  if(!vcm->IsValid(_cfg, tmpInfo, callee)) {
    if(this->m_debug)
      std::cout << callee <<": Returning false due to already invalid"
                << std::endl;
    return false;
  }

  //Get the first witness:
  CfgType firstWitnessCfg(_cfg.GetRobot());
  if(!this->CollisionInfo(_cfg, firstWitnessCfg, _b, tmpInfo, true)) {
    if(this->m_debug)
      std::cout << callee << ": Returning false from not being able to"
                             " find first witness" << std::endl;
    return false;
  }

  Vector3d normalDirection = (tmpInfo.m_robotPoint - tmpInfo.m_objectPoint).normalize();

  //Find the unit normal:
//  CfgType unitDirectionToTickCfgAlong = _Cfg - firstWitnessCfg;
//  unitDirectionToTickCfgAlong /= (unitDirectionToTickCfgAlong.Magnitude());
  CfgType unitDirectionToTickCfgAlong;

  if(this->m_exactClearance)
    unitDirectionToTickCfgAlong = CfgType(normalDirection, _cfg.GetRobot());
  else {
    unitDirectionToTickCfgAlong = _cfg - firstWitnessCfg;
    unitDirectionToTickCfgAlong /= unitDirectionToTickCfgAlong.Magnitude();
  }

  if(this->m_debug)
    std::cout << callee << ": first normal directional cfg from witness = "
              << unitDirectionToTickCfgAlong << std::endl
              << "_cfg = " << _cfg << std::endl << "firstWitnessCfg = "
              << firstWitnessCfg << std::endl;

  //Move along normal until anther witness is found.
  //A better maxIterations might be in order, but this should be fine for now.
  const double maSearchResolution = this->GetEnvironment()->GetPositionRes()
                                    * this->m_maSearchResolutionFactor;
  const size_t maxIterations = _b->GetMaxDist()/this->m_rayTickResolution;
  CfgType magnitudeAndDirectionToTick = unitDirectionToTickCfgAlong
                                        * maSearchResolution;

  bool inBounds = true;
  bool valid = true;
  bool passed = false;
  CfgType tickedCfg = _cfg;
  mathtool::Vector3d firstWitnessVertex = tmpInfo.m_objectPoint;

  //The MA search loop:
  for(size_t tick = 1; tick < maxIterations; tick++) {
    //Tick the cfg:
    tickedCfg += magnitudeAndDirectionToTick;

    //Check if in bounds:
    inBounds = tickedCfg.InBounds(_b);
    if(!inBounds && !this->m_useBBX) {
      if(this->m_debug)
        std::cout << "Returning false from going OOB but not using BBX as "
                     "obstacle" << std::endl;
      return false;
    }

    if(this->m_exactClearance) {
      //Exact, so we want all CD info, specifically the witness point.
      // GetNearestVertexWitness will do this for us, including boundary witness
      // information, if BBX is active as an obstacle. It also returns the
      // IsValid() boolean for _cfg and puts all data in tmpInfo:
      valid = this->GetNearestVertexWitness(tickedCfg, tmpInfo, _b);

      if(!valid || !inBounds) {
        if(this->m_debug)
          std::cout << "Returning false from going invalid when looking for "
                       "second exact witness." << std::endl
                    << "_cfg = " << _cfg << std::endl << "first witness cfg = "
                    << firstWitnessCfg << std::endl << "unit tick direction = "
                    << unitDirectionToTickCfgAlong << std::endl << "Current "
                    "ticked cfg = " << tickedCfg << std::endl << std::endl;
        return false;
      }
      //For exact, we can quit as soon as we have a witness change.
      //I found some issues initially when  doing a pure equality, so check
      // the two witness vertices are sufficiently different.
      static const double threshold = this->GetEnvironment()->GetPositionRes() * 2;

      // @TODO: check the witnesses are sufficiently different before it is
      // considered valid. Currently, using double the environment resolution is
      // a quick fix; however, we should be checking the relative difference
      // between the cfg and its witness to check if the witnesses are
      // sufficiently different from the first witness.
      passed = !FuzzyVectorEquality(
                tmpInfo.m_objectPoint, firstWitnessVertex, threshold);
    }
    else {
      //Do the most simple validity check (not returning all info).
      valid = vcm->IsValid(tickedCfg, callee);
      passed = !inBounds || !valid;
    }

    if(passed) {
      if(this->m_debug && this->m_exactClearance)
        std::cout << callee << ": passed on tick " << tick << " with original "
                  << "witness vertex = " << firstWitnessVertex << " and second "
                  "witness vertex = " << tmpInfo.m_objectPoint
                  << std::endl << std::endl;
      break;// we have found the second and final witness, break out of the loop
    }
  }

  //check that it passed, and didn't just time out.
  if(!passed) {
    if(this->m_debug)
      std::cout << callee << " Returning false as no second witness could "
                              "be found" << std::endl;
    return false;
  }

  //If doing exact, then the MA sample is the midpoint of the two last ticks.
  if(this->m_exactClearance)
    firstWitnessCfg = tickedCfg - magnitudeAndDirectionToTick;

  //If doing approx, the first witness is already set, and the last witness is
  // simply the tickedCfg.

  //Find the midpoint of the two cfgs as the MA cfg:
  CfgType cfgMA = (firstWitnessCfg + tickedCfg)/2.0;

  //Now we have to double check that it's valid, since it's possible that it's
  // not, especially with increase in complexity of environment/narrow passages.
  if(!cfgMA.InBounds(_b) || !vcm->IsValid(cfgMA, callee)) {
    //It's either OOB or it's invalid. This pair of witness won't work for a
    // MA sample, so return false.
    if(this->m_debug)
      std::cout << callee << "Returning false due to invalid midpoint cfg"
                << std::endl;
    return false;
  }

  _cfg = cfgMA;

  if(this->m_debug) {
    VDComment("Final CFG");
    VDAddTempCfg(cfgMA, true);
    VDClearLastTemp();
    VDClearComments();
    VDClearLastTemp();
    VDClearLastTemp();
    cout << "FINAL Cfg: " << _cfg << endl;
  }

  return true;
}

#endif
