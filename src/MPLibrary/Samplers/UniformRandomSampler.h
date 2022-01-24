#ifndef PMPL_UNIFORM_RANDOM_SAMPLER_H_
#define PMPL_UNIFORM_RANDOM_SAMPLER_H_

#include "SamplerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// This sampler only validity-checks the input sample. It is only a
/// 'uniform random' sampler if given a uniform random distribution of input
/// samples.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class UniformRandomSampler : virtual public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename SamplerMethod<MPTraits>::BoundaryMap;

    ///@}
    ///@name Construction
    ///@{

    UniformRandomSampler();

    UniformRandomSampler(XMLNode& _node);

    virtual ~UniformRandomSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid)
        override;

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid)
        override;

    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid)
        override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_vcLabel;  ///< The validity checker to use.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler() {
  this->SetName("UniformRandomSampler");
}


template <typename MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("UniformRandomSampler");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
UniformRandomSampler<MPTraits>::
Print(std::ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel
      << std::endl;
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
bool
UniformRandomSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid) {
  // Check Validity.
  auto vc = this->GetValidityChecker(m_vcLabel);
  const std::string callee = this->GetNameAndLabel() + "::Sampler";
  const bool isValid = vc->IsValid(_cfg, callee);

  // Store result.
  if(isValid)
    _valid.push_back(_cfg);
  else
    _invalid.push_back(_cfg);

  // Debug.
  if(this->m_debug) {
    std::cout << "Sampled Cfg: " << _cfg.PrettyPrint()
              << "\n\tBoundary: " << *_boundary
              << "\n\tValidity:  " << isValid
              << std::endl;

    VDClearAll();
    VDAddTempCfg(_cfg, isValid);
    VDComment("UniformSampling::Cfg " + std::string(isValid ? "" : "in") +
        "valid");
  }

  return isValid;
}


template <typename MPTraits>
bool
UniformRandomSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {
  BoundaryMap emptyMap;
  return Sampler(_cfg, emptyMap, _valid, _invalid);
}


template <typename MPTraits>
bool
UniformRandomSampler<MPTraits>::
Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
    std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {
  // Check Validity.
  auto vc = this->GetValidityChecker(m_vcLabel);
  const std::string callee = this->GetNameAndLabel() + "::Sampler";
  const bool isValid = vc->IsValid(_cfg, callee);

  // Store result.
  if(isValid)
    _valid.push_back(_cfg);
  else
    _invalid.push_back(_cfg);

  // Debug.
  if(this->m_debug) {
    std::cout << "Sampled Cfg: " << _cfg.PrettyPrint()
              << "\n\tValidity:  " << isValid
              << std::endl;
  }

  return isValid;
}

/*----------------------------------------------------------------------------*/

#endif
