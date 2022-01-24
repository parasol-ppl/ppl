#ifndef PMPL_VALIDATION_STRATEGY_H_
#define PMPL_VALIDATION_STRATEGY_H_

#include "MPStrategyMethod.h"
#include "nonstd/io.h"


////////////////////////////////////////////////////////////////////////////////
/// Runs another MPStrategy (the target) and compares the output(s) to gold
/// standard(s) specified as input files. This exists to test changes in the
/// code and ensure that the target's behavior hasn't changed.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ValidationStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Local Types
    ///@{

    /// A gold standard input to validate against the target strategy.
    struct GoldStandard {

      /// The types of gold standard files we can read.
      enum class Type {Roadmap,
                       BlockRoadmap,
                       CfgPath,
                       FullCfgPath,
                       History};

      Type        type;       ///< The type of output to check.
      std::string label;      ///< Label for the generated object.
      Robot*      robot;      ///< The robot represented in the output.
      std::string filename;   ///< The input filename.

    };

    ///@}
    ///@name Construction
    ///@{

    ValidationStrategy();

    ValidationStrategy(XMLNode& _node);

    virtual ~ValidationStrategy() = default;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Run() override;
    virtual void Finalize() override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Validate the output of this strategy against a gold standard.
    void ValidateResult(const GoldStandard& _g);

    /// Validate a roadmap output.
    void ValidateRoadmap(const GoldStandard& _g, RoadmapType* const _output);

    /// Validate a path output.
    void ValidatePath(const GoldStandard& _g,
        const std::vector<CfgType>& _output);

    /// Validate a history output.
    void ValidateHistory(const GoldStandard& _g);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_targetLabel;                  ///< Target MPStrategy label.
    std::vector<GoldStandard> m_goldStandards;  ///< Standards to validate.

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
ValidationStrategy<MPTraits>::
ValidationStrategy() : MPStrategyMethod<MPTraits>() {
  this->SetName("ValidationStrategy");
}


template <typename MPTraits>
ValidationStrategy<MPTraits>::
ValidationStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("ValidationStrategy");

  m_targetLabel = _node.Read("target", true, "",
      "The label of the target strategy to validate.");


  // Read gold standards from child nodes.
  for(auto& child : _node) {
    if(child.Name() != "GoldStandard")
      throw ParseException(child.Where()) << "Only child nodes of type "
                                          << "'GoldStandard' are permitted.";

    GoldStandard g;

    // Parse the output type.
    std::string type = child.Read("type", true, "", "The output type.");
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    if(type == "roadmap")
      g.type = GoldStandard::Type::Roadmap;
    else if(type == "blockroadmap")
      g.type = GoldStandard::Type::BlockRoadmap;
    else if(type == "cfgpath")
      g.type = GoldStandard::Type::CfgPath;
    else if(type == "fullcfgpath")
      g.type = GoldStandard::Type::FullCfgPath;
    else if(type == "history")
      g.type = GoldStandard::Type::History;
    else
      throw ParseException(child.Where()) << "Unrecognized type '" << type
                                          << "'.";

    // Get the object label.
    switch(g.type) {
      case GoldStandard::Type::Roadmap:
      case GoldStandard::Type::BlockRoadmap:
      case GoldStandard::Type::CfgPath:
      case GoldStandard::Type::FullCfgPath:
        g.label = child.Read("robot", true, "", "The robot label.");
        break;
      case GoldStandard::Type::History:
        g.label = child.Read("label", true, "", "The history label.");
        break;
      default:
        throw RunTimeException(WHERE) << "Unrecognized type.";
    }

    // Parse the filename.
    g.filename = child.Read("filename", true, "", "The input file name.");

    m_goldStandards.push_back(g);
  }

  if(m_goldStandards.empty())
    throw ParseException(_node.Where()) << "At least one GoldStandard is "
                                        << "required.";
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
ValidationStrategy<MPTraits>::
Initialize() {
  auto problem = this->GetMPProblem();

  // Validate each gold standard.
  for(auto& g : m_goldStandards) {
    // Fetch the robot if needed (throws if not found).
    switch(g.type) {
      case GoldStandard::Type::Roadmap:
      case GoldStandard::Type::BlockRoadmap:
      case GoldStandard::Type::CfgPath:
      case GoldStandard::Type::FullCfgPath:
        g.robot = problem->GetRobot(g.label);
        break;
      case GoldStandard::Type::History:
        break;
      default:
        throw RunTimeException(WHERE) << "Unrecognized type.";
    }

    // Check that file exists.
    g.filename = problem->GetPath(g.filename);
    if(!std::ifstream(g.filename).good())
      throw ParseException(WHERE) << "Could not open file '"
                                  << g.filename << "'.";
  }
}


template <typename MPTraits>
void
ValidationStrategy<MPTraits>::
Run() {
  // Get the target, disable outputs, and run.
  auto target = this->GetMPStrategy(m_targetLabel);
  target->EnableOutputFiles(false);
  (*target)();
}


template <typename MPTraits>
void
ValidationStrategy<MPTraits>::
Finalize() {
  // Verify each gold standard against the output.
  for(const auto& g : m_goldStandards)
    ValidateResult(g);
}

/*-------------------------------- Helpers -----------------------------------*/

template <typename MPTraits>
void
ValidationStrategy<MPTraits>::
ValidateResult(const GoldStandard& _g) {
  switch(_g.type) {
    case GoldStandard::Type::Roadmap:
    {
      auto output = this->GetRoadmap(_g.robot);
      ValidateRoadmap(_g, output);
      break;
    }
    case GoldStandard::Type::BlockRoadmap:
    {
      auto output = this->GetBlockRoadmap(_g.robot);
      ValidateRoadmap(_g, output);
      break;
    }
    case GoldStandard::Type::CfgPath:
    {
      const std::vector<CfgType> output = this->GetPath(_g.robot)->Cfgs();
      ValidatePath(_g, output);
      break;
    }
    case GoldStandard::Type::FullCfgPath:
    {
      const std::vector<CfgType> output = this->GetPath(_g.robot)->FullCfgs(
          this->GetMPLibrary());
      ValidatePath(_g, output);
      break;
    }
    case GoldStandard::Type::History:
    {
      ValidateHistory(_g);
      break;
    }
    default:
      throw RunTimeException(WHERE) << "Unrecognized type.";
  }
}


template <typename MPTraits>
void
ValidationStrategy<MPTraits>::
ValidateRoadmap(const GoldStandard& _g, RoadmapType* const _output) {
  if(this->m_debug)
    std::cout << "Validating "
              << (_g.type == GoldStandard::Type::BlockRoadmap ? "blocked " : "")
              << "roadmap file '" << _g.filename << "'"
              << " for robot '" << _g.label << "'"
              << std::endl;

  // Get the input roadmap.
  RoadmapType input(_g.robot);
  ::Read(&input, _g.filename);

  // Compare.
  const bool ok = *_output == input;

  // Quit on pass.
  if(this->m_debug)
    std::cout << "\t" << (ok ? "Passed" : "Failed") << std::endl;
  if(ok)
    return;

  // Maps are not equal, write the output for inspection.
  const std::string base = this->GetBaseFilename();

  if(_g.type == GoldStandard::Type::Roadmap)
    _output->Write(base + ".map", this->GetEnvironment());
  else
    _output->Write(base + ".block.map", this->GetEnvironment());
}


template <typename MPTraits>
void
ValidationStrategy<MPTraits>::
ValidatePath(const GoldStandard& _g, const std::vector<CfgType>& _output) {
  if(this->m_debug)
    std::cout << "Validating "
              << (_g.type == GoldStandard::Type::FullCfgPath ? "full " : "")
              << "cfg path file '" << _g.filename << "'"
              << " for robot '" << _g.label << "'"
              << std::endl;

  // Get the input path.
  const std::vector<CfgType> input = ReadPath<CfgType>(_g.filename, _g.robot);

  // Compare.
  const bool ok = _output == input;

  // Quit on pass.
  if(this->m_debug)
    std::cout << "\t" << (ok ? "Passed" : "Failed") << std::endl;
  if(ok)
    return;

  // Paths are not equal, write the output for inspection.
  const std::string base = this->GetBaseFilename();
  if(_g.type == GoldStandard::Type::CfgPath)
    ::WritePath(base + ".rdmp.path", _output);
  else
    ::WritePath(base + ".path", _output);
}


template <typename MPTraits>
void
ValidationStrategy<MPTraits>::
ValidateHistory(const GoldStandard& _g) {
  if(this->m_debug)
    std::cout << "Validating history file '" << _g.filename << "' for history "
              << "stat '" << _g.label << "'"
              << std::endl;

  // Get the output.
  auto stats = this->GetStatClass();
  const std::vector<double> output = stats->GetHistory(_g.label);

  // Read the input file.
  double buffer;
  std::vector<double> input;
  input.reserve(output.size());
  std::ifstream ifs(_g.filename);
  while(ifs >> buffer)
    input.push_back(buffer);

  // Compare.
  constexpr double tolerance = std::numeric_limits<double>::epsilon() * 100;
  bool ok = output.size() == input.size();
  for(size_t i = 0; ok and i < output.size(); ++i)
    ok &= nonstd::approx(output[i], input[i], tolerance);

  // Quit on pass.
  if(this->m_debug)
    std::cout << "\t" << (ok ? "Passed" : "Failed") << std::endl;
  if(ok)
    return;

  // Write history.
  stats->WriteHistory(_g.label);
}

/*----------------------------------------------------------------------------*/

#endif
