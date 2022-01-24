#ifndef PRINT_MAP_EVALUATION_H
#define PRINT_MAP_EVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Prints the roadmap to a file
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class PrintMapEvaluation : public MapEvaluatorMethod<MPTraits> {

  public:

    /// Constructs a PrintMapEvaluation object
    PrintMapEvaluation();

    /// Constructs a PrintMapEvaluation object with a base filename
    /// @param _basename The base name of the output file.
    PrintMapEvaluation(string _baseName);

    /// Constructs a PrintMapEvaluation object from an XML node
    /// @param _node The XML node to use.
    PrintMapEvaluation(XMLNode& _node);

    virtual ~PrintMapEvaluation();

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  protected:

    string m_baseName; ///< The basename of the file to write
};

template <typename MPTraits>
PrintMapEvaluation<MPTraits>::PrintMapEvaluation() {
  this->SetName("PrintMapEvaluation");
}

template <typename MPTraits>
PrintMapEvaluation<MPTraits>::PrintMapEvaluation(string _baseName)
  : m_baseName(_baseName) {
  this->SetName("PrintMapEvaluation");
}

template <typename MPTraits>
PrintMapEvaluation<MPTraits>::PrintMapEvaluation(XMLNode& _node)
  : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("PrintMapEvaluation");
  m_baseName = _node.Read("base_name", true, "", "base filename for map output");
}

template <typename MPTraits>
PrintMapEvaluation<MPTraits>::~PrintMapEvaluation() {
}

template <typename MPTraits>
void
PrintMapEvaluation<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
  _os << "\tbase filename = " << m_baseName << endl;
}

/// Write the map to a file
template <typename MPTraits>
bool
PrintMapEvaluation<MPTraits>::operator()() {
  int numNodes = this->GetRoadmap()->get_num_vertices();
  int numEdges = this->GetRoadmap()->get_num_edges();
  ostringstream osName;
  osName << m_baseName << "." << numNodes << "." << numEdges << ".map";
  this->GetRoadmap()->Write(osName.str(), this->GetEnvironment());

  int numCollNodes = this->GetBlockRoadmap()->get_num_vertices();
  int numCollEdges = this->GetBlockRoadmap()->get_num_edges();
  if(numCollNodes) {
    ostringstream osCollName;
    osCollName << m_baseName << "." << numCollNodes << "." << numCollEdges << ".block.map";
    this->GetBlockRoadmap()->Write(osCollName.str(), this->GetEnvironment());
  }

  return true;
}
#endif
