#include "TMPLibraryTests.h"

#include "Traits/TMPTestTraits.h"

/*--------------------------- Construction ---------------------------*/

TMPLibraryTests::
TMPLibraryTests() {
  InitializeMethodSets();
}

TMPLibraryTests::
TMPLibraryTests(const std::string& _xmlFile) {
  InitializeMethodSets();
  ReadXMLFile(_xmlFile);
}

TMPLibraryTests::
~TMPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

typename TMPLibraryTests::TestResult
TMPLibraryTests::
RunTest() {

  size_t passed = 0;
  size_t failed = 0;
  size_t total = 0;

  // Task allocator tests
  RunMethodSetTests(*this->m_taskAllocatorTests,passed,failed,total);

  // Task decomposer tests
  RunMethodSetTests(*this->m_taskAllocatorTests,passed,failed,total);

  // Task evaluator tests
  RunMethodSetTests(*this->m_taskAllocatorTests,passed,failed,total);

  bool success = (failed == 0);
  std::string message = "COMPLETED TESTS"
                        "\nTotal: "  + std::to_string(total)  +
                        "\nPassed: " + std::to_string(passed) +
                        "\nFailed: " + std::to_string(failed) +
                        "\n\n\n";

  return std::make_pair(success,message);
}

/*-------------------------- Helper Functions ------------------------*/

void
TMPLibraryTests::
InitializeMethodSets() {
  m_taskAllocatorTests = new TaskAllocatorMethodTestSet(this,
      typename TMPTraits::TaskAllocatorMethodList(), "TaskAllocators");
  m_taskDecomposerTests = new TaskDecomposerMethodTestSet(this,
      typename TMPTraits::TaskDecomposerMethodList(), "TaskDecomposer");
  m_taskEvaluatorTests = new TaskEvaluatorMethodTestSet(this,
      typename TMPTraits::TaskEvaluatorMethodList(), "TaskEvaluators");
}
/*---------------------------- XML Helpers -----------------------------------*/

void
TMPLibraryTests::
ReadXMLFile(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode tmpNode(_filename, "MotionPlanning");

  // Find the 'TMPLibrary' node.
  XMLNode* library = nullptr;
  for(auto& child : tmpNode)
    if(child.Name() == "TMPLibrary")
      library = &child;

  // Throw exception if we can't find it.
  if(!library)
    throw ParseException(WHERE) << "Cannot find TMPLibrary node in XML file '"
                                << _filename << "'.";

  // Parse the library node to set algorithms and parameters.
  for(auto& child : *library)
    ParseChild(child);

}


bool
TMPLibraryTests::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "TaskAllocators") {
    m_taskAllocatorTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskDecomposers") {
    m_taskDecomposerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "TaskEvaluators") {
    m_taskEvaluatorTests->ParseXML(_node);
    return true;
  }
  else
    return false;
}
/*-------------------------------- Debugging ---------------------------------*/
/*--------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
