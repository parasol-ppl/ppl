#ifndef PMPL_LKH_SEARCH_H_
#define PMPL_LKH_SEARCH_H_


#include "MPLibrary/MPBaseObject.h"
#include "MPProblem/TRPGoalMap.h"
#include "Utilities/XMLNode.h"
#include <LKHInclude/LKH.h>
#include <LKHInclude/Genetic.h>
////////////////////////////////////////////////////////////////////////////////
/// Interfaces with the LKH libray in utils. Uses a heursitic to evaluate
/// variants of the Traveling Salesman Problem.
///
/// Paramters for the LKH library are set in parameter file designated in the
/// XML file. The parameter file must designate the problem file being passed
/// to the library which is written here (problem file called ProblemFile.atsp
/// for now but it needs to be updated to take in different types of problems
/// as ".atsp" is specific to the atypical traveling saleman problem).
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LKHSearch final : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType        CfgType;
    typedef typename MPTraits::WeightType     WeightType;
    typedef typename MPTraits::RoadmapType    Roadmap;


    ///@}
    ///@name Construction
    ///@{

    LKHSearch();

    LKHSearch(XMLNode& _node);

    virtual ~LKHSearch();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Graph Seach

    /// Runs the LKH library ATSP search on a transformed TRP.
    /// @param _map Map representing the TRP problem.
    /// @return Set of path indices for the robots in the group. Index of the
    /// set corresponds to the index of the robot inthe container in the goal map
    /// as input in the TRPTool::Initialize function.
    std::vector<std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor>> SearchTRP(
            TRPGoalMap<MPTraits>* _map);

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// generate adjacency matrix
    /// @param _map goal map that the matrix is being dreated for.
    void CreateAdjacencyMatrix(TRPGoalMap<MPTraits>* _map);

    /// Write graph and parameters to file in LKH Library format from the
    /// adjacency matrix.
    void CreateProblemFile();

    /// Translated LKHmain file from LKH Library. Calls the functionality of the
    /// LKH library.
    void LKHmain();

    /// Reads the output file from the LKH Library.
    /// @return List of vertices visited in the ATSP solution to the transformed
    /// TRP problem.
    std::vector<size_t> ReadPathFile();

    /// Converts the ATSP solution back into a TRP solution for the set of
    /// robots.
    /// @param _nodes List of vertices visted in ATSP solution to the problem.
    /// @param _map TRPGoalMap that represents the TRP problem being solved.
    /// @return Set of paths that each robot in the group takes.
    std::vector<std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor>>
            DeconstructATSPPath(std::vector<size_t> _nodes, TRPGoalMap<MPTraits>* _map);
    ///@}
    ///@name Internal State
    ///{

    size_t m_MAX = 9999;
    bool m_debug{true};

    std::string m_label;

    std::string m_parameterFile{"Examples/LKHParameterFile.par"};
    std::string m_problemFile{"ProblemFile.atsp"};
    std::string m_problemType{"ATSP"}; //TODO need to be able to change in XML
    std::string m_outputFile{"ATSP.txt"};

    std::vector<std::vector<size_t>> m_adjMatrix;
    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
LKHSearch<MPTraits>::
LKHSearch() : MPBaseObject<MPTraits>() {
  this->SetName("LKHSearch");
}

template <typename MPTraits>
LKHSearch<MPTraits>::
LKHSearch(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("LKHSearch");
  m_parameterFile = _node.Read("parameterFile", false,
      "Examples/LKHParameterFile.par", "LKH Parameter File");
  m_label = _node.Read("label", true, "", "LKHSearch Label");
}

template <typename MPTraits>
LKHSearch<MPTraits>::
~LKHSearch()= default;

/*-------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
LKHSearch<MPTraits>::
Initialize() {
}

/*------------------------------- Graph Search  -------------------------------*/

template <typename MPTraits>
std::vector<std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor>>
LKHSearch<MPTraits>::
SearchTRP(TRPGoalMap<MPTraits>* _map){
  CreateAdjacencyMatrix(_map);

  //make problem file for the LKH Library to take in
  CreateProblemFile();

  //call LKH Library on the problem file
  LKHmain();

  //read path from LKH Library output file
  std::vector<size_t> nodes = ReadPathFile();

  return DeconstructATSPPath(nodes, _map);

}



/*---------------------------------- Helpers ----------------------------------*/
template <typename MPTraits>
void
LKHSearch<MPTraits>::
CreateAdjacencyMatrix(TRPGoalMap<MPTraits>* _map){

  if(m_debug){
    std::cout << "creating adjacency matrix" << std::endl;
  }
  const size_t maxDistance = m_MAX;//(size_t)-1;
  //TODO need to figure out how to calculate this rather than arbitrarily
  //picking 100
  const size_t buffer = 100;
  //values from each of the goals
  for(auto start : *_map->GetGoalDescriptors()){
    std::vector<size_t> pathLengths;
    for(auto end : *_map->GetGoalDescriptors()){

      if(start == end){
        pathLengths.push_back(maxDistance);
      }
      else {
        pathLengths.push_back(_map->GetPath(start,end).Length()+buffer);
      }
    }
    //non-edges from goal to depot
    for(size_t i = 0; i <_map->GetDepotDescriptors()->size(); i++){
      pathLengths.push_back(maxDistance);
    }
    //edges from goal to terminal
    for(auto terminal : *_map->GetDepotDescriptors()){
      pathLengths.push_back(_map->GetPath(start,terminal).Length()+buffer);
    }
    //add edges for "start" to the adjacency matrix
    m_adjMatrix.push_back(pathLengths);
  }
  for(auto depot : *_map->GetDepotDescriptors()){
    std::vector<size_t> pathLengths;
    //depot to goal edges
    for(auto goal : *_map->GetGoalDescriptors()){
      auto path = _map->GetPath(depot,goal);
      if(m_debug){
        std::cout << "printing vertices: " << goal << " " << depot << std::endl;
      }
      pathLengths.push_back(path.Length()+buffer);
    }
    for(size_t i = 0; i < _map->GetDepotDescriptors()->size(); i++){
      pathLengths.push_back(maxDistance);
    }
    for(auto depot2 : *_map->GetDepotDescriptors()){
      if(depot == depot2){
        pathLengths.push_back(buffer);
      }
      else {
        pathLengths.push_back(maxDistance);
      }
    }
    m_adjMatrix.push_back(pathLengths);
  }
  for(auto terminal : *_map->GetDepotDescriptors()){
    std::vector<size_t> pathLengths;
    //terminal to goal edges
    for(size_t i = 0; i < _map->GetGoalDescriptors()->size(); i++){
      pathLengths.push_back(maxDistance);
    }
    //terminal to depot edges
    for(auto depot : *_map->GetDepotDescriptors()){
      if(terminal == depot){
        pathLengths.push_back(maxDistance);
      }
      else {
        pathLengths.push_back(0);
      }
    }
    //terminal to terminal edges
    for(size_t i = 0; i < _map->GetDepotDescriptors()->size(); i++){
      pathLengths.push_back(maxDistance);
    }
    m_adjMatrix.push_back(pathLengths);
  }
}


template <typename MPTraits>
void
LKHSearch<MPTraits>::
CreateProblemFile(){
  if(m_debug){
    std::cout << "creating problem file: " << m_problemFile << std::endl;
  }
  std::ofstream myfile;
  myfile.open(m_problemFile);
  myfile << "TYPE: " << m_problemType << std::endl
         << "DIMENSION: " << m_adjMatrix.size() << std::endl
         << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl
         << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl
         << "EDGE_WEIGHT_SECTION" << std::endl;
  for(auto vertex : m_adjMatrix){
    for(auto edge : vertex){
      myfile << edge << "   ";
    }
    myfile << std::endl;
  }

  myfile << "EOF";
  myfile.close();
  if(m_debug){
    std::cout << "closing problem file" << endl;
  }
}



template <typename MPTraits>
void
LKHSearch<MPTraits>::
LKHmain(){
    GainType Cost, OldOptimum;
    double Time, LastTime = GetTime();

    /* Read the specification of the problem */
    //if (argc >= 2)
    ParameterFileName = m_parameterFile.c_str();
    ReadParameters();
    MaxMatrixDimension = 10000;
    ReadProblem();

    if (SubproblemSize > 0) {
        if (DelaunayPartitioning)
            SolveDelaunaySubproblems();
        else if (KarpPartitioning)
            SolveKarpSubproblems();
        else if (KCenterPartitioning)
            SolveKCenterSubproblems();
        else if (KMeansPartitioning)
            SolveKMeansSubproblems();
        else if (RohePartitioning)
            SolveRoheSubproblems();
        else if (MoorePartitioning || SierpinskiPartitioning)
            SolveSFCSubproblems();
        else
            SolveTourSegmentSubproblems();
        return;
    }
    AllocateStructures();
    CreateCandidateSet();
    InitializeStatistics();

    if (Norm != 0)
        BestCost = PLUS_INFINITY;
    else {
        /* The ascent has solved the problem! */
        Optimum = BestCost = (GainType) LowerBound;
        UpdateStatistics(Optimum, GetTime() - LastTime);
        RecordBetterTour();
        RecordBestTour();
        WriteTour(OutputTourFileName, BestTour, BestCost);
        WriteTour(TourFileName, BestTour, BestCost);
        Runs = 0;
    }

    /* Find a specified number (Runs) of local optima */
    for (Run = 1; Run <= Runs; Run++) {
        LastTime = GetTime();
        Cost = FindTour();      /* using the Lin-Kernighan heuristic */
        if (MaxPopulationSize > 1) {
            /* Genetic algorithm */
            int i;
            for (i = 0; i < PopulationSize; i++) {
                GainType OldCost = Cost;
                Cost = MergeTourWithIndividual(i);
                if (TraceLevel >= 1 && Cost < OldCost) {
                    //printff("  Merged with %d: Cost = " GainFormat, i + 1,
                    //        Cost);
                    //if (Optimum != MINUS_INFINITY && Optimum != 0)
                        //printff(", Gap = %0.4f%%",
                        //        100.0 * (Cost - Optimum) / Optimum);
                    //printff("\n");
                }
            }
            if (!HasFitness(Cost)) {
                if (PopulationSize < MaxPopulationSize) {
                    AddToPopulation(Cost);
                    if (TraceLevel >= 1)
                        PrintPopulation();
                } else if (Cost < Fitness[PopulationSize - 1]) {
                    i = ReplacementIndividual(Cost);
                    ReplaceIndividualWithTour(i, Cost);
                    if (TraceLevel >= 1)
                        PrintPopulation();
                }
            }
        } else if (Run > 1)
            Cost = MergeTourWithBestTour();
        if (Cost < BestCost) {
            BestCost = Cost;
            RecordBetterTour();
            RecordBestTour();
            WriteTour(OutputTourFileName, BestTour, BestCost);
            WriteTour(TourFileName, BestTour, BestCost);
        }
        OldOptimum = Optimum;
        if (Cost < Optimum) {
            if (FirstNode->InputSuc) {
                Node *N = FirstNode;
                while ((N = N->InputSuc = N->Suc) != FirstNode);
            }
            Optimum = Cost;
            //printff("*** New optimum = " GainFormat " ***\n\n", Optimum);
        }
        Time = fabs(GetTime() - LastTime);
        UpdateStatistics(Cost, Time);
        //if (TraceLevel >= 1 && Cost != PLUS_INFINITY) {
            //printff("Run %d: Cost = " GainFormat, Run, Cost);
            //if (Optimum != MINUS_INFINITY && Optimum != 0)
                //printff(", Gap = %0.4f%%",
                //        100.0 * (Cost - Optimum) / Optimum);
            //printff(", Time = %0.2f sec. %s\n\n", Time,
            //        Cost < Optimum ? "<" : Cost == Optimum ? "=" : "");
        //}
        if (StopAtOptimum && Cost == OldOptimum && MaxPopulationSize >= 1) {
            Runs = Run;
            break;
        }
        if (PopulationSize >= 2 &&
            (PopulationSize == MaxPopulationSize ||
             Run >= 2 * MaxPopulationSize) && Run < Runs) {
            Node *N;
            int Parent1, Parent2;
            Parent1 = LinearSelection(PopulationSize, 1.25);
            do
                Parent2 = LinearSelection(PopulationSize, 1.25);
            while (Parent2 == Parent1);
            ApplyCrossover(Parent1, Parent2);
            N = FirstNode;
            do {
                if (ProblemType != HCP && ProblemType != HPP) {
                    int d = C(N, N->Suc);
                    AddCandidate(N, N->Suc, d, INT_MAX);
                    AddCandidate(N->Suc, N, d, INT_MAX);
                }
                N = N->InitialSuc = N->Suc;
            }
            while (N != FirstNode);
        }
        SRandom(++Seed);
    }
    PrintStatistics();
    return;
}

template <typename MPTraits>
std::vector<size_t>
LKHSearch<MPTraits>::
ReadPathFile(){
  std::vector<size_t> goals;
  std::ifstream myfile;
  myfile.open(m_outputFile);
  std::string input;
  while(std::getline(myfile, input)){
    //filters other info in output file
    if(isdigit(input[0])){
      std::stringstream ss(input);
      size_t goal;
      ss >> goal;
      goals.push_back(goal-1);
    }
  }
  if(m_debug){
    std::cout << "Goals read from file:";
    for(auto g : goals){
      std::cout << " " << g;
    }
    std::cout << std::endl;
  }
  return goals;
}

template <typename MPTraits>
std::vector<std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor>>
LKHSearch<MPTraits>::
DeconstructATSPPath(std::vector<size_t> _nodes, TRPGoalMap<MPTraits>* _map){
  std::vector<std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor>> pathSets;
  for(size_t i = 0; i < _map->GetDepotDescriptors()->size(); i++){
    //find tour from each depot (aka worker location)
/*    std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor>* tour;
    for(size_t j = 0; j < _nodes.size(); j++){
      //if not on a tour and at the depot in the set of nodes from the LKH file
      // the + goal descriptors size is bc the Depots are listed after the goals
      // in the adjacency matrix and thus their index in the LKH file.
      // Might need to re-evaluate if functionality to add goals in is added
      // into library, but it should still be fine.
      if(!tour and _nodes[j] == i+_map->GetGoalDescriptors()->size()){
        tour = new std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor>();
        tour->push_back(_map->GetGoalDescriptors()->at(j));
      }
      else if(tour){
        //if in a tour and at the terminal created for the depot the tour
        // started from then end the tour. The number of goal and depot
        // descriptors is added becasue the goals and depots are listed before
        // the terminal in the adjacency matrix so the sum of those + i shou;d
        // be the index for the desired terminal. Again this should work even if
        // goals are added later, but that should be double checked.
        if(_nodes[j] == i+_map->GetGoalDescriptors()->size()+_map->GetDepotDescriptors()->size()){
          //tour->push_back(_map->GetGoalDescriptors()->at(j));
          //pathSets.push_back(*tour);
          //delete tour;
          //TODO prevent memory leaks
          break;
        }
        //condition needed to make sure no unused depots get included in tour
        //form another depot.
        else if(j < _map->GetGoalDescriptors()->size()){
          tour->push_back(_map->GetGoalDescriptors()->at(j));
        }
      }
    }
    pathSets.push_back(*tour);
*/

    std::cout << "Looking for: " << _map->GetDepotDescriptors()->at(i) << std::endl;
    std::vector<typename TRPGoalMap<MPTraits>::vertex_descriptor> tour;
    bool complete = false;
    bool onTour = false;
    size_t j = 0;
    while(!complete){
      if(m_debug){
        std::cout << "Looking at " << _nodes[j] << std::endl;
      }
      //if not on a tour and at the depot in the set of nodes from the LKH file
      // the + goal descriptors size is bc the Depots are listed after the goals
      // in the adjacency matrix and thus their index in the LKH file.
      // Might need to re-evaluate if functionality to add goals in is added
      // into library, but it should still be fine.
      if(!onTour and _nodes[j] == _map->GetDepotDescriptors()->at(i)){
        if(m_debug){
          std::cout << "Starting new tour" << std::endl;
        }
        onTour = true;
        tour.push_back(_nodes[j]);
      }
      else if(onTour){
        //if in a tour and at the terminal created for the depot the tour
        // started from then end the tour. The number of goal and depot
        // descriptors is added becasue the goals and depots are listed before
        // the terminal in the adjacency matrix so the sum of those + i shou;d
        // be the index for the desired terminal. Again this should work even if
        // goals are added later, but that should be double checked.
        if(_nodes[j] == _map->GetDepotDescriptors()->at(i)+_map->GetDepotDescriptors()->size()){
          if(m_debug){
            std::cout << "Finished tour" << std::endl;
            for(auto v : tour){
              std::cout << v << " ";
            }
            std::cout << std::endl;
          }
          complete = true;
          pathSets.push_back(tour);
        }
        //condition needed to make sure no unused depots get included in tour
        //form another depot.
        else if(_nodes[j] < _map->GetGoalDescriptors()->size()){
          if(m_debug){
            std::cout << "adding " << _nodes[j] << " to the tour" << std::endl;
          }
          tour.push_back(_nodes[j]);
          if(m_debug){
            std::cout << "added " << _nodes[j] << " to the tour" << std::endl;
          }
        }
      }
      j++;
      if(j >= _nodes.size())
        j = 0;
    }

  }
  return pathSets;


}



/*-----------------------------------------------------------------------------*/

#endif










