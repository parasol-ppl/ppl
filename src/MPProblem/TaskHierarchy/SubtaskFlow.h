#ifndef PPL_SUBTASK_FLOW_H_
#define PPL_SUBTASK_FLOW_H_

#include "SemanticTask.h"

#include <containers/sequential/graph/graph.h>

struct TBDFunction {
	enum Operator {Max,Min};
	Operator m_operator{Max};
	std::vector<TBDFunction> m_subFunctions;
	std::vector<size_t> m_elems;
};

struct FlowNode {
	std::vector<size_t> m_subNodes;
	SemanticTask* m_task;
	TBDFunction m_initiationFunction;
};


class SubtaskFlow : public stapl::sequential::graph<stapl::DIRECTED,
													 stapl::NONMULTIEDGES, FlowNode, SemanticTask::DependencyType> {

	public: 
		///@name Local Types
		///@{

		typedef std::pair<std::vector<size_t>,TBDFunction> ParentInfo;

    using STAPLGraph =
#ifdef _PARALLEL
    stapl::dynamic_graph<stapl::DIRECTED,stapl::NONMULTIEDGES,
                         FlowNode, SemanticTask::DependencyType>
#else
    stapl::sequential::graph<stapl::DIRECTED,stapl::NONMULTIEDGES,
                             FlowNode, SemanticTask::DependencyType>
#endif
    ;

		typedef typename STAPLGraph::vertex_iterator VI;

		///@}
		///@name Construction
		///@{
		
		SubtaskFlow(SemanticTask* _task);

		~SubtaskFlow();

		///@}
		///@name Debug
		///@{

		void Print();

		///@}
		///@name Accessors
		///@{

		size_t Size();

		VI GetFlowNodeIter(SemanticTask* _task);
		VI GetFlowNodeIter(size_t _vid);

		FlowNode GetFlowNode(size_t _vid);

		VI GetRootIter();

		size_t GetSuperNode(size_t _vid);
		///@}

	private:
		///@name Base Class Unhiding
		///@{

		using STAPLGraph::add_vertex;
		using STAPLGraph::delete_vertex;
		using STAPLGraph::add_edge;
		using STAPLGraph::delete_edge;

		///@}
		///@name Helper Functions
		///@{

		ParentInfo EvalNode(SemanticTask* _task, ParentInfo _parentInfo);

		ParentInfo HandleDependencies(SemanticTask* _task, ParentInfo _parentInfo);

		ParentInfo MergeNodes(SemanticTask* _one, SemanticTask* _two, ParentInfo _parentInfo);

		///@}
		///@name Internal State
		///@{

		std::unordered_map<SemanticTask*,size_t> m_taskNodeMap;

		size_t m_root;

		std::unordered_map<size_t,size_t> m_superNodeMap;

		///@}

};

#endif
