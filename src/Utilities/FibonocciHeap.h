#ifndef FIBONACCI_HEAP_H_
#define FIBONACCI_HEAP_H_

#include "ConfigurationSpace/RoadmapGraph.h"

class FibonacciHeap {
	public:

		typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> Roadmap;

		/** node structure for a node in fibonacci heap */
	struct FibonacciNode {
		int degree;                 //Number of children for this node.
		FibonacciNode *parent;     //Parent pointer.
		FibonacciNode *child;      //Pointer to the first child of a node.
		FibonacciNode *left;       //Pointer to the left sibling.
		FibonacciNode *right;      //Pointer to the right sibling.
		bool mark;                  //Whether the node is marked. Used for cascading cut operation.
		bool is_infinity;
		double key;                    // This corresponds to the distance from the source node.
		size_t m_VID;             //Pointer to the node. Simplifying, we use an int-index to represent each node.
	};

	Fibonacci(Roadmap* _roadmap){
		m_roadmap = _roadmap;
	}

	void fib_heap_insert(FibonacciNode *new_node, double key);

	void fib_heap_existing_to_root(FibonacciNode *new_node);

	void fib_heap_add_child(FibonacciNode *parent_node, FibonacciNode *new_child_node);

	void fib_heap_remove_node_from_root(FibonacciNode *node);

	void fib_heap_link(FibonacciNode *high_node, FibonacciNode *low_node);

	void fib_heap_consolidate();

	FibonacciNode* fib_heap_extract_min();

	void fib_heap_cut(FibonacciNode *node,
		FibonacciNode *node_parent);

	void fib_heap_cascading_cut(FibonacciNode *node);

	void fib_heap_decrease_key(FibonacciNode *node_inst,
		int new_key);

	void fib_heap_delete(FibonacciNode *node);

	int get_min_distant_unmarked_node(int *distance_to_dest, 
			std::unordered_map<size_t,bool> marked);

	int get_min_distant_unmarked_node_fib_heap(std::unordered_map<size_t,FibonacciNode*> node_array, 
			std::unordered_map<size_t,bool> marked);

	void dijkstra_fibanocci(int src);

	/** This function runs the Depth-First-Search to find if the graph is completely connected.*/
	//void run_DFS(int node_index, std::unordered_map<size_t,bool>& discovered);

	bool check_connected(graph *my_graph);



	private:
		FibonacciNode* m_minNode{nullptr};
		size_t m_numNodes{0};
		Roadmap* m_roadmap{nullptr};
		Roadmap* m_highLevelGraph{nullptr};
};
