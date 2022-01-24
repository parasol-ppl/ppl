/*
 To run the program:
 compile:
 g++ dijkstra.cpp
 ----> this generated a.out in current directory
 run:
 ./a.out -s input
 -----> for simple scheme with input as input file
 ./a.out -f input
 -----> for fibonacci scheme with input as input file
 ./a.out -r n d x
 -----> for random mode n vertices and d% density and x source node.
 This program implements dijkstra's algorithm for obtaining single source shortest paths in two ways.
 1.Simple sheme: In this scheme the edge weights are extracted and updated in an array
 2.Fibonacci heap: In this scheme the weights are extracted and are updated through a fibonacci heap
 The code supports two modes for running the program.
 1.Random mode
 2.User input mode
 In random mode, a graph is generated with number of vertices and density as input. Using rand function, all the edges and their
 costs are generated and added to the graph. Then both the schemes are called to run the algorithm.
 In user input mode, the input from the a file is read and along with the input as which scheme should the algorithm run in.
 The implementation of Fibonacci heap is based on the algorithm described on CLRS while the adjacency list representation is
 based on the one suggested by Guido van Rossum where hash table is used to associate each vertex in a graph with an array of
 adjacent vertices. There is no explicit representation of edges as objects..
 */
#include <iostream>
#include <map>
#include <list>
#include <climits>
#include <stdio.h>
#include <memory>
#include <cmath>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <vector>

using namespace std;

//vector<int> *shortest_order;
//int *distance_to_dest;
//int num_vertices;

/** Node structure in a Graph. Instead, we are simplifying by representing each node by its index (int) */
/*struct node {
	int index;
};
*/
/** Structure for each element in a Graph's adjacency list*/
/*
struct adj_node {
	int dest_node; //Index of the node to which the current node is connected to.
	int edge_len; //Edge weight.
};
*/
/** Adjacency list representation of the graph. We use a Hash Table to map each vertex to its
    adjacent nodes
*/
/*
struct graph {
	int num_vertices; //Number of vertices in the graph. This is provided as an input from the user.
	map<int, list<adj_node*> > adjacent_list_map; //Map for each vertex.
};
*/

/** This is the fibonacci heap data structure which has min pointer
    of type fibonacci node and total nodes in the heap*/
/*struct fibonacci_heap {
	FibonacciNode *min_node;
	int num_nodes;
	fibonacci_heap()
	{
	    num_nodes = 0;
	    min_node = NULL;
	}
};
*/

/*
int get_min_distant_unmarked_node(graph* graph_obj, int *distance_to_dest,
		bool *marked);
*/

/** This function inserts a new node into the fibonacci heap. Depending on the key mentioned,
    heap min pointer is changed appropriately and degree incremented by one.
*/
void 
FibonacciHeap::
fib_heap_insert(FibonacciNode *new_node, double key) {

	FibonacciNode *min_node = m_minNode;
	new_node->key = key;
	new_node->degree = 0;
	new_node->parent = NULL;
	new_node->child = NULL;
	new_node->left = new_node;
	new_node->right = new_node;
	new_node->mark = false;
	new_node->is_infinity = false;
	//Concatenating the root list containing new_node with root list of the heap. We place the new node next to the min node in case it is not NULL.
	if (min_node != NULL) {
		FibonacciNode* min_left_temp = min_node->left;
		min_node->left = new_node;
		new_node->right = min_node;
		new_node->left = min_left_temp;
		min_left_temp->right = new_node;
	}

	//Updating the min pointer.
	if (min_node == NULL || min_node->key > new_node->key) {
		m_minNode = new_node;
	}

	m_numNodes++;
}

/** This function deals with adding truncated nodes to the root list.
    mark value of the node is set to FALSE as it is equivalent to the node being insert first time. */
void 
FibonacciHeap::
fib_heap_existing_to_root(FibonacciNode *new_node) {

	FibonacciNode *min_node = m_minNode;
	new_node->parent = NULL; //Updating the parent pointer to null.
	new_node->mark = false;   //Setting the mark value of the node to false.

	/*Concatenating the root list containing new_node with root list of the heap.
	We place the new node to the left of the min node in case it is not NULL.*/
	if (min_node != NULL) {
		FibonacciNode* min_left_temp = min_node->left;
		min_node->left = new_node;
		new_node->right = min_node;
		new_node->left = min_left_temp;
		min_left_temp->right = new_node;

		if (min_node->key > new_node->key) {
			m_minNode = new_node;
		}
	} else { //Case when there are no nodes in the root list already existing.
		m_minNode = new_node;
		new_node->right = new_node;
		new_node->left = new_node;
	}
}

/** This function combines two fibonacci heaps. This function isn't required for Dijkstra implementation.*/
/*fibonacci_heap *fib_heap_union(fibonacci_heap *fib_heap_first,
		fibonacci_heap *fib_heap_second) {

	fibonacci_heap *appended_fib_heap = fib_heap_make();
	FibonacciNode *first_heap_min_node = fib_heap_first->min_node;
	FibonacciNode *second_heap_min_node = fib_heap_second->min_node;

	if (fib_heap_second->min_node == NULL) {
		appended_fib_heap->min_node = first_heap_min_node;
	} else if (fib_heap_first->min_node == NULL) {
		appended_fib_heap->min_node = second_heap_min_node;
	} else { //When both are not NULL
		//Concatenating the root list of H2 with the root list of H
		FibonacciNode *first_heap_min_node_left_temp =
				first_heap_min_node->left;
		FibonacciNode *second_heap_min_node_left_temp =
				second_heap_min_node->left;
		first_heap_min_node->left = second_heap_min_node_left_temp;
		second_heap_min_node->left = first_heap_min_node_left_temp;
		first_heap_min_node_left_temp->right = second_heap_min_node;
		second_heap_min_node_left_temp->right = first_heap_min_node;

		if (second_heap_min_node->key < first_heap_min_node->key) {
			appended_fib_heap->min_node = second_heap_min_node;
		} else {
			appended_fib_heap->min_node = first_heap_min_node;
		}
	}

	appended_fib_heap->num_nodes = fib_heap_first->num_nodes
			+ fib_heap_second->num_nodes;
	return appended_fib_heap;
}*/

/** This function adds the new_child_node to the parent_node child list. Degree of the parent in incremented
    by one as well to reflect to the node*/
void 
FibonacciHeap::
fib_heap_add_child(FibonacciNode *parent_node,
		FibonacciNode *new_child_node) {

	if (parent_node->degree == 0) {
		parent_node->child = new_child_node;
		new_child_node->right = new_child_node;
		new_child_node->left = new_child_node;
		new_child_node->parent = parent_node;
	} else {
		FibonacciNode* first_child = parent_node->child;
		FibonacciNode* first_child_left_temp = first_child->left;
		first_child->left = new_child_node;
		new_child_node->right = first_child;
		new_child_node->left = first_child_left_temp;
		first_child_left_temp->right = new_child_node;
	}
	new_child_node->parent = parent_node;
	parent_node->degree = parent_node->degree + 1;
}

/** This function is used to truncate a child node from a sibling list. It could be a root list as well. */
void 
FibonacciHeap::
fib_heap_remove_node_from_root(FibonacciNode *node) {

    //if chld has siblings,then remove it from sibling list by traversing
	if (node->right != node) {
		node->right->left = node->left;
		node->left->right = node->right;
	}

    /*if the node being removed is the child node of parent, we have to set
	 some other sibling of child as child for parent.*/
	if (node->parent != NULL) {
		int parent_degree = node->parent->degree;
		if (parent_degree == 1) {
            // if its the only child then parent has no more children. so set to null.
			node->parent->child = NULL;
		} else {
			node->parent->child = node->right;
		}
		//Parent's degree is updated from truncating the child node.
		node->parent->degree = node->parent->degree - 1;
	}
}

/** This function links two nodes as part of the consolidation operation.
    First the larger node is sliced from its sibling list and is then
    added as a child to the smaller node
    */
void 
FibonacciHeap::
fib_heap_link(FibonacciNode *high_node, FibonacciNode *low_node) {

	fib_heap_remove_node_from_root(high_node);
	fib_heap_add_child(low_node, high_node);
	//mark value of the larger node is marked false.
	high_node->mark = false;

}

/** This method is the crucial method for fibonacci heaps where all the actual time
    is spent in pairwise merging of all the trees, to ensure at the end of the operation
    there are no two trees with the same degree.
    we scan the root list with the help of min pointer in the heap and combine trees with
    same degree into a single tree. We do this with the help of a auxillary table to see if
    there is a tree with degree already existing in our heap.
*/
void 
FibonacciHeap::
fib_heap_consolidate(/*fibonacci_heap *heap_inst*/) {

	int node_degree;
	int count = 0, root_count = 0; //root_count is used to count the number of nodes in the root list.

	if (m_numNodes > 1) { //When the number of nodes is less then 1, consolidate makes no sense.
		int degree_table_size = m_numNodes;
		vector<FibonacciNode*> degree_table; //This is the table via which the degrees are compared for consolidation.
		FibonacciNode *current_node = m_minNode, *start_node = m_minNode;
		
		FibonacciNode *existing_node_degree_array, *current_consolidating_node;

		FibonacciNode *temp_node = m_minNode, *iterating_node = m_minNode;
        //Calculating the number of nodes in the root list.
		do {
			root_count++;
			iterating_node = iterating_node->right;
		} while (iterating_node != temp_node);

		while (count < root_count) {
			current_consolidating_node = current_node;
			current_node = current_node->right; // This actually means node for next iternation.
			node_degree = current_consolidating_node->degree;
			while (true) { //We traverse the table each time until all same degree nodes are merged.
				while (node_degree >= degree_table.size()) {
					degree_table.push_back(NULL); //This is a way by which we avoid segmentation fault for accessing a index in the degree table.
				}
				if (degree_table[node_degree] == NULL) { //Adding the current node to the degree table since empty.
					degree_table[node_degree] = current_consolidating_node;
					break;
				} else {//When there is already a node existing with the same degree
					existing_node_degree_array = degree_table[node_degree];

					if (current_consolidating_node->key
							> existing_node_degree_array->key) {

						//swapping the nodes.
						FibonacciNode * temp_node = current_consolidating_node;
						current_consolidating_node = existing_node_degree_array;
						existing_node_degree_array = temp_node;
					}
					if (existing_node_degree_array
							== current_consolidating_node)break;
                    //Linking the larger of the nodes to the smaller one.
					fib_heap_link(/*heap_inst, */existing_node_degree_array,
							current_consolidating_node);
                    //Making the current degree index as null as it is incremented now.
					degree_table[node_degree] = NULL;
					node_degree++;
				}
			}
			count++;
		}

        //Adding the different degree nodes back to the root list.
		m_minNode = NULL;
		for (int i = 0; i < degree_table.size(); i++) {
			if (degree_table[i] != NULL) {
				fib_heap_existing_to_root(/*heap_inst, */degree_table[i]);
			}
		}
	}
}

/** This function extracts the minimum value from the heap based on the min_node pointer that
    each heap structure maintains. In order to verify the correctness of the fibonacci implementation
    with the array one, I am writing out each extracted min node to a file for making the comparision
    easier.
*/
FibonacciNode*
FibonacciHeap::
fib_heap_extract_min(/*fibonacci_heap *heap_inst*/) {

	FibonacciNode *min_node = m_minNode;
	//ofstream myfile;
	//myfile.open("order_fib.txt", std::ios_base::app);

    //Writing out the minimum node to the file.
	//myfile << "FIB -- MIN NODE " << min_node->m_VID << endl;

	if (min_node != NULL) {
		//Add each child in the extracted node to the root list.
		int degree = min_node->degree;
		FibonacciNode *current_child = min_node->child;
		FibonacciNode *removed_child;
		int count = 0; //Count for the children being added to the root list.

        //Iterate till all the children nodes are added to the root list.
		while (count < degree) {
			removed_child = current_child;
			current_child = current_child->right;
			fib_heap_existing_to_root(removed_child);
			count++;
		}

        //Removing the extracted node from the root list.
		fib_heap_remove_node_from_root(min_node);
		//Decrementing the number of nodes as a node has been removed.
		m_numNodes--;
		if (m_numNodes == 0) { //only one node at the root level.
			m_minNode = NULL;
		} else { //More than one node at the root level.
			m_minNode = min_node->right; //It may not be the real min node.
			//Removing from the root list.
			FibonacciNode *min_node_left_temp = min_node->left;
			m_minNode->left = min_node_left_temp;
			min_node_left_temp->right = m_minNode;
			//Once
			fib_heap_consolidate();
		}
	}
	return min_node;
}

/** This method removes the child node from its parent in the heap.*/
void 
FibonacciHeap::
fib_heap_cut(/*fibonacci_heap *heap_inst, */FibonacciNode *node,
		FibonacciNode *node_parent) {

	fib_heap_remove_node_from_root(node);
	fib_heap_existing_to_root(/*heap_inst, */node);

}

/** This recursive function removes the nodes from heap if the child mark
    values are true. It goes from child to parent until it sees a parent
    whose child mark value is false. It sets the child mark of last parent
    as true since it just lost a child.*/
void 
FibonacciHeap::
fib_heap_cascading_cut(/*fibonacci_heap *heap_inst, */FibonacciNode *node) {

	FibonacciNode *parent_node = node->parent;
	if (parent_node != NULL) {
        // case when parent is having child mark false
		if (node->mark == false) {
			node->mark = true;
		} else {
		    // remove this node and recurse up the path until mark is false
			fib_heap_cut(/*heap_inst, */node, parent_node);
			fib_heap_cascading_cut(/*heap_inst, */parent_node);
		}
	}

}

/** This method sets the key field of node to amount specified as argument.
    Based on the new value it may be removed from the parent and called for
    cut and cascade methods.*/
void 
FibonacciHeap::
fib_heap_decrease_key(/*fibonacci_heap *heap_inst, */FibonacciNode *node_inst,
		double new_key) {

	int old_key = node_inst->key;

	if (new_key > old_key) {
		return;
	}

	node_inst->key = new_key;
	if (node_inst->parent != NULL) {
	    /*  new weight is less than parent weight, so it must be cut
            from the parent*/
		if (node_inst->key < node_inst->parent->key) {
			FibonacciNode *parent_node = node_inst->parent;
			fib_heap_cut(/*heap_inst, */node_inst, node_inst->parent);
			fib_heap_cascading_cut(/*heap_inst, */parent_node);
		}
	}

    //Updating the heap's min pointer appropriately.
	if (node_inst->key < m_minNode->key) {
		m_minNode = node_inst;
	}

}

/** This function can be used to delete a key from the heap. This is
    achieved by decreasing the key value to the minimum value available
    in the data type being used. This function is not in use for our current
    implementation.
    */
void 
FibonacciHeap::
fib_heap_delete(/*fibonacci_heap *heap_inst, */FibonacciNode *node) {
	fib_heap_decrease_key(/*heap_inst, */node, INT_MIN);
	fib_heap_extract_min(/*heap_inst*/);
}

/** This function creates a graph with the number of vertices specified by
    the user.*/
/*graph* create_graph(int graph_size) {
	graph *graph_inst = new graph;
	graph_inst->num_vertices = graph_size;
	return graph_inst;
}
*/

/** This function implements the Dijkstra algorithm using the routine array way for calculating
    the next minimum distant vertex from the source node.*/
/*
void dijkstra_normal(graph* graph_instance, int src) {
    //Setting the start time.
	clock_t Time, Start = clock();
	int count_marked = 0;
	//Maintaining the distance from the source for each of the destination nodes.
	distance_to_dest = new int[graph_instance->num_vertices];
	bool *marked = new bool[graph_instance->num_vertices];
    //Distances initialized to INT_MAX or infinity.
	for (int i = 0; i < graph_instance->num_vertices; i++) {
		distance_to_dest[i] = INT_MAX;
	}
    //PS:This marked variable is different fromt he fibonnaci mark value. This denotes
    //whether the node has already reached min distance.
	for (int i = 0; i < graph_instance->num_vertices; i++) {
		marked[i] = false;
	}

	distance_to_dest[src] = 0; //distance to the source to make equal to zero.
	while (count_marked < graph_instance->num_vertices) { // Add each min distant node in each iteration.
		get_min_distant_unmarked_node(graph_instance, distance_to_dest, marked);
		count_marked++;
	}

    //Total execution time.
	Time = clock() - Start;
	//Printing out the output to a file for better comparision with fibonacci.
	ofstream myfile;
	myfile.open("output_normal.txt", std::fstream::out | std::fstream::trunc);

	for (int i = 0; i < graph_instance->num_vertices; i++) {
		myfile << i << " : " << distance_to_dest[i] << endl;
	}

	cout << "\nTotal execution time using Simple Scheme : "
			<< (double) Time / CLOCKS_PER_SEC << " secs\n";
	myfile.close();
}
*/

/** This function calculates the minimum distant node using the fibonacci heap.*/
/*int 
FibonacciHeap::
get_min_distant_unmarked_node_fib_heap(graph* graph_obj,
		fibonacci_heap *heap, FibonacciNode **node_array, bool *marked);*/

/** This function implements the Dijkstra algorithm using the fibonacci heap. Source vertex
    is specified by the user as input.*/
void 
FibonacciHeap::
dijkstra_fibanocci(/*graph* graph_instance, */int src) {
//TODO NEED TO MODIFY FOR MAMTP METHOD AND REMOVE UNNECESSARY STUFF

	int count_marked = 0;
    //List of all fibonacci node pointers to each of the vertices.
	//FibonacciNode **node_array =
	//		new FibonacciNode*[m_roadmap->num_vertices];

	std::unordered_map<size_t,FibonacciNode*> node_array(m_roadmap->num_vertices);
	//This is a boolean variable denoting is the destination vertex has reached minimum distant value.
	//bool *marked = new bool[graph_instance->num_vertices];
	std::unordered_map<size_t,bool> marked;
	//fibonacci_heap *heap_inst = new fibonacci_heap;
    //First, we insert all the vertices into the fibonacci heap with their weights being maximum except for the source node.
	/*for (int i = 0; i < graph_instance->num_vertices; i++) {
		marked[i] = false;
		node_array[i] = new FibonacciNode;
		if (i == src) {
			fib_heap_insert(node_array[i], 0);
		} else {
			fib_heap_insert(node_array[i], INT_MAX);
		}
		node_array[i]->m_VID = i;
	}*/

	//First, we insert all the vertices into the fibonacci heap with their weights being maximum except for the source node.
	for(auto vit = m_roadmap->begin(); vit != m_roadmap->end(); vit++){
		marked[vit->descriptor()] = false;
		auto fibNode = new FibonacciNode();
		if(vit->descriptor() == src){
			fib_heap_insert(fibNode, 0);
		} else {
			fib_heap_insert(fibNode, INT_MAX);
		}
		node_array[i]->m_VID = i;

	}

	while (count_marked < m_roadmap->Size()) { // Add each min distant node in each iteration.
		get_min_distant_unmarked_node_fib_heap(node_array, marked);
		count_marked++;
	}
}

/** This function calculates the min distant unmarked node using the normal array implementation.
    This uses the Greedy approach.*/
int 
FibonacciHeap::
get_min_distant_unmarked_node(/*graph* graph_obj, */int *distance_to_dest,
		std::unordered_map<size_t,bool>& marked) {
    //Initializing the min distance of all vertices except the source as MAX_INT.
	int min_distance = INT_MAX;
	int min_node_index;

	for (int i = 0; i < m_roadmap->Size()/*graph_obj->num_vertices*/; i++) {
		if ((!marked[i]) && (min_distance >= distance_to_dest[i])) {
			min_distance = distance_to_dest[i];
			min_node_index = i;
		}
	}
    //Writing out the extracted min distant node to a file for knowing the order later.
	ofstream myfile;
	myfile.open("order_normal.txt", std::ios_base::app);
	myfile << "NORMAL -- MIN NODE " << min_node_index << ":";

	if (shortest_order[min_node_index].size() < 0) {
		for (int i = 0; i < shortest_order[min_node_index].size(); i++) {
			myfile << shortest_order[min_node_index][i] << "-";
		}
	}

	myfile << endl;
	myfile.close();
	//marking the node as already calculated.
	marked[min_node_index] = true;

	//TODO NEED to update this to new format
	//Updating the distances for the adjacent vertices of min distant unmarked node.
	/*list<adj_node*> adj_list_for_node =
			graph_obj->adjacent_list_map[min_node_index];
	for (list<adj_node*>::iterator it = adj_list_for_node.begin();
			it != adj_list_for_node.end(); ++it) {
		int adj_node_index = (*it)->dest_node;
		if (marked[adj_node_index] == false) {
			if (distance_to_dest[adj_node_index]
					> distance_to_dest[min_node_index] + (*it)->edge_len) {
				distance_to_dest[adj_node_index] =
						distance_to_dest[min_node_index] + (*it)->edge_len;
				shortest_order[adj_node_index].push_back(min_node_index);
			}
		}
	}*/
	//Updating the distances for the adjacent vertices of min distant unmarked node.
	auto vit = m_roadmap->GetVertex(min_node_index);
		for(auto eit = vit->begin(); eit != vit->end(); eit++){
			if(eit->source() != min_node_index)
				continue;
			auto adj_node_index = eit->target();
			if (marked[adj_node_index] == false) {
				//THIS IS THE NEW EXTRA MODIFIED STUFF
				//Check if Node is virtual
				//TODO::Pass in the ROBOTSELECTION Function as a lambda
				if(node_array[adj_node_index]->key == -1){
					Robot* m_roadmap->GetVertex(adj_node_index)
					double readyTime;
				}
				else if (node_array[adj_node_index]->key
						> node_array[min_node_index]->key + (*it)->edge_len) {

					fib_heap_decrease_key(/*heap, */node_array[adj_node_index],
							node_array[min_node_index]->key + eit->property().GetWeight(););
				}
			}
		}

	return min_node_index;
}

/** This function calculates the minimum distant node using the fibonacci heap.*/
int 
FibonacciHeap::
get_min_distant_unmarked_node_fib_heap(/*graph* graph_obj,
		fibonacci_heap *heap, */std::unordered_map<size_t,FibonacciNode*>& node_array, 
		std::unordered_map<size_t,bool>& marked) {

	int min_distance = INT_MAX;
	FibonacciNode *min_node;
	int min_node_index;
	//Extracting the min distant node.
	min_node = fib_heap_extract_min(/*heap*/);

	if (min_node != NULL) {
		min_node_index = min_node->m_VID;
		//Marking that the node has reached its min distance.
		marked[min_node_index] = true;
		

		//TODO NEEDS to be changed to new format
		//Updating the distances for the adjacent vertices of min distant unmarked node.
		/*list<adj_node*> adj_list_for_node =
				graph_obj->adjacent_list_map[min_node_index];
		for (list<adj_node*>::iterator it = adj_list_for_node.begin();
				it != adj_list_for_node.end(); ++it) {
			int adj_node_index = (*it)->dest_node;

			if (marked[adj_node_index] == false) {
				if (node_array[adj_node_index]->key
						> node_array[min_node_index]->key + (*it)->edge_len) {

					fib_heap_decrease_key(node_array[adj_node_index],
							node_array[min_node_index]->key + (*it)->edge_len);
				}
			}
		}*/

		auto vit = m_roadmap->GetVertex(min_node_index);
		for(auto eit = vit->begin(); eit != vit->end(); eit++){
			if(eit->source() != min_node_index)
				continue;
			auto adj_node_index = eit->target();
			if (marked[adj_node_index] == false) {
				if (node_array[adj_node_index]->key
						> node_array[min_node_index]->key + (*it)->edge_len) {

					fib_heap_decrease_key(/*heap, */node_array[adj_node_index],
							node_array[min_node_index]->key + eit->property().GetWeight(););
				}
			}
		}

	}
	return min_node_index;
}

/** This function runs the Depth-First-Search to find if the graph is completely connected.*/
/*void 
FibonacciHeap::
run_DFS(int node_index, std::unordered_map<size_t,bool>& discovered) {

	discovered[node_index] = true;

	//TODO NEED to update this to new format
	list<adj_node*> adj_list_for_node = my_graph->adjacent_list_map[node_index];



	for (list<adj_node*>::iterator it = adj_list_for_node.begin();
			it != adj_list_for_node.end(); ++it) {
        //Running recursively DFS on the child nodes.
		if (discovered[(*it)->dest_node] != true)
			run_DFS((*it)->dest_node, discovered);
	}
}
*/
/** This function check if the graph is connected by running Depth-Frist-Search on the graph.*/
bool 
FibonacciHeap::
check_connected(/*graph *my_graph*/) {
	//TODO NEED to update all of this and may be able to use RoadmapGraph's base functions
    //Array showing if a node is connected the graoh.
	bool *discovered = new bool[my_graph->num_vertices];
	for (int i = 0; i < my_graph->num_vertices; i++) {
		discovered[i] == false;
	}
    //Running Depth-First-Search.
	run_DFS(my_graph, 0, discovered);
	for (int i = 0; i < my_graph->num_vertices; i++) {
		if (discovered[i] == false) {
			return false;
		}
	}
	return true;
}

