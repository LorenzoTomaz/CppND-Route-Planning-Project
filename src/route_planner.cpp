#include "route_planner.h"
#include <algorithm>
#include <vector>
using std::vector;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    //OK TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

}


//OK TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node* node) { 
    
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
   current_node->FindNeighbors();
   for(auto elem: current_node->neighbors){
        elem->parent = current_node;
        elem->h_value = CalculateHValue(elem);
        elem->g_value = current_node->g_value + current_node->distance(*elem);
        open_list.push_back(elem);
        elem->visited = true;
   }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
bool Compare(RouteModel::Node * node1, RouteModel::Node * node2 ){
    float f1 = node1->g_value + node1->h_value; // f1 = g1 + h1
    float f2 = node2->g_value + node2->h_value; // f2 = g2 + h2
    return f1 > f2; 
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *lowest = open_list.back();
    open_list.pop_back();
    return lowest;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    std::vector<RouteModel::Node> path_found;

    distance = 0.0f;
    while (current_node->parent != nullptr)
    {
        // add current_node to path_found
        path_found.push_back(*current_node);

        // compute the distance between current_node and its parent node
        distance += current_node->distance(*current_node->parent);

        // update current_node to its parent
        current_node = current_node->parent;

    
    }
    
    path_found.push_back(*current_node);
    std::vector<RouteModel::Node> rpath (path_found.rbegin(), path_found.rend());
    distance *= m_Model.MetricScale();
    return rpath;

}


// TODO 7: Write the A* Search algorithm here.
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// Tips:
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
	open_list.push_back(start_node);
    std::vector<RouteModel::Node*> ptr = open_list;

	/* Our algorithm is running here */
	while(!open_list.empty())
	{
		/* Find best next node for exploration */
		RouteModel::Node* current_node = NextNode();

		/* Check if we have reached our goal already */
		if(current_node->distance(*end_node) == 0)
		{
                 
			 m_Model.path = ConstructFinalPath(end_node);
           
			return;
		}

		/* Otherwise, add our current node to the neighbors */
		AddNeighbors(current_node);
	}
    std::reverse(ptr.begin(), ptr.end());

}
