#include "route_planner.h"
#include <algorithm>

using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return this->end_node->distance(*node);
    
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {
    
    current_node->FindNeighbors();
    
    for(RouteModel::Node* node:current_node->neighbors){
        node->parent = current_node;
        float f_value = current_node->distance(*node);
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + f_value;
        node->visited = true;
        this->open_list.push_back(node);
        
    }
    
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool Compare(RouteModel::Node* node1, RouteModel::Node* node2){
    return((node1->g_value + node1->h_value) > (node2->g_value + node2->h_value));
}

RouteModel::Node *RoutePlanner::NextNode() {
    
    
    
    sort(open_list.begin(), open_list.end(), Compare);
    
    /*std::vector<std::vector<float>> f_values;
    float id = 0;
    for(RouteModel::Node* nodes : open_list){
        std::vector<float> f_node{nodes->g_value+nodes->h_value, id};
        id++;
        f_values.push_back(f_node);
    }
    
    std::vector<std::vector<float>> f_values_sorted;
    
    float f_highest = f_values[f_values.size()-1][0];
    float id_f = f_values.size()-1;
    int id_fs = 0;
    
    while(f_values.size() > 0){
        
        for(int i = 0; i < f_values.size(); i++){
            if(f_highest < f_values[i][0]){
                f_highest = f_values[i][0];
                id_f = f_values[i][1];
                id_fs = i;
                }
            }
        
        
        f_values_sorted.push_back({f_highest, id_f});
        f_values.erase(f_values.begin()+id_fs);
        
        f_highest = f_values[f_values.size()-1][0];
        id_f = f_values[f_values.size()-1][1];
        
    }
    
    std::vector<RouteModel::Node* > sorted_list;
    
    for(int i = 0; i < f_values_sorted.size(); i++){
        for(int k = 0; k < open_list.size(); k++){
            if((int)f_values_sorted[i][1] == k){
                sorted_list.push_back(open_list[k]);
            }
        }
    }
    
    open_list = sorted_list;*/
    
    RouteModel::Node* lowest_f = open_list[open_list.size()-1];
    open_list.pop_back();
    
    return lowest_f;
    
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
    RouteModel::Node* parent_node = nullptr;
    distance = 0.0f;
    
    path_found.push_back(*current_node);
    
    while(current_node->parent){
        parent_node = current_node->parent;
        distance += current_node->distance(*parent_node);
        path_found.insert(path_found.begin(), *parent_node);
        current_node = parent_node;
        
    }
    

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    
    
    this->start_node->visited = true;
    
    AddNeighbors(current_node);
    
    while(current_node->h_value){
        
        current_node = NextNode();
        
        AddNeighbors(current_node);
        
    }
    m_Model.path = ConstructFinalPath(current_node);
    

}
