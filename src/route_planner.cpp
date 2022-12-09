#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node =  &m_Model.FindClosestNode(start_x, start_y );
    end_node = &m_Model.FindClosestNode(end_x, end_y );
   

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
    for ( RouteModel::Node* nodes : current_node->neighbors ) {
        nodes->parent = current_node;
        nodes->h_value = RoutePlanner::CalculateHValue(nodes);
        nodes->g_value = current_node->g_value + current_node->distance(*nodes);
        nodes->visited = true;
        open_list.push_back(nodes);    //open_list has values of node in form of pointer stored in vector format.
    }

}

bool compare(RouteModel::Node* node_1 , RouteModel::Node* node_2 ){
    return( node_1->f_value > node_2->f_value  );
}

RouteModel::Node *RoutePlanner::NextNode() {
  for ( RouteModel::Node* nodes : open_list ) {
    nodes->f_value = (nodes->g_value + nodes->h_value );  
  }
  std::sort(open_list.begin(),open_list.end(), compare);
  RouteModel::Node * point = open_list.back();
    open_list.pop_back();
  return point;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) { 
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
  
    while (current_node != start_node) {
    path_found.push_back(*current_node); //this is end to start ka path
    distance = distance + current_node->distance(*current_node->parent);
    current_node = current_node->parent;
    }
    path_found.push_back(*start_node);

    reverse(path_found.begin(), path_found.end()) ;
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
   
  RouteModel::Node *current_node = nullptr;
  start_node->visited = true;
  open_list.push_back(start_node);  
  while (open_list.size() > 0 ){
    current_node =  NextNode();    
    if (current_node  == end_node) {
      m_Model.path  =  ConstructFinalPath(current_node);
      break;
    }
    AddNeighbors(current_node);
  }

}
