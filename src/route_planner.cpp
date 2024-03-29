#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel& model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
	// Convert inputs to percentage:
	start_x *= 0.01;
	start_y *= 0.01;
	end_x *= 0.01;
	end_y *= 0.01;
	m_Model = model;


	RoutePlanner::start_node = &model.FindClosestNode(start_x, start_y);
	RoutePlanner::end_node = &model.FindClosestNode(end_x, end_y);
	// Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

}



float RoutePlanner::CalculateHValue(RouteModel::Node const* node) {
	return node->distance(*end_node);
}




void RoutePlanner::AddNeighbors(RouteModel::Node* current_node) {
	current_node->FindNeighbors();
	for (auto node : current_node->neighbors) {
		node->parent = current_node;
		node->h_value = CalculateHValue(node);
		node->g_value = current_node->g_value + current_node->distance(*node);
		RoutePlanner::open_list.emplace_back(node);
		node->visited = true;
	}
}



RouteModel::Node* RoutePlanner::NextNode() {
	sort(open_list.begin(), open_list.end(), [](const auto& _1st, const auto& _2nd) { return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value; });
	RouteModel::Node* p = *(open_list.begin());
	open_list.erase(open_list.begin());
	return p;
}




std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node* current_node) {
	// Create path_found vector
	distance = 0.0f;
	std::vector<RouteModel::Node> path_found;


	while (current_node != start_node) {
		path_found.push_back(*(current_node));
		distance += current_node->distance(*(current_node->parent));
		current_node = current_node->parent;
	}
	path_found.push_back(*start_node);
	distance += (path_found.end() - 1)->distance(*start_node);
	reverse(path_found.begin(), path_found.end());
	distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
	return path_found;

}



void RoutePlanner::AStarSearch() {
	RouteModel::Node* current_node = nullptr;


	start_node->visited = true;
	open_list.emplace_back(start_node);

	while (!open_list.empty()) {
		current_node = NextNode();

		if ((current_node->distance(*end_node) == 0) && current_node == end_node) {
			m_Model.path = ConstructFinalPath(current_node);
			return;
		}

		AddNeighbors(current_node);
	}
}