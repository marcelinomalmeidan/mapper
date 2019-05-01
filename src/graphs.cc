/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "mapper/graphs.h"
#include <limits>
#include <vector>
#include <set>

namespace octoclass {

// Classes for graphs and trees
// Graphs and trees also have associated octomap keys

// Graph Node class ----------------------------------------------------
GraphNode::GraphNode(const Eigen::Vector3d &pos_in) {
    pos_ = pos_in;
    neighbors_.n_neighbors = 0;
}

GraphNode::GraphNode(const Eigen::Vector3d &pos_in,
                     const octomap::OcTreeKey key_in) {
    pos_ = pos_in;
    key_ = key_in;
    neighbors_.n_neighbors = 0;
}

GraphNode::GraphNode() {
    neighbors_.n_neighbors = 0;
}


// Add one neighbor to the set
void GraphNode::AddNeighbor(const uint &index, const double &cost) {
    neighbors_.n_neighbors = neighbors_.n_neighbors + 1;
    neighbors_.indexes.push_back(index);
    neighbors_.costs.push_back(cost);
}

// Graph class ---------------------------------------------------
Graph::Graph() {
    // Do Nothing
}

void Graph::AddNode(const Eigen::Vector3d &pos) {
    n_nodes_ = n_nodes_ + 1;
    nodes_.push_back(GraphNode(pos));
}

void Graph::AddNode(const Eigen::Vector3d &pos,
                    const octomap::OcTreeKey &key_in,
                    uint *index) {
    nodes_.push_back(GraphNode(pos, key_in));
    *index = n_nodes_;
    nodeSet_.insert(IndexedOcTreeKey(key_in, *index));
    n_nodes_ = n_nodes_ + 1;
}

void Graph::AddEdge(const uint &index1,
                    const uint &index2,
                    const double &cost) {
    nodes_[index1].AddNeighbor(index2, cost);
    nodes_[index2].AddNeighbor(index1, cost);
    n_edges_ = n_edges_ + 1;
}

// Return visualization markers for tree visualization
void Graph::GraphVisualization(visualization_msgs::Marker* line_list) {
    // Initializa array
    line_list->header.frame_id = "/map";
    line_list->header.stamp = ros::Time::now();
    line_list->ns = "/Graph";
    line_list->action = visualization_msgs::Marker::ADD;
    line_list->pose.orientation.w = 1.0;
    line_list->type = visualization_msgs::Marker::LINE_LIST;
    line_list->id = 0;
    line_list->scale.x = 0.01;  // Line width
    line_list->color = visualization_functions::Color::Red();

    // Populate visualization markers
    geometry_msgs::Point node1, node2;
    uint neighbor_index;
    // ROS_INFO("Plotting %d nodes!", int(RRTree.n_nodes));
    for (uint index1 = 0; index1 < n_nodes_; index1++) {
        // ROS_INFO("Parent %d has %d children!", int(i), int(RRTree.tree[i].children.size()));
        msg_conversions::eigen_to_ros_point(nodes_[index1].pos_, &node1);
        for (uint j = 0; j < nodes_[index1].neighbors_.indexes.size(); j++) {
            neighbor_index = nodes_[index1].neighbors_.indexes[j];
            if (neighbor_index > index1) {
                msg_conversions::eigen_to_ros_point(nodes_[neighbor_index].pos_, &node2);
                line_list->points.push_back(node1);
                line_list->points.push_back(node2);
                // ROS_INFO("Node (%d), Neighbor (%d), cost: %f", int(index1), int(neighborIndex),
                //                                                (nodes[neighborIndex].Pos-nodes[index1].Pos).norm());
            }
        }
    }
    // ROS_INFO("n_edges: %d", int(nEdges));
    // ROS_INFO("n_edges in Graph: %d", int(n_edges));
}

void Graph::PathVisualization(std::vector<uint> &total_path,
                              std::vector<uint> &waypoints,
                              visualization_msgs::MarkerArray* markers) {
    // Initialize edges marker
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "/Path";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = 0;
    line_list.scale.x = 0.02;  // Line width
    line_list.color = visualization_functions::Color::Green();

    // Populate edges
    uint path_n_points = total_path.size();
    geometry_msgs::Point node;
    if (path_n_points >= 2) {
        msg_conversions::eigen_to_ros_point(nodes_[total_path[0]].pos_, &node);
        line_list.points.push_back(node);
        for (uint i = 1; i < total_path.size()-1; i++) {
            msg_conversions::eigen_to_ros_point(nodes_[total_path[i]].pos_, &node);
            line_list.points.push_back(node);
            line_list.points.push_back(node);
        }
        msg_conversions::eigen_to_ros_point(nodes_[total_path[total_path.size()-1]].pos_, &node);
        line_list.points.push_back(node);
    }

    // Populate the waypoints
    visualization_msgs::Marker endpoints;
    endpoints.header.frame_id = "/world";
    endpoints.header.stamp = ros::Time::now();
    endpoints.ns = "/Path";
    endpoints.action = visualization_msgs::Marker::ADD;
    endpoints.type = visualization_msgs::Marker::SPHERE_LIST;
    endpoints.id = 1;
    endpoints.scale.x = 0.1;
    endpoints.scale.y = 0.1;
    endpoints.scale.z = 0.1;
    endpoints.color = visualization_functions::Color::Blue();
    for (uint i = 0; i < waypoints.size(); i++) {
        msg_conversions::eigen_to_ros_point(nodes_[waypoints[i]].pos_, &node);
        endpoints.points.push_back(node);
    }

    markers->markers.push_back(endpoints);
    markers->markers.push_back(line_list);
}

void Graph::Astar(uint init_index, uint final_index, std::vector<uint> &total_path) {
    std::set<uint> closed_set;
    std::set<uint> open_set;
    std::vector<uint> comeFrom(n_nodes_);
    std::vector<double> f_score(n_nodes_, std::numeric_limits<float>::infinity());
    std::vector<double> g_score(n_nodes_, std::numeric_limits<float>::infinity());
    uint current_index, neighbor_index, n_neighbors;
    double cur_cost, tentative_cost;

    // Initialize algorithm
    open_set.insert(init_index);
    g_score[init_index] = 0;
    f_score[init_index] = NodeDistance(init_index, final_index);

    while (open_set.size() > 0) {
        // Find the node in open_set having the lowest fScore[] value
        cur_cost = std::numeric_limits<float>::infinity();
        for (std::set<uint>::iterator it = open_set.begin(); it != open_set.end(); ++it) {
            // ROS_INFO("fscores: %f", fScore[*it]);
            if (f_score[*it] < cur_cost) {
                current_index = *it;
                cur_cost = f_score[current_index];
                // openSetCurIndex = idx;
            }
        }

        // Delete from open_set, increment closedSet
        open_set.erase(current_index);
        closed_set.insert(current_index);

        // Check whether we reached goal
        if (current_index == final_index) {
            // total_path.push_back(current_index);
            total_path.insert(total_path.begin(), current_index);
            while (current_index != init_index) {
                current_index = comeFrom[current_index];
                // total_path.push_back(current_index);
                total_path.insert(total_path.begin(), current_index);
            }
            break;
        }

        // Check neighbors
        n_neighbors = nodes_[current_index].neighbors_.indexes.size();
        // std::vector<uint> neighbors(n_neighbors);
        for (uint i = 0; i < n_neighbors; i++) {
            neighbor_index = nodes_[current_index].neighbors_.indexes[i];

            // Check if neighbor is in closed set already
            if (closed_set.find(neighbor_index) != closed_set.end()) {
                continue;  // If already in set, go to next neighbor
            }

            // Also, if not in open set, add to it
            open_set.insert(neighbor_index);

            // Check whether this neighbor leades to a new path
            tentative_cost = g_score[current_index] + nodes_[current_index].neighbors_.costs[i];
            if (tentative_cost >= g_score[neighbor_index]) {
                continue;
            }

            // Best path until now: record it!
            comeFrom[neighbor_index] = current_index;
            g_score[neighbor_index] = tentative_cost;
            f_score[neighbor_index] = g_score[neighbor_index] + NodeDistance(neighbor_index, final_index);
        }
    }
}

void Graph::Astar2(uint init_index, uint final_index, std::vector<uint> &total_path) {
    // Declare variables
    PriorityQueue<uint, double> queue;      // uint index, double cost
    std::vector<uint> comeFrom(n_nodes_);
    std::vector<double> costSoFar(n_nodes_, std::numeric_limits<float>::infinity());
    uint current_index, neighbor_index, n_neighbors;
    double tentative_cost, priority;

    // Initialize algorithm
    queue.put(init_index, 0.0);
    comeFrom[init_index] = init_index;
    costSoFar[init_index] = 0;

    while (!queue.empty()) {
        current_index = queue.get();

        // Check whether we reached goal
        if (current_index == final_index) {
            total_path.insert(total_path.begin(), current_index);
            // totalPtotal_pathath.push_front(current_index);
            while (current_index != init_index) {
                current_index = comeFrom[current_index];
                total_path.insert(total_path.begin(), current_index);
                // total_path.push_front(current_index);
            }
            break;
        }

        // Check neighbors
        n_neighbors = nodes_[current_index].neighbors_.indexes.size();
        for (uint i = 0; i < n_neighbors; i++) {
            neighbor_index = nodes_[current_index].neighbors_.indexes[i];

            // Check whether this neighbor leades to a new path
            tentative_cost = costSoFar[current_index] + nodes_[current_index].neighbors_.costs[i];
            if (tentative_cost < costSoFar[neighbor_index]) {
                costSoFar[neighbor_index] = tentative_cost;
                priority = tentative_cost + NodeDistance(neighbor_index, final_index);
                queue.put(neighbor_index, priority);
                comeFrom[neighbor_index] = current_index;
            }
        }
    }
}

double Graph::NodeDistance(uint index1, uint index2) {
    return (nodes_[index1].pos_ - nodes_[index2].pos_).norm();
}

// Tree Node class -----------------------------------------------
TreeNode::TreeNode(const Eigen::Vector3d &pos_in,
                   const uint &parent_in,
                   const double &cost_in) {
    pos_ = pos_in;
    parent_ = parent_in;
    cost_ = cost_in;
}

TreeNode::TreeNode(const Eigen::Vector3d &pos_in,
                   const octomap::OcTreeKey key_in,
                   const uint &parent_in,
                   const double &cost_in) {
    pos_ = pos_in;
    key_ = key_in;
    parent_ = parent_in;
    cost_ = cost_in;
}

void TreeNode::AddChild(const uint &child_index) {
    n_children_ = n_children_ + 1;
    children_.push_back(child_index);
}


// Tree class ----------------------------------------------------
Tree::Tree(const Eigen::Vector3d &root) {
    // Create Root node, set the parent to itself and cost = 0
    tree_.push_back(TreeNode(root, 0, 0.0));
    n_nodes_ = 1;
}

Tree::Tree(const Eigen::Vector3d &root,
           const octomap::OcTreeKey key) {
    tree_.push_back(TreeNode(root, key, 0, 0.0));
    n_nodes_ = 1;
}

Tree::Tree() {
    // Do Nothing
}

// Return the position of the root of the tree
Eigen::Vector3d Tree::GetRootPos() {
    return tree_[0].pos_;
}

void Tree::AddNode(const Eigen::Vector3d &pos,
                   const uint &parent,
                   const double &cost) {
    tree_.push_back(TreeNode(pos, parent, cost));
    tree_[parent].AddChild(n_nodes_);
    n_nodes_ = n_nodes_ + 1;
}

void Tree::AddNode(const Eigen::Vector3d &pos,
                   const octomap::OcTreeKey key,
                   const uint &parent,
                   const double &cost) {
    tree_.push_back(TreeNode(pos, key, parent, cost));
    tree_[parent].AddChild(n_nodes_);
    n_nodes_ = n_nodes_ + 1;
}

void Tree::AddNode(const Eigen::Vector3d &pos,
                   const octomap::OcTreeKey key,
                   const uint &parent,
                   const double &cost,
                   uint *index) {
    tree_.push_back(TreeNode(pos, key, parent, cost));
    tree_[parent].AddChild(n_nodes_);
    *index = n_nodes_;
    n_nodes_ = n_nodes_ + 1;
}

// Return visualization markers for tree visualization
void Tree::TreeVisualization(visualization_msgs::Marker* line_list) {
    // Initializa array
    line_list->header.frame_id = "/world";
    line_list->header.stamp = ros::Time::now();
    line_list->ns = "/Tree";
    line_list->action = visualization_msgs::Marker::ADD;
    line_list->pose.orientation.w = 1.0;
    line_list->type = visualization_msgs::Marker::LINE_LIST;
    line_list->id = 0;
    line_list->scale.x = 0.01;  // Line width
    line_list->color = visualization_functions::Color::Red();

    // Populate visualization markers
    geometry_msgs::Point parent, child;
    uint child_index;
    // ROS_INFO("Plotting %d nodes!", int(RRTree.n_nodes));
    for (uint i = 0; i < n_nodes_; i++) {
        // ROS_INFO("Parent %d has %d children!", int(i), int(RRTree.tree[i].children.size()));
        msg_conversions::eigen_to_ros_point(tree_[i].pos_, &parent);
        for (uint j = 0; j < tree_[i].children_.size(); j++) {
            child_index = tree_[i].children_[j];
            msg_conversions::eigen_to_ros_point(tree_[child_index].pos_, &child);
            line_list->points.push_back(parent);
            line_list->points.push_back(child);
            // ROS_INFO("Parent: (%f, %f, %f)", parent.x, parent.y, parent.z);
            // ROS_INFO("Child: (%f, %f, %f)", child.x, child.y, child.z);
        }
    }
    // ROS_INFO("n_edges: %d", int(line_list->points.size()));
}

}  // octoclass