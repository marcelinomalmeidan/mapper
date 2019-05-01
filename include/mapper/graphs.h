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

#ifndef MAPPER_GRAPHS_H_
#define MAPPER_GRAPHS_H_

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include "mapper/visualization_functions.h"
#include "mapper/indexed_octree_key.h"
#include "mapper/priority_queue.h"
#include "mapper/msg_conversions.h"

namespace octoclass {

struct Neighbors{
    std::vector<uint> indexes;
    std::vector<double> costs;
    int n_neighbors;
};

class GraphNode {
 public:
    Eigen::Vector3d pos_;
    Neighbors neighbors_;
    octomap::OcTreeKey key_;

    // Constructors
    explicit GraphNode(const Eigen::Vector3d &pos_in);
    GraphNode(const Eigen::Vector3d &pos_in,
              const octomap::OcTreeKey key_in);
    GraphNode();

    // Methods
    void AddNeighbor(const uint &index, const double &cost);  // Add one neighbor to the set
};


class Graph {
 public:
    std::vector<GraphNode> nodes_;
    uint n_nodes_ = 0;
    uint n_edges_ = 0;

    // Constructors
    Graph();

    // Methods
    void AddNode(const Eigen::Vector3d &pos);
    void AddNode(const Eigen::Vector3d &pos,
                 const octomap::OcTreeKey &key_in,
                 uint *index);
    void AddEdge(const uint &index1,
                 const uint &index2,
                 const double &cost);
    void GraphVisualization(visualization_msgs::Marker *line_list);
    void PathVisualization(std::vector<uint> &total_path,
                           std::vector<uint> &waypoints,
                           visualization_msgs::MarkerArray *markers);
    void Astar(uint init_index, uint final_index, std::vector<uint> &total_path);
    void Astar2(uint init_index, uint final_index, std::vector<uint> &total_path);  // Faster implementation

 protected:
    KeySet nodeSet_;  // Structure used for hashing keysets to find indexes

    // Methods
    double NodeDistance(uint index1, uint index2);
};


class TreeNode {
 public:
    Eigen::Vector3d pos_;
    uint parent_;
    std::vector<uint> children_;
    uint n_children_ = 0;
    double cost_ = 0.0;
    octomap::OcTreeKey key_;

    // Constructor
    TreeNode(const Eigen::Vector3d &pos_in,
             const uint &parent_in,
             const double &cost_in);  // Non-keyed node
    TreeNode(const Eigen::Vector3d &pos_in,
             const octomap::OcTreeKey key_in,
             const uint &parent_in,
             const double &cost_in);  // Keyed node

    // Methods
    void AddChild(const uint &child_index);
};

class Tree {
 public:
    std::vector<TreeNode> tree_;
    uint n_nodes_ = 0;

    // Constructor
    explicit Tree(const Eigen::Vector3d &root);
    Tree(const Eigen::Vector3d &root,
         const octomap::OcTreeKey key);
    Tree();

    // Methods
    Eigen::Vector3d GetRootPos();
    void AddNode(const Eigen::Vector3d &pos,
                 const uint &parent,
                 const double &cost);
    void AddNode(const Eigen::Vector3d &pos,
                 const octomap::OcTreeKey key,
                 const uint &parent,
                 const double &cost);
    void AddNode(const Eigen::Vector3d &pos,
                 const octomap::OcTreeKey key,
                 const uint &parent,
                 const double &cost,
                 uint *index);
    void TreeVisualization(visualization_msgs::Marker *line_list);
};

}  // octoclass

#endif  // MAPPER_GRAPHS_H_
