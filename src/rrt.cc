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

#include <limits>
#include <vector>
#include "mapper/rrt.h"

namespace octoclass {

// RRT class -----------------------------------------------------
RRT::RRT(const Eigen::Vector3d &Root, const double steer_param) {
    rrtree_ = Tree(Root);
    steer_param_ = steer_param;
    cloud_ptr_->push_back(pcl::PointXYZ(Root[0], Root[1], Root[2]));
    kdtree_.setInputCloud(cloud_ptr_);
}

// Return the position of the root of the tree
Eigen::Vector3d RRT::GetRootPos() {
    return rrtree_.GetRootPos();
}

// Get a sample within a cube with side = 2*boxLim
void RRT::SampleNodeBox(const double box_lim,
                   Eigen::Vector3d *sample) {
    *sample = rrtree_.tree_[0].pos_ + box_lim*Eigen::Vector3d::Random();
}

void RRT::SampleNodeBox(const Eigen::Vector3d &box_min,
                        const Eigen::Vector3d &box_max,
                        Eigen::Vector3d *sample) {
    const Eigen::Vector3d center = (box_max + box_min)/2.0;
    const Eigen::Vector3d range = (box_max - box_min)/2.0;
    *sample = SampleNodeBox(center, range);
}

Eigen::Vector3d RRT::SampleNodeBox(const Eigen::Vector3d &center,
                                   const Eigen::Vector3d &range) {
    return (center + range.cwiseProduct(Eigen::Vector3d::Random()));
}

void RRT::BruteNN(const Eigen::Vector3d &sample,
                  uint *NNindex,
                  double *NNcost) {
    static double curDist;
    *NNcost = std::numeric_limits<float>::infinity();
    *NNindex = 0;
    for (uint i = 0; i < rrtree_.n_nodes_; i++) {
        curDist = DistanceToNode(i, sample);
        if (curDist < *NNcost) {
            *NNcost = curDist;
            *NNindex = i;
        }
    }
}

// Steer function: modifies sample and cost
void RRT::Steer(const uint &node_index,
                Eigen::Vector3d *sample,
                double *cost) {
    // Change the values of sample and cost based on distance from nearest neighbor
    static Eigen::Vector3d direction;
    if (*cost > steer_param_) {
        direction = *sample - rrtree_.tree_[node_index].pos_;
        *sample = rrtree_.tree_[node_index].pos_ + steer_param_*direction.normalized();
        *cost = steer_param_;
    }
}

// Get the euclidean distance between two nodes
double RRT::NodeDistance(const uint index1,
                         const uint index2) {
    return (rrtree_.tree_[index1].pos_ - rrtree_.tree_[index2].pos_).norm();
}

// Get the euclidean distance between a point and a node
double RRT::DistanceToNode(const uint index,
                           const Eigen::Vector3d pos) {
    return (rrtree_.tree_[index].pos_ - pos).norm();
}

void RRT::SetKdtree() {
    kdtree_.setInputCloud(cloud_ptr_);
}

// octoRRT class -----------------------------------------------------
OctoRRT::OctoRRT(const Eigen::Vector3d &root,
                 const double &steer_param,
                 const double &resolution_in) {
    const octomap::point3d query = octomap::point3d(root[0], root[1], root[2]);
    const octomap::OcTreeKey key = node_octree_.coordToKey(query);
    // octomap::point3d nodeCenter = node_octree_.keyToCoord(key);
    // Root = Eigen::Vector3d(nodeCenter.x(), nodeCenter.y(), nodeCenter.z());
    rrtree_ = Tree(root, key);
    steer_param_ = steer_param;
    node_octree_.setResolution(resolution_in);
    node_octree_.updateNode(key, true);
    node_set_.insert(IndexedOcTreeKey(key, 0));
}

// Return the position of the root of the tree
Eigen::Vector3d OctoRRT::GetRootPos() {
    return rrtree_.GetRootPos();
}

// Add a new node to the tree (if not added already)
void OctoRRT::AddNode(const Eigen::Vector3d &pos,
                      const uint &parent,
                      const double &cost) {
    const octomap::point3d query = octomap::point3d(pos[0], pos[1], pos[2]);
    const octomap::OcTreeKey key = node_octree_.coordToKey(query);
    const octomap::OcTreeNode* node = node_octree_.search(key);
    if (node != NULL) {
        if (!node_octree_.isNodeOccupied(node)) {
            // octomap::point3d nodeCenter = node_octree_.keyToCoord(key);
            // pos = Eigen::Vector3d(nodeCenter.x(), nodeCenter.y(), nodeCenter.z());
            node_octree_.updateNode(key, true);
            uint index;
            rrtree_.AddNode(pos, key, parent, cost, &index);
            node_set_.insert(IndexedOcTreeKey(key, index));
        }
    } else {
        // octomap::point3d nodeCenter = node_octree_.keyToCoord(key);
        // pos = Eigen::Vector3d(nodeCenter.x(), nodeCenter.y(), nodeCenter.z());
        node_octree_.updateNode(key, true);
        uint index;
        rrtree_.AddNode(pos, key, parent, cost, &index);
        node_set_.insert(IndexedOcTreeKey(key, index));
    }
}

// Get a sample within a cube with side = 2*boxLim
void OctoRRT::SampleNodeBox(const double box_lim,
                            Eigen::Vector3d *sample) {
    *sample = rrtree_.tree_[0].pos_ + box_lim*Eigen::Vector3d::Random();
}

void OctoRRT::SampleNodeBox(const Eigen::Vector3d &box_min,
                            const Eigen::Vector3d &box_max,
                            Eigen::Vector3d *sample) {
    Eigen::Vector3d center = (box_max + box_min)/2.0;
    Eigen::Vector3d range = (box_max - box_min)/2.0;
    *sample = SampleNodeBox(center, range);
}

Eigen::Vector3d OctoRRT::SampleNodeBox(const Eigen::Vector3d &center,
                                       const Eigen::Vector3d &range) {
    return (center + range.cwiseProduct(Eigen::Vector3d::Random()));
}

void OctoRRT::NodesWithinBox(const Eigen::Vector3d &box_min,
                             const Eigen::Vector3d &box_max,
                             std::vector<Eigen::Vector3d> *indexes) {
    octomap::OcTree::leaf_bbx_iterator it;
    for (it = node_octree_.begin_leafs_bbx(octomap::point3d(box_min[0], box_min[1], box_min[2]),
                                         octomap::point3d(box_max[0], box_max[1], box_max[2]));
                                         it != node_octree_.end_leafs_bbx(); ++it) {
        octomap::point3d pos = it.getCoordinate();
        indexes->push_back(Eigen::Vector3d(pos.x(), pos.y(), pos.z()));
        // key = it.getKey();
        // key2index = node_set_.find(IndexedOcTreeKey(key, 0));
        // indexes.push_back(key2index->index);
    }
}

void OctoRRT::OctoNN(const Eigen::Vector3d &sample,
                     uint *nn_index,
                     double *nn_cost) {
    static Eigen::Vector3d box_min, box_max;
    static float step, radius;
    step = 1.5;
    std::vector<Eigen::Vector3d> candidates, radius_candidates;
    do {
        do {
            radius = step*steer_param_;
            box_min << sample[0]-radius,
                      sample[1]-radius,
                      sample[2]-radius;
            box_max << sample[0]+radius,
                      sample[1]+radius,
                      sample[2]+radius;
            NodesWithinBox(box_min, box_max, &candidates);
            step = step + 1;
        } while (candidates.size() == 0);

        // Check if any of the candidates are within radius
        for (uint i = 0; i < candidates.size(); i++) {
            if ((candidates[i] - sample).norm() <= radius) {
                radius_candidates.push_back(candidates[i]);
            }
        }
    } while (radius_candidates.size() == 0);


    // Search for the nearest neighbor within candidates
    *nn_cost = std::numeric_limits<float>::infinity();
    static double cur_dist;
    static uint min_index;
    min_index = 0;
    for (uint i = 0; i < radius_candidates.size(); i++) {
        cur_dist = (radius_candidates[i] - sample).norm();
        if (cur_dist < *nn_cost) {
            *nn_cost = cur_dist;
            min_index = i;
        }
    }

    // Find the graph index of the nearest neighbor
    static octomap::point3d query;
    static octomap::OcTreeKey key;
    static std::tr1::unordered_set<IndexedOcTreeKey, IndexedOcTreeKey::KeyHash>::const_iterator key2index;
    query = octomap::point3d(radius_candidates[min_index][0],
                             radius_candidates[min_index][1],
                             radius_candidates[min_index][2]);
    key = node_octree_.coordToKey(query);
    key2index = node_set_.find(IndexedOcTreeKey(key, 0));
    *nn_index = key2index->index_;
}

// Steer sample towards root
void OctoRRT::SteerToRoot(Eigen::Vector3d *sample,
                          const double &steer_param) {
    // Change the values of sample and cost based on distance from nearest neighbor
    static Eigen::Vector3d direction;
    direction = *sample - this->GetRootPos();
    const double directionNorm = direction.norm();
    if (directionNorm > steer_param) {
        *sample = this->GetRootPos() + (steer_param/directionNorm)*direction;
    }
}

// Steer function: modifies sample and cost
void OctoRRT::Steer(const uint &node_index,
                    Eigen::Vector3d *sample,
                    double *cost) {
    // Change the values of sample and cost based on distance from nearest neighbor
    static Eigen::Vector3d direction;
    if (*cost > steer_param_) {
        direction = *sample - rrtree_.tree_[node_index].pos_;
        *sample = rrtree_.tree_[node_index].pos_ + steer_param_*direction.normalized();
        *cost = steer_param_;
    }
}

// Get the euclidean distance between two nodes
double OctoRRT::NodeDistance(const uint index1,
                             const uint index2) {
    return (rrtree_.tree_[index1].pos_ - rrtree_.tree_[index2].pos_).norm();
}

// Get the euclidean distance between a point and a node
double OctoRRT::DistanceToNode(const uint index,
                               const Eigen::Vector3d pos) {
    return (rrtree_.tree_[index].pos_ - pos).norm();
}

} // octoclass
