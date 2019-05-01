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
#include "mapper/rrg.h"

namespace octoclass {

// RRG Class -----------------------------------------------------
RRG::RRG(const double &steer_param) {
    steer_param_ = steer_param;
}

// index is returned as zero if the node cannot be added
void RRG::AddNode(const Eigen::Vector3d &pos,
                  uint *index) {
    const octomap::point3d query = octomap::point3d(pos[0], pos[1], pos[2]);
    octomap::OcTreeKey key = node_octree_.coordToKey(query);
    const octomap::OcTreeNode* node = node_octree_.search(key);
    *index = 0;
    if (node != NULL) {
        if (!node_octree_.isNodeOccupied(node)) {
            // octomap::point3d nodeCenter = node_octree_.keyToCoord(key);
            // pos = Eigen::Vector3d(nodeCenter.x(), nodeCenter.y(), nodeCenter.z());
            node_octree_.updateNode(key, true);
            rrgraph_.AddNode(pos, key, index);
            node_set_.Insert(key, *index);
        }
    } else {
        // octomap::point3d nodeCenter = node_octree_.keyToCoord(key);
        // pos = Eigen::Vector3d(nodeCenter.x(), nodeCenter.y(), nodeCenter.z());
        node_octree_.updateNode(key, true);
        rrgraph_.AddNode(pos, key, index);
        node_set_.Insert(key, *index);
    }
    // cloudPtr->push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
}

void RRG::AddEdge(const uint &index1,
                  const uint &index2,
                  const double &cost) {
    rrgraph_.AddEdge(index1, index2, cost);
}

void RRG::AddEdge(const uint &index1,
                  const uint &index2) {
    AddEdge(index1, index2, this->NodeDistance(index1, index2));
}

void RRG::SampleNodeBox(const double &box_lim,
                        Eigen::Vector3d *sample) {
    *sample = box_lim*Eigen::Vector3d::Random();
}

void RRG::SampleNodeBox(const Eigen::Vector3d &box_min,
                        const Eigen::Vector3d &box_max,
                        Eigen::Vector3d *sample) {
    const Eigen::Vector3d center = (box_max + box_min)/2.0;
    const Eigen::Vector3d range = (box_max - box_min)/2.0;
    *sample = center + range.cwiseProduct(Eigen::Vector3d::Random());
}

Eigen::Vector3d RRG::SampleNodeBox(const Eigen::Vector3d &center,
                                   const Eigen::Vector3d &range) {
    return (center + range.cwiseProduct(Eigen::Vector3d::Random()));
}

void RRG::NodesWithinBox(const Eigen::Vector3d &box_min,
                         const Eigen::Vector3d &box_max,
                         std::vector<Eigen::Vector3d> *nodes) {
    octomap::OcTree::leaf_bbx_iterator it;
    for (it = node_octree_.begin_leafs_bbx(octomap::point3d(box_min[0], box_min[1], box_min[2]),
                                           octomap::point3d(box_max[0], box_max[1], box_max[2]));
                                           it != node_octree_.end_leafs_bbx(); ++it) {
        octomap::point3d pos = it.getCoordinate();
        nodes->push_back(Eigen::Vector3d(pos.x(), pos.y(), pos.z()));
        // key = it.getKey();
        // key2index = node_set_.find(IndexedOcTreeKey(key, 0));
        // indexes.push_back(key2index->index);
    }
}

void RRG::NodesWithinRadius(const double radius,
                            const Eigen::Vector3d center,
                            std::vector<Eigen::Vector3d> *nodes,
                            std::vector<double> *costs) {
    std::vector<Eigen::Vector3d> candidates;
    static Eigen::Vector3d boxMin, boxMax;
    boxMin << center[0]-radius,
              center[1]-radius,
              center[2]-radius;
    boxMax << center[0]+radius,
              center[1]+radius,
              center[2]+radius;
    NodesWithinBox(boxMin, boxMax, &candidates);

    // Check if any of the candidates are within radius
    const double radiusSquare = radius*radius;
    for (uint i = 0; i < candidates.size(); i++) {
        const Eigen::Vector3d distVec = candidates[i] - center;
        const double distSqr = distVec.transpose()*distVec;
        if (distSqr <= radiusSquare) {
            nodes->push_back(candidates[i]);
            costs->push_back(sqrt(distSqr));
        }
    }
}

void RRG::NodesWithinRadius(const double radius,
                            const Eigen::Vector3d center,
                            std::vector<uint> *indexes,
                            std::vector<double> *costs) {
    std::vector<Eigen::Vector3d> node_pos;
    NodesWithinRadius(radius, center, &node_pos, costs);

    static octomap::point3d query;
    static octomap::OcTreeKey key;
    static uint index;
    for (uint i = 0; i < node_pos.size(); i++) {
        query = octomap::point3d(node_pos[i][0], node_pos[i][1], node_pos[i][2]);
        key = node_octree_.coordToKey(query);
        node_set_.Key2Index(key, &index);
        indexes->push_back(index);
    }
}

void RRG::OctoNN(const Eigen::Vector3d &sample,
                 uint *nn_index,
                 double *nn_cost) {
    static Eigen::Vector3d boxMin, boxMax;
    static float step, radius;
    step = 1.5;
    std::vector<Eigen::Vector3d> candidates, radiusCandidates;
    do {
        do {
            radius = step*steer_param_;
            boxMin << sample[0]-radius,
                      sample[1]-radius,
                      sample[2]-radius;
            boxMax << sample[0]+radius,
                      sample[1]+radius,
                      sample[2]+radius;
            NodesWithinBox(boxMin, boxMax, &candidates);
            step = step + 1;
        } while (candidates.size() == 0);

        // Check if any of the candidates are within radius
        for (uint i = 0; i < candidates.size(); i++) {
            if ((candidates[i] - sample).norm() <= radius) {
                radiusCandidates.push_back(candidates[i]);
            }
        }
    } while (radiusCandidates.size() == 0);


    // Search for the nearest neighbor within candidates
    *nn_cost = std::numeric_limits<float>::infinity();
    static double cur_dist;
    static uint min_index;
    min_index = 0;
    for (uint i = 0; i < radiusCandidates.size(); i++) {
        cur_dist = (radiusCandidates[i] - sample).norm();
        if (cur_dist < *nn_cost) {
            *nn_cost = cur_dist;
            min_index = i;
        }
    }

    // Find the graph index of the nearest neighbor
    static octomap::point3d query;
    static octomap::OcTreeKey key;
    // static std::tr1::unordered_set<IndexedOcTreeKey, IndexedOcTreeKey::KeyHash>::const_iterator key2index;
    query = octomap::point3d(radiusCandidates[min_index][0],
                             radiusCandidates[min_index][1],
                             radiusCandidates[min_index][2]);
    key = node_octree_.coordToKey(query);
    node_set_.Key2Index(key, nn_index);
    // key2index = node_set_.find(IndexedOcTreeKey(key, 0));
    // nn_index = key2index->index;
}

// Steer function: modifies sample and cost
void RRG::Steer(const uint &node_index,
                Eigen::Vector3d *sample,
                double *cost) {
    // Change the values of sample and cost based on distance from nearest neighbor
    static Eigen::Vector3d direction;
    if (*cost > steer_param_) {
        direction = *sample - rrgraph_.nodes_[node_index].pos_;
        *sample = rrgraph_.nodes_[node_index].pos_ + steer_param_*direction.normalized();
        *cost = steer_param_;
    }
}

// Get the euclidean distance between two nodes
double RRG::NodeDistance(const uint index1,
                         const uint index2) {
    return (rrgraph_.nodes_[index1].pos_ - rrgraph_.nodes_[index2].pos_).norm();
}

// Get the euclidean distance between a point and a node
double RRG::DistanceToNode(const uint index,
                           const Eigen::Vector3d pos) {
    return (rrgraph_.nodes_[index].pos_ - pos).norm();
}

} // octoclass
