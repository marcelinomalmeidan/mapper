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

#ifndef MAPPER_RRG_H_
#define MAPPER_RRG_H_

#include <vector>
#include "mapper/graphs.h"
#include "mapper/indexed_octree_key.h"

namespace octoclass {

// Rapidly-exploring random graph
class RRG {
 public:
    Graph rrgraph_;
    double steer_param_;
    octomap::OcTree node_octree_ = octomap::OcTree(0.1);  // create empty tree with resolution 0.1
    IndexedKeySet node_set_;

    // Constructor
    explicit RRG(const double &steer_param);

    // Methods
    void AddNode(const Eigen::Vector3d &pos,
                 uint *index);
    void AddEdge(const uint &index1,
                 const uint &index2);
    void AddEdge(const uint &index1,
                 const uint &index2,
                 const double &cost);
    void SampleNodeBox(const double &box_lim,
                       Eigen::Vector3d *sample);
    void SampleNodeBox(const Eigen::Vector3d &box_min,
                       const Eigen::Vector3d &box_max,
                       Eigen::Vector3d *sample);
    Eigen::Vector3d SampleNodeBox(const Eigen::Vector3d &center,
                                  const Eigen::Vector3d &range);
    void NodesWithinBox(const Eigen::Vector3d &box_min,
                        const Eigen::Vector3d &box_max,
                        std::vector<Eigen::Vector3d> *nodes);
    void NodesWithinRadius(const double radius,
                           const Eigen::Vector3d center,
                           std::vector<Eigen::Vector3d> *nodes,
                           std::vector<double> *costs);
    void NodesWithinRadius(const double radius,
                           const Eigen::Vector3d center,
                           std::vector<uint> *indexes,
                           std::vector<double> *costs);
    void OctoNN(const Eigen::Vector3d &sample,
                uint *nn_index,
                double *nn_cost);
    void Steer(const uint &node_index,
               Eigen::Vector3d *sample,
               double *cost);
    double NodeDistance(const uint index1,
                        const uint index2);  // Distance between two nodes
    double DistanceToNode(const uint index,
                          const Eigen::Vector3d pos);  // Distance to a node
};

}  // octoclass

#endif  // MAPPER_RRG_H_
