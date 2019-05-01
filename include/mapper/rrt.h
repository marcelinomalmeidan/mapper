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

#ifndef MAPPER_RRT_H_
#define MAPPER_RRT_H_


#include <vector>
#include "mapper/graphs.h"

namespace octoclass {

class RRT {
 public:
    Tree rrtree_;
    double steer_param_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_ =
            pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

    // Constructor
    RRT(const Eigen::Vector3d &Root, const double steer_param);

    // Methods
    Eigen::Vector3d GetRootPos();
    void SampleNodeBox(const double box_lim,
                       Eigen::Vector3d *sample);
    void SampleNodeBox(const Eigen::Vector3d &box_min,
                       const Eigen::Vector3d &box_max,
                       Eigen::Vector3d *sample);
    Eigen::Vector3d SampleNodeBox(const Eigen::Vector3d &center,
                                  const Eigen::Vector3d &range);
    void BruteNN(const Eigen::Vector3d &sample,
                 uint *NNindex,
                 double *NNcost);
    void Steer(const uint &node_index,
               Eigen::Vector3d *sample,
               double *cost);
    double NodeDistance(const uint index1,
                        const uint index2);  // Distance between two nodes
    double DistanceToNode(const uint index,
                          const Eigen::Vector3d pos);  // Distance to a node
    void SetKdtree();
};


// Similar to RRT, but uses an octomap to store nodes, making an efficient nearest neighbor search
class OctoRRT {
 public:
    Tree rrtree_;
    double steer_param_;
    octomap::OcTree node_octree_ = octomap::OcTree(0.1);  // create empty tree with resolution 0.1
    KeySet node_set_;

    // Constructor
    OctoRRT(const Eigen::Vector3d &root,
            const double &steer_param,
            const double &resolution_in);

    // Methods
    Eigen::Vector3d GetRootPos();
    void AddNode(const Eigen::Vector3d &pos,
                 const uint &parent,
                 const double &cost);
    void SampleNodeBox(const double box_lim,
                       Eigen::Vector3d *sample);
    void SampleNodeBox(const Eigen::Vector3d &box_min,
                       const Eigen::Vector3d &box_max,
                       Eigen::Vector3d *sample);
    Eigen::Vector3d SampleNodeBox(const Eigen::Vector3d &center,
                                  const Eigen::Vector3d &range);
    void NodesWithinBox(const Eigen::Vector3d &box_min,
                        const Eigen::Vector3d &box_max,
                        std::vector<Eigen::Vector3d> *indexes);
    void OctoNN(const Eigen::Vector3d &sample,
                uint *nn_ndex,
                double *nn_cost);  // Nearest neighbor using octomap
    void SteerToRoot(Eigen::Vector3d *sample,
                     const double &steer_param);
    void Steer(const uint &node_index,
               Eigen::Vector3d *sample,
               double *cost);
    double NodeDistance(const uint index1,
                        const uint index2);  // Distance between two nodes
    double DistanceToNode(const uint index,
                          const Eigen::Vector3d Pos);  // Distance to a node
};

}  // octoclass

#endif  // MAPPER_RRT_H_
