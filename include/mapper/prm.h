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

#ifndef MAPPER_PRM_H_
#define MAPPER_PRM_H_

#include "mapper/graphs.h"

namespace octoclass {

// Probabilistic Roadmaps
class PRM {
 public:
    Graph prm_graph_;
    double max_dist_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_ =
            pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

    // Constructor
    explicit PRM(double max_dist);

    // Methods
    void AddNode(const Eigen::Vector3d &pos);
    void AddEdge(const uint &index1,
                 const uint &index2,
                 const double &cost);
    void SampleNodeBox(const double &box_lim,
                       Eigen::Vector3d *sample);
    void SampleNodeBox(const Eigen::Vector3d &box_min,
                       const Eigen::Vector3d &box_max,
                       Eigen::Vector3d *sample);
    void SetKdtree();
};

#endif  // MAPPER_PRM_H_

}  // octoclass
