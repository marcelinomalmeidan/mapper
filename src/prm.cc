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

#include "mapper/prm.h"

namespace octoclass {

// PRM Class -----------------------------------------------------
PRM::PRM(double max_dist) {
    max_dist_ = max_dist;
}

void PRM::AddNode(const Eigen::Vector3d &pos) {
    prm_graph_.AddNode(pos);
    cloud_ptr_->push_back(pcl::PointXYZ(pos[0], pos[1], pos[2]));
}

void PRM::AddEdge(const uint &index1,
                  const uint &index2,
                  const double &cost) {
    prm_graph_.AddEdge(index1, index2, cost);
}

void PRM::SampleNodeBox(const double &box_lim,
                        Eigen::Vector3d *sample) {
    *sample = box_lim*Eigen::Vector3d::Random();
}

void PRM::SampleNodeBox(const Eigen::Vector3d &box_min,
                        const Eigen::Vector3d &box_max,
                        Eigen::Vector3d *sample) {
    const Eigen::Vector3d center = (box_max + box_min)/2.0;
    const Eigen::Vector3d range = (box_max - box_min)/2.0;
    *sample = center + range.cwiseProduct(Eigen::Vector3d::Random());
}

void PRM::SetKdtree() {
    kdtree_.setInputCloud(cloud_ptr_);
}

}  // octoclass