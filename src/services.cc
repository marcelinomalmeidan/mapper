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

#include <mapper/mapper_class.h>
#include <limits>
#include <vector>

namespace mapper {

// Update resolution of the map
bool MapperClass::UpdateResolution(mapper::SetFloat::Request &req,
                                   mapper::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetResolution(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

// Update map memory time
bool MapperClass::UpdateMemoryTime(mapper::SetFloat::Request &req,
                                   mapper::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetMemory(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

bool MapperClass::MapInflation(mapper::SetFloat::Request &req,
                               mapper::SetFloat::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.SetMapInflation(req.data);
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    return true;
}

bool MapperClass::ResetMap(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res) {
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.ResetMap();
    pthread_mutex_unlock(&mutexes_.octomap);

    res.success = true;
    res.message = "Map has been reset!";
    return true;
}

bool MapperClass::SaveMap(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {
    std::string filename1 = local_path_ + "/maps/octomap.ot";
    std::string filename2 = local_path_ + "/maps/octomap_inflated.ot";
    pthread_mutex_lock(&mutexes_.octomap);
        globals_.octomap.tree_.write(filename1);
        globals_.octomap.tree_inflated_.write(filename2);
    pthread_mutex_unlock(&mutexes_.octomap);
    
    ROS_INFO("Maps saved in:\n%s \n%s\n", filename1.c_str(), filename2.c_str());
}

bool MapperClass::LoadMap(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {
    std::string filename1 = local_path_ + "/maps/octomap.ot";
    std::string filename2 = local_path_ + "/maps/octomap_inflated.ot";
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename1));
    octomap::OcTree* tree_inflated = dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename2));
    if (tree && tree_inflated) {
        pthread_mutex_lock(&mutexes_.octomap);
            globals_.octomap.CopyMap(*tree, *tree_inflated);
        pthread_mutex_unlock(&mutexes_.octomap);
    }
}

bool MapperClass::OctomapProcessPCL(std_srvs::SetBool::Request &req,
                                    std_srvs::SetBool::Response &res) {
    pthread_mutex_lock(&mutexes_.update_map);
        globals_.update_map = req.data;
    pthread_mutex_unlock(&mutexes_.update_map);
    if (req.data) {
        ROS_INFO("PCL data will be processed!");
    } else {
        ROS_INFO("PCL data will not be processed!");
    }
}

bool MapperClass::RRGService(mapper::RRT_RRG_PRM::Request &req,
                             mapper::RRT_RRG_PRM::Response &res) {
    std::vector<Eigen::Vector3d> e_path;
    visualization_msgs::Marker graph_markers;
    pthread_mutex_lock(&mutexes_.octomap);
    res.success = globals_.octomap.OctoRRG(
        msg_conversions::ros_point_to_eigen_vector(req.origin),
        msg_conversions::ros_point_to_eigen_vector(req.destination),
        msg_conversions::ros_point_to_eigen_vector(req.box_min),
        msg_conversions::ros_point_to_eigen_vector(req.box_max),
        req.max_time, req.max_nodes, req.steer_param, req.free_space_only,
        req.prune_result, req.publish_rviz, &res.planning_time, &res.n_nodes, 
        &e_path, &graph_markers);
    pthread_mutex_unlock(&mutexes_.octomap);
    for (uint i = 0 ; i < e_path.size(); i++) {
        res.path.push_back(msg_conversions::eigen_to_ros_point(e_path[i]));
    }

    if (req.publish_rviz) {
        graph_tree_marker_pub_.publish(graph_markers);
    }
    return true;
}

}  // namespace mapper
