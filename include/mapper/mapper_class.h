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

#ifndef MAPPER_MAPPER_CLASS_H_
#define MAPPER_MAPPER_CLASS_H_

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

// ROS libraries
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>

// // Service messages
#include <mapper/SetFloat.h>

// C++ libraries
#include <fstream>
#include <vector>
#include <string>
#include <exception>
#include <thread>         // std::thread

// Astrobee message types
#include "mapper/Segment.h"
#include "mapper/ControlState.h"

// // Classes
#include "mapper/tf_class.h"
#include "mapper/octoclass.h"
#include "mapper/polynomials.h"
#include "mapper/sampled_trajectory.h"

// Data structures
#include "mapper/structs.h"

// My defined libraries
#include "mapper/visualization_functions.h"

namespace mapper {

class MapperClass {
 public:
  MapperClass();
  ~MapperClass();

  virtual void Initialize(ros::NodeHandle *nh);


 protected:
  // Callbacks (see callbacks.cpp for implementation) ----------------
  // Callback for handling incoming point cloud messages
  void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
                   const uint& cam_index);

  // Callback for handling incoming new trajectory messages
  void SegmentCallback(const mapper::Segment::ConstPtr &msg);


  // Services (see services.cpp for implementation) -----------------
  // Update resolution of the map
  bool UpdateResolution(mapper::SetFloat::Request &req,
                        mapper::SetFloat::Response &res);

  // Update map memory time
  bool UpdateMemoryTime(mapper::SetFloat::Request &req,
                        mapper::SetFloat::Response &res);

  // Update map inflation
  bool MapInflation(mapper::SetFloat::Request &req,
                    mapper::SetFloat::Response &res);

  // Reset the map
  bool ResetMap(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res);

  // Threads (see threads.cpp for implementation) -----------------
  // Thread for fading memory of the octomap
  void FadeTask();

  // Threads for constantly updating the tfTree values
  void HazTfTask();
  void PerchTfTask();
  void BodyTfTask();
  void TfTask(const std::string& parent_frame,
              const std::string& child_frame,
              const uint& index); // Returns the transform from child to parent frame, expressed in parent frame

  // Thread for collision checking
  void CollisionCheckTask();

  // Thread for getting pcl data and populating the octomap
  void OctomappingTask();

 private:
  // Declare global variables (structures defined in structs.h)
  globalVariables globals_;  // These variables are all mutex-protected
  mutexStruct mutexes_;
  semaphoreStruct semaphores_;

  // Thread variables
  std::thread h_haz_tf_thread_, h_perch_tf_thread_, h_body_tf_thread_;
  std::thread h_octo_thread_, h_fade_thread_, h_collision_check_thread_;
  std::vector<std::thread> h_cameras_tf_thread_;

  // Subscriber variables
  ros::Subscriber haz_sub_, perch_sub_, segment_sub_;
  std::vector<ros::Subscriber> cameras_sub_;

  // Octomap services
  ros::ServiceServer resolution_srv_, memory_time_srv_;
  ros::ServiceServer map_inflation_srv_, reset_map_srv_;

  // Thread rates (hz)
  double tf_update_rate_, fading_memory_update_rate_;

  // // Path planning services
  ros::ServiceServer RRT_srv, octoRRT_srv, PRM_srv, graph_srv, Astar_srv;
  ros::ServiceServer newTraj_srv;

  // Marker publishers
  ros::Publisher sentinel_pub_;
  ros::Publisher obstacle_marker_pub_;
  ros::Publisher free_space_marker_pub_;
  ros::Publisher inflated_obstacle_marker_pub_;
  ros::Publisher inflated_free_space_marker_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher cam_frustum_pub_;
  ros::Publisher map_keep_in_out_pub_;
};

}  // namespace mapper

#endif  // MAPPER_MAPPER_CLASS_H_
