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

#include <vector>
#include <string>
#include "mapper/mapper_class.h"
#include "mapper/pcl_conversions.h"

namespace mapper {

void MapperClass::PclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
                              const uint& cam_index) {
    // Structure to include pcl and its frame
    stampedPcl new_pcl;
    uint max_queue_size = 2;

    // Convert message into pcl type
    pcl::PointCloud< pcl::PointXYZ > cloud;
    pcl_conversions::FromROSMsg(*msg, &cloud);
    new_pcl.cloud = cloud;

    // Get transform from camera to world
    new_pcl.tf_cam2world = globals_.tf_cameras2world[cam_index];
    // const std::string pcl_frame = cloud.header.frame_id;
    // pthread_mutex_lock(&mutexes_.tf);
    //     if (!pcl_frame.compare("haz_cam")) {
    //         new_pcl.tf_cam2world = globals_.tf_cam2world;
    //     } else if (!pcl_frame.compare("perch_cam")) {
    //         new_pcl.tf_cam2world = globals_.tf_perch2world;
    //     }
    // pthread_mutex_unlock(&mutexes_.tf);

    // save into global variables
    pthread_mutex_lock(&mutexes_.point_cloud);
        globals_.pcl_queue.push(new_pcl);
        if (globals_.pcl_queue.size() > max_queue_size) {
            globals_.pcl_queue.pop();
        } else {
            // signal octomap thread to process new pcl data
            sem_post(&semaphores_.pcl);
        }
    pthread_mutex_unlock(&mutexes_.point_cloud);

    // printf("queue_size = %d\n", sem_getvalue());
    
}


void MapperClass::SegmentCallback(const mapper::Segment::ConstPtr &msg) {
    ros::Time t0 = ros::Time::now();
    while (t0.toSec() == 0) {
        t0 = ros::Time::now();
    }

    // set segments
    mapper::Segment segments = *msg;

    if (segments.segment.size() == 0) {  // Empty trajectory
        return;
    }

    // transform message into set of polynomials
    polynomials::Trajectory3D poly_trajectories(segments);

    // sample trajectory at 10hz
    double ts = 0.1;
    sampled_traj::SampledTrajectory3D sampled_traj(ts, poly_trajectories);
    // SampledTrajectory3D sampledTraj(Time, Positions); // fake trajectory

    pthread_mutex_lock(&mutexes_.sampled_traj);
        globals_.sampled_traj.pos_ = sampled_traj.pos_;
        globals_.sampled_traj.time_ = sampled_traj.time_;
        globals_.sampled_traj.n_points_ = sampled_traj.n_points_;

        // compress trajectory into points with max deviation of 1cm from original trajectory
        globals_.sampled_traj.CompressSamples();

        //  Transform compressed trajectory into a set of pixels in octomap
        //  Octomap insertion avoids repeated points
        globals_.sampled_traj.thick_traj_.clear();
        for (int i = 0; i < globals_.sampled_traj.n_compressed_points_-1; i++) {
            globals_.sampled_traj.ThickBresenham(globals_.sampled_traj.compressed_pos_[i],
                                                globals_.sampled_traj.compressed_pos_[i+1]);
        }

        // populate trajectory node centers in a point cloud
        globals_.sampled_traj.ThickTrajToPcl();

        // populate kdtree for finding nearest neighbor w.r.t. collisions
        globals_.sampled_traj.CreateKdTree();
    pthread_mutex_unlock(&mutexes_.sampled_traj);

    // Notify the collision checker to check for collision
    sem_post(&semaphores_.collision_check);

    // ros::Duration solver_time = ros::Time::now() - t0;
    // ROS_DEBUG("Time to compute octotraj: %f", solver_time.toSec());
}


}  // namespace mapper
