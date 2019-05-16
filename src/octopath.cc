
#include "mapper/octoclass.h"

namespace octoclass {

// Prune a path to minimize waypoints
void OctoClass::PathPruning(const std::vector<Eigen::Vector3d> &path,
                            const bool &free_space_only,
                            std::vector<Eigen::Vector3d> *compressed_path) {
    // the minimum number of points for final vector
    static int min_points = 2;

    // initialize compressed points as all samples
    *compressed_path = path;
    int compressed_points = path.size();

    // first delete colinear points
    static double epsilon = 0.0001, dist;
    static int delete_index;
    static Eigen::Vector3d p1, p2, p;
    while (true) {
        delete_index = -1;
        for (int i = 1; i < compressed_points-1; i++) {
            p1 << (*compressed_path)[i-1][0],
                  (*compressed_path)[i-1][1],
                  (*compressed_path)[i-1][2];
            p2 << (*compressed_path)[i+1][0],
                  (*compressed_path)[i+1][1],
                  (*compressed_path)[i+1][2];
            p  << (*compressed_path)[i][0],
                  (*compressed_path)[i][1],
                  (*compressed_path)[i][2];
            algebra_3d::Line3d line(p1, p2);
            line.DistancePoint2Line(p, &dist);
            if (dist < epsilon) {
                delete_index = i;
                break;
            }
        }
        if (delete_index > 0) {
            compressed_path->erase(compressed_path->begin() + delete_index);
            compressed_points = compressed_points - 1;
        } else {
            break;
        }
    }

    // Compress the remaining points
    static double max_dist;
    int col_check;
    while (compressed_points > min_points) {
        // first find the point that deviates the least in the whole set
        max_dist = -1;
        delete_index = -1;
        for (int i = 1; i < compressed_points-1; i++) {
            p1 << (*compressed_path)[i-1][0],
                  (*compressed_path)[i-1][1],
                  (*compressed_path)[i-1][2];
            p2 << (*compressed_path)[i+1][0],
                  (*compressed_path)[i+1][1],
                  (*compressed_path)[i+1][2];
            p  << (*compressed_path)[i][0],
                  (*compressed_path)[i][1],
                  (*compressed_path)[i][2];
            algebra_3d::Line3d line(p1, p2);
            line.DistancePoint2Line(p, &dist);

            // Check for collision
            if (free_space_only) {
                col_check = CheckCollision(p1, p2);
            } else {
                col_check = CheckOccupancy(p1, p2);
            }
            if ((dist > max_dist) && (col_check != 1)) {
                max_dist = dist;
                delete_index = i;
            }
        }
        if (delete_index > 0) {
            compressed_path->erase(compressed_path->begin() + delete_index);
            compressed_points = compressed_points - 1;
        } else {
            break;
        }
    }

    // ROS_INFO("Compressed points: %d", compressed_points);
}

bool OctoClass::OctoRRG(const Eigen::Vector3d &p0,
			            const Eigen::Vector3d &pf,
			            const Eigen::Vector3d &box_min,
			            const Eigen::Vector3d &box_max,
			            const double &max_time,
			            const int &max_nodes,
			            const double &steer_param,
			            const bool &free_space_only,
			            const bool &prune_result,
			            const bool &publish_rviz,
			            float *plan_time,
			            int *n_rrg_nodes,
			            std::vector<Eigen::Vector3d> *path,
                        visualization_msgs::Marker *graph_markers) {
    // Check whether initial and final points are within box
    if ((p0[0] < box_min[0]) || (p0[1] < box_min[1]) || (p0[2] < box_min[2]) ||
        (p0[0] > box_max[0]) || (p0[1] > box_max[1]) || (p0[2] > box_max[2])) {
        ROS_WARN("[mapper] RRG Error: Initial point not within box!");
        return false;
    }
    if ((pf[0] < box_min[0]) || (pf[1] < box_min[1]) || (pf[2] < box_min[2]) ||
        (pf[0] > box_max[0]) || (pf[1] > box_max[1]) || (pf[2] > box_max[2])) {
        ROS_WARN("[mapper] RRG Error: Final point not within box!");
        return false;
    }

    // Check whether the initial and final points are colliding
    if (free_space_only) {
        if (CheckCollision(p0)) {
            ROS_WARN("[mapper] RRG Error: Initial point is colliding!");
            return false;
        }
        if (CheckCollision(pf)) {
            ROS_WARN("[mapper] RRG Error: Final point is colliding!");
            return false;
        }
    } else {
        if (CheckOccupancy(p0) == 1) {
            ROS_WARN("[mapper] RRG Error: Initial point is colliding!");
            return false;
        }
        if (CheckOccupancy(pf) == 1) {
            ROS_WARN("[mapper] RRG Error: Final point is colliding!");
            return false;
        }
    }

    const ros::Time t0 = ros::Time::now();

    // Get free area volume
    double free_vol;
    if (free_space_only) {
        BBXFreeVolume(box_min, box_max, &free_vol);
    } else {
        Eigen::Vector3d range = box_max - box_min;
        double box_vol = range[0]*range[1]*range[2];
        double occ_vol;
        BBXOccVolume(box_min, box_max, &occ_vol);
        free_vol = box_vol - occ_vol;
    }

    // RRG specific parameters
    const double unit_sphere_vol = M_PI*3.0/4.0;
    const double dim = 3.0;
    const double dim_inv = 1.0/dim;
    const double gamma_star = 2.0*pow(1.0+dim_inv, dim_inv)*pow(free_vol/unit_sphere_vol, dim_inv);
    const double gamma = 1.1*gamma_star;

    // Create RRG class and add initial position to the graph
    uint index;
    RRG obj_rrg(steer_param);
    obj_rrg.AddNode(p0, &index);

    // Run RRG until maximum allowed time
    Eigen::Vector3d sample, neighbor_pos;
    static double cost;
    static uint min_index, n_nodes, final_index = 0;
    n_nodes = obj_rrg.rrgraph_.n_nodes_;
    bool connected_graph = false;  // Becomes true when a path has been found from p0 to pf
    while (((ros::Time::now() - t0).toSec() < max_time) && (n_nodes < max_nodes)) {
        // Get a new sample
        obj_rrg.SampleNodeBox(box_min, box_max, &sample);

        // Find nearest node for sample
        obj_rrg.OctoNN(sample, &min_index, &cost);

        // Steer sample
        obj_rrg.Steer(min_index, &sample, &cost);

        // Check for collision
        int col_check;
        neighbor_pos = obj_rrg.rrgraph_.nodes_[min_index].pos_;
        if (free_space_only) {
            col_check = CheckCollision(neighbor_pos, sample);
        } else {
            col_check = CheckOccupancy(neighbor_pos, sample);
        }

        // Stop if collides with nearest neighbor
        if (col_check == 1) {
            continue;
        }

        // Try to add node: fails if there is another node in the same voxel
        obj_rrg.AddNode(sample, &index);
        if (index == 0) {  // Node was not added succesfully
            continue;
        }

        // Get RRG parameter
        n_nodes = obj_rrg.rrgraph_.n_nodes_;
        const double max_dist = std::min(gamma*pow(log(n_nodes)/n_nodes,
                                        dim_inv), steer_param);

        // Get nodes within max_dist radius
        std::vector<uint> near_nodes;
        std::vector<double> costs;
        obj_rrg.NodesWithinRadius(max_dist, sample, &near_nodes, &costs);

        // Add non-colliding nodes within max_dist radius
        for (uint i = 0; i < near_nodes.size(); i++) {
            neighbor_pos = obj_rrg.rrgraph_.nodes_[near_nodes[i]].pos_;
            if (free_space_only) {
                col_check = CheckCollision(neighbor_pos, sample);
            } else {
                col_check = CheckOccupancy(neighbor_pos, sample);
            }

            if (col_check != 1) {
                obj_rrg.AddEdge(index, near_nodes[i], costs[i]);
            }
        }

        // Go back to 'while' if there is a connection between p0 and pf
        if (connected_graph) {
            continue;
        }

        // Check if new node connects with the final destination
        // This portion does not need to execute after one path has been found between p0 and pf
        cost = obj_rrg.DistanceToNode(index, pf);
        if (cost <= max_dist) {
            if (free_space_only) {
                col_check = CheckCollision(pf, sample);
            } else {
                col_check = CheckOccupancy(pf, sample);
            }

            if (col_check != 1) {
                obj_rrg.AddNode(pf, &final_index);
                obj_rrg.AddEdge(index, final_index, cost);
                connected_graph = true;
                ROS_INFO("[mapper] Found connection to destination!");
            }
        }
    }

    // Publish graph into Rviz if requested
    if (publish_rviz) {
        obj_rrg.rrgraph_.GraphVisualization(graph_markers);
    }

    // Calculate Astar from initial node to final node
    std::vector<Eigen::Vector3d> sol_path;
    if (final_index > 0) {
        std::vector<uint> index_path;
        ROS_INFO("[mapper] Trying A* from node %d to node %d!", 0,
                             static_cast<int>(final_index));
        obj_rrg.rrgraph_.Astar2(0, final_index, index_path);

        // Populate final path
        if (index_path.size() == 0) {
            ROS_WARN("[mapper] RRG Error: Couldn't find solution through Astar!");
            return false;
        } else {
            for (uint i = 0; i < index_path.size(); i++) {
                sol_path.push_back(obj_rrg.rrgraph_.nodes_[index_path[i]].pos_);
            }
            // ROS_INFO("Path size: %d", static_cast<int>(sol_path.size()));
        }
    } else {
        ROS_WARN("[mapper] RRG Error: Could not find a path from p0 to pf!");
        return false;
    }

    // Prune results if requested
    if(prune_result) {
    	this->PathPruning(sol_path, free_space_only, path);
    } else {
    	*path = sol_path;
    }

    // Populate time and nodes
    *n_rrg_nodes = n_nodes;
    *plan_time = (ros::Time::now() - t0).toSec();

    // ROS_INFO("[mapper] nNodes: %d", static_cast<int>(obj_rrg.rrgraph_.n_nodes_));
    // ROS_INFO("[mapper] nEdges: %d", static_cast<int>(obj_rrg.rrgraph_.n_edges_));

    return true;
}

}  // octoclass
