/*
* visibility.h
*
* ---------------------------------------------------------------------
* Copyright (C) 2022 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#ifndef VISIBILITY_H
#define VISIBILITY_H

#include "visilibity.hpp"

#include <string>
#include <thread>   
#include <mutex>
#include <iostream>
#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

using namespace Eigen;
using namespace std;
using namespace std::chrono; // nanoseconds, system_clock, seconds
using namespace VisiLibity;

typedef time_point<std::chrono::system_clock> t_p_sc; // giving a typename

namespace visibility_graph
{
    class visibility
    {
        public:

            /** @brief Obstacle structure
             * @param v Flattened vertices of the obstacle
             * @param h Height that the obstacle is extruded to
             * @param c Flattened centroid of the obstacle
            **/
            struct obstacle
            {
                vector<Eigen::Vector2d> v; // Base vertices
                double h; // Height that is it extruded to
                Eigen::Vector2d c; // Centroid
            };

            /** @brief Global map structure
             * @param start_end Start and end point pair (3D)
             * @param obs Obstacles in the map
             * @param inflation Safety margin that is used to expand the map
             * @param t Transform and oriented to start-end vector
             * @param rpy RPY that is recorded for the transform
            **/
            struct global_map
            {
                std::pair<Eigen::Vector3d, Eigen::Vector3d> start_end; // Start and end pair
                vector<obstacle> obs; // Obstacles
                double inflation; // Safety margin
                Eigen::Affine3d t; // Transform and oriented to start-end vector
                Eigen::Vector3d rpy; // RPY that is recorded for the transform
            };

            visibility(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
            {
                std::vector<double> obstacles_list, start_list, end_list;

                /** @brief Define ROS Params */
                _nh.param<int>("map/polygon_vertices_size", polygon_vertices_size, -1);
                _nh.param<string>("map/frame", frame, "");
                _nh.param<double>("map/change_start_end_hz", 
                    change_start_end_hz, -1.0);
                _nh.getParam("map/height_limit", height_list);

                _nh.getParam("planning/start", start_list);
                _nh.getParam("planning/end", end_list);

                assert(!start_list.empty() || ((int)start_list.size() == 3));
                assert(!end_list.empty() || ((int)end_list.size() == 3));
                
                Eigen::Vector3d start = Eigen::Vector3d(start_list[0], start_list[1], start_list[2]);
                Eigen::Vector3d end = Eigen::Vector3d(end_list[0], end_list[1], end_list[2]);
                map.start_end = make_pair(start, end);

                _nh.getParam("planning/obstacles", obstacles_list);
                int obs_list_size = (int)obstacles_list.size();
                // Assume each polygon is made up of 4 vertices
                // 0-1, 2-3, 4-5, 6-7 since we need both x and y
                // 8 element will be height
                assert(obs_list_size % (polygon_vertices_size*2 + 1) == 0);
                int obstacle_size = obs_list_size / (polygon_vertices_size*2 + 1);
                for (int i = 0; i < obstacle_size; i++)
                {
                    obstacle obs;
                    vector<Eigen::Vector2d> v_tmp;
                    for (int j = 0; j < polygon_vertices_size; j++)
                    {
                        int x_idx = i*(polygon_vertices_size*2 + 1) + j*2+0;
                        int y_idx = i*(polygon_vertices_size*2 + 1) + j*2+1;
                        v_tmp.push_back(Eigen::Vector2d(
                            obstacles_list[x_idx], obstacles_list[y_idx]));
                        // std::cout << v_tmp[(int)v_tmp.size()-1].transpose() << ", ";
                    }
                    // std::cout << std::endl;

                    obs.c = get_centroid_2d(v_tmp);

                    // enforces outer boundary vertices are listed ccw and
                    // holes listed cw
                    graham_scan(v_tmp, obs.c, "cw", obs.v);
                    obs.h = obstacles_list[(i+1)*(polygon_vertices_size*2 + 1) - 1];
                    
                    map.obs.push_back(obs);                    
                }

                _nh.param<double>("planning/protected_zone", protected_zone, -1.0);

                /** @brief Publishers */
                obstacle_pub = _nh.advertise<
                    visualization_msgs::Marker>("/obstacle", 10);
                plane_polygon_pub = _nh.advertise<
                    visualization_msgs::Marker>("/plane_obstacles", 10);
                start_end_pub = _nh.advertise<
                    visualization_msgs::Marker>("/start_end", 10);
                visibility_graph_pub = _nh.advertise<
                    visualization_msgs::Marker>("/visbility", 10);

                /** @brief Timer functions */
                timer = _nh.createTimer(ros::Duration(0.1), 
                    &visibility::main_timer, this, false, false);
                visual_timer = _nh.createTimer(ros::Duration(0.1), 
                    &visibility::visualization_timer, this, false, false);

                /** @brief Choose a color for the trajectory using random values **/
                std:mt19937 generator(dev());
                std::uniform_real_distribution<double> dis(0.0, 1.0);
                // We will generate colors for 
                // 1. start and end (points)
                // 2. obstacles edges (line list)
                // 3. 2d plane polygons (line list)
                // 4. visibility graph (line list and points)
                // 5. shortest path (line list)
                int color_count = 5;
                for (int i = 0; i < color_count; i++)
                    color_range.push_back(Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5));

                timer.start();
                visual_timer.start();

                switch_time = system_clock::now();
            }

            ~visibility(){}

            double constrain_to_pi(double x)
            {
                x = fmod(x + M_PI, 2 * M_PI);
                if (x < 0)
                    x += 2 * M_PI;
                return x - M_PI;
            }

        private:

            ros::NodeHandle _nh;

            global_map map;

            ros::Publisher obstacle_pub, plane_polygon_pub, start_end_pub, visibility_graph_pub;

            t_p_sc start_time;

            int polygon_vertices_size;

            std::mutex main_mutex;

            vector<Eigen::Vector4d> color_range;
            vector<Eigen::Vector3d> debug_point_vertices;
            std::pair<Eigen::Vector2d, Eigen::Vector2d> boundary;
            vector<Eigen::Vector3d> shortest_path_3d;
            vector<obstacle> rot_polygons;
            double protected_zone;
            string frame;

            double change_start_end_hz;
            time_point<std::chrono::system_clock> switch_time;
            std::vector<double> height_list;
            std::random_device dev;

            bool found = false;

            /** 
             * @brief Command callback
             * @param msg is in the geometry msgs type
            **/
            void command_callback(
                const geometry_msgs::PointConstPtr& msg);

            /** 
             * @brief Sum up from a range of int values
             * @param s start of range
             * @param e end of the range
            **/
            int sum_of_range(int s, int e);

            /** 
             * @brief gift_wrapping algorithm
             * @param points_in points that are passed into the algorithm
             * @param (Return) Vector of points that are at the edge (convex hull)
            **/
            vector<Eigen::Vector2d> gift_wrapping(
                vector<Eigen::Vector2d> points_in);

            /** 
             * @brief graham_scan, sorts the polygon clockwise https://stackoverflow.com/a/57454410
             * Holes listed in clockwise or counterclockwise up to the direction
             * @param points_in Vector of points in
             * @param centroid Centroid of the flattened polygon
             * @param dir Clockwise or counterclockwise direction sorting of points_out
             * @param points_out (Return) Vector of points out
            **/
            void graham_scan(
                vector<Eigen::Vector2d> points_in, Eigen::Vector2d centroid,
                string dir, vector<Eigen::Vector2d> &points_out);

            /** 
             * @brief find_nearest_distance_2d_polygons_and_fuse
             * Find out whether the nearest distance between the 2 polygons are within the threshold
             * If it is within, the 2 polygons will be fused
             * Uses a shortcut to derive the closest vertex pair, use the vector of the centroids and dot product
             * @param o1 Obstacle 1 input
             * @param o2 Obstacle 2 input
             * @param safety_radius Safety radius that is acting as a threshold
             * @param points_out (Return) Pair of points that are the closest 2 points between the 2 polygons
             * (Only will return points out if the edges are not parallel)
             * @param nearest_distance (Return) Nearest distance between the 2 polygons
             * @param o3 (Return) The fused obstacle
             * (Only will return points out if the edges are not parallel)
             * @param (Return) A boolean representing whether there is a fused polygon
            **/
            bool find_nearest_distance_2d_polygons_and_fuse(
                obstacle o1, obstacle o2, double safety_radius,
                std::pair<Eigen::Vector2d, Eigen::Vector2d> &points_out, 
                double &nearest_distance, obstacle &o3);

            /** 
             * @brief closest_points_between_lines
             * @param a0 First point of line 1
             * @param a1 Second point of line 1
             * @param b0 First point of line 2
             * @param b1 Second point of line 2
             * @param c_p The pair of points that are the closest points
             * (Only will return points out if the edges are not parallel)
             * @param distance Nearest distance between the 2 lines
             * @param (Return) A boolean representing whether the line is not parallel
            **/
            bool closest_points_between_lines(
                Eigen::Vector2d a0, Eigen::Vector2d a1,
                Eigen::Vector2d b0, Eigen::Vector2d b1,
                std::pair<Eigen::Vector2d, Eigen::Vector2d> &c_p,
                double &distance);
            
            /** 
             * @brief set_2d_min_max_boundary
             * @param obstacles Vector of flattened obstacles
             * @param start_end Start and end flattened points
             * @param boundary (Return) The minimum and maximum of the inputs
            **/
            void set_2d_min_max_boundary(
                vector<obstacle> obstacles, std::pair<Eigen::Vector2d, Eigen::Vector2d> start_end, 
                std::pair<Eigen::Vector2d, Eigen::Vector2d> &boundary);

            /** 
             * @brief boundary_to_polygon_vertices
             * @param min_max The minimum and maximum of the inputs
             * @param dir Pass the direction into the graham search to sort the vertices
             * @param (Return) The AABB of the inputs represented in the 4 vertices
            **/
            vector<Eigen::Vector2d> boundary_to_polygon_vertices(
                std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max, string dir);
            
            /** 
             * @brief get_line_plane_intersection
             * @param s_e Start and end pair
             * @param normal Normal of the plane
             * @param pop Point on plane
             * @param p (Return) Point of intersection
            **/
            bool get_line_plane_intersection(
                std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e, 
                Eigen::Vector3d normal, Eigen::Vector3d pop, Eigen::Vector3d &p);

            /** 
             * @brief get_polygons_on_plane
             * @param g_m Pass in the global map
             * @param normal Normal of the plane
             * @param polygons (Return) Return the vector of flattened polygons
             * @param v (Return) Return the vertices in 3d (not transformed)
            **/
            void get_polygons_on_plane(
                global_map g_m, Eigen::Vector3d normal, 
                vector<obstacle> &polygons, vector<Eigen::Vector3d> &v);
        
            /** 
             * @brief get_expansion_of_obs
             * @param obs Obstacle as input and output 
             * @param inflation Inflation amount
            **/
            void get_expanded_obs(
                obstacle &obs, double inflation);

            /** 
             * @brief get_centroid_2d
             * @param vect Vector of points used to get centroid
             * @param (Return) Centroid
            **/
            Eigen::Vector2d get_centroid_2d(
                vector<Eigen::Vector2d> vect);

            /** 
             * @brief get_rotation
             * @param rpy Euler angles in Eigen::Vector3d
             * @param frame Coordinate frame used
             * @param (Return) 3x3 Rotation matrix
            **/
            Eigen::Matrix3d get_rotation(
                Eigen::Vector3d rpy, std::string frame);

            /**
             * @brief get_affine_transform
             * @param pos Translational position
             * @param rpy Euler angles
             * @param frame Coordinate frame used
             * @param (Return) Affine3d matrix
            **/
            Eigen::Affine3d get_affine_transform(
                Eigen::Vector3d pos, Eigen::Vector3d rpy, 
                std::string frame);

            /**
             * @brief Main loop
            **/
            void get_visibility_path();

            visualization_msgs::Marker visualize_line_list(
                vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vect_vert, 
                Eigen::Vector4d color, double scale, int index, double transparency);
            
            visualization_msgs::Marker visualize_points(
                vector<Eigen::Vector3d> points_vect, 
                Eigen::Vector4d color, double scale, int index);

            /** @brief Timers for searching and agent movement **/
            ros::Timer timer, visual_timer;
            void main_timer(const ros::TimerEvent &);
            void visualization_timer(const ros::TimerEvent &);            
            
    };
}

#endif