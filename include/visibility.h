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

            struct obstacle
            {
                vector<Eigen::Vector2d> v; // Base vertices
                double h; // Height that is it extruded to
                Eigen::Vector2d c; // Centroid
            };

            struct global_map
            {
                std::pair<Eigen::Vector3d, Eigen::Vector3d> min_max_bnd; // min and max xyz
                std::pair<Eigen::Vector3d, Eigen::Vector3d> start_end; // start and end pair
                vector<obstacle> obs; // obstacles
                double inflation; // agent safety margin
                Eigen::Affine3d t; // Transform from start and aligned
            };

            visibility(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
            {
                /** @brief ROS Params */
                std::vector<double> obstacles_list, start_list, end_list, min_max_bnd_list;

                _nh.param<int>("map/polygon_vertices_size", polygon_vertices_size, -1);

                _nh.getParam("planning/start", start_list);
                _nh.getParam("planning/end", end_list);
                assert(start_list.empty() || start_list.size() != 3);
                assert(end_list.empty() || end_list.size() != 3);
                
                Eigen::Vector3d start = Eigen::Vector3d(start_list[0], start_list[1], start_list[2]);
                Eigen::Vector3d end = Eigen::Vector3d(end_list[0], end_list[1], end_list[2]);
                map.start_end = make_pair(start, end);

                _nh.getParam("planning/min_max_bnd", min_max_bnd_list);
                Eigen::Vector3d min_bnd = Eigen::Vector3d(min_max_bnd_list[0], min_max_bnd_list[2], min_max_bnd_list[4]);
                Eigen::Vector3d max_bnd = Eigen::Vector3d(min_max_bnd_list[1], min_max_bnd_list[3], min_max_bnd_list[5]);
                map.min_max_bnd = make_pair(min_bnd, max_bnd);

                _nh.getParam("planning/obstacles", obstacles_list);
                int obs_list_size = (int)obstacles_list.size();
                // Assume each polygon is made up of 4 vertices
                // 0-1, 2-3, 4-5, 6-7 since we need both x and y
                // 8 element will be height
                assert(obs_list_size % (polygon_vertices_size*2 + 1) != 0);
                int obstacle_size = obs_list_size / (polygon_vertices_size + 1);
                for (int i = 0; i < obstacle_size; i++)
                {
                    obstacle obs;
                    for (int j = 0; j < polygon_vertices_size; j++)
                    {
                        int x_idx = i*(polygon_vertices_size*2 + 1) + j*2+0;
                        int y_idx = i*(polygon_vertices_size*2 + 1) + j*2+1;
                        obs.v.push_back(Eigen::Vector2d(
                            obstacles_list[x_idx], obstacles_list[y_idx]));
                    }
                    
                    obs.h = obstacles_list[(i+1)*(polygon_vertices_size + 1) - 1];
                    obs.c = get_centroid_2d(obs.v);

                    map.obs.push_back(obs);
                }

                // _nh.param<double>("planning/sub_runtime_error", rrt_param.r_e.first, -1.0);

                /** @brief For debug */
                obstacle_pub = _nh.advertise<
                    visualization_msgs::Marker>("/obstacle", 10);
                plane_polygon_pub = _nh.advertise<
                    visualization_msgs::Marker>("/plane_obstacles", 10);
                start_end_pub = _nh.advertise<
                    visualization_msgs::Marker>("/start_end", 10);

                visibility_graph_pub = _nh.advertise<
                    visualization_msgs::Marker>("/visbility", 10);

                /** @brief Timer for the rrt search and agent */
                timer = _nh.createTimer(ros::Duration(0.1), 
                    &visibility::main_timer, this, false, false);

                /** @brief Choose a color for the trajectory using random values **/
                std::random_device dev;
                std:mt19937 generator(dev());
                std::uniform_real_distribution<double> dis(0.0, 1.0);
                color = Eigen::Vector4d(dis(generator), dis(generator), dis(generator), 0.5);

                timer.start();
            }

            ~visibility(){}

            double constrain_to_pi(double x){
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

            Eigen::Vector4d color;

            /** @brief Callbacks, mainly for loading pcl and commands **/
            void command_callback(
                const geometry_msgs::PointConstPtr& msg);

            /** @brief graham_scan, sorts the polygon clockwise https://stackoverflow.com/a/57454410
             * Holes listed in clockwise
             * @param points_in Vector of points in
             * @param points_out Vector of points out
            **/
            void graham_scan(
                vector<Eigen::Vector2d> points_in, Eigen::Vector2d centroid,
                vector<Eigen::Vector2d> &points_out);
            
            /** @brief get_line_plane_intersection
             * https://stackoverflow.com/a/71407596
             * @param obs Obstacles will provide the line 
             * @param normal Normal of the plane
             * @param pop Point on plane
            **/
            bool get_line_plane_intersection(
                std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e, 
                Eigen::Vector3d normal, Eigen::Vector3d pop, Eigen::Vector3d &p);

            /** @brief get_polygons_on_plane
             * @param g_m Pass in the global map
             * @param normal Normal of the plane
            **/
            void get_polygons_on_plane(
                global_map g_m, Eigen::Vector3d normal, 
                vector<obstacle> &polygons, vector<Eigen::Vector3d> &v);
        
            /** @brief get_expansion_of_obs
             * @param obs Obstacle as input and output 
             * @param inflation Inflation amount
            **/
            void get_expanded_obs(
                obstacle &obs, double inflation);

            /** @brief get_centroid_2d
             * @param vect Vector of points used to get centroid
            **/
            Eigen::Vector2d get_centroid_2d(
                vector<Eigen::Vector2d> vect);

            /** @brief get_affine_transform
             * @param pos Translational position
             * @param rpy Euler angles
            **/
            Eigen::Affine3d get_affine_transform(
                Eigen::Vector3d pos, Eigen::Vector3d rpy, string frame);

            // Using https://karlobermeyer.github.io/VisiLibity1/doxygen_html/annotated.html
            void get_visibility_path();

            /** @brief Timers for searching and agent movement **/
            ros::Timer timer;
            void main_timer(const ros::TimerEvent &);

            void visualize_points(double scale_small, double scale_big)
            {
                visualization_msgs::Marker sphere_points, search;
                sphere_points.header.frame_id = search.header.frame_id = "world";
                sphere_points.header.stamp = search.header.stamp = ros::Time::now();
                sphere_points.type = visualization_msgs::Marker::SPHERE;
                search.type = visualization_msgs::Marker::SPHERE;
                sphere_points.action = search.action = visualization_msgs::Marker::ADD;

                sphere_points.id = 0;
                search.id = 1;

                sphere_points.pose.orientation.w = search.pose.orientation.w = 1.0;
                sphere_points.color.r = search.color.g = color(0);
                sphere_points.color.g = search.color.r = color(1);
                sphere_points.color.b = search.color.b = color(2);

                sphere_points.color.a = color(3);
                search.color.a = 0.1;

                sphere_points.scale.x = scale_small;
                sphere_points.scale.y = scale_small;
                sphere_points.scale.z = scale_small;

                search.scale.x = scale_big;
                search.scale.y = scale_big;
                search.scale.z = scale_big;

            }
            
    };
}

#endif