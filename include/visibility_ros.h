/*
* visibility_ros.h
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

#ifndef VISIBILITY_ROS_H
#define VISIBILITY_ROS_H

#include "visibility.h"

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
using namespace visibility_graph;

typedef time_point<std::chrono::system_clock> t_p_sc; // giving a typename

class visibility_ros
{
    public:

        visibility_ros(ros::NodeHandle &nodeHandle) : _nh(nodeHandle)
        {
            std::vector<double> obstacles_list, start_list, end_list;

            /** @brief Define ROS Params */
            _nh.param<int>("map/polygon_vertices_size", polygon_vertices_size, -1);
            _nh.param<std::string>("map/frame", frame, "");
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
            timer = _nh.createTimer(ros::Duration(0.05), 
                &visibility_ros::main_timer, this, false, false);
            visual_timer = _nh.createTimer(ros::Duration(0.05), 
                &visibility_ros::visualization_timer, this, false, false);

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

        ~visibility_ros(){}

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
        vector<Eigen::Vector3d> shortest_path_3d;
        vector<obstacle> rot_polygons;
        string frame;

        double change_start_end_hz;
        double protected_zone;
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

#endif