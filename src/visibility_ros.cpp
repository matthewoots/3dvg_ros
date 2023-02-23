/*
* visibility.cpp
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

#include "visibility_ros.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

visualization_msgs::Marker visibility_ros::visualize_line_list(
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vect_vert, 
    Eigen::Vector4d color, double scale, int index, double transparency)
{
    visualization_msgs::Marker lines;
    lines.header.frame_id = "world";
    lines.header.stamp = ros::Time::now();
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;

    lines.id = index;

    lines.color.r = color(0);
    lines.color.g = color(1);
    lines.color.b = color(2);

    lines.color.a = transparency;

    lines.scale.x = scale;
    
    // Create the vertices line list
    for (std::pair<Eigen::Vector3d, Eigen::Vector3d> &vert_pair : vect_vert)
    {
        geometry_msgs::Point p1, p2;
        p1.x = vert_pair.first.x();
        p2.x = vert_pair.second.x();

        p1.y = vert_pair.first.y();
        p2.y = vert_pair.second.y();

        p1.z = vert_pair.first.z();
        p2.z = vert_pair.second.z();

        lines.points.push_back(p1);
        lines.points.push_back(p2);
    }

    return lines;
}

visualization_msgs::Marker visibility_ros::visualize_points(
    vector<Eigen::Vector3d> points_vect, 
    Eigen::Vector4d color, double scale, int index)
{
    visualization_msgs::Marker points;
    points.header.frame_id = "world";
    points.header.stamp = ros::Time::now();
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;

    points.id = index;

    points.color.r = color(0);
    points.color.g = color(1);
    points.color.b = color(2);

    points.color.a = color(3);

    points.scale.x = scale;
    points.scale.y = scale;
    
    // Create the vertices point
    for (Eigen::Vector3d &vect : points_vect)
    {
        geometry_msgs::Point p1;
        p1.x = vect.x();
        p1.y = vect.y();
        p1.z = vect.z();

        points.points.push_back(p1);
    }

    return points;
}

void visibility_ros::main_timer(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> lock(main_mutex);

    double duration_switch = duration<double>(system_clock::now() - switch_time).count();
    if (duration_switch > 1/change_start_end_hz)
    {
        std:mt19937 generator(dev());
        std::uniform_real_distribution<double> dis_angle(-M_PI, M_PI);
        std::uniform_real_distribution<double> dis_height(height_list[0], height_list[1]);
        
        double rand_angle = dis_angle(generator);
        double opp_rand_angle = constrain_to_pi(rand_angle - M_PI);

        double h = (map.start_end.first - map.start_end.second).norm() / 2;

        map.start_end.first = Eigen::Vector3d(h * cos(rand_angle), 
            h * sin(rand_angle), dis_height(generator));
        map.start_end.second = Eigen::Vector3d(h * cos(opp_rand_angle), 
            h * sin(opp_rand_angle), dis_height(generator));

        std::cout << "start_position = " << KBLU << map.start_end.first.transpose() << KNRM << " " <<
                "end_position = " << KBLU << map.start_end.second.transpose() << KNRM << std::endl;

        switch_time = system_clock::now();

        visibility vis_graph(map, frame, 1);

        time_point<std::chrono::system_clock> start_time = system_clock::now();
        vis_graph.calculate_path();
        rot_polygons = vis_graph.get_rotated_poly();
        shortest_path_3d = vis_graph.get_path();
        map = vis_graph.get_map();
        double search_time = duration<double>(system_clock::now() - start_time).count();
        std::cout << "get_visibility_path time (" << KBLU << search_time * 1000 << KNRM << "ms)" << std::endl;
        found = true;
    }
    
}

void visibility_ros::visualization_timer(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> lock(main_mutex);

    visualization_msgs::Marker obs_visualize, start_end_visualize, debug_vertices;
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vect_vert;

    vector<Eigen::Vector3d> s_e;
    s_e.push_back(map.start_end.first);
    s_e.push_back(map.start_end.second);

    start_end_visualize = visualize_points(
        s_e, color_range[0], 0.3, 1);
    start_end_pub.publish(start_end_visualize);


    // Change global_map into 
    // vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> format
    for (obstacle &obs : map.obs)
    {
        // Get the centroid 
        Eigen::Vector2d centroid;
        vector<Eigen::Vector2d> vert;             

        int obs_vert_pair_size, obs_hori_pair_size;
        obs_vert_pair_size = obs_hori_pair_size = obs.v.size();

        // Add lines for verticals
        for (int i = 0; i < obs_vert_pair_size; i++)
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
            vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.first);
            vert_pair.second = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.second);
            vect_vert.push_back(vert_pair);

            /** @brief For debug purpose **/
            // std::cout << vert_pair.first.transpose() << " to " << vert_pair.second.transpose() << std::endl;
        }
        // Add lines for horizontals
        for (int i = 0; i < obs_hori_pair_size; i++)
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
            vert_pair.first = Eigen::Vector3d(
                obs.v[i % obs_hori_pair_size].x(), 
                obs.v[i % obs_hori_pair_size].y(), obs.h.first);
            vert_pair.second = Eigen::Vector3d(
                obs.v[(i+1) % obs_hori_pair_size].x(), 
                obs.v[(i+1) % obs_hori_pair_size].y(), obs.h.first);
            vect_vert.push_back(vert_pair);

            vert_pair.first = Eigen::Vector3d(
                obs.v[i % obs_hori_pair_size].x(), 
                obs.v[i % obs_hori_pair_size].y(), obs.h.second);
            vert_pair.second = Eigen::Vector3d(
                obs.v[(i+1) % obs_hori_pair_size].x(), 
                obs.v[(i+1) % obs_hori_pair_size].y(), obs.h.second);
            vect_vert.push_back(vert_pair);
        }

    }

    obs_visualize = visualize_line_list(
        vect_vert, color_range[1], 0.25, 2, 0.75);
    obstacle_pub.publish(obs_visualize);

    if (!found)
        return;

    // debug_vertices = visualize_points(
    //     debug_point_vertices, color_range[2], 0.2, 3);
    // obstacle_pub.publish(debug_vertices);

    // vect_vert.clear();
    for (obstacle &obs : rot_polygons)
    {
        int obs_pair_size = obs.v.size();
        for (int i = 0; i < obs_pair_size; i++)
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
            vert_pair.first = map.t.inverse() * 
                Eigen::Vector3d(obs.v[i % obs_pair_size].x(), 
                obs.v[i % obs_pair_size].y(), 0.0);
            vert_pair.second = map.t.inverse() *
                Eigen::Vector3d(obs.v[(i+1) % obs_pair_size].x(), 
                obs.v[(i+1) % obs_pair_size].y(), 0.0);
            vect_vert.push_back(vert_pair);
        }
    }
    visualization_msgs::Marker plane_visualize = visualize_line_list(
        vect_vert, color_range[0], 0.15, 5, 0.5);
    obstacle_pub.publish(plane_visualize);

    // Debugging vector
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debug_pair;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
    Eigen::Vector3d middle = 
        (map.start_end.second + map.start_end.first) / 2;
    Eigen::Vector3d normal_proj = middle +
        get_rotation(map.rpy, frame).inverse() * Eigen::Vector3d(0.0, 0.0, 1.0);
    vert_pair.first = middle;
    vert_pair.second = normal_proj;

    debug_pair.push_back(map.start_end);
    debug_pair.push_back(vert_pair);
    
    visualization_msgs::Marker vector_visualize = 
        visualize_line_list(debug_pair, color_range[3], 0.25, 4, 0.15);
    obstacle_pub.publish(vector_visualize);


    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> shortest_path_3d_pair;
    for (int i = 0; i < (int)shortest_path_3d.size() - 1; i++)
        shortest_path_3d_pair.push_back(
            make_pair(shortest_path_3d[i], shortest_path_3d[i+1]));
    
    visualization_msgs::Marker shortest_path_visualize = 
        visualize_line_list(shortest_path_3d_pair, color_range[4], 0.25, 5, 0.5);
    visibility_graph_pub.publish(shortest_path_visualize);

} 
