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

#include "visibility.h"

#define KNRM  "\033[0m"
#define KRED  "\033[31m"
#define KGRN  "\033[32m"
#define KYEL  "\033[33m"
#define KBLU  "\033[34m"
#define KMAG  "\033[35m"
#define KCYN  "\033[36m"
#define KWHT  "\033[37m"

namespace visibility_graph
{
    void visibility::graham_scan(
        vector<Eigen::Vector2d> points_in, Eigen::Vector2d centroid, 
        vector<Eigen::Vector2d> &points_out)
    {
        std::vector<std::pair<double, int>> point_angle_pair;
        int point_size = points_in.size();
        // Begin sorting by ordering using angles
        for (int i = 0; i < point_size; i++) 
        {
            double angle;

            // check to make sure the angle won't be "0"
            if (points_in[i].x() == centroid.x()) 
                angle = 0.0;
            else
                angle = atan((points_in[i].y() - centroid.y()) / (points_in[i].x() - centroid.x()));
            
            point_angle_pair.push_back(
                make_pair(angle, i));
        }

        // Using simple sort() function to sort
        sort(point_angle_pair.begin(), point_angle_pair.end());

        points_out.clear();
        for (int i = 0; i < point_size; i++) 
            points_out.push_back(points_in[point_angle_pair[i].second]);
    }

    bool visibility::get_line_plane_intersection(
        std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e, 
        Eigen::Vector3d normal, Eigen::Vector3d pop, Eigen::Vector3d &p)
    {
        // A code from OpenGL C# application
        // https://stackoverflow.com/a/71407596
        double epsilon = 0.0001;

        Eigen::Vector3d ray = s_e.second - s_e.first;
        double ray_dist = ray.norm(); 

        // Dot product of normal and point
        double d_n_p = normal.x() * pop.x() + normal.y() * pop.y() + normal.z() * pop.z();
        // Dot product of normal and ray
        double d_n_r = normal.x() * ray.x() + normal.y() * ray.y() + normal.z() * ray.z();
        if (d_n_r <= epsilon && d_n_r >= -epsilon)
            return false;
        
        // Dot product of normal and ray_origin
        double d_n_ro = normal.x() * s_e.first.x() + normal.y() * s_e.first.y() + normal.z() * s_e.first.z();
        double t = (d_n_p - d_n_ro) / d_n_r;

        if (t > ray_dist)
            return false;
        
        p = s_e.first + ray * t; 

        return true;
    }

    void visibility::get_polygons_on_plane(
        global_map g_m, Eigen::Vector3d normal, 
        vector<obstacle> &polygons, vector<Eigen::Vector3d> &v)
    {        

        // https://karlobermeyer.github.io/VisiLibity1/doxygen_html/class_visi_libity_1_1_environment.html
        // the outer boundary vertices must be listed ccw and the hole vertices cw 
        for (obstacle &obs : g_m.obs)
        {
            // Get the centroid 
            Eigen::Vector2d centroid;
            vector<Eigen::Vector2d> vert;             

            int obs_vert_size = obs.v.size();
            for (int i = 0; i < obs_vert_size; i++)
            {
                Eigen::Vector3d o_vert;
                std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
                vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), 0.0);
                vert_pair.second = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h);

                if (!get_line_plane_intersection(vert_pair, normal, g_m.start_end.first, o_vert))
                    continue;
                
                // Transform original vertex into 2d, o_vert to t_vert
                Eigen::Vector3d t_vert;
                v.push_back(o_vert);
                
                vert.push_back(Eigen::Vector2d(t_vert.x(), t_vert.y()));
            }

            obs.c = get_centroid_2d(vert);
            
            // No height data since its a flat plane
            obs.h = 0.0;

            // Organize vertices of holes in clockwise format
            graham_scan(vert, obs.c, obs.v);

            polygons.push_back(obs);
        }
    }

    void visibility::get_expanded_obs(obstacle &obs, double inflation)
    {
        obstacle tmp = obs;
        int vect_size = tmp.v.size();
        for (int i = 0; i < vect_size; i++)
        {
            // Write over the obstacle vertices 
            Eigen::Vector2d norm_vect = (tmp.v[i] - tmp.c).normalized();
            obs.v[i] = tmp.v[i] + norm_vect * inflation;
        }
    }

    Eigen::Vector2d visibility::get_centroid_2d(
        vector<Eigen::Vector2d> vert)
    {
        int point_size = (int)vert.size();
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (Eigen::Vector2d &p : vert)
            centroid += p;
        
        return centroid/point_size;
    }

    Eigen::Affine3d get_affine_transform(
        Eigen::Vector3d pos, Eigen::Vector3d rpy, string frame)
    {
        Eigen::Affine3d affine;

        // Get rotation matrix from RPY
        // https://stackoverflow.com/a/21414609
        Eigen::AngleAxisd rollAngle(rpy.x(), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(rpy.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(rpy.z(), Eigen::Vector3d::UnitX());

        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

        Eigen::Matrix3d rot = q.matrix();

        affine.translation() = pos;
        affine.linear() = rot;

        return affine;
    }

    void visibility::get_visibility_path()
    {

    }

    void visibility::main_timer(const ros::TimerEvent &)
    {

    }

}