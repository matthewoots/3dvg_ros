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
    /** 
     * @brief Sum up from a range of int values
     * @param s start of range
     * @param e end of the range
    **/
    int visibility::sum_of_range(int s, int e)
    {
        return e*(e+1)/2 - s*(s+1)/2 + s;
    }

    /** 
     * @brief gift_wrapping algorithm
     * @param points_in points that are passed into the algorithm
     * @param (Return) Vector of points that are at the edge (convex hull)
    **/
    vector<Eigen::Vector2d> visibility::gift_wrapping(
        vector<Eigen::Vector2d> points_in)
    {
        vector<Eigen::Vector2d> vect;
        int n = (int)points_in.size();
        int next[n];
        int l = 0;
        for (int i = 0; i < n; i++)
        {
            next[i] = -1;
            // Find the leftmost point
            if (points_in[i].x() < points_in[l].x())
                l = i;
        }
        int p = l, q = -1;
        int total_possible_attempts = pow(n,2);
        int tries = 0;
        do
        {
            q = (p+1) % n;
            for (int i = 0; i < n; i++)
            {
                int val = 
                    (points_in[i].y() - points_in[p].y()) * (points_in[q].x() - points_in[i].x()) -
                    (points_in[i].x() - points_in[p].x()) * (points_in[q].y() - points_in[i].y());
                // clockwise direction
                if (val > 0)
                    q = i;
            }
            next[p] = q;
            p = q;
            tries++;
        }
        while (p != l && tries < total_possible_attempts);

        for (int i = 0; i < n; i++)
        {
            if (next[i] != -1)
                vect.push_back(points_in[i]);
        }

        return vect;

    }

    /** 
     * @brief graham_scan, sorts the polygon clockwise https://stackoverflow.com/a/57454410
     * Holes listed in clockwise or counterclockwise up to the direction
     * @param points_in Vector of points in
     * @param centroid Centroid of the flattened polygon
     * @param dir Clockwise or counterclockwise direction sorting of points_out
     * @param points_out (Return) Vector of points out
    **/
    void visibility::graham_scan(
        vector<Eigen::Vector2d> points_in, Eigen::Vector2d centroid, 
        string dir, vector<Eigen::Vector2d> &points_out)
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
                angle = atan2((points_in[i].y() - centroid.y()), (points_in[i].x() - centroid.x()));
            
            point_angle_pair.push_back(
                make_pair(angle, i));
        }

        // Using simple sort() function to sort
        // By default it is 
        sort(point_angle_pair.begin(), point_angle_pair.end());

        points_out.clear();
        if (strcmp(dir.c_str(), "ccw") == 0)
        {
            for (int i = 0; i < point_size; i++) 
                points_out.push_back(points_in[point_angle_pair[i].second]);
        }
        else if (strcmp(dir.c_str(), "cw") == 0)
        {
            for (int i = point_size-1; i >= 0; i--) 
                points_out.push_back(points_in[point_angle_pair[i].second]);
        }
    }

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
    bool visibility::find_nearest_distance_2d_polygons_and_fuse(
        obstacle o1, obstacle o2, double safety_radius,
        std::pair<Eigen::Vector2d, Eigen::Vector2d> &points_out, 
        double &nearest_distance, obstacle &o3)
    {
        // The polygon vertices should be sorted
        // Use the euclidean distance to estimate the closest vertices
        std::pair<Eigen::Vector2d, Eigen::Vector2d> direction_pair;
        direction_pair.first = (o2.c - o1.c).normalized(); // o1 -> o2
        direction_pair.second = (o1.c - o2.c).normalized(); // o2 -> o1

        vector<std::pair<int,int>> i1, i2;
        double dot1 = 0.0, dot2 = 0.0;
        for (int i = 0; i < (int)o1.v.size(); i++)
        {
            std::pair<int,int> p_i;
            Eigen::Vector2d v1 = o1.v[i] - o1.c;
            double d1 = 
                direction_pair.first.x() * v1.x() + direction_pair.first.y() * v1.y();

            // Same direction
            if (d1 > dot1)
            {
                i1.clear();
                p_i = make_pair(
                    i-1 < 0 ? i-1+(int)o1.v.size() : i-1, i);
                i1.push_back(p_i);
                p_i = make_pair(
                    i, (i+1)%(int)o1.v.size());
                i1.push_back(p_i);

                dot1 = d1;
            }
                
        }

        for (int i = 0; i < (int)o2.v.size(); i++)
        {
            std::pair<int,int> p_i;
            Eigen::Vector2d v2 = o2.v[i] - o2.c;
            double d2 = 
                direction_pair.second.x() * v2.x() + direction_pair.second.y() * v2.y();
            
            // Same direction
            if (d2 > dot2)
            {
                i2.clear();
                p_i = make_pair(
                    i-1 < 0 ? i-1+(int)o2.v.size() : i-1, i);
                i2.push_back(p_i);
                p_i = make_pair(
                    i, (i+1)%(int)o2.v.size());
                i2.push_back(p_i);

                dot2 = d2;
            }
        }
        
        bool fuse = false;
        nearest_distance = safety_radius * 1.25;

        for (std::pair<int,int> &idx1 : i1)
        {
            for (std::pair<int,int> &idx2 : i2)
            {
                std::pair<Eigen::Vector2d, Eigen::Vector2d> c_p;
                double dist;
                bool is_parallel = !closest_points_between_lines(
                    o1.v[idx1.first], o1.v[idx1.second],
                    o2.v[idx2.first], o2.v[idx2.second],
                    c_p, dist);
                /** @brief For debug purpose **/
                // std::cout << "closest dist = " << dist << std::endl;

                if (dist < nearest_distance)
                {
                    fuse = true;
                    nearest_distance = dist;
                    points_out = c_p;
                }
                // Lines are parallel and distance is very close
                // Drop 1 edge and fuse the polygons together
                // if (is_parallel && dist < safety_radius)
            }
        }        

        if (!fuse)
            return false;

        o3.h = 0.0;

        // Append the 2 vectors
        vector<Eigen::Vector2d> tmp_vertices, new_vertices;
        tmp_vertices = o1.v;
        tmp_vertices.insert(tmp_vertices.end(), o2.v.begin(), o2.v.end());
        /** @brief For debug purpose **/
        // std::cout << "tmp_vertices.size() = " << (int)tmp_vertices.size() << std::endl;
        
        new_vertices = gift_wrapping(tmp_vertices);
        /** @brief For debug purpose **/
        // std::cout << "gift_wrapping.size() = " << (int)new_vertices.size() << std::endl;
        o3.c = get_centroid_2d(new_vertices);
        graham_scan(new_vertices, o3.c, "cw", o3.v);

        return true;
    }

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
    bool visibility::closest_points_between_lines(
        Eigen::Vector2d a0, Eigen::Vector2d a1,
        Eigen::Vector2d b0, Eigen::Vector2d b1,
        std::pair<Eigen::Vector2d, Eigen::Vector2d> &c_p,
        double &distance)
    {
        // Converted from Python code from
        // https://stackoverflow.com/a/18994296

        double epsilon = 0.000001;

        Eigen::Vector2d a = a1 - a0;
        Eigen::Vector2d b = b1 - b0;
        double m_a = a.norm();
        double m_b = b.norm();

        Eigen::Vector2d a_v = a.normalized();
        Eigen::Vector2d b_v = b.normalized();

        double denom = a_v.x() * b_v.y() - a_v.y() * b_v.x();
        // double denom = pow(cross.norm(), 2);

        // If lines are parallel (denom=0) test if lines overlap.
        // If they don't overlap then there is a closest point solution.
        // If they do overlap, there are infinite closest positions, but there is a closest distance
        if (abs(denom) < epsilon)
        {
            // Lines are parallel, there can be closest points
            double d0 = a_v.x() * (b0-a0).x() + a_v.y() * (b0-a0).y();
            double d1 = a_v.x() * (b1-a0).x() + a_v.y() * (b1-a0).y();

            // Is segment B before A?
            if (d0 <= m_a && d1 <= m_a)
            {
                if (abs(d0) < abs(d1))
                {
                    c_p.first = a0; c_p.second = b0;
                    distance = (a0-b0).norm();
                    return true;
                }
                else
                {
                    c_p.first = a0; c_p.second = b1;
                    distance = (a0-b1).norm();
                    return true;
                }
            }
            // Is segment B after A?
            else if (d0 >= m_a && d1 >= m_a)
            {
                if (abs(d0) < abs(d1))
                {
                    c_p.first = a1; c_p.second = b0;
                    distance = (a1-b0).norm();
                    return true;
                }
                else
                {
                    c_p.first = a1; c_p.second = b1;
                    distance = (a1-b1).norm();
                    return true;
                }
            }
            // Segments overlap, return distance between parallel segments
            else
            {
                distance = (((d0 * a_v) + a0) - b0).norm();
                return false;
            }
        }
        
        // Lines criss-cross: Calculate the projected closest points
        Eigen::Vector2d t = b0 - a0;
        double det_a = t.x() * a_v.y() - t.y() * a_v.x();
        double det_b = t.x() * b_v.y() - t.y() * b_v.x();

        double t0 = det_a / denom;
        double t1 = det_b / denom;

        Eigen::Vector2d p_a = a0 + (a_v * t0); // Projected closest point on segment A
        Eigen::Vector2d p_b = b0 + (b_v * t1); // Projected closest point on segment B

        // Clamp projection A
        if (t0 < 0.0 || t0 > m_a)
        {
            if (t0 < 0.0)
                p_a = a0;
            else if (t0 > m_a)
                p_a = a1;
            
            double dot = 
                b_v.x() * (p_a-b0).x() + b_v.y() * (p_a-b0).y();
            if (dot < 0.0)
                dot = 0;
            else if (dot > m_b)
                dot = m_b;
            p_b = b0 + (b_v * dot);
        }
        
        
        // Clamp projection B
        if (t0 < 0.0 || t0 > m_a)
        {
            if (t1 < 0)
                p_b = b0;
            else if (t1 > m_a)
                p_b = b1;

            double dot = 
                a_v.x() * (p_b-a0).x() + a_v.y() * (p_b-a0).y();
            if (dot < 0.0)
                dot = 0;
            else if (dot > m_a)
                dot = m_a;
            p_a = a0 + (a_v * dot);
        }

        c_p.first = p_a; c_p.second = p_b;
        distance = (p_a - p_b).norm();

        return true;
    }

    /** 
     * @brief set_2d_min_max_boundary
     * @param obstacles Vector of flattened obstacles
     * @param start_end Start and end flattened points
     * @param boundary (Return) The minimum and maximum of the inputs
    **/
    void visibility::set_2d_min_max_boundary(
        vector<obstacle> obstacles, std::pair<Eigen::Vector2d, Eigen::Vector2d> start_end, std::pair<Eigen::Vector2d, Eigen::Vector2d> &boundary)
    {
        vector<Eigen::Vector2d> query_vector;
        for (obstacle &obs : obstacles)
        {
            for (int j = 0; j < (int)obs.v.size(); j++)
                query_vector.push_back(obs.v[j]);
        }
        query_vector.push_back(start_end.first);
        query_vector.push_back(start_end.second);

        Eigen::Vector2d max(-1000.0, -1000.0), min(1000.0, 1000.0);
        for (Eigen::Vector2d &v : query_vector)
        {
            if (v.x() < min.x())
                min.x() = v.x();
            if (v.x() > max.x())
                max.x() = v.x();
            
            if (v.y() < min.y())
                min.y() = v.y();
            if (v.y() > max.y())
                max.y() = v.y();
        }

        boundary.first = min;
        boundary.second = max;
    }

    /** 
     * @brief boundary_to_polygon_vertices
     * @param min_max The minimum and maximum of the inputs
     * @param dir Pass the direction into the graham search to sort the vertices
     * @param (Return) The AABB of the inputs represented in the 4 vertices
    **/
    vector<Eigen::Vector2d> visibility::boundary_to_polygon_vertices(
        std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max, string dir)
    {
        vector<Eigen::Vector2d> tmp, vect;
        // Establish 4 corners
        tmp.push_back(min_max.first);
        tmp.push_back(min_max.second);
        tmp.push_back(Eigen::Vector2d(min_max.first.x(), min_max.second.y()));
        tmp.push_back(Eigen::Vector2d(min_max.second.x(), min_max.first.y()));

        graham_scan(tmp, Eigen::Vector2d(
            (min_max.first.x() + min_max.second.x()) / 2, (min_max.first.y() + min_max.second.y()) / 2), 
            dir, vect);

        return vect;
    }

    /** 
     * @brief get_line_plane_intersection
     * @param s_e Start and end pair
     * @param normal Normal of the plane
     * @param pop Point on plane
     * @param p (Return) Point of intersection
    **/
    bool visibility::get_line_plane_intersection(
        std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e, 
        Eigen::Vector3d normal, Eigen::Vector3d pop, Eigen::Vector3d &p)
    {
        // https://stackoverflow.com/a/23976134
        double epsilon = 0.0001;
        double t;

        Eigen::Vector3d ray_raw = s_e.second - s_e.first;
        Eigen::Vector3d ray = ray_raw.normalized();
        double ray_dist = ray_raw.norm(); 
        Eigen::Vector3d ray_to_p = pop - s_e.first; 
        double d_rp_n = normal.x() * ray_to_p.x() + normal.y() * ray_to_p.y() + normal.z() * ray_to_p.z();
        double d_n_r = normal.x() * ray.x() + normal.y() * ray.y() + normal.z() * ray.z();
        
        if (abs(d_n_r) > epsilon)
        {
            t = d_rp_n / d_n_r;
            if (t < 0 || t > ray_dist) 
                return false; 
        }
        
        p = s_e.first + ray * t; 
        /** @brief For debug purpose **/
        // std::cout << "p = " << p.transpose() << " t = " << t << std::endl;

        return true;
    }

    /** 
     * @brief get_polygons_on_plane
     * @param g_m Pass in the global map
     * @param normal Normal of the plane
     * @param polygons (Return) Return the vector of flattened polygons
     * @param v (Return) Return the vertices in 3d (not transformed)
    **/
    void visibility::get_polygons_on_plane(
        global_map g_m, Eigen::Vector3d normal, 
        vector<obstacle> &polygons, vector<Eigen::Vector3d> &v)
    {        
        v.clear();
        
        // https://karlobermeyer.github.io/VisiLibity1/doxygen_html/class_visi_libity_1_1_environment.html
        // the outer boundary vertices must be listed ccw and the hole vertices cw 
        for (obstacle &obs : g_m.obs)
        {
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
                v.push_back(o_vert);

                Eigen::Vector3d t_vert = map.t * o_vert;
                /** @brief For debug purpose **/
                // std::cout << t_vert.transpose() << std::endl;
                
                vert.push_back(Eigen::Vector2d(t_vert.x(), t_vert.y()));
            }

            obs.c = get_centroid_2d(vert);
            
            // No height data since its a flat plane
            obs.h = 0.0;

            // Organize vertices of holes in clockwise format
            graham_scan(vert, obs.c, "cw", obs.v);

            polygons.push_back(obs);
        }
    }

    /** 
     * @brief get_expansion_of_obs
     * @param obs Obstacle as input and output 
     * @param inflation Inflation amount
    **/
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

    /** 
     * @brief get_centroid_2d
     * @param vect Vector of points used to get centroid
     * @param (Return) Centroid
    **/
    Eigen::Vector2d visibility::get_centroid_2d(
        vector<Eigen::Vector2d> vert)
    {
        int point_size = (int)vert.size();
        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (Eigen::Vector2d &p : vert)
            centroid += p;
        
        return centroid/point_size;
    }

    /** 
     * @brief get_rotation
     * @param rpy Euler angles in Eigen::Vector3d
     * @param frame Coordinate frame used
     * @param (Return) 3x3 Rotation matrix
    **/
    Eigen::Matrix3d visibility::get_rotation(
        Eigen::Vector3d rpy, std::string frame)
    {
        Eigen::Vector3d orientated_rpy;
        // y is pitch, and RHR indicates positive to be anticlockwise (z is neg)
        // Hence to counter pitch direction we can leave rpy.y() positive
        // z is yaw, and RHR indicates positive to be anticlockwise (y is pos)
        // Hence to counter yaw direction we need to make rpy.z() negative
        if (strcmp(frame.c_str(), "nwu") == 0)
            orientated_rpy = Eigen::Vector3d(0.0, rpy.y(), -rpy.z());
        // x is pitch, and RHR indicates positive to be anticlockwise (z is pos)
        // Hence to counter pitch direction we need to make rpy.x() negative
        // z is yaw, and RHR indicates positive to be anticlockwise (y is pos)
        // Hence to counter yaw direction we need to make rpy.z() negative
        else if (strcmp(frame.c_str(), "enu") == 0)
            orientated_rpy = Eigen::Vector3d(0.0, rpy.x(), -rpy.z());

        // Get rotation matrix from RPY
        // https://stackoverflow.com/a/21414609
        Eigen::AngleAxisd rollAngle(orientated_rpy.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(orientated_rpy.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(orientated_rpy.z(), Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;

        return q.matrix();
    }

    /**
     * @brief get_affine_transform
     * @param pos Translational position
     * @param rpy Euler angles
     * @param frame Coordinate frame used
     * @param (Return) Affine3d matrix
    **/
    Eigen::Affine3d visibility::get_affine_transform(
        Eigen::Vector3d pos, Eigen::Vector3d rpy, 
        std::string frame)
    {
        Eigen::Affine3d affine;
        Eigen::Matrix3d rot = get_rotation(rpy, frame);
        Eigen::Vector3d rot_pos = rot * -pos;

        affine.translation() = rot_pos;
        affine.linear() = rot;

        return affine;
    }

    /**
     * @brief Main loop
    **/
    void visibility::get_visibility_path()
    {
        debug_point_vertices.clear();

        Eigen::Vector3d direction = map.start_end.second - map.start_end.first;
        direction.normalized();

        if (strcmp(frame.c_str(), "nwu") == 0)
        {
            double yaw = atan2(direction.y(), direction.x());
            Eigen::Vector2d h_xy = Eigen::Vector2d(direction.x(), direction.y());
            double length_h_xy = h_xy.norm();
            double pitch = atan2(direction.z(), length_h_xy);
            map.rpy = Eigen::Vector3d(0.0, pitch, yaw);
            map.t = get_affine_transform(map.start_end.first, map.rpy, frame);
        }

        else if (strcmp(frame.c_str(), "enu") == 0)
        {
            double yaw = atan2(direction.y(), direction.x());
            Eigen::Vector2d h_xy = Eigen::Vector2d(direction.x(), direction.y());
            double length_h_xy = h_xy.norm();
            double pitch = atan2(direction.z(), length_h_xy);
            map.rpy = Eigen::Vector3d(pitch, 0.0, yaw);
            map.t = get_affine_transform(map.start_end.first, map.rpy, frame);
        }

        std::pair<Eigen::Vector3d, Eigen::Vector3d> rot_pair;
        rot_pair.first = map.t * map.start_end.first;
        rot_pair.second = map.t * map.start_end.second;
        std::pair<Eigen::Vector2d, Eigen::Vector2d> rot_pair_2d;
        rot_pair_2d.first = Eigen::Vector2d(rot_pair.first.x(), rot_pair.first.y());
        rot_pair_2d.second = Eigen::Vector2d(rot_pair.second.x(), rot_pair.second.y());

        /** @brief Check transform **/
        // std::cout << "original = " << (map.t.inverse() * rot_pair.first).transpose() << " to " << 
        //      (map.t.inverse() * rot_pair.second).transpose() << std::endl;
        // std::cout << "transformed = " << rot_pair.first.transpose() << " to " << 
        //     rot_pair.second.transpose() << std::endl;

        // Get the plane normal
        Eigen::Vector3d normal = 
            get_rotation(map.rpy, frame).inverse() * Eigen::Vector3d(0.0, 0.0, 1.0);

        rot_polygons.clear();
        get_polygons_on_plane(map, normal, rot_polygons, debug_point_vertices);

        int count = 0, check_size = 1;

        int total_tries = sum_of_range(1, (int)rot_polygons.size()-1);
        int tries = 0;
        while (count != check_size && tries < pow(total_tries,1.5))
        {
            int poly_size = (int)rot_polygons.size();
            check_size = sum_of_range(1, poly_size-1);
            count = 0;

            for (int i = 0; i < poly_size; i++)
            {
                bool early_break = false;
                for (int j = 0; j < poly_size; j++)
                {
                    if (i <= j)
                        continue;
                    
                    std::pair<Eigen::Vector2d, Eigen::Vector2d> p_o;
                    double n_d;
                    obstacle o3;
                    if (rot_polygons[i].v.empty() || rot_polygons[j].v.empty())
                    {
                        count++;
                        continue;
                    }

                    if (find_nearest_distance_2d_polygons_and_fuse(
                        rot_polygons[i], rot_polygons[j], protected_zone*1.5, p_o, n_d, o3))
                    {
                        /** @brief For debug purpose **/
                        // std::cout << "fuse" << std::endl;

                        vector<obstacle> tmp = rot_polygons;
                        rot_polygons.clear(); 
                        for (int k = 0; k < poly_size; k++)
                        {
                            if (k != i && k != j)
                                rot_polygons.push_back(tmp[k]);
                        }
                        rot_polygons.push_back(o3);
                        early_break = true;
                        break;
                    }
                    count++;
                }
                if (early_break)
                    break;
            }

            tries++;
            /** @brief For debug purpose **/
            // std::cout << count << "/" << check_size << "/" << poly_size << std::endl;
        }

        for (int i = 0; i < (int)rot_polygons.size(); i++)
            get_expanded_obs(rot_polygons[i], protected_zone);

        /** @brief For debug purpose **/
        // std::cout << "final_polygon_size = " << (int)rot_polygons.size() << std::endl;

        /** @brief For debug purpose **/
        // for (Eigen::Vector3d &p : debug_point_vertices)
        //     std::cout << p.transpose() << std::endl;
        // std::cout << std::endl;

        vector<VisiLibity::Polygon> vector_polygon;

        // Create the polygon for boundary
        std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max;
        set_2d_min_max_boundary(rot_polygons, rot_pair_2d, min_max);
        vector<Eigen::Vector2d> boundary =
            boundary_to_polygon_vertices(min_max, "ccw");
        VisiLibity::Polygon boundary_polygon;
        std::vector<VisiLibity::Point> boundary_vertices;
        for (Eigen::Vector2d &p : boundary)
        {
            VisiLibity::Point vis_vert(p.x(), p.y());
            boundary_vertices.push_back(vis_vert);
        }
        boundary_polygon.set_vertices(boundary_vertices);

        VisiLibity::Environment my_environment;
        my_environment.set_outer_boundary(boundary_polygon);

        if (!rot_polygons.empty())
        {
            // Create the polygons for holes
            for (obstacle &poly : rot_polygons)
            {
                VisiLibity::Polygon polygon;
                
                if (!poly.v.empty())
                {
                    /** @brief For debug purpose **/
                    // printf("poly_vert_size %d\n",(int)poly.v.size());

                    for (int i = 0; i < (int)poly.v.size(); i++)
                        polygon.push_back(
                            VisiLibity::Point(poly.v[i].x(), poly.v[i].y()));

                    polygon.eliminate_redundant_vertices(0.1);
                    polygon.enforce_standard_form();

                    /** @brief For debug purpose **/
                    // printf("polygon_size %d, standard %s, simple %s\n", 
                    //     polygon.n(), polygon.is_in_standard_form() ? "y" : "n",
                    //     polygon.is_simple() ? "y" : "n");
                    
                    my_environment.add_hole(polygon);
                    vector_polygon.push_back(polygon);
                }
            }
        }
        else
            printf("empty environment\n");

        VisiLibity::Polyline shortest_path_poly;
        VisiLibity::Point start_vis(rot_pair_2d.first.x(), rot_pair_2d.first.y());
        VisiLibity::Point end_vis(rot_pair_2d.second.x(), rot_pair_2d.second.y());
        shortest_path_poly = my_environment.shortest_path(start_vis, end_vis, 0.2);

        shortest_path_3d.clear();
        for (int i = 0; i < shortest_path_poly.size(); i++)
        {
            VisiLibity::Point point = shortest_path_poly[i];
            shortest_path_3d.push_back(
                map.t.inverse() * Eigen::Vector3d(
                point.x(), point.y(), 0.0));
        }

        return;
    }

    visualization_msgs::Marker visibility::visualize_line_list(
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

    visualization_msgs::Marker visibility::visualize_points(
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

    void visibility::main_timer(const ros::TimerEvent &)
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
        }

        time_point<std::chrono::system_clock> start_time = system_clock::now();
        get_visibility_path();
        double search_time = duration<double>(system_clock::now() - start_time).count();
        std::cout << "get_visibility_path time (" << KBLU << search_time * 1000 << KNRM << "ms)" << std::endl;
        found = true;
    }

    void visibility::visualization_timer(const ros::TimerEvent &)
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
                vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), 0.0);
                vert_pair.second = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h);
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
                    obs.v[i % obs_hori_pair_size].y(), 0.0);
                vert_pair.second = Eigen::Vector3d(
                    obs.v[(i+1) % obs_hori_pair_size].x(), 
                    obs.v[(i+1) % obs_hori_pair_size].y(), 0.0);
                vect_vert.push_back(vert_pair);

                vert_pair.first = Eigen::Vector3d(
                    obs.v[i % obs_hori_pair_size].x(), 
                    obs.v[i % obs_hori_pair_size].y(), obs.h);
                vert_pair.second = Eigen::Vector3d(
                    obs.v[(i+1) % obs_hori_pair_size].x(), 
                    obs.v[(i+1) % obs_hori_pair_size].y(), obs.h);
                vect_vert.push_back(vert_pair);
            }

        }

        obs_visualize = visualize_line_list(
            vect_vert, color_range[1], 0.1, 2, 0.75);
        obstacle_pub.publish(obs_visualize);

        if (!found)
            return;

        debug_vertices = visualize_points(
            debug_point_vertices, color_range[2], 0.2, 3);
        obstacle_pub.publish(debug_vertices);

        vect_vert.clear();
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
            visualize_line_list(debug_pair, color_range[3], 0.08, 4, 0.15);
        obstacle_pub.publish(vector_visualize);


        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> shortest_path_3d_pair;
        for (int i = 0; i < (int)shortest_path_3d.size() - 1; i++)
            shortest_path_3d_pair.push_back(
                make_pair(shortest_path_3d[i], shortest_path_3d[i+1]));
        
        visualization_msgs::Marker shortest_path_visualize = 
            visualize_line_list(shortest_path_3d_pair, color_range[4], 0.1, 5, 0.5);
        visibility_graph_pub.publish(shortest_path_visualize);

    } 

}