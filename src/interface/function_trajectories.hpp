/**
 * Module:      function_trajectories.hpp
 * Description: trajectory util functions for AutoKU
 * 
 * Authors: Yuseung Na (ys.na0220@gmail.com)
 *          XXXXXXX XXX (@gmail.com)
 * 
 * Revision History
 *      June 14, 2023: Yuseung Na - Created.
 *      XXXX XX, XXXX: XXXXXXX XX - 
 */

#ifndef __FUNCTION_TRAJECTORIES_HPP__
#define __FUNCTION_TRAJECTORIES_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// Utility header
#include <spline.h>

// Interface Header
#include "interface_constants.hpp"
#include "interface_trajectories.hpp"
#include "function_print.hpp"

using namespace autoku_types;

namespace autoku_functions{
    inline tk::Map GenerateRoadMap(const Trajectory& trajectory) {
        if ((uint16_t)trajectory.point.size() < 4) {
            tk::Map road_map;
            autoku_functions::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return road_map;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
                double x = trajectory.point[i].x;
                double y = trajectory.point[i].y;
                double dx = i == 0 ? 0.0 : x - trajectory.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - trajectory.point[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }
            
            return tk::Map(sx, sy, ss, sdx, sdy);
        }
    }

    inline tk::Map GenerateBoundaryMap(const TrajectoryBoundary& trajectory_boundary) {
        if ((uint16_t)trajectory_boundary.point.size() < 4) {
            tk::Map road_map;
            autoku_functions::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return road_map;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)trajectory_boundary.point.size(); i++) {
                double x = trajectory_boundary.point[i].x;
                double y = trajectory_boundary.point[i].y;
                double dx = i == 0 ? 0.0 : x - trajectory_boundary.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - trajectory_boundary.point[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }
            
            return tk::Map(sx, sy, ss, sdx, sdy);
        }
    }

    inline bool GetPredictionFrenetReference(const Trajectory& reference, const int& reference_line,
                                             std::vector<double>& sx, std::vector<double>& sy, std::vector<double>& ss,
                                             std::vector<double>& sdx, std::vector<double>& sdy) {
        if (reference.point.size() < 4) {
            autoku_functions::DebugPrintValueError(
                "Size of reference_map point is not enough to calculate frenet reference!\n"
                "Size of reference_map point: ", (int)reference.point.size()
            );
            return false;
        }
        
        // Boundary
        if (reference_line == 0 || reference_line == 1 || reference_line == 2) {
            TrajectoryBoundary boundary;
            if (reference_line == 0) {
                boundary = reference.left_boundary;
            }
            else if (reference_line == 1) {
                boundary = reference.center_boundary;
            }
            else if (reference_line == 2) {
                boundary = reference.right_boundary;
            }
            
            if ((int)boundary.point.size() < 4) {
                autoku_functions::DebugPrintValueError(
                    "Size of boundary point is not enough to calculate frenet reference!\n"
                    "Size of boundary point: ", (int)boundary.point.size()
                );
                return false;
            }
            
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)boundary.point.size(); i++) {
                double x = boundary.point[i].x;
                double y = boundary.point[i].y;
                double dx = i == 0 ? 0.0 : x - boundary.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - boundary.point[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }
        }
        // Race line
        else {
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)reference.point.size(); i++) {
                double x = reference.point[i].x;
                double y = reference.point[i].y;
                double dx = i == 0 ? 0.0 : x - reference.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - reference.point[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }
        }
        
        if ((int)ss.size() < 4) {
            autoku_functions::DebugPrintValueError(
                "Size of output frenet point is not enough!\n"
                "Size of output frenet point:", (int)ss.size()
            );
            return false;
        }

        return true;
    }

    inline void GenerateBoundarySpline(const Trajectory& reference, 
                                       tk::spline& left_boundary, 
                                       tk::spline& right_boundary) {
        if (reference.left_boundary.point.size() < 4 || reference.right_boundary.point.size() < 4) {            
            autoku_functions::DebugPrintValueError(
                "Boundary points are not enough!",
                (uint16_t)reference.left_boundary.point.size()
            );            
            return;
        }
        else {
            std::vector<double> left_s, left_n, right_s, right_n;
            for (auto lp : reference.left_boundary.point) {
                left_s.push_back(lp.s);
                left_n.push_back(lp.n);
            }
            for (auto rp : reference.right_boundary.point) {
                right_s.push_back(rp.s);
                right_n.push_back(rp.n);
            }

            left_boundary.set_points(left_s, left_n, false);
            right_boundary.set_points(right_s, right_n, false);
        }
    }

    inline void GenerateBoundarySplineCenter(const Trajectory& reference, 
                                             tk::Map& center_map,
                                             tk::spline& left_boundary, 
                                             tk::spline& right_boundary) {
        if (reference.left_boundary.point.size() < 4 || reference.right_boundary.point.size() < 4) {            
            autoku_functions::DebugPrintValueError(
                "Boundary points are not enough!",
                (uint16_t)reference.left_boundary.point.size()
            );            
            return;
        }
        else {
            std::vector<double> left_s, left_n, right_s, right_n;
            for (auto lp : reference.left_boundary.point) {
                std::vector<double> bp_sn = center_map.ToFrenet(lp.x, lp.y);
                left_s.push_back(bp_sn.at(0));
                left_n.push_back(bp_sn.at(1));
            }
            for (auto rp : reference.right_boundary.point) {
                std::vector<double> bp_sn = center_map.ToFrenet(rp.x, rp.y);
                right_s.push_back(bp_sn.at(0));
                right_n.push_back(bp_sn.at(1));
            }

            left_boundary.set_points(left_s, left_n);
            right_boundary.set_points(right_s, right_n);
        }
    }

    inline Trajectory ConvertTrajectoryByCenter(const Trajectory& trajectory,
                                                tk::Map& center_map) {
        Trajectory converted_trajectory = trajectory;

        for (auto& point : converted_trajectory.point) {
            std::vector<double> bp_sn = center_map.ToFrenet(point.x, point.y);
            point.s = bp_sn.at(0);
            point.n = bp_sn.at(1);
            point.distance = point.s;
        }

        return converted_trajectory;
    }

    inline Trajectory ResampleTrajectoryDistance(const Trajectory& trajectory,
                                                 const double& resample_distance) {

        Trajectory resampled_trajectory = trajectory;

        if (resampled_trajectory.point.size() < 4) {
            return resampled_trajectory;
        }        
        else {
            std::vector<double> time, x, y, speed, distance, curvature;
            for (auto point : trajectory.point) {
                time.push_back(point.time - trajectory.point.at(0).time);
                x.push_back(point.x);
                y.push_back(point.y);
                speed.push_back(point.speed);
                distance.push_back(point.distance);
                curvature.push_back(point.curvature);

                // autoku_functions::DebugPrintInfo(
                //     "time: " + std::to_string(point.time) 
                //     + ", speed: " + std::to_string(point.speed) 
                //     + ", distance: " + std::to_string(point.distance)
                // );
            }

            resampled_trajectory.point.clear();
            tk::spline sx, sy, sv, sa, st, sk;
            sx.set_points(distance, x);
            sy.set_points(distance, y);
            sv.set_points(distance, speed);
            st.set_points(distance, time);
            sk.set_points(distance, curvature);

            int num_total_point = std::ceil((distance.back() - distance.front()) / resample_distance);
            double start_s = distance.front();

            for (uint16_t i = 0; i < num_total_point; i++) {
                TrajectoryPoint tp;
                tp.time = st(resample_distance * i + start_s) + trajectory.point.at(0).time;
                tp.x = sx(resample_distance * i + start_s);
                tp.y = sy(resample_distance * i + start_s);
                tp.z = 0.0;
                tp.speed = sv(resample_distance * i + start_s);
                tp.acceleration = 0.0;
                tp.distance = resample_distance * i;
                double dx = sx(resample_distance * (i+1) + start_s) - tp.x;
                double dy = sy(resample_distance * (i+1) + start_s) - tp.y;
                tp.yaw = atan2(dy, dx);
                tp.curvature = sk(resample_distance * i + start_s);
                resampled_trajectory.point.push_back(tp);

                // autoku_functions::DebugPrintInfo(
                //     "time: " + std::to_string(tp.time) 
                //     + ", speed: " + std::to_string(tp.speed) 
                //     + ", distance: " + std::to_string(tp.distance)
                // );
            }
        }

        return resampled_trajectory;
    }

    inline Trajectory ResampleTrajectoryTime(const Trajectory& trajectory,
                                             const double& resample_time) {

        Trajectory resampled_trajectory = trajectory;

        if (resampled_trajectory.point.size() < 4) {
            return resampled_trajectory;
        }        
        else {
            std::vector<double> time, x, y, speed, distance;
            for (auto point : resampled_trajectory.point) {
                time.push_back(point.time);
                x.push_back(point.x);
                y.push_back(point.y);
                speed.push_back(point.speed);
                distance.push_back(point.distance);
            }

            resampled_trajectory.point.clear();
            
            tk::spline st, tx, ty, tv, ts;
            st.set_points(distance, time);
            tx.set_points(time, x);
            ty.set_points(time, y);
            tv.set_points(time, speed);
            ts.set_points(time, distance);

            int num_total_point = std::ceil(time.back() / resample_time);
            double time_offset = 0.0;

            for (uint16_t i = 0; i < num_total_point; i++) {
                TrajectoryPoint tp;
                tp.time = resample_time * i;
                tp.x = tx(resample_time * i + time_offset);
                tp.y = ty(resample_time * i + time_offset);
                tp.z = 0.0;
                tp.speed = tv(resample_time * i + time_offset);
                tp.acceleration = 0.0;
                tp.distance = ts(resample_time * i + time_offset) - ts(time_offset);
                double dx = tx(resample_time * (i+1) + time_offset) - tp.x;
                double dy = ty(resample_time * (i+1) + time_offset) - tp.y;
                tp.yaw = atan2(dy, dx);       
                tp.curvature = 0.0;

                resampled_trajectory.point.push_back(tp);
            }
        }

        return resampled_trajectory;
    }

    inline Trajectory RestrictLateralAccel(const Trajectory& trajectory,
                                           const float& max_lateral_accel,
                                           const float& max_longitudinal_accel,
                                           const float& dt) {
        Trajectory restricted_trajectory = trajectory;  

        float max_restricted_speed = std::numeric_limits<float>::max();

        for(uint32_t i = 1; i < restricted_trajectory.point.size(); i++) {
            // Apply speed limit
            float restricted_speed = sqrt(fabs(max_lateral_accel / restricted_trajectory.point[i].curvature));

            if (trajectory.decision_behavior == OVERTAKE_STATIC_VEHICLE) {
                if (restricted_speed < restricted_trajectory.point[i].speed) {
                    if ((restricted_speed - restricted_trajectory.point[i-1].speed)/dt <= -max_longitudinal_accel) {
                        //return trajectory;
                        max_restricted_speed = restricted_trajectory.point[i-1].speed - max_longitudinal_accel * dt;
                    }
                    else {
                        max_restricted_speed = std::min(max_restricted_speed, restricted_speed);
                    }
                }
                restricted_trajectory.point[i].speed = std::max(0.0f, std::min(restricted_trajectory.point[i].speed, max_restricted_speed));            
            }
            else {
                if ((restricted_speed - restricted_trajectory.point[i-1].speed)/dt <= -max_longitudinal_accel) {
                    restricted_speed = restricted_trajectory.point[i-1].speed - max_longitudinal_accel * dt;
                }
                restricted_trajectory.point[i].speed = std::min(restricted_trajectory.point[i].speed, restricted_speed);
            }
        }

        // Forward smoothing
        for (uint32_t i = 0; i < restricted_trajectory.point.size(); i++) {
            // Calculate acceleration
            double v = restricted_trajectory.point.at(i).speed;
            double v_old = i == 0 ? v : restricted_trajectory.point.at(i-1).speed;
            double a = (pow(v, 2) - pow(v_old, 2)) / (2);

            // Smoothen the speed not to exceed the maximum acceleration
            if (a > max_longitudinal_accel) {
                restricted_trajectory.point.at(i).speed = sqrt(pow(v_old, 2) + 2 * max_longitudinal_accel);
            }
        }

        // Backward smoothing
        for (uint32_t i = restricted_trajectory.point.size() - 1; i > 0; i--) {
            // Calculate acceleration
            double v = restricted_trajectory.point.at(i).speed;
            double v_old = restricted_trajectory.point.at(i-1).speed;
            double a = (pow(v, 2) - pow(v_old, 2)) / (2);

            // Backward smoothing according to minimum acceleration
            if (a < -max_longitudinal_accel) {  
                restricted_trajectory.point.at(i-1).speed = sqrt(pow(v, 2) - 2 * (-max_longitudinal_accel));
            }
        }

        // Recalculate time
        for(uint32_t i = 1; i < restricted_trajectory.point.size(); i++){
            TrajectoryPoint& current_point = restricted_trajectory.point[i];
            const TrajectoryPoint& previous_point = restricted_trajectory.point[i-1];

            // time = distance / speed
            double distance = current_point.distance - previous_point.distance;
            double speed = (current_point.speed + previous_point.speed) / 2.0;

            if (speed >= 0.1) {
                current_point.time = previous_point.time + (distance / speed);
            }
            else {
                current_point.time = previous_point.time + 0.1;
            }
        }

        return restricted_trajectory;
    }
} // namespace autoku_functions

#endif  // __FUNCTION_TRAJECTORIES_HPP__