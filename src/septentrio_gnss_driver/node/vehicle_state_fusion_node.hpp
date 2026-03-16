// *****************************************************************************
//
// © Copyright 2024, Septentrio NV/SA.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//    1. Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//    2. Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//    3. Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <septentrio_gnss_driver/msg/ins_nav_geod.hpp>
#include <septentrio_gnss_driver/msg/ext_sensor_meas.hpp>
#include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <array>

namespace vehicle_state_fusion {

/**
 * @brief Node that fuses INS and IMU data into a vehicle state message
 * 
 * Subscribes to:
 *  - /septentrio/insnavgeod (INSNavGeod): Fused GNSS/INS position, velocity, attitude
 *  - /septentrio/extsensormeas (ExtSensorMeas): Raw IMU measurements
 * 
 * Publishes:
 *  - /SensFusion_vehicle_states (nav_msgs::Odometry): Vehicle state [x, y, yaw, vx, vy, yaw_rate, ax]
 */
class VehicleStateFusionNode : public rclcpp::Node
{
public:
    VehicleStateFusionNode();

private:
    void insNavGeodCallback(const septentrio_gnss_driver::msg::INSNavGeod::SharedPtr msg);
    void extSensorMeasCallback(const septentrio_gnss_driver::msg::ExtSensorMeas::SharedPtr msg);
    void localizationCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pvtGeodeticCallback(const septentrio_gnss_driver::msg::PVTGeodetic::SharedPtr msg);
    
    void publishVehicleState();
    void publishDiagnostics();
    
    // Load origin from JSON file based on selected scenario
    bool loadOriginFromJSON(const std::string& json_path, const std::string& scenario);
    
    // Convert lat/lon to local ENU coordinates (accurate version with ellipsoid)
    std::array<double, 3> lla2enu(double lat, double lon, double alt);
    
    // Convert heading/pitch/roll to quaternion
    geometry_msgs::msg::Quaternion eulerToQuaternion(double heading, double pitch, double roll);
    
    // Transform velocities from ENU to vehicle body frame
    void transformVelocitiesToBodyFrame(double ve, double vn, double yaw, double& vx, double& vy);
    
    // Rotation matrix structure
    struct RotationMatrix {
        double data[3][3];
    };
    
    // Calculate rotation matrix from roll, pitch, yaw
    RotationMatrix calculateRotationMatrix(double roll, double pitch, double yaw);
    
    // Rotate accelerations from sensor frame to vehicle body frame
    std::array<double, 3> rotateAccelerations(double roll, double pitch, double yaw,
                                               double ax_sensor, double ay_sensor, double az_sensor);

    // Subscribers
    rclcpp::Subscription<septentrio_gnss_driver::msg::INSNavGeod>::SharedPtr ins_sub_;
    rclcpp::Subscription<septentrio_gnss_driver::msg::ExtSensorMeas>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
    rclcpp::Subscription<septentrio_gnss_driver::msg::PVTGeodetic>::SharedPtr pvt_sub_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_state_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    
    // Timer for periodic diagnostics
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    
    // Latest data
    septentrio_gnss_driver::msg::INSNavGeod::SharedPtr latest_ins_;
    septentrio_gnss_driver::msg::ExtSensorMeas::SharedPtr latest_imu_;
    nav_msgs::msg::Odometry::SharedPtr latest_localization_;
    septentrio_gnss_driver::msg::PVTGeodetic::SharedPtr latest_pvt_;
    
    // Reference point for local ENU frame (loaded from JSON or set on first message)
    bool origin_set_;
    double origin_lat_;   // radians
    double origin_lon_;   // radians
    double origin_alt_;   // meters
    
    // Frame IDs
    std::string frame_id_;
    std::string child_frame_id_;
    
    // Parameters
    bool use_ins_velocities_;  // If true, use INS velocities; if false, compute from position
    std::string scenarios_json_path_;
    std::string selected_scenario_;
    bool manual_origin_enabled_;
    double pitch_offset_;  // INS pitch mounting offset [rad] - measured on flat ground
    double roll_offset_;   // INS roll mounting offset [rad] - measured on flat ground
    
    // Acceleration computation from velocity derivative
    bool first_velocity_;
    double prev_vx_body_;
    double prev_vy_body_;
    rclcpp::Time prev_velocity_time_;
    
    // DEPRECATED: Old IMU-based filtering (kept for reference but not used)
    std::vector<double> ax_window_;
    std::vector<double> ay_window_;
    int window_size_accel_;
    int filter_counter_;
    double moving_average_filter(const std::vector<double>& data);
};

} // namespace vehicle_state_fusion
