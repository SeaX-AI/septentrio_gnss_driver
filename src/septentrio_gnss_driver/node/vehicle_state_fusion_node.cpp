// *****************************************************************************
//
// © Copyright 2024, Septentrio NV/SA.
// All rights reserved.
//
// *****************************************************************************

#include "vehicle_state_fusion_node.hpp"

namespace vehicle_state_fusion {

VehicleStateFusionNode::VehicleStateFusionNode() 
    : Node("vehicle_state_fusion_node"),
      origin_set_(false),
      origin_lat_(0.0),
      origin_lon_(0.0),
      origin_alt_(0.0),
      first_velocity_(true),
      prev_vx_body_(0.0),
      prev_vy_body_(0.0),
      window_size_accel_(20),
      filter_counter_(0)
{
    // Initialize filter windows
    ax_window_.resize(window_size_accel_, 0.0);
    ay_window_.resize(window_size_accel_, 0.0);
    // Declare parameters
    this->declare_parameter<std::string>("frame_id", "odom");
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->declare_parameter<bool>("use_ins_velocities", true);
    this->declare_parameter<std::string>("scenarios_json_path", "");
    this->declare_parameter<std::string>("selected_scenario", "Torremocha");
    this->declare_parameter<bool>("manual_origin.enabled", false);
    this->declare_parameter<double>("manual_origin.latitude", 0.0);
    this->declare_parameter<double>("manual_origin.longitude", 0.0);
    this->declare_parameter<double>("manual_origin.altitude", 0.0);
    this->declare_parameter<double>("imu_calibration.pitch_offset", 0.0);
    this->declare_parameter<double>("imu_calibration.roll_offset", 0.0);
    
    // Get parameters
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    use_ins_velocities_ = this->get_parameter("use_ins_velocities").as_bool();
    scenarios_json_path_ = this->get_parameter("scenarios_json_path").as_string();
    selected_scenario_ = this->get_parameter("selected_scenario").as_string();
    manual_origin_enabled_ = this->get_parameter("manual_origin.enabled").as_bool();
    pitch_offset_ = this->get_parameter("imu_calibration.pitch_offset").as_double();
    roll_offset_ = this->get_parameter("imu_calibration.roll_offset").as_double();
    
    // Log calibration offsets if set
    if (pitch_offset_ != 0.0 || roll_offset_ != 0.0) {
        RCLCPP_INFO(this->get_logger(), 
                   "IMU calibration offsets: pitch=%.4f rad (%.2f°), roll=%.4f rad (%.2f°)",
                   pitch_offset_, pitch_offset_ * 180.0 / M_PI,
                   roll_offset_, roll_offset_ * 180.0 / M_PI);
    }
    
    // Try to load origin from JSON file
    if (!scenarios_json_path_.empty() && !manual_origin_enabled_) {
        if (loadOriginFromJSON(scenarios_json_path_, selected_scenario_)) {
            RCLCPP_INFO(this->get_logger(), 
                       "Origin loaded from JSON for scenario '%s': lat=%.6f°, lon=%.6f°, alt=%.2f m",
                       selected_scenario_.c_str(),
                       origin_lat_ * 180.0 / M_PI,
                       origin_lon_ * 180.0 / M_PI,
                       origin_alt_);
            origin_set_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), 
                       "Failed to load origin from JSON. Will use first valid GNSS position.");
        }
    }
    
    // Use manual origin if enabled
    if (manual_origin_enabled_) {
        origin_lat_ = this->get_parameter("manual_origin.latitude").as_double() * M_PI / 180.0;
        origin_lon_ = this->get_parameter("manual_origin.longitude").as_double() * M_PI / 180.0;
        origin_alt_ = this->get_parameter("manual_origin.altitude").as_double();
        origin_set_ = true;
        RCLCPP_INFO(this->get_logger(), 
                   "Manual origin set: lat=%.6f°, lon=%.6f°, alt=%.2f m",
                   origin_lat_ * 180.0 / M_PI,
                   origin_lon_ * 180.0 / M_PI,
                   origin_alt_);
    }
    
    // Create subscribers
    ins_sub_ = this->create_subscription<septentrio_gnss_driver::msg::INSNavGeod>(
        "/septentrio/insnavgeod", 10,
        std::bind(&VehicleStateFusionNode::insNavGeodCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<septentrio_gnss_driver::msg::ExtSensorMeas>(
        "/septentrio/extsensormeas", 10,
        std::bind(&VehicleStateFusionNode::extSensorMeasCallback, this, std::placeholders::_1));
    
    localization_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/septentrio/localization", 10,
        std::bind(&VehicleStateFusionNode::localizationCallback, this, std::placeholders::_1));
    
    pvt_sub_ = this->create_subscription<septentrio_gnss_driver::msg::PVTGeodetic>(
        "/septentrio/pvtgeodetic", 10,
        std::bind(&VehicleStateFusionNode::pvtGeodeticCallback, this, std::placeholders::_1));
    
    // Create publisherss
    vehicle_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/SensFusion_vehicle_states_new", 10);
    
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    
    // Create timer for periodic diagnostics (1 Hz)
    diagnostics_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&VehicleStateFusionNode::publishDiagnostics, this));
    
    RCLCPP_INFO(this->get_logger(), "Vehicle State Fusion Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Child Frame ID: %s", child_frame_id_.c_str());
}

void VehicleStateFusionNode::insNavGeodCallback(
    const septentrio_gnss_driver::msg::INSNavGeod::SharedPtr msg)
{
    // CRITICAL: Validate message before processing
    // Note: gnss_mode=0 means INS is in pure inertial mode, but data can still be valid
    // The INS system can blend GNSS and IMU, and operate temporarily without GNSS
    
    // Error code 23: IMU/INS sync issues - warn but continue (position data still valid)
    // Other errors: reject message
    if (msg->error != 0 && msg->error != 23) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "INS: Error code %d, skipping message", msg->error);
        return;
    }
    
    if (msg->error == 23) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "INS: Warning - Error code 23 (IMU/INS sync issue, using data anyway)");
    }
    
    // Check for NaN values in critical fields
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || 
        std::isnan(msg->height) || std::isnan(msg->heading)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "INS: NaN values detected, skipping message");
        return;
    }
    
    // Check for NaN in velocities
    if (std::isnan(msg->ve) || std::isnan(msg->vn) || std::isnan(msg->vu)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "INS: NaN velocities detected, skipping message");
        return;
    }
    
    // Log when INS is in pure inertial mode (for debugging only)
    if (msg->gnss_mode == 0) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "INS: Operating in pure inertial mode (gnss_mode=0)");
    }
    
    // Message is valid, update state
    latest_ins_ = msg;
    
    // Set origin on first valid message if NOT already set from JSON/manual config
    // Accept any valid message (data quality checks above ensure validity)
    if (!origin_set_) {
        origin_lat_ = msg->latitude;
        origin_lon_ = msg->longitude;
        origin_alt_ = msg->height;
        origin_set_ = true;
        RCLCPP_INFO(this->get_logger(), 
                    "ENU origin set from first valid INS message: lat=%.6f°, lon=%.6f°, alt=%.2f m (gnss_mode=%d)", 
                    origin_lat_ * 180.0 / M_PI, 
                    origin_lon_ * 180.0 / M_PI,
                    origin_alt_, msg->gnss_mode);
    }
    
    // Publish vehicle state if we have origin set and valid data
    publishVehicleState();
}

void VehicleStateFusionNode::extSensorMeasCallback(
    const septentrio_gnss_driver::msg::ExtSensorMeas::SharedPtr msg)
{
    latest_imu_ = msg;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "IMU callback: ax=%.3f, ay=%.3f, timestamp=%d.%d",
                         msg->acceleration_x, msg->acceleration_y,
                         msg->header.stamp.sec, msg->header.stamp.nanosec);
}

void VehicleStateFusionNode::localizationCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_localization_ = msg;
}

void VehicleStateFusionNode::pvtGeodeticCallback(
    const septentrio_gnss_driver::msg::PVTGeodetic::SharedPtr msg)
{
    latest_pvt_ = msg;
}

void VehicleStateFusionNode::publishVehicleState()
{
    if (!latest_ins_ || !origin_set_) {
        return;
    }
    
    nav_msgs::msg::Odometry odom;
    
    // Header
    odom.header.stamp = latest_ins_->header.stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    
    // Position: Convert geodetic to local ENU coordinates (accurate version)
    std::array<double, 3> enu = lla2enu(latest_ins_->latitude, 
                                        latest_ins_->longitude, 
                                        latest_ins_->height);
    
    odom.pose.pose.position.x = enu[1];  // North
    odom.pose.pose.position.y = enu[0];  // East
    odom.pose.pose.position.z = enu[2];  // Up
    
    // Orientation: Use quaternion from /septentrio/localization if available, else convert from heading/pitch/roll
    // NOTE: With use_ros_axis_orientation=false, heading is in NAUTICAL convention (North=0°, CW)
    double heading_rad;  // Heading in nautical convention (North=0°, clockwise)
    double roll_rad;     // Roll angle in radians
    double pitch_rad;    // Pitch angle in radians
    
    if (latest_localization_) {
        // Use orientation directly from localization topic
        odom.pose.pose.orientation = latest_localization_->pose.pose.orientation;
        
        // Extract yaw (heading) from quaternion - in nautical convention with use_ros_axis_orientation=false
        tf2::Quaternion q(
            latest_localization_->pose.pose.orientation.x,
            latest_localization_->pose.pose.orientation.y,
            latest_localization_->pose.pose.orientation.z,
            latest_localization_->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        heading_rad = yaw;   // Nautical convention (with use_ros_axis_orientation=false)
        roll_rad = roll;     // Roll angle
        pitch_rad = pitch;   // Pitch angle
    } else {
        // Fallback: Convert heading/pitch/roll to quaternion
        // Septentrio heading: North=0°, clockwise (nautical convention)
        // With use_ros_axis_orientation=false, keep it as is (no conversion)
        heading_rad = latest_ins_->heading * M_PI / 180.0;  // Nautical convention
        pitch_rad = latest_ins_->pitch * M_PI / 180.0;
        roll_rad = latest_ins_->roll * M_PI / 180.0;
        odom.pose.pose.orientation = eulerToQuaternion(heading_rad, pitch_rad, roll_rad);
    }
    
    // Remove mounting offsets to get true pitch/roll relative to horizon
    double pitch_real = pitch_rad - pitch_offset_;  // Remove installation bias
    double roll_real = roll_rad - roll_offset_;     // Remove installation bias
    
    // Position covariance (if available)
    if (latest_ins_->sb_list & 0x01) {  // Position StdDev available
        odom.pose.covariance[0] = latest_ins_->longitude_std_dev * latest_ins_->longitude_std_dev;
        odom.pose.covariance[7] = latest_ins_->latitude_std_dev * latest_ins_->latitude_std_dev;
        odom.pose.covariance[14] = latest_ins_->height_std_dev * latest_ins_->height_std_dev;
    }
    
    // Velocity: Transform from NED to vehicle body frame
    // ve = velocity East, vn = velocity North, vu = velocity Up
    // heading is in nautical convention (North=0°, CW) with use_ros_axis_orientation=false
    double vx_body, vy_body;
    transformVelocitiesToBodyFrame(latest_ins_->ve, latest_ins_->vn, heading_rad, vx_body, vy_body);
    
    odom.twist.twist.linear.x = vx_body;  // Longitudinal velocity
    odom.twist.twist.linear.y = vy_body;  // Lateral velocity
    odom.twist.twist.linear.z = latest_ins_->vu;  // Up velocity (unchanged)
    
    // PRIMARY: Accelerations from velocity differentiation (main source for control)
    double ax_body = 0.0;
    double ay_body = 0.0;
    double dt = 0.0;
    
    if (!first_velocity_) {
        rclcpp::Time current_time = latest_ins_->header.stamp;
        dt = (current_time - prev_velocity_time_).seconds();
        
        if (dt > 0.001 && dt < 1.0) {  // Sanity check
            // Numerical differentiation
            double ax_raw = (vx_body - prev_vx_body_) / dt;
            double ay_raw = (vy_body - prev_vy_body_) / dt;
            
            // Exponential moving average filter
            static double filtered_ax_vel = 0.0;
            static double filtered_ay_vel = 0.0;
            static bool filter_vel_init = false;
            
            double alpha = 0.15;  // Tuning parameter
            
            if (!filter_vel_init) {
                filtered_ax_vel = ax_raw;
                filtered_ay_vel = ay_raw;
                filter_vel_init = true;
            } else {
                // Outlier detection: reject unrealistic accelerations
                const double max_accel = 5.0;  // m/s²
                if (std::abs(ax_raw) < max_accel && std::abs(ay_raw) < max_accel) {
                    filtered_ax_vel = alpha * ax_raw + (1.0 - alpha) * filtered_ax_vel;
                    filtered_ay_vel = alpha * ay_raw + (1.0 - alpha) * filtered_ay_vel;
                }
            }
            
            ax_body = filtered_ax_vel;
            ay_body = filtered_ay_vel;
        }
    }
    
    // Update previous values for next iteration
    prev_vx_body_ = vx_body;
    prev_vy_body_ = vy_body;
    prev_velocity_time_ = latest_ins_->header.stamp;
    first_velocity_ = false;
    
    // IMU accelerations: Rotate from sensor frame to body frame to compensate roll/pitch mounting errors
    double ax_imu_raw = 0.0;
    double ay_imu_raw = 0.0;
    double az_imu_raw = 0.0;
    double ax_imu_rotated = 0.0;
    double ay_imu_rotated = 0.0;
    
    if (latest_imu_ && latest_imu_->n > 0) {
        // Get raw IMU measurements with correct axis mapping
        // IMU is mounted rotated 90° - experimentally verified axis mapping:
        // Vehicle Forward (ax) = -IMU_Y
        // Vehicle Lateral (ay) = IMU_X  
        // Vehicle Up (az) = IMU_Z (assuming FRD, negate for FLU)
        ax_imu_raw = -latest_imu_->acceleration_y;  // Forward = -IMU_Y
        ay_imu_raw =  latest_imu_->acceleration_x;  // Lateral = IMU_X
        az_imu_raw = -latest_imu_->acceleration_z;  // Up (FRD→FLU)
        
        // Gravity compensation: IMU measures specific force (acceleration + gravity projected)
        // When vehicle has pitch/roll, gravity projects into body frame axes
        // Using pitch_real and roll_real (already computed with offsets removed)
        // Subtract gravitational components to get true inertial acceleration:
        //   ax_inertial = ax_measured - g*sin(pitch_real)
        //   ay_inertial = ay_measured + g*sin(roll_real)  (sign depends on FRD/FLU convention)
        const double g = 9.80665;  // Gravitational acceleration [m/s²]
        double ax_imu_compensated = ax_imu_raw - g * std::sin(pitch_real);
        double ay_imu_compensated = ay_imu_raw + g * std::sin(roll_real);
        
        // Exponential moving average filter to reduce IMU noise
        static double filtered_ax_imu = 0.0;
        static double filtered_ay_imu = 0.0;
        static bool filter_imu_init = false;
        
        double alpha_imu = 0.1;  // Tuning parameter (0.1=more filtering, 0.5=less filtering)
        
        if (!filter_imu_init) {
            filtered_ax_imu = ax_imu_compensated;
            filtered_ay_imu = ay_imu_compensated;
            filter_imu_init = true;
        } else {
            // Outlier detection: reject unrealistic accelerations
            const double max_accel = 10.0;  // m/s²
            if (std::abs(ax_imu_compensated) < max_accel && std::abs(ay_imu_compensated) < max_accel) {
                filtered_ax_imu = alpha_imu * ax_imu_compensated + (1.0 - alpha_imu) * filtered_ax_imu;
                filtered_ay_imu = alpha_imu * ay_imu_compensated + (1.0 - alpha_imu) * filtered_ay_imu;
            }
        }
        
        ax_imu_rotated = filtered_ax_imu;
        ay_imu_rotated = filtered_ay_imu;
        
        // Log time difference for debugging (only if significant)
        double imu_age = (latest_ins_->header.stamp.sec - latest_imu_->header.stamp.sec) + 
                         (latest_ins_->header.stamp.nanosec - latest_imu_->header.stamp.nanosec) * 1e-9;
        if (std::abs(imu_age) > 5.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "IMU/INS time difference: age=%.3f s (using IMU data anyway)", imu_age);
        }
    }
    
    // Angular velocity: yaw_rate from IMU (convert from deg/s to rad/s)
    if (latest_imu_ && latest_imu_->n > 0) {
        double yaw_rate_rad = latest_imu_->angular_rate_z * M_PI / 180.0;
        odom.twist.twist.angular.z = -yaw_rate_rad;
    }
    
    // PRIMARY: Store velocity-derived accelerations in angular.x and angular.y (for control)
    odom.twist.twist.angular.x = ax_body;  // Longitudinal acceleration (from velocity)
    odom.twist.twist.angular.y = ay_body;  // Lateral acceleration (from velocity)
    
    // ADDITIONAL: Store IMU accelerations in covariance for comparison
    odom.pose.covariance[1] = ax_imu_rotated;  // IMU rotated longitudinal (gravity compensated)
    odom.pose.covariance[2] = ay_imu_rotated;  // IMU rotated lateral (gravity compensated)
    odom.pose.covariance[3] = ax_imu_raw;      // IMU raw longitudinal (for debugging)
    odom.pose.covariance[4] = ay_imu_raw;      // IMU raw lateral (for debugging)
    
    // Velocity covariance (if available)
    if (latest_ins_->sb_list & 0x20) {  // Velocity StdDev available
        odom.twist.covariance[0] = latest_ins_->ve_std_dev * latest_ins_->ve_std_dev;
        odom.twist.covariance[7] = latest_ins_->vn_std_dev * latest_ins_->vn_std_dev;
        odom.twist.covariance[14] = latest_ins_->vu_std_dev * latest_ins_->vu_std_dev;
    }
    
    // [DEBUG] Store heading, roll, and pitch in covariance for debugging (non-standard use)
    odom.pose.covariance[8] = heading_rad;  // Heading in nautical convention (North=0°, CW)
    odom.pose.covariance[9] = roll_real;    // Roll angle (calibrated, offset removed) in radians
    odom.pose.covariance[10] = pitch_real;  // Pitch angle (calibrated, offset removed) in radians
    
    // Publish
    vehicle_state_pub_->publish(odom);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Vehicle state: x=%.2f, y=%.2f, yaw=%.2f rad, vx=%.2f, vy=%.2f m/s", 
                enu[0], enu[1], heading_rad, vx_body, vy_body);
}

bool VehicleStateFusionNode::loadOriginFromJSON(const std::string& json_path, const std::string& scenario)
{
    try {
        std::ifstream file(json_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", json_path.c_str());
            return false;
        }
        
        nlohmann::json j;
        file >> j;
        file.close();
        
        if (!j.contains(scenario)) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Scenario '%s' not found in JSON file", 
                        scenario.c_str());
            return false;
        }
        
        if (!j[scenario].contains("LLA0")) {
            RCLCPP_ERROR(this->get_logger(), 
                        "LLA0 key not found for scenario '%s'", 
                        scenario.c_str());
            return false;
        }
        
        auto lla0 = j[scenario]["LLA0"];
        if (lla0.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), 
                        "LLA0 must contain exactly 3 values [lat, lon, alt]");
            return false;
        }
        
        // Convert from degrees to radians
        origin_lat_ = lla0[0].get<double>() * M_PI / 180.0;
        origin_lon_ = lla0[1].get<double>() * M_PI / 180.0;
        origin_alt_ = lla0[2].get<double>();
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), 
                    "Exception while loading JSON: %s", 
                    e.what());
        return false;
    }
}

std::array<double, 3> VehicleStateFusionNode::lla2enu(double lat, double lon, double alt)
{
    // Convert Lat, Lon, Altitude to East-North-Up (ENU) coordinate system
    // Input: lat, lon (radians), alt (meters)
    // Output: [east, north, up] in meters
    // This is the accurate version considering ellipsoid curvature
    
    // WGS84 ellipsoid parameters
    const double a = 6378137.0;        // Semi-major axis
    const double b = 6356752.3142;     // Semi-minor axis
    const double e2 = 1.0 - std::pow(b / a, 2);  // First eccentricity squared
    
    // Location deltas
    double dlat = lat - origin_lat_;
    double dlon = lon - origin_lon_;
    double dalt = alt - origin_alt_;
    
    // Useful definitions at origin
    double sin_lat0 = std::sin(origin_lat_);
    double cos_lat0 = std::cos(origin_lat_);
    double tmp1 = std::sqrt(1.0 - e2 * sin_lat0 * sin_lat0);
    
    std::array<double, 3> enu;
    
    // Accurate transformations accounting for ellipsoid curvature
    // East component
    enu[0] = (a / tmp1 + origin_alt_) * cos_lat0 * dlon 
           - (a * (1.0 - e2) / std::pow(tmp1, 3) + origin_alt_) * sin_lat0 * dlat * dlon 
           + cos_lat0 * dlon * dalt;
    
    // North component
    enu[1] = (a * (1.0 - e2) / std::pow(tmp1, 3) + origin_alt_) * dlat 
           + 1.5 * cos_lat0 * sin_lat0 * a * e2 * dlat * dlat 
           + sin_lat0 * sin_lat0 * dalt * dlat 
           + 0.5 * sin_lat0 * cos_lat0 * (a / tmp1 + origin_alt_) * dlon * dlon;
    
    // Up component
    enu[2] = dalt 
           - 0.5 * (a - 1.5 * a * e2 * cos_lat0 * cos_lat0 + 0.5 * a * e2 + origin_alt_) * dlat * dlat 
           - 0.5 * cos_lat0 * cos_lat0 * (a / tmp1 - origin_alt_) * dlon * dlon;
    
    return enu;
}

void VehicleStateFusionNode::transformVelocitiesToBodyFrame(double ve, double vn, double yaw, 
                                                             double& vx, double& vy)
{
    // Transform velocities from NED (North-East-Down) frame to vehicle body frame
    // yaw is in NAUTICAL convention: North=0°, clockwise (Septentrio native)
    // With use_ros_axis_orientation=false, heading and velocities are in Septentrio convention
    
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    
    // Rotation from NED to body frame (nautical heading):
    // vx (forward) =  cos(yaw)*vn + sin(yaw)*ve
    // vy (left)    = -sin(yaw)*vn + cos(yaw)*ve  
    vx =  cos_yaw * vn + sin_yaw * ve;  // Longitudinal velocity (forward)
    vy = -sin_yaw * vn + cos_yaw * ve;  // Lateral velocity (left)
}

geometry_msgs::msg::Quaternion VehicleStateFusionNode::eulerToQuaternion(
    double heading, double pitch, double roll)
{
    // Convert Euler angles to quaternion
    // Heading = yaw (rotation around Z)
    // Pitch = rotation around Y
    // Roll = rotation around X
    
    tf2::Quaternion q;
    q.setRPY(roll, pitch, heading);
    
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();
    
    return quat_msg;
}

double VehicleStateFusionNode::moving_average_filter(const std::vector<double>& data)
{
    double sum = 0.0;
    for (const auto& val : data) {
        sum += val;
    }
    return sum / data.size();
}

void VehicleStateFusionNode::publishDiagnostics()
{
    if (!latest_pvt_) {
        return;
    }
    
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = this->now();
    
    diagnostic_msgs::msg::DiagnosticStatus diag_status;
    diag_status.name = "Vehicle State Fusion: GNSS/RTK Status";
    diag_status.hardware_id = "Septentrio GNSS";
    
    // Determine overall status level
    uint8_t mode = latest_pvt_->mode;
    uint8_t error = latest_pvt_->error;
    uint16_t h_acc = latest_pvt_->h_accuracy;  // cm
    uint16_t v_acc = latest_pvt_->v_accuracy;  // cm
    uint8_t nr_sv = latest_pvt_->nr_sv;
    uint16_t corr_age = latest_pvt_->mean_corr_age;  // 0.01s
    
    // Status evaluation logic
    if (error != 0) {
        diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_status.message = "GNSS Error: code " + std::to_string(error);
    } else if (mode == 4) {  // RTK Fixed
        if (h_acc <= 5 && v_acc <= 10 && nr_sv >= 12 && corr_age < 300) {
            diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diag_status.message = "RTK Fixed - Excellent";
        } else if (h_acc <= 10 && nr_sv >= 8) {
            diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            diag_status.message = "RTK Fixed - Good";
        } else {
            diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag_status.message = "RTK Fixed - Degraded";
        }
    } else if (mode == 5) {  // RTK Float
        if (h_acc <= 20 && nr_sv >= 8) {
            diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag_status.message = "RTK Float";
        } else {
            diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag_status.message = "RTK Float - Degraded";
        }
    } else if (mode == 2) {  // DGPS
        diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diag_status.message = "DGPS (No RTK)";
    } else if (mode == 1) {  // Stand Alone
        diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diag_status.message = "Stand Alone (No Corrections)";
    } else {  // No PVT
        diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_status.message = "No GNSS Fix";
    }
    
    // Add key-value pairs with detailed information
    diagnostic_msgs::msg::KeyValue kv;
    
    // Mode string
    kv.key = "GNSS Mode";
    switch (mode) {
        case 0: kv.value = "No PVT"; break;
        case 1: kv.value = "Stand Alone"; break;
        case 2: kv.value = "DGPS"; break;
        case 3: kv.value = "Fixed"; break;
        case 4: kv.value = "RTK Fixed"; break;
        case 5: kv.value = "RTK Float"; break;
        case 6: kv.value = "SBAS"; break;
        case 7: kv.value = "Moving Base RTK Fixed"; break;
        case 8: kv.value = "Moving Base RTK Float"; break;
        case 9: kv.value = "PPP"; break;
        default: kv.value = "Unknown"; break;
    }
    diag_status.values.push_back(kv);
    
    // Horizontal accuracy
    kv.key = "Horizontal Accuracy";
    kv.value = std::to_string(h_acc) + " cm";
    diag_status.values.push_back(kv);
    
    // Vertical accuracy
    kv.key = "Vertical Accuracy";
    kv.value = std::to_string(v_acc) + " cm";
    diag_status.values.push_back(kv);
    
    // Number of satellites
    kv.key = "Satellites Used";
    kv.value = std::to_string(nr_sv);
    diag_status.values.push_back(kv);
    
    // Correction age
    kv.key = "Correction Age";
    kv.value = std::to_string(corr_age * 0.01) + " s";
    diag_status.values.push_back(kv);
    
    // Number of base stations
    kv.key = "Base Stations";
    kv.value = std::to_string(latest_pvt_->nr_bases);
    diag_status.values.push_back(kv);
    
    // Error code
    kv.key = "Error Code";
    kv.value = std::to_string(error);
    diag_status.values.push_back(kv);
    
    // Latency
    kv.key = "Latency";
    kv.value = std::to_string(latest_pvt_->latency * 0.0001) + " s";
    diag_status.values.push_back(kv);
    
    diag_array.status.push_back(diag_status);
    diagnostics_pub_->publish(diag_array);
}

VehicleStateFusionNode::RotationMatrix VehicleStateFusionNode::calculateRotationMatrix(
    double roll, double pitch, double yaw)
{
    RotationMatrix R;
    double c_roll = std::cos(roll);
    double s_roll = std::sin(roll);
    double c_pitch = std::cos(pitch);
    double s_pitch = std::sin(pitch);
    double c_yaw = std::cos(yaw);
    double s_yaw = std::sin(yaw);

    R.data[0][0] = c_yaw * c_pitch;
    R.data[0][1] = -s_yaw * c_roll + c_yaw * s_pitch * s_roll;
    R.data[0][2] = s_yaw * s_roll + c_yaw * s_pitch * c_roll;
    R.data[1][0] = s_yaw * c_pitch;
    R.data[1][1] = c_yaw * c_roll + s_yaw * s_pitch * s_roll;
    R.data[1][2] = -c_yaw * s_roll + s_yaw * s_pitch * c_roll;
    R.data[2][0] = -s_pitch;
    R.data[2][1] = c_pitch * s_roll;
    R.data[2][2] = c_pitch * c_roll;
    
    return R;
}

std::array<double, 3> VehicleStateFusionNode::rotateAccelerations(
    double roll, double pitch, double yaw,
    double ax_sensor, double ay_sensor, double az_sensor)
{
    // Calculate rotation matrix to transform from sensor frame to body frame
    // This properly accounts for roll/pitch mounting errors
    RotationMatrix R = calculateRotationMatrix(roll, pitch, yaw);

    double ax_body = R.data[0][0] * ax_sensor + R.data[0][1] * ay_sensor + R.data[0][2] * az_sensor;
    double ay_body = R.data[1][0] * ax_sensor + R.data[1][1] * ay_sensor + R.data[1][2] * az_sensor;
    double az_body = R.data[2][0] * ax_sensor + R.data[2][1] * ay_sensor + R.data[2][2] * az_sensor;

    return {ax_body, ay_body, az_body};
}

} // namespace vehicle_state_fusion

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vehicle_state_fusion::VehicleStateFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
