/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  ROS2ISOBUS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  ROS2ISOBUS is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with ROS2ISOBUS.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "NMEA2000ClientNode.hpp"

namespace ros2_isobus
{

NMEA2000ClientROS2::NMEA2000ClientROS2(const rclcpp::NodeOptions &options)
    : NMEA2000Client("map"), rclcpp::Node("ISOBUS_nmea2000_node", options)
{
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(ros2_isobus::kNmea2000GnssPositionTopic, 10);
    rapid_position_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(ros2_isobus::kNmea2000RapidPositionTopic, 10);
    pseudo_noise_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(ros2_isobus::kNmea2000PseudoNoiseTopic, 10);
    cog_sog_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(ros2_isobus::kNmea2000CogSogTopic, 10);
    attitude_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(ros2_isobus::kNmea2000AttitudeTopic, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(ros2_isobus::kNmea2000ImuTopic, 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(ros2_isobus::kNmea2000VelocityTopic, 10);

    bus_sub_ = create_subscription<ros2_isobus::msg::IsobusFrame>(
        ros2_isobus::kBusRxTopic, 100,
        std::bind(&NMEA2000ClientROS2::HandleMsg, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NMEA2000 node ready");
}

void NMEA2000ClientROS2::publishNavSatFix()
{
    sensor_msgs::msg::NavSatFix msg;
    // PositionDate is days since 1970-01-01, PositionTime is seconds since midnight
    const auto sec_from_epoch = static_cast<int64_t>(PositionDate) * 24 * 60 * 60 + static_cast<int64_t>(PositionTime);
    msg.header.stamp = nowStamp();
    msg.header.frame_id = frame_id_;
    msg.latitude = latitude;
    msg.longitude = longitude;
    msg.altitude = altitude;
    msg.status.status = GPSFix >= 4 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    msg.status.service = gnss_service(GPSType);

    const bool has_std = (STD_lat > 0 && STD_lon > 0 && STD_alt > 0);
    const double variance_x = has_std ? std::pow(STD_lat, 2) : (HDOP > 0 ? std::pow(HDOP, 2) : -1.0);
    const double variance_y = has_std ? std::pow(STD_lon, 2) : (HDOP > 0 ? std::pow(HDOP, 2) : -1.0);
    const double variance_z = has_std ? std::pow(STD_alt, 2) : (PDOP > 0 ? std::pow(PDOP, 2) : -1.0);
    msg.position_covariance[0] = variance_x;
    msg.position_covariance[4] = variance_y;
    msg.position_covariance[8] = variance_z;
    msg.position_covariance_type = (variance_x > 0 && variance_y > 0 && variance_z > 0)
        ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
        : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    if (gnss_pub_)
        gnss_pub_->publish(msg);

}

void NMEA2000ClientROS2::publishCogSog()
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = nowStamp();
    msg.header.frame_id = frame_id_;
    msg.twist.linear.x = speed_ms;
    msg.twist.angular.z = compass_rad;
    
    if (cog_sog_pub_)
        cog_sog_pub_->publish(msg);

    geometry_msgs::msg::TwistWithCovarianceStamped velocity;
    velocity.header = msg.header;
    const double yaw = -compass_rad + M_PI / 2.0;
    velocity.twist.twist.linear.x = speed_ms * std::cos(yaw);
    velocity.twist.twist.linear.y = speed_ms * std::sin(yaw);
    if (velocity_pub_)
        velocity_pub_->publish(velocity);

}

void NMEA2000ClientROS2::publishRapidPosition()
{
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = nowStamp();
    msg.header.frame_id = frame_id_;
    msg.latitude = rapid_latitude;
    msg.longitude = rapid_longitude;
    msg.altitude = altitude;
    msg.status.status = GPSFix >= 4 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    msg.status.service = gnss_service(GPSType);
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
 
    if (rapid_position_pub_)
        rapid_position_pub_->publish(msg);
}

void NMEA2000ClientROS2::publishAttitude()
{
    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = nowStamp();
    msg.header.frame_id = frame_id_;
    msg.vector.x = Roll;
    msg.vector.y = Pitch;
    msg.vector.z = Yaw;
    
    if (attitude_pub_)
        attitude_pub_->publish(msg);

    sensor_msgs::msg::Imu imu;
    imu.header = msg.header;
    const double cr = std::cos(Roll * 0.5);
    const double sr = std::sin(Roll * 0.5);
    const double cp = std::cos(Pitch * 0.5);
    const double sp = std::sin(Pitch * 0.5);
    const double cy = std::cos(Yaw * 0.5);
    const double sy = std::sin(Yaw * 0.5);
    imu.orientation.w = cr * cp * cy + sr * sp * sy;
    imu.orientation.x = sr * cp * cy - cr * sp * sy;
    imu.orientation.y = cr * sp * cy + sr * cp * sy;
    imu.orientation.z = cr * cp * sy - sr * sp * cy;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    if (imu_pub_)
        imu_pub_->publish(imu);
}

void NMEA2000ClientROS2::publishPseudoNoise()
{
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "GNSS Pseudo Noise Statistics";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "GNSS pseudo noise stats";

    auto add_kv = [&status](const std::string &key, double value)
    {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = std::to_string(value);
        status.values.push_back(kv);
    };

    add_kv("RMS_uncertainty", RMS_uncertainty);
    add_kv("STD_major", STD_of_Major);
    add_kv("STD_minor", STD_of_Minor);
    add_kv("Orientation_major_rad", Orientation_of_Major);
    add_kv("STD_lat", STD_lat);
    add_kv("STD_lon", STD_lon);
    add_kv("STD_alt", STD_alt);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = nowStamp();
    arr.status.push_back(status);
    
    if (pseudo_noise_pub_)
        pseudo_noise_pub_->publish(arr);
}

}  // namespace ros2_isobus

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_isobus::NMEA2000ClientROS2>());
    rclcpp::shutdown();
    return 0;
}
