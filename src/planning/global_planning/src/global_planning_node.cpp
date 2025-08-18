#include "global_planning/global_planning_node.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "rcutils/logging_macros.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

GlobalPlanningNode::GlobalPlanningNode() : Node("global_planning_node")
{
    // Get parameters
    this->declare_parameter("debug", false);
    this->declare_parameter("csv_file_path", "LVMS_SVL_ENU_TTL_15.csv");
    this->declare_parameter("global_path_publish_rate", 1);
    this->declare_parameter("local_path_publish_rate", 10);
    this->declare_parameter("local_path_length_m", 50.0);  // Length in meters
    this->declare_parameter("planning_odom_publish_rate", 10);
    this->declare_parameter("visualize", false);

    auto debug_ = this->get_parameter("debug").as_bool();
    if (debug_) {
        RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
        rcutils_ret_t ret = rcutils_logging_set_logger_level(
            this->get_logger().get_name(),
            RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set logger level to DEBUG");
        }
    }
    visualize_ = this->get_parameter("visualize").as_bool();

    csv_file_path_ = this->get_parameter("csv_file_path").as_string();
    local_path_length_m_ = this->get_parameter("local_path_length_m").as_double();

    // Initialize publishers
    path_pub_ = this->create_publisher<autoware_planning_msgs::msg::Path>(
        "/planning/global_path", 1);
    local_path_pub_ = this->create_publisher<autoware_planning_msgs::msg::Path>(
        "/planning/local_path", 1);
    planning_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/planning/odom", 1);
    if (visualize_) {
        global_path_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planning/global_path_vis", 1);
        local_path_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planning/local_path_vis", 1);
    }

    // Initialize subscriber
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps_top/fix", 10,
        std::bind(&GlobalPlanningNode::gpsCallback, this, std::placeholders::_1));
    lgsvl_odom_sub_ = this->create_subscription<lgsvl_msgs::msg::VehicleOdometry>(
        "/lgsvl/odometry", 10,
        std::bind(&GlobalPlanningNode::on_lgsvl_odom_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&GlobalPlanningNode::on_odom_callback, this, std::placeholders::_1));

    // Load waypoints from CSV
    loadWaypointsFromCSV(csv_file_path_);

    // Create timers for publishing
    const int global_path_publish_rate_ = this->get_parameter("global_path_publish_rate").as_int();
    global_path_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/global_path_publish_rate_),
        std::bind(&GlobalPlanningNode::publishGlobalPath, this));

    const int local_path_publish_rate_ = this->get_parameter("local_path_publish_rate").as_int();
    local_path_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/local_path_publish_rate_),  // Use same rate as global path
        std::bind(&GlobalPlanningNode::publishLocalPath, this));

    const int planning_odom_publish_rate_ = this->get_parameter("planning_odom_publish_rate").as_int();
    planning_odom_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/planning_odom_publish_rate_),
        std::bind(&GlobalPlanningNode::publishPlanningOdom, this));
}

void GlobalPlanningNode::loadWaypointsFromCSV(const std::string& csv_path)
{
    RCLCPP_INFO(this->get_logger(), "Loading waypoints from CSV file: %s", csv_path.c_str());
    std::ifstream file(csv_path);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
        return;
    }

    // Add error handling for string to double conversion
    auto safe_stod = [](const std::string& str) -> double {
        try {
            return std::stod(str);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("global_planning_node"), 
                "Failed to convert string to double: %s", str.c_str());
            return 0.0;
        }
    };

    // Read first line to get origin coordinates
    std::string line;
    if (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ','))
        {
            // Trim whitespace from value
            value.erase(0, value.find_first_not_of(" \t\r\n"));
            value.erase(value.find_last_not_of(" \t\r\n") + 1);
            if (!value.empty()) {
                row.push_back(safe_stod(value));
            }
        }

        if (row.size() >= 5)  // Ensure we have all values including origin coordinates
        {
            // Get the last three values as origin coordinates
            this->origin_x_ = row[3];  // origin_x
            this->origin_y_ = row[4];  // origin_y
            this->origin_z_ = row[5];  // origin_z
            RCLCPP_INFO(this->get_logger(), "Origin coordinates: x=%f, y=%f, z=%f", 
                       this->origin_x_, this->origin_y_, this->origin_z_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid CSV file format");
            return;
        }
    }

    // Initialize the projection
    const GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();
    proj = GeographicLib::LocalCartesian(origin_x_, origin_y_, origin_z_, earth);
    global_path_ = std::make_unique<autoware_planning_msgs::msg::Path>();
    global_path_->header.frame_id = "map";

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<double> row;

        while (std::getline(ss, value, ','))
        {
            row.push_back(safe_stod(value));
        }

        if (row.size() >= 3)  // Ensure we have at least x, y, z coordinates
        {
            autoware_planning_msgs::msg::PathPoint point;
            
            point.pose.position.x = row[0];  // x coordinate
            point.pose.position.y = row[1];  // y coordinate
            point.pose.position.z = row[2];  // z coordinate
            point.longitudinal_velocity_mps = row[4]; // target speed
        
            global_path_->points.push_back(point);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from CSV", global_path_->points.size());
}

void GlobalPlanningNode::p_convertGPStoENU(double latitude, double longitude, double altitude, Point3D* output) const
{
    // Convert GPS coordinates to ENU coordinates
    double east, north, up;
    proj.Forward(latitude, longitude, altitude, east, north, up); 
    output->x = east;
    output->y = north;
    output->z = up;
}

void GlobalPlanningNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    Point3D current_position;
    p_convertGPStoENU(msg->latitude, msg->longitude, msg->altitude, &current_position);
    // RCLCPP_DEBUG(this->get_logger(), "GPS: lat: %f, lon: %f | ENU: x=%f, y=%f", 
    //             msg->latitude, msg->longitude, 
    //             current_position.x, current_position.y);
    current_position_ = std::make_shared<Point3D>(current_position);
    received_gps_ = true;
}

void GlobalPlanningNode::on_lgsvl_odom_callback(const lgsvl_msgs::msg::VehicleOdometry::SharedPtr msg) {
    lgsvl_odom_ = msg;
}

size_t GlobalPlanningNode::findClosestWaypoint(
    const autoware_planning_msgs::msg::Path::SharedPtr path,
    const Point3D::SharedPtr current_position,
    const size_t search_radius,
    size_t prev_closest_index)
{
    if (path->points.empty()) {
        return 0;
    }

    auto closest_iter = std::min_element(path->points.begin(), path->points.end(), 
        [this, current_position](const autoware_planning_msgs::msg::PathPoint& a, const autoware_planning_msgs::msg::PathPoint& b) {
            return calculateDistance(*current_position, Point3D{a.pose.position.x, a.pose.position.y, a.pose.position.z}) < 
                   calculateDistance(*current_position, Point3D{b.pose.position.x, b.pose.position.y, b.pose.position.z});
    });
    
    size_t closest_index = std::distance(path->points.begin(), closest_iter);
    // double min_distance = calculateDistance(*current_position, Point3D{closest_iter->pose.position.x, closest_iter->pose.position.y, closest_iter->pose.position.z});
    // RCLCPP_DEBUG(this->get_logger(), "[findClosestWaypoint] Closest waypoint index: %zu, distance: %f", closest_index, min_distance);
    return closest_index;
}

size_t GlobalPlanningNode::findNextWaypoint(
    const Point3D::SharedPtr current_position,
    const size_t search_radius)
{
    // find the closest waypoint
    auto closest_index = findClosestWaypoint(global_path_, current_position, search_radius, current_waypoint_index_); // update the closest waypoint index
    current_waypoint_index_ = closest_index;

    // find the next waypoint by taking points from closest_index to closest point + N meters away
    size_t next_index = current_waypoint_index_;
    size_t start_index = current_waypoint_index_;  // Remember where we started
    double distance = 0;
    
    while (distance < search_radius) {
        size_t temp_next = (next_index + 1) % global_path_->points.size();
        
        // Break if we've wrapped around completely
        if (temp_next == start_index) {
            break;
        }
        
        next_index = temp_next;
        distance += calculateDistance(*current_position, Point3D{global_path_->points[next_index].pose.position.x,
                                                       global_path_->points[next_index].pose.position.y,
                                                       global_path_->points[next_index].pose.position.z});
    }
    RCLCPP_DEBUG(this->get_logger(), "[findNextWaypoint] Next waypoint index: %zu, distance: %f", next_index, distance);
    return next_index;
}

double GlobalPlanningNode::calculateDistance(const Point3D& p1, const Point3D& p2) const
{
    return std::sqrt(
        std::pow(p2.x - p1.x, 2) +
        std::pow(p2.y - p1.y, 2) +
        std::pow(p2.z - p1.z, 2));
}

void GlobalPlanningNode::publishGlobalPath()
{
    if (!received_gps_) {
        RCLCPP_WARN(this->get_logger(), "No GPS data received yet");
        return;
    }

    if (global_path_->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No waypoints available to publish");
        return;
    }

    auto message = autoware_planning_msgs::msg::Path();
    message.header.stamp = this->now();
    message.header.frame_id = "map";
    message.points = global_path_->points;

    path_pub_->publish(message);
    if (visualize_) {
        auto global_path_vis_msg = nav_msgs::msg::Path();
        convertAutowarePathToNavPath(message, global_path_vis_msg);
        this->global_path_vis_pub_->publish(global_path_vis_msg);
    }
}

void GlobalPlanningNode::convertAutowarePathToNavPath(const autoware_planning_msgs::msg::Path& autoware_path, nav_msgs::msg::Path& nav_path) {
    nav_path.header = autoware_path.header;
    nav_path.header.frame_id = "map";
    for (const auto& point : autoware_path.points) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = nav_path.header;
        pose_stamped.pose = point.pose;
        nav_path.poses.push_back(pose_stamped);
    }
}

void GlobalPlanningNode::publishLocalPath()
{
    if (!received_gps_ || this->current_position_ == nullptr) {
        // RCLCPP_WARN(this->get_logger(), "No GPS data or current position received yet");
        return;
    }

    if (global_path_->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "No waypoints available to publish");
        return;
    }
    // find the next waypoint
    size_t next_index = findNextWaypoint(current_position_, local_path_length_m_);
    // RCLCPP_DEBUG(this->get_logger(), "[publishLocalPath] Next waypoint index: %zu", next_index);

    // Create a new path message and populate it with a slice of the global path
    auto message = autoware_planning_msgs::msg::Path();
    message.header.stamp = this->now();
    message.header.frame_id = "map";
    
    // Copy points from current_waypoint_index_ to next_index
    message.points.insert(
        message.points.end(),
        global_path_->points.begin() + current_waypoint_index_,
        global_path_->points.begin() + next_index + 1  // +1 to include the end point
    );

    local_path_pub_->publish(message);

    if (visualize_) {
        auto local_path_vis_msg = nav_msgs::msg::Path();
        convertAutowarePathToNavPath(message, local_path_vis_msg);
        this->local_path_vis_pub_->publish(local_path_vis_msg);
    }
}

void GlobalPlanningNode::publishPlanningOdom() {
    if (!received_gps_ || this->current_position_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "No GPS data or current position received yet");
        return;
    }
    if (odom_ == nullptr) {
        // RCLCPP_WARN(this->get_logger(), "No LGSVL odom received yet");
        return;
    }
    const auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header = lgsvl_odom_->header;
    odom->header.frame_id = "map";
    odom->pose.pose.position.x = current_position_->x;
    odom->pose.pose.position.y = current_position_->y;
    odom->pose.pose.position.z = current_position_->z;
    odom->pose.pose.orientation = odom_->pose.pose.orientation;
    odom->twist = odom_->twist;

    planning_odom_pub_->publish(*odom);

    // Convert geometry_msgs quaternion to tf2 quaternion
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(odom->pose.pose.orientation, tf2_quat);
    const auto yaw = tf2::impl::getYaw(tf2_quat);
    RCLCPP_DEBUG(this->get_logger(), 
                "[publishPlanningOdom] Publishing odom: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, vel=%.2f", 
                odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z, 
                yaw, odom->twist.twist.linear.x);
}

void GlobalPlanningNode::on_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_ = msg;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanningNode>());
    rclcpp::shutdown();
    return 0;
}