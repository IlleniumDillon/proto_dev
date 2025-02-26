#ifndef HAV_ROS_NODE_HPP
#define HAV_ROS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "hav_plan.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

class HAVROSNode : public rclcpp::Node
{
public:
    std::shared_ptr<hav::GridGraph> graph;
    hav::HAVPlanner planner;
    nav_msgs::msg::Path path;
    nav_msgs::msg::OccupancyGrid map;
    geometry_msgs::msg::Pose2D start;
    geometry_msgs::msg::Pose2D goal;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dm_pub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr exchange_sub;

    std::vector<cv::Point2i> exchange_obstacles;
public:
    HAVROSNode();
    ~HAVROSNode() = default;

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void exchangeCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timerCallback();
};

#endif // HAV_ROS_NODE_HPP