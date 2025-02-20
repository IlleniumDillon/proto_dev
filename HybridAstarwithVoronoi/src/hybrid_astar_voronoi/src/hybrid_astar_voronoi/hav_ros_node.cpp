#include "hav_ros_node.hpp"
#include "tf2/utils.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

HAVROSNode::HAVROSNode()
    : Node("hav_ros_node")
{
    declare_parameter("mapfile", ament_index_cpp::get_package_share_directory("hybrid_astar_voronoi") + "/config/map.png");
    std::string mapfile = this->get_parameter("mapfile").as_string();
    cv::Mat map = cv::imread(mapfile, cv::IMREAD_GRAYSCALE);
    if (map.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map file: %s", mapfile.c_str());
        return;
    }
    cv::Point3d origin(0, 0, 0);
    cv::Point3d resolution(0.05, 0.05, 0.05);
    cv::Point3d state_min(0, 0, -M_PI);
    cv::Point3d state_max(map.cols * resolution.x, map.rows * resolution.y, M_PI);
    graph = std::make_shared<hav::GridGraph>(origin, resolution, state_min, state_max, map);
    planner.bindGraph(graph.get());
    planner.setExecuteSpace(0.3, M_PI / 4, 2, 2, 0.5);

    this->map.header.frame_id = "map";
    this->map.info.resolution = resolution.x;
    this->map.info.width = map.cols;
    this->map.info.height = map.rows;
    this->map.info.origin.position.x = state_min.x;
    this->map.info.origin.position.y = state_min.y;
    this->map.info.origin.position.z = 0;
    this->map.info.origin.orientation.x = 0;
    this->map.info.origin.orientation.y = 0;
    this->map.info.origin.orientation.z = 0;
    this->map.info.origin.orientation.w = 1;
    this->map.data.resize(map.cols * map.rows);
    for (int i = 0; i < map.rows; i++)
    {
        for (int j = 0; j < map.cols; j++)
        {
            this->map.data[i * map.cols + j] = map.at<uchar>(i, j) == 0 ? 100 : 0;
        }
    }

    path_pub = this->create_publisher<nav_msgs::msg::Path>("path", 10);
    map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 10, std::bind(&HAVROSNode::goalCallback, this, std::placeholders::_1));
    timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HAVROSNode::timerCallback, this));


    // cv::Mat dmshow(map.rows, map.cols, CV_8UC1);
    // for (int i = 0; i < map.rows; i++)
    // {
    //     for (int j = 0; j < map.cols; j++)
    //     {
    //         dmshow.at<uchar>(i, j) = (*graph).gnodes[j][i]->distance_to_obstacle;
    //     }
    // }
    // double min = 0, max = 0;
    // cv::minMaxIdx(dmshow, &min, &max);
    // dmshow = (dmshow - min) / (max - min) * 255;
    // cv::imshow("distance to obstacle", dmshow);
    // cv::waitKey(0);
}

void HAVROSNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    static bool first = true;
    if (first)
    {
        start.x = msg->pose.position.x;
        start.y = msg->pose.position.y;
        start.theta = tf2::getYaw(msg->pose.orientation);
        first = false;
    }
    else
    {
        goal.x = msg->pose.position.x;
        goal.y = msg->pose.position.y;
        goal.theta = tf2::getYaw(msg->pose.orientation);
        auto result = planner.search(cv::Point3d(start.x, start.y, start.theta), cv::Point3d(goal.x, goal.y, goal.theta));
        RCLCPP_INFO(this->get_logger(), "Plan time: %f ms", result.plan_time_ms);
        if (result.success)
        {
            path.header.frame_id = "map";
            path.header.stamp = this->now();
            path.poses.clear();
            for (auto& state: result.path)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = this->now();
                pose.pose.position.x = state.x;
                pose.pose.position.y = state.y;
                pose.pose.position.z = 0;
                pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), state.z));
                path.poses.emplace_back(pose);
            }
            path_pub->publish(path);
        }
        first = true;
    }
}

void HAVROSNode::timerCallback()
{
    map_pub->publish(map);
}
