#ifndef SC_DEFINE_HPP
#define SC_DEFINE_HPP

#include <string>
#include <map>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

namespace sc
{
class PlanNodeCost;
class PlanNode;
class GridNode;
class GridGraph;

enum PlanNodeFlag
{
    NOT_VISITED,
    IN_OPENSET,
    IN_CLOSESET
};

class PlanNodeCost
{
public:
    GridNode* gnode = nullptr;
    double distance = 0.0;
public:
    PlanNodeCost(){}
    PlanNodeCost(GridNode* node, double distance = 0): gnode(node), distance(distance){}

    bool operator<(const PlanNodeCost& other) const;
    bool operator>(const PlanNodeCost& other) const;
    bool operator==(const PlanNodeCost& other) const
    {
        return distance == other.distance;
    }

    PlanNodeCost operator+(const PlanNodeCost& other) const
    {
        return PlanNodeCost(gnode, distance + other.distance);
    }
    PlanNodeCost operator+(const double& dist) const
    {
        return PlanNodeCost(gnode, distance + dist);
    }
    PlanNodeCost operator-(const PlanNodeCost& other) const
    {
        return PlanNodeCost(gnode, distance - other.distance);
    }
    PlanNodeCost operator-(const double& dist) const
    {
        return PlanNodeCost(gnode, distance - dist);
    }
};

class PlanNode
{
public:
    GridNode* gnode = nullptr;
    cv::Point3d state;
    cv::Point3i index;
    PlanNode* parent = nullptr;
    PlanNodeCost g, h, f;
    PlanNodeFlag flag;
    // bool is_obstacle = false;
    // double distance_to_obstacle;
    cv::Point2d control_input;
    std::multimap<PlanNodeCost, PlanNode*>::iterator it;
public:
    PlanNode(GridNode* pos)
    {
        gnode = pos;
        state = cv::Point3d(0, 0, 0);
        index = cv::Point3i(0, 0, 0);
        parent = nullptr;
        g = PlanNodeCost(gnode, 0.0);
        h = PlanNodeCost(gnode, 0.0);
        f = PlanNodeCost(gnode, 0.0);
        flag = NOT_VISITED;
        // is_obstacle = false;
        // distance_to_obstacle = 0.0;
        control_input = cv::Point2d(0, 0);
    }
    bool operator==(const PlanNode& other) const
    {
        return index == other.index;
    }
};

enum GridNodeFlag
{
    DM_UNKNOWN,
    DM_UNCONFIRMED,
    DM_CONFIRMED,
    DM_RISE,
    DM_FALL
};

class GridNode
{
public:
    cv::Point2d position;
    cv::Point2i index;
    std::vector<PlanNode*> pnodes;

    GridNodeFlag flag;
    std::multimap<double, GridNode*>::iterator it;
    double distance_to_obstacle = 0.0;
    cv::Point2i nearest_obstacle;
    bool is_obstacle = false;
    bool need_rise = false;
public:
    GridNode()
    {
        position = cv::Point2d(0, 0);
        index = cv::Point2i(0, 0);
        flag = DM_UNKNOWN;
        distance_to_obstacle = std::numeric_limits<double>::max();
        nearest_obstacle = cv::Point2i(-1, -1);
        is_obstacle = false;
        need_rise = false;
    }
    ~GridNode()
    {
        for(auto& pnode: pnodes)
        {
            delete pnode;
        }
    }
};

class GridGraph
{
public:
    GridGraph(){}
    GridGraph(cv::Point3d origin, cv::Point3d resolution, cv::Point3d state_min, cv::Point3d state_max, const cv::Mat& map);
    ~GridGraph();
// for hybrid A* search
public:
    cv::Point3d origin;
    cv::Point3d resolution;
    cv::Point3i size;
    cv::Point3d state_min;
    cv::Point3d state_max;

    std::vector<std::vector<GridNode*>> gnodes;
public:
    PlanNode* operator()(int x, int y, int z);
    PlanNode* operator()(const cv::Point3i& index);
    PlanNode* operator()(double x, double y, double z);
    PlanNode* operator()(const cv::Point3d& state);

    GridNode* operator()(int x, int y);
    GridNode* operator()(const cv::Point2i& index);
    GridNode* operator()(double x, double y);
    GridNode* operator()(const cv::Point2d& state);
// for Dynamic Distance Map
public:
    std::multimap<double, GridNode*> open_set;
    std::vector<GridNode*> close_set;

    // std::vector<cv::Point2i> insert_obstacles;
    // std::vector<cv::Point2i> remove_obstacles;
public:
    void initDistanceToObstacle();
    void insertObstacle(const cv::Point2i& index);
    void removeObstacle(const cv::Point2i& index);
    void updateObstacle(const std::vector<cv::Point2i>& map);
    void updateDistanceToObstacle();
    void reset();
};
}   // namespace sc

#endif // SC_DEFINE_HPP