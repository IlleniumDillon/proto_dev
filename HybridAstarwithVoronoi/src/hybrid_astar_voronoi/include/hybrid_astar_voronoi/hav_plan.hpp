#ifndef HAV_PLAN_HPP
#define HAV_PLAN_HPP

#include "hav_define.hpp"

namespace hav
{

class PlanResult
{
public:
    bool success = false;
    int iterations = 0;
    double plan_time_ms = -1.0;
    double path_length = -1.0;
    std::vector<cv::Point3d> path;
    std::vector<cv::Point2d> control_inputs;
};

class HAVPlanner
{
public:
    std::multimap<PlanNodeCost, PlanNode*> open_set;
    std::vector<PlanNode*> close_set;
    GridGraph* graph;
    PlanResult result;
    std::vector<cv::Point2d> control_space;
private:
    std::vector<cv::Point3d> neighbor_space;
    std::vector<double> neighbor_cost;
public:
    HAVPlanner(){}
    void setExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt);
    void bindGraph(GridGraph* graph){this->graph = graph;}
    void reset();
    double heuristic(PlanNode* node, PlanNode* goal);
    void getNeighbors(PlanNode* node, 
        std::vector<PlanNode*>& neighbors,
        std::vector<PlanNodeCost>& costs,
        std::vector<cv::Point3f>& neighbor_state,
        std::vector<cv::Point2d>& vws);
    bool endCondition(PlanNode* node, PlanNode* goal);
    PlanResult search(PlanNode* start, PlanNode* goal);
    PlanResult search(cv::Point3d start, cv::Point3d goal);
};

} // namespace hav

#endif // HAV_PLAN_HPP