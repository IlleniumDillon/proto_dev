#include "hav_plan.hpp"

using namespace hav;

void hav::HAVPlanner::setExecuteSpace(double max_v, double max_w, int step_v, int step_w, double dt)
{
    control_space.clear();
    neighbor_space.clear();
    neighbor_cost.clear();

    double v_ = max_v / step_v;
    double w_ = max_w / step_w;
    for (int i = -step_v; i <= step_v; i++)
    {
        for (int j = -step_w; j <= step_w; j++)
        {
            if (i == 0 && j == 0)
            {
                continue;
            }
            double v = i * v_;
            double w = j * w_;
            
            cv::Point3d neighbor;
            neighbor.z = w * dt;
            neighbor.y = neighbor.z / 2;
            if (j == 0)
            {
                neighbor.x = v * dt;
            }
            else
            {
                double R = v / w;
                double L = std::sqrt(2*R*R*(1 - std::cos(w * dt)));
                neighbor.x = L;
            }
            control_space.emplace_back(cv::Point2d(v, w));
            neighbor_space.emplace_back(neighbor);

            neighbor_cost.emplace_back(std::abs(v*dt));
            if (v < 0)
            {
                neighbor_cost.back() *= 2;
            }
            else if (v == 0)
            {
                neighbor_cost.back() = dt;
            }
        }
    }
}

void hav::HAVPlanner::reset()
{
    for (auto& n : open_set)
    {
        n.second->parent = nullptr;
        n.second->flag = NOT_VISITED;
    }
    open_set.clear();
    for (auto n : close_set)
    {
        n->parent = nullptr;
        n->flag = NOT_VISITED;
    }
    close_set.clear();
}

double hav::HAVPlanner::heuristic(PlanNode *node, PlanNode *goal)
{
    cv::Point2d diff = node->gnode->position - goal->gnode->position;
    return std::sqrt(diff.x*diff.x + diff.y*diff.y);
}

void hav::HAVPlanner::getNeighbors(PlanNode *node, std::vector<PlanNode *> &neighbors, std::vector<PlanNodeCost> &costs, std::vector<cv::Point3f> &neighbor_state, std::vector<cv::Point2d> &vws)
{
    neighbors.clear();
    costs.clear();
    neighbor_state.clear();
    vws.clear();

    for(size_t i = 0; i < neighbor_space.size(); i++)
    {
        auto&d = neighbor_space[i];
        cv::Point3f dstate(
            d.x*std::cos(node->state.z + d.y),
            d.x*std::sin(node->state.z + d.y),
            d.z
        );
        cv::Point3f state(
            node->state.x + dstate.x,
            node->state.y + dstate.y,
            node->state.z + dstate.z
        );
        if (state.z > M_PI)
        {
            state.z -= 2*M_PI;
        }
        else if (state.z <= -M_PI)
        {
            state.z += 2*M_PI;
        }
        auto neighbor = (*graph)(state.x, state.y, state.z);

        if(neighbor != nullptr && !neighbor->gnode->is_obstacle)
        {
            // std::cout << "neighbor: " << state.x << "," << state.y << "," << state.z << std::endl;
            neighbors.emplace_back(neighbor);
            costs.emplace_back(PlanNodeCost(nullptr, neighbor_cost[i]));
            neighbor_state.emplace_back(state);
            vws.emplace_back(control_space[i]);
        }
    }
}

bool hav::HAVPlanner::endCondition(PlanNode *node, PlanNode *goal)
{
    auto diff = node->gnode->position - goal->gnode->position;
    if(sqrt(diff.x*diff.x + diff.y*diff.y) < 0.1)
    {
        return true;
    }
    return false;
}

PlanResult hav::HAVPlanner::search(PlanNode *start, PlanNode *goal)
{
    result = PlanResult();

    auto start_time = std::chrono::high_resolution_clock::now();
    start->g = PlanNodeCost(start->gnode, 0);
    start->h = PlanNodeCost(nullptr, heuristic(start, goal));
    start->f = start->g + start->h;
    start->parent = nullptr;
    start->flag = IN_OPENSET;
    start->it = open_set.insert(std::make_pair(start->f, start));

    std::vector<PlanNode*> neighbors;
    std::vector<PlanNodeCost> costs;
    std::vector<cv::Point3f> neighbor_state;
    std::vector<cv::Point2d> vws;
    PlanNodeCost tentative_g;

    while (!open_set.empty())
    {
        result.iterations++;
        auto current = open_set.begin()->second;
        if (endCondition(current, goal))
        {
            result.success = true;
            result.path_length = current->g.distance;
            while (current != nullptr)
            {
                result.path.emplace_back(current->state);
                result.control_inputs.emplace_back(current->control_input);
                current = current->parent;
            }
            std::reverse(result.path.begin(), result.path.end());
            std::reverse(result.control_inputs.begin(), result.control_inputs.end());
            break;
        }
        open_set.erase(current->it);
        close_set.emplace_back(current);
        current->flag = IN_CLOSESET;
        getNeighbors(current, neighbors, costs, neighbor_state, vws);
        for (size_t i = 0; i < neighbors.size(); i++)
        {
            auto neighbor = neighbors[i];
            auto cost = costs[i];
            auto state = neighbor_state[i];
            auto vw = vws[i];
            tentative_g = current->g + cost;
            if (neighbor->flag != IN_OPENSET)
            {
                neighbor->state = state;
                neighbor->h = PlanNodeCost(nullptr, heuristic(neighbor, goal));
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                neighbor->control_input = vw;
                neighbor->flag = IN_OPENSET;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
            else if (tentative_g < neighbor->g)
            {
                open_set.erase(neighbor->it);
                neighbor->state = state;
                // neighbor->h = heuristic(neighbor, goal);
                neighbor->g = tentative_g;
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                neighbor->control_input = vw;
                neighbor->it = open_set.insert(std::make_pair(neighbor->f, neighbor));
            }
        }
    }
    reset();
    auto end_time = std::chrono::high_resolution_clock::now();
    result.plan_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    return result;
}

PlanResult hav::HAVPlanner::search(cv::Point3d start, cv::Point3d goal)
{
    auto start_node = (*graph)(start.x, start.y, start.z);
    auto goal_node = (*graph)(goal.x, goal.y, goal.z);
    if (start_node == nullptr || goal_node == nullptr)
    {
        return PlanResult();
    }
    return search(start_node, goal_node);
}
