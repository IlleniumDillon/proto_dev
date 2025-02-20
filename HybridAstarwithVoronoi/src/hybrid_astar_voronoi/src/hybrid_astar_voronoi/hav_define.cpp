#include "hav_define.hpp"

using namespace hav;

hav::GridGraph::GridGraph(cv::Point3d origin, cv::Point3d resolution, cv::Point3d state_min, cv::Point3d state_max, const cv::Mat &map)
{
    this->origin = origin;
    this->resolution = resolution;
    this->state_min = state_min;
    this->state_max = state_max;
    this->size.x = (state_max.x - state_min.x) / resolution.x;
    this->size.y = (state_max.y - state_min.y) / resolution.y;
    this->size.z = (state_max.z - state_min.z) / resolution.z;

    this->gnodes.resize(size.x);
    for (int i = 0; i < size.x; i++)
    {
        this->gnodes[i].resize(size.y);
        for (int j = 0; j < size.y; j++)
        {
            this->gnodes[i][j] = new GridNode();
            this->gnodes[i][j]->position.x = state_min.x + i * resolution.x;
            this->gnodes[i][j]->position.y = state_min.y + j * resolution.y;
            this->gnodes[i][j]->index.x = i;
            this->gnodes[i][j]->index.y = j;
            // this->gnodes[i][j]->is_obstacle = map.at<uchar>(j, i) == 0 ? true : false;
            if(map.at<uchar>(j, i) == 0)
            {
                this->gnodes[i][j]->is_obstacle = true;
                this->gnodes[i][j]->distance_to_obstacle = 0;
                this->gnodes[i][j]->nearest_obstacle = cv::Point2i(i, j);

                bool surrounded = true;
                for(int dx = -1; dx <= 1; dx++)
                {
                    for(int dy = -1; dy <= 1; dy++)
                    {
                        if(dx == 0 && dy == 0)
                        {
                            continue;
                        }
                        int nx = i + dx;
                        int ny = j + dy;
                        if(nx < 0 || nx >= size.x || ny < 0 || ny >= size.y)
                        {
                            continue;
                        }
                        if(map.at<uchar>(ny, nx) != 0)
                        {
                            surrounded = false;
                            break;
                        }
                    }
                }
                if(!surrounded)
                {
                    this->gnodes[i][j]->it = open_set.insert(std::make_pair(0, this->gnodes[i][j]));
                }
            }

            this->gnodes[i][j]->pnodes.resize(size.z);
            for (int k = 0; k < size.z; k++)
            {
                this->gnodes[i][j]->pnodes[k] = new PlanNode(this->gnodes[i][j]);
                this->gnodes[i][j]->pnodes[k]->state.x = state_min.x + i * resolution.x;
                this->gnodes[i][j]->pnodes[k]->state.y = state_min.y + j * resolution.y;
                this->gnodes[i][j]->pnodes[k]->state.z = state_min.z + k * resolution.z;
                this->gnodes[i][j]->pnodes[k]->index.x = i;
                this->gnodes[i][j]->pnodes[k]->index.y = j;
                this->gnodes[i][j]->pnodes[k]->index.z = k;
            }
        }
    }

    initDistanceToObstacle();
}

hav::GridGraph::~GridGraph()
{
    for (int i = 0; i < size.x; i++)
    {
        for (int j = 0; j < size.y; j++)
        {
            delete this->gnodes[i][j];
        }
    }
}

void hav::GridGraph::updateObstacle(const std::vector<cv::Point2i> &map)
{

}

void hav::GridGraph::initDistanceToObstacle()
{
    while(!open_set.empty())
    {
        GridNode* current = open_set.begin()->second;
        open_set.erase(open_set.begin());
        close_set.push_back(current);
        current->flag = DM_CONFIRMED;
        for(int dx = -1; dx <= 1; dx++)
        {
            for(int dy = -1; dy <= 1; dy++)
            {
                if(dx == 0 && dy == 0)
                {
                    continue;
                }
                int nx = current->index.x + dx;
                int ny = current->index.y + dy;
                if(nx < 0 || nx >= size.x || ny < 0 || ny >= size.y)
                {
                    continue;
                }
                GridNode* neighbor = this->gnodes[nx][ny];
                if(neighbor->is_obstacle)
                {
                    continue;
                }
                cv::Point2i diff = current->nearest_obstacle - neighbor->index;
                double distance = std::sqrt(diff.x*diff.x + diff.y*diff.y);
                if(neighbor->flag == DM_UNKNOWN)
                {
                    neighbor->distance_to_obstacle = distance;
                    neighbor->nearest_obstacle = current->nearest_obstacle;
                    neighbor->flag = DM_UNCONFIRMED;
                    neighbor->it = open_set.insert(std::make_pair(distance, neighbor));
                }
                else if(neighbor->flag == DM_UNCONFIRMED && neighbor->distance_to_obstacle > distance)
                {
                    open_set.erase(neighbor->it);
                    neighbor->distance_to_obstacle = distance;
                    neighbor->nearest_obstacle = current->nearest_obstacle;
                    neighbor->it = open_set.insert(std::make_pair(distance, neighbor));
                }
            }
        }
    }
    reset();
}

void hav::GridGraph::updateDistanceToObstacle()
{
    
}

void hav::GridGraph::reset()
{
    for(auto& n : open_set)
    {
        n.second->flag = DM_UNKNOWN;
    }
    open_set.clear();
    for(auto n : close_set)
    {
        n->flag = DM_UNKNOWN;
    }
    close_set.clear();
}

PlanNode *hav::GridGraph::operator()(int x, int y, int z)
{
    if(x < 0 || x >= size.x || y < 0 || y >= size.y || z < 0 || z >= size.z)
    {
        return nullptr;
    }
    return this->gnodes[x][y]->pnodes[z];
}

PlanNode *hav::GridGraph::operator()(const cv::Point3i &index)
{
    return this->operator()(index.x, index.y, index.z);
}

PlanNode *hav::GridGraph::operator()(double x, double y, double z)
{
    int ix = (x - state_min.x) / resolution.x;
    int iy = (y - state_min.y) / resolution.y;
    int iz = (z - state_min.z) / resolution.z;
    return this->operator()(ix, iy, iz);
}

PlanNode *hav::GridGraph::operator()(const cv::Point3d &state)
{
    return this->operator()(state.x, state.y, state.z);
}

GridNode *hav::GridGraph::operator()(int x, int y)
{
    if(x < 0 || x >= size.x || y < 0 || y >= size.y)
    {
        return nullptr;
    }
    return this->gnodes[x][y];
}

GridNode *hav::GridGraph::operator()(const cv::Point2i &index)
{
    return this->operator()(index.x, index.y);
}

GridNode *hav::GridGraph::operator()(double x, double y)
{
    int ix = (x - state_min.x) / resolution.x;
    int iy = (y - state_min.y) / resolution.y;
    return this->operator()(ix, iy);
}

GridNode *hav::GridGraph::operator()(const cv::Point2d &state)
{
    return this->operator()(state.x, state.y);
}

bool hav::PlanNodeCost::operator<(const PlanNodeCost &other) const
{
    return distance == other.distance ? 
            gnode->distance_to_obstacle > other.gnode->distance_to_obstacle :
            distance < other.distance;
}

bool hav::PlanNodeCost::operator>(const PlanNodeCost &other) const
{
    return distance == other.distance ? 
            gnode->distance_to_obstacle < other.gnode->distance_to_obstacle :
            distance > other.distance;
}