#include "sc_define.hpp"

#include <iostream>

int main()
{
    cv::Mat map = cv::imread("../map.png", cv::IMREAD_GRAYSCALE);
    // cv::imshow("map", map);
    // cv::waitKey(0);

    cv::Point3d origin(0, 0, 0);
    cv::Point3d resolution(0.05, 0.05, 0.05);
    cv::Point3d state_min(0, 0, -M_PI);
    cv::Point3d state_max(map.cols * resolution.x, map.rows * resolution.y, M_PI);
    auto graph = std::make_shared<sc::GridGraph>(origin, resolution, state_min, state_max, map);

    double d = 0.2;
    double f = resolution.x / d / 2;
    std::vector<cv::Point2i> heightLine;
    cv::Mat distanceMapf = cv::Mat::zeros(map.size(), CV_32FC1);
    for(int i = 0; i < map.rows; i++)
    {
        for(int j = 0; j < map.cols; j++)
        {
            distanceMapf.at<float>(i, j) = graph->operator()(j, i)->distance_to_obstacle * resolution.x;
            double dev = (2*distanceMapf.at<float>(i, j)-d) / 2 / d;
            double floor = std::floor(dev);
            double ceil = std::ceil(dev);
            double err = std::min(std::abs(dev - floor), std::abs(dev - ceil));
            if(err < f)
            {
                heightLine.push_back(cv::Point2i(j, i));
            }
        }
    }


    double min, max;
    cv::minMaxLoc(distanceMapf, &min, &max);
    distanceMapf = (distanceMapf - min) / (max - min) * 255;
    cv::Mat distanceMap = cv::Mat::zeros(map.size(), CV_8UC3);
    for(int i = 0; i < map.rows; i++)
    {
        for(int j = 0; j < map.cols; j++)
        {
            distanceMap.at<cv::Vec3b>(i, j) = cv::Vec3b(distanceMapf.at<float>(i, j), distanceMapf.at<float>(i, j), distanceMapf.at<float>(i, j));
        }
    }
    for(auto & p : heightLine)
    {
        distanceMap.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0, 0, 255);
    }
    // cv::imshow("distance map", distanceMap);
    // cv::waitKey(0);

    // cv::imwrite("../distance_map.png", distanceMap);

    cv::Point2i start(132,109);
    std::vector<cv::Point2i> path;
    auto current = graph->operator()(start.x, start.y);
    while(current!=nullptr)
    {
        path.push_back(current->index);
        current->flag = sc::DM_CONFIRMED;

        double minT = std::numeric_limits<double>::max();
        cv::Point2i maxIndex = cv::Point2i(-1, -1);
        int pathIndex = 0;

        for(;pathIndex < path.size(); pathIndex++)
        {
            cv::Point2i pathPoint = path[path.size() - 1 - pathIndex];
            current = (*graph)(pathPoint.x, pathPoint.y);
            
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
                    if(nx < 0 || nx >= map.cols || ny < 0 || ny >= map.rows)
                    {
                        continue;
                    }
                    auto neighbor = (*graph)(nx, ny);
                    if(neighbor->flag == sc::DM_CONFIRMED || neighbor->is_obstacle)
                    {
                        continue;
                    }
                    if(neighbor->distance_to_obstacle < minT)
                    {
                        minT = neighbor->distance_to_obstacle;
                        maxIndex = neighbor->index;
                    }
                }
            }

            if (maxIndex.x >= 0)
            {
                break;
            }
        }
        if (maxIndex.x < 0)
        {
            break;
        }
        else 
        {
            current = (*graph)(maxIndex.x, maxIndex.y);
        }
    }

    cv::Mat pathMap = cv::Mat::zeros(map.size(), CV_8UC3);
    for(int i = 0; i < map.rows; i++)
    {
        for(int j = 0; j < map.cols; j++)
        {
            pathMap.at<cv::Vec3b>(i, j) = cv::Vec3b(map.at<uchar>(i, j), map.at<uchar>(i, j), map.at<uchar>(i, j));
        }
    }
    for(auto & p : path)
    {
        pathMap.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0, 255, 0);
        cv::imshow("path map", pathMap);
        cv::waitKey(1);
    }

    return 0;
}