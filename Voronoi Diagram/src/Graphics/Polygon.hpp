#ifndef POLYGON_HPP
#define POLYGON_HPP

/// TODO:

#include "Coordinate.hpp"
#include "Vertex.hpp"
#include "Halfedge.hpp"

#include "Algebra.hpp"

#include <algorithm>

class Polygon
{
public:
    std::vector<Coordinate> points;
    double min_x, min_y, max_x, max_y;
    Coordinate center;
    std::vector<Vertex*> vertices;
public:
    Polygon(std::vector<Coordinate> points)
    {
        this->points = points;
        orderPoints(this->points);
        min_x = min_y = std::numeric_limits<double>::max();
        max_x = max_y = std::numeric_limits<double>::min();
        for (auto& point : points)
        {
            min_x = std::min(min_x, point.x);
            min_y = std::min(min_y, point.y);
            max_x = std::max(max_x, point.x);
            max_y = std::max(max_y, point.y);
            vertices.push_back(new Vertex(point.x, point.y));
        }
        center = Coordinate((min_x + max_x) / 2, (min_y + max_y) / 2);
    }
    ~Polygon()
    {
        for (auto vertex : vertices)
            delete vertex;
    }
private:
    void orderPoints(std::vector<Coordinate>& points)
    {
        // clockwise
        auto keyFunc = [center = &center](const Coordinate& a, const Coordinate& b) -> bool
        {
            double angle1 = atan2(a.y - center->y, a.x - center->x);
            double angle2 = atan2(b.y - center->y, b.x - center->x);
            if (angle1 < 0)
                angle1 += 2 * M_PI;
            if (angle2 < 0)
                angle2 += 2 * M_PI;
            return angle1 < angle2;
        };
        std::sort(points.begin(), points.end(), keyFunc);
    }
    Coordinate& getClosestPoint(const Coordinate& point)
    {
        double min_dist = std::numeric_limits<double>::max();
        Coordinate* closest = nullptr;
        for (auto& p : points)
        {
            double dist = distance(p, point);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest = &p;
            }
        }
        return *closest;
    }
};

#endif // POLYGON_HPP