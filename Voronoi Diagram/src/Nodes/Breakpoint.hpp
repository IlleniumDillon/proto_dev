#ifndef BREAKPOINT_HPP
#define BREAKPOINT_HPP

#include "../Graphics/Coordinate.hpp"
#include "../Graphics/Point.hpp"
#include "../Graphics/Halfedge.hpp"
#include <utility>
#include <cmath>

class Breakpoint
{
public:
    std::pair<Point, Point> points;
    Halfedge* _edge = nullptr;
    Halfedge* edge = nullptr;

    Breakpoint(const Point& pl, const Point& pr, Halfedge* edge = nullptr)
        : points(std::make_pair(pl, pr)), edge(edge)
    {
    }
    bool doseIntersect()
    {
        return !((points.first.y == points.second.y) && 
            (points.second.x < points.first.x));
    }
    Coordinate getIntersection(double l, double max_y = 0)
    {
        Coordinate res;
        auto& i = points.first;
        auto& j = points.second;
        auto& p = i;

        double a = i.x;
        double b = i.y;
        double c = j.x;
        double d = j.y;
        double u = 2 * (b - l);
        double v = 2 * (d - l);

        if(i.y == j.y)
        {
            res.x = (i.x + j.x) / 2;
            if(j.x < i.x)
            {
                res.y = max_y ? max_y : std::numeric_limits<double>::max();
                return res;
            }
        }
        else if(i.y == l)
        {
            res.x = i.x;
            p = j;
        }
        else if(j.y == l)
        {
            res.x = j.x;
        }
        else
        {
            res.x = -(sqrt(v * (a * a * u - 2 * a * c * u + b * b * (u - v) + c * c * u) + d * d * u * (v - u) + l * l * (u - v) * (u - v)) + a * v - c * u) / (u - v);
        }

        a = p.x;
        b = p.y;
        u = 2 * (b - l);

        if(u == 0)
        {
            res.y = std::numeric_limits<double>::max();
            return res;
        }

        res.y = 1 / u * (res.x * res.x - 2 * a * res.x + a * a + b * b - l * l);
        return res;
    }
};

#endif // BREAKPOINT_HPP