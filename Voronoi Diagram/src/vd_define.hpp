#ifndef VD_DEFINE_HPP
#define VD_DEFINE_HPP

#include <vector>
#include <list>
#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <limits>

namespace voronoi
{

class Point;
class Rect;
class Site;
class Edge;
class GraphEdge;
class DelauneyEdge;
class DelauneyIter;
class Diagram;
class Clipper;
class HalfEdge;
class PriorityQ;
class _Context;

class Point
{
public:
    double x, y;
public:
    Point() : x(0), y(0) {}
    Point(double x, double y) : x(x), y(y) {}
    Point(const Point &p) : x(p.x), y(p.y) {}
    Point &operator=(const Point &p)
    {
        x = p.x;
        y = p.y;
        return *this;
    }

    bool operator==(const Point &p) const
    {
        return abs(x-p.x) < 1e-6 && abs(y-p.y) < 1e-6;
    }
    bool operator<(const Point &p) const
    {
        return (y == p.y) ? (x < p.x) : (y < p.y);
    }
    bool operator>(const Point &p) const
    {
        return (y == p.y) ? (x > p.x) : (y > p.y);
    }

    bool isOnRect(const Point &min, const Point &max) const
    {
        return x == min.x || x == max.x || y == min.y || y == max.y;
    }

    double distance(const Point &p) const
    {
        return sqrt((x-p.x)*(x-p.x) + (y-p.y)*(y-p.y));
    }
    double distance_squared(const Point &p) const
    {
        return (x-p.x)*(x-p.x) + (y-p.y)*(y-p.y);
    }
};

class Rect
{
public:
    Point min, max;
public:
    Rect() 
        : min(std::numeric_limits<double>::min(), std::numeric_limits<double>::min()), 
          max(std::numeric_limits<double>::max(), std::numeric_limits<double>::max())
        {}
    Rect(double min_x, double min_y, double max_x, double max_y) : min(min_x, min_y), max(max_x, max_y) {}
    Rect(const Point &min, const Point &max) : min(min), max(max) {}
    Rect(const Rect &r) : min(r.min), max(r.max) {}
    Rect &operator=(const Rect &r)
    {
        min = r.min;
        max = r.max;
        return *this;
    }
};

class Site
{
public:
    Point p;
    int index;
    GraphEdge *edges;
};

class GraphEdge
{
public:
    GraphEdge *next;
    Edge *edge;
    Site *neighbor;
    Point pos[2];
    double angle;
};

class Edge
{
public:
    Edge *next;
    Site *sites[2];
    Point pos[2];
    double a, b, c;
};

class DelauneyEdge
{
public:
    Edge *edge;
    Site *sites[2];
    Point pos[2];
};

class DelauneyIter
{
public:
    Edge *sentinel;
    Edge *current;
};

class Clipper
{
public:
    Point min, max;
};

class Diagram
{
public:
    _Context *internal;
    int numSites;
    Point min, max;
};

class HalfEdge
{
public:
    Edge *edge;
    HalfEdge* left;
    HalfEdge* right;
    Point vertex;
    double y;
    bool direction;
    int pqPos;
};

class PriorityQ
{
public:
    int maxItems;
    int numItems;
    void** items;
};

class _Context
{
public:
    Edge *edges;
    HalfEdge *beachline_start;
    HalfEdge *beachline_end;
    HalfEdge *last_inserted;
    PriorityQ* eventqueue;

    Site *sites;
    Site *bottomsite;
    int numsites;
    int currentsite;

    Clipper clipper;

    Rect rect;
};

};  // namespace voronoi

#endif // VD_DEFINE_HPP