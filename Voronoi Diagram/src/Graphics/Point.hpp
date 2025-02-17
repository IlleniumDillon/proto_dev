#ifndef POINT_HPP
#define POINT_HPP

#include <cmath>
#include "Coordinate.hpp"
#include "Vertex.hpp"

class Halfedge;

class Point : public Coordinate
{
public:
    Halfedge* halfedge_head;
public:
    Point(double x, double y) : Coordinate(x, y), halfedge_head(nullptr) {}
    double area();
    std::vector<Halfedge*> borders();
    std::vector<Vertex*> vertices();    
};

#endif // POINT_HPP