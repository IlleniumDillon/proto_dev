#ifndef VERTEX_HPP
#define VERTEX_HPP

#include "Coordinate.hpp"
#include <vector>

class Halfedge;

class Vertex : public Coordinate
{
public:
    std::vector<Halfedge*> halfedges;
    Vertex(double x, double y) : Coordinate(x, y) {}
    Vertex() : Coordinate() {}
};

#endif // VERTEX_HPP