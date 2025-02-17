#ifndef BOUNDING_HPP
#define BOUNDING_HPP

#include "Polygon.hpp"
#include "Coordinate.hpp"

class BoundingBox : public Polygon
{
public:
    BoundingBox(double lx, double rx, double by, double ty)
        : Polygon({Coordinate(lx, by), Coordinate(rx, by), Coordinate(rx, ty), Coordinate(lx, ty)})
    {
    }
};

#endif // BOUNDING_HPP