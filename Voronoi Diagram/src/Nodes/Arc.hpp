#ifndef ARC_HPP
#define ARC_HPP

#include "../Graphics/Coordinate.hpp"
#include "../Graphics/Point.hpp"

class CircleEvent;

class Arc
{
public:
    Point* origin;
    CircleEvent* circle_event;
public:
    Arc(Point* origin, CircleEvent* circle_event = nullptr)
        : origin(origin), circle_event(circle_event)
    {
    }
};

#endif // ARC_HPP