#ifndef SITE_EVENT_HPP
#define SITE_EVENT_HPP

#include "Event.hpp"
#include "../Graphics/Point.hpp"

class SiteEvent : public Event
{
public:
    Point point;
public:
    SiteEvent(Point point) : point(point) 
    {
        isCircleEvent = false;
    }
    double x() const override { return point.x; }
    double y() const override { return point.y; }
};

#endif // SITE_EVENT_HPP