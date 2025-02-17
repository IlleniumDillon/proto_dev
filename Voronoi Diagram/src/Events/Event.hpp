#ifndef EVENT_HPP
#define EVENT_HPP

class Event
{
public:
    bool isCircleEvent = false;
    virtual double x() const { return 0; }
    virtual double y() const { return 0; }

    bool operator<(const Event &other) const
    {
        if (this->y() == other.y() && this->x() == other.x())
            return this->isCircleEvent && !other.isCircleEvent;
        if (this->y() == other.y())
            return this->x() < other.x();
        return this->y() > other.y();
    }
    bool operator==(const Event &other) const
    {
        return this->y() == other.y() && this->x() == other.x();
    }
    bool operator!=(const Event &other) const
    {
        return !(*this == other);
    }
};

#endif // EVENT_HPP