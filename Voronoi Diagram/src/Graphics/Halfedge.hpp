#ifndef HALFEDGE_HPP
#define HALFEDGE_HPP

#include "Vertex.hpp"
#include "Point.hpp"

#include <boost/variant.hpp>

class Breakpoint;

class Halfedge
{
private:
    Halfedge *twin;
    Halfedge *_twin;
public:
    boost::variant<Breakpoint *, Vertex *> origin;
    Point incident_point;
    Halfedge *next;
    Halfedge *prev;
    bool removed;
public:
    Halfedge(Vertex *origin, Halfedge *twin, Point incident_point)
    : origin(origin), twin(twin), incident_point(incident_point), next(nullptr), prev(nullptr), removed(false), _twin(nullptr)
    {
    }
    void setNext(Halfedge *next)
    {
        if(next!=nullptr)
            next->prev = this;
        this->next = next;
    }
    Coordinate getOrigin();
    Halfedge * getTwin(){ return _twin; }
    void setTwin(Halfedge *twin)
    {
        if(twin!=nullptr)
            twin->_twin = this;
        this->_twin = twin;
    }
    Vertex* getTarget()
    {
        if(this->twin == nullptr)
            return nullptr;
        return boost::get<Vertex *>(this->twin->origin);
    }
    void deleteThis();
};

#endif // HALFEDGE_HPP