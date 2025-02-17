#ifndef CIRCLE_EVENT_HPP
#define CIRCLE_EVENT_HPP

#include "Event.hpp"
#include"../Graphics/Coordinate.hpp"
#include "../Nodes/LeafNode.hpp"
#include "../Nodes/Arc.hpp"

#include <vector>

class CircleEvent : public Event
{
public:
    Coordinate center;
    double radius;
    LeafNode* arcNode;
    bool isValid = true;
    std::vector<Point> pointTriple;
    std::vector<LeafNode*> arcTriple;
public:
    CircleEvent(Coordinate center, double radius, LeafNode* arcNode,
        std::vector<Point> pointTriple = {}, 
        std::vector<LeafNode*> arcTriple = {})
        : center(center), radius(radius), arcNode(arcNode),
        pointTriple(pointTriple), arcTriple(arcTriple)
    {
        isCircleEvent = true;
    }
    double x() const override { return center.x; }
    double y() const override { return center.y - radius; }
    CircleEvent& remove()
    {
        isValid = false;
        return *this;
    }
    static CircleEvent* createCircleEvent(LeafNode* leftNode, LeafNode* midNode, LeafNode* rightNode, double sweepLine)
    {
        if(leftNode == nullptr || rightNode == nullptr || midNode == nullptr)
            return nullptr;
        /// TODO:
    }
private:

};

#endif // CIRCLE_EVENT_HPP