#include "Point.hpp"
#include "Halfedge.hpp"
#include <Eigen>
double Point::area()
{
    auto vertices = this->vertices();
    Eigen::MatrixXd matrix(vertices.size(), 2);
    for (int i = 0; i < vertices.size(); i++)
    {
        matrix(i, 0) = vertices[i]->x;
        matrix(i, 1) = vertices[i]->y;
    }
    double area = 0;
    for (int i = 0; i < vertices.size(); i++)
    {
        area += matrix(i, 0) * matrix((i + 1) % vertices.size(), 1) - matrix(i, 1) * matrix((i + 1) % vertices.size(), 0);
    }
    return std::abs(area) / 2;
}

std::vector<Halfedge *> Point::borders()
{
    if (halfedge_head == nullptr)
        return std::vector<Halfedge *>();
    std::vector<Halfedge *> borders;
    Halfedge *current = halfedge_head;
    do
    {
        borders.push_back(current);
        current = current->next;
    } while (current != halfedge_head && current != nullptr);
    return borders;
}

std::vector<Vertex *> Point::vertices()
{
    auto borders = this->borders();
    std::vector<Vertex *> vertices;
    for (auto border : borders)
    {
        if(border->origin.type() == typeid(Vertex*))
            vertices.push_back(boost::get<Vertex *>(border->origin));
    }
    return vertices;
}
