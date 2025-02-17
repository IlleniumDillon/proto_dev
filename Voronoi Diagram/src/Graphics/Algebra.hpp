#ifndef ALGEBRA_HPP
#define ALGEBRA_HPP

#include <cmath>

#include "Coordinate.hpp"

double distance(const Coordinate& a, const Coordinate& b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
double magnitude(const Coordinate& a)
{
    return sqrt(pow(a.x, 2) + pow(a.y, 2));
}
Coordinate norm(const Coordinate& a)
{
    if (a.x == 0 && a.y == 0)
        return Coordinate(0, 0);
    return a / magnitude(a);
}
bool get_intersection(const Coordinate& ray_ori, const Coordinate& ray_end, const Coordinate& p1, const Coordinate& p2, Coordinate& intersection)
{
    Coordinate dir = ray_end - ray_ori;
    Coordinate v1 = ray_ori - p1;
    Coordinate v2 = p2 - p1;
    Coordinate v3 = Coordinate(-dir.y, dir.x);
    if (v2.dot(v3) == 0)
        return false;
    double t1 = v2.cross(v1) / v2.dot(v3);
    double t2 = v1.dot(v3) / v2.dot(v3);
    if (t1 >= 0 && t2 >= 0 && t2 <= 1)
    {
        intersection = ray_ori + dir * t1;
        return true;
    }
    return false;
}
double get_angled(const Coordinate& p, const Coordinate& c)
{
    double angle = atan2(p.y - c.y, p.x - c.x);
    if (angle < 0)
        angle += 2 * M_PI;
    return angle;
}
bool check_clockwise(const Coordinate& a, const Coordinate& b, const Coordinate& c, const Coordinate& cen)
{
    double angle1 = get_angled(a, cen);
    double angle2 = get_angled(b, cen);
    double angle3 = get_angled(c, cen);
    double diff1 = angle3 - angle1;
    double diff2 = angle3 - angle2;
    if (diff1 < 0)
        diff1 += 2 * M_PI;
    if (diff2 < 0)
        diff2 += 2 * M_PI;
    return diff1 < diff2;
}


#endif // ALGEBRA_HPP