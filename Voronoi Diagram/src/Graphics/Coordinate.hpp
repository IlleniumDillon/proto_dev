#ifndef COORDINATE_HPP
#define COORDINATE_HPP

class Coordinate
{
public:
    double x, y;
public:
    Coordinate(double x, double y) : x(x), y(y) {}
    Coordinate() : x(0), y(0) {}
    Coordinate operator+(const Coordinate& other) const
    {
        return Coordinate(x + other.x, y + other.y);
    }
    Coordinate operator-(const Coordinate& other) const
    {
        return Coordinate(x - other.x, y - other.y);
    }
    Coordinate operator*(double scalar) const
    {
        return Coordinate(x * scalar, y * scalar);
    }
    Coordinate operator/(double scalar) const
    {
        return Coordinate(x / scalar, y / scalar);
    }

    double cross(const Coordinate& other) const
    {
        return x * other.y - y * other.x;
    }
    double dot(const Coordinate& other) const
    {
        return x * other.x + y * other.y;
    }
};

#endif // COORDINATE_HPP