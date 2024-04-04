#ifndef POINT_H
#define POINT_H

struct Point {
    Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    double x;
    double y;

    bool inBounds(Point lower, Point upper)
    {
        if (x >= lower.x && x <= upper.x && y >= lower.y && y <= upper.y)
            return true;
        return false;
    }

    std::string toString() {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }

    double magnitude() {
        return std::sqrt(x*x + y*y);
    }
};

Point operator+ (const Point& a, const Point& b) {
    return Point(a.x + b.x, a.y + b.y);
}

Point operator- (const Point& a, const Point& b) {
    return Point(a.x - b.x, a.y - b.y);
}

Point operator- (const Point& p) {
    return Point(-p.x, -p.y);
}

Point operator* (const Point& p, double scalar) {
    return Point(p.x * scalar, p.y * scalar);
}

Point operator/ (const Point& p, double scalar) {
    return Point(p.x / scalar, p.y / scalar);
}

#endif // POINT_H