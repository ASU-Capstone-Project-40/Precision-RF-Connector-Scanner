#ifndef XYZ_H
#define XYZ_H

struct XY {
    XY(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    double x;
    double y;

    bool inBounds(XY lower, XY upper)
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

XY operator+ (const XY& a, const XY& b) {
    return XY(a.x + b.x, a.y + b.y);
}

XY operator- (const XY& a, const XY& b) {
    return XY(a.x - b.x, a.y - b.y);
}

XY operator- (const XY& p) {
    return XY(-p.x, -p.y);
}

XY operator* (const XY& p, double scalar) {
    return XY(p.x * scalar, p.y * scalar);
}

XY operator/ (const XY& p, double scalar) {
    return XY(p.x / scalar, p.y / scalar);
}

#endif // XYZ_H