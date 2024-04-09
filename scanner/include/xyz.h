#ifndef XYZ_H
#define XYZ_H

struct XYZ {
    XYZ(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    double x;
    double y;

    bool inBounds(XYZ lower, XYZ upper)
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

XYZ operator+ (const XYZ& a, const XYZ& b) {
    return XYZ(a.x + b.x, a.y + b.y);
}

XYZ operator- (const XYZ& a, const XYZ& b) {
    return XYZ(a.x - b.x, a.y - b.y);
}

XYZ operator- (const XYZ& p) {
    return XYZ(-p.x, -p.y);
}

XYZ operator* (const XYZ& p, double scalar) {
    return XYZ(p.x * scalar, p.y * scalar);
}

XYZ operator/ (const XYZ& p, double scalar) {
    return XYZ(p.x / scalar, p.y / scalar);
}

#endif // XYZ_H