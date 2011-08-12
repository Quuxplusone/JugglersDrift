
#ifndef H_FINDSHAPES
 #define H_FINDSHAPES

#include <vector>

enum ShapeKind {
    SK_UNKNOWN,
    SK_CIRCLE,
    SK_LINE_SEGMENT
};

struct ShapePixel {
    int x, y;
    ShapePixel(int i, int j): x(i), y(j) { }
};

struct Shape {
    std::vector<ShapePixel> pixels;
    unsigned char color[3];
    double cx, cy;  /* center coordinates */

    virtual ShapeKind kind() { return SK_UNKNOWN; }
    bool is_black() const { return !(color[0] || color[1] || color[2]); }

    Shape() { }
    Shape(const unsigned char[3]);
    void push_back(int i, int j) {
        pixels.push_back(ShapePixel(i,j));
    }
};

struct UnknownShape : public Shape {
    UnknownShape(const Shape &sh, int x, int y): Shape(sh) { cx = x; cy = y; }
};

struct Circle : public Shape {
    virtual ShapeKind kind() { return SK_CIRCLE; }
    double radius;
    Circle(const Shape &sh, int x, int y, int r): Shape(sh), radius(r) { cx = x; cy = y; }
};

struct LineSegment : public Shape {
    virtual ShapeKind kind() { return SK_LINE_SEGMENT; }
    double angle;
    bool directed;
    LineSegment(const Shape &sh, int x, int y, double a, bool d):
        Shape(sh), angle(a), directed(d) { cx = x; cy = y; }
};

std::vector<Shape*> find_shapes_in_image(unsigned char (*im)[3], int w, int h);

#endif /* H_FINDSHAPES */
