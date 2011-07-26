
#ifndef H_FINDSHAPES
 #define H_FINDSHAPES

#include <vector>

struct ShapePixel {
    int x, y;
    ShapePixel(int i, int j): x(i), y(j) { }
};

struct Shape {
    std::vector<ShapePixel> pixels;
    unsigned char color[3];
    int cx, cy;  /* center coordinates */
    int radius;
    bool handled;
    Shape() { }
    Shape(const unsigned char[3]);
    void push_back(int i, int j) {
        pixels.push_back(ShapePixel(i,j));
    }
};

std::vector<Shape> find_shapes_in_image(unsigned char (*im)[3], int w, int h);

#endif /* H_FINDSHAPES */
