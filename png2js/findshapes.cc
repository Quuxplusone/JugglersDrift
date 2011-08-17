
#include <assert.h>
#include <math.h>
#include <string.h>
#include <map>
#include <vector>
#include "findshapes.h"

Shape::Shape(const unsigned char c[3])
{
    memcpy(color, c, 3);
}


struct UnionFind {
    int *nodes, *pop;
    int cap;
    UnionFind(int cap_);
    ~UnionFind() { delete [] nodes; delete [] pop; }
    void merge(int a, int b);
    int findparent(int a);
};

UnionFind::UnionFind(int cap_): cap(cap_)
{
    assert(cap_ > 0);
    nodes = new int[cap_];
    pop = new int[cap_];
    assert(nodes != NULL);
    assert(pop != NULL);
    for (int i=0; i < cap_; ++i) {
        nodes[i] = i;
        pop[i] = 1;
    }
}

void UnionFind::merge(int a, int b)
{
    assert(0 <= a && a < b && b < cap);
    int pa = findparent(a);
    int pb = findparent(b);
    assert(0 <= pa && pa < cap);
    assert(0 <= pb && pb < cap);
    assert(pop[pa] >= 1);
    assert(pop[pb] >= 1);
    if (pa == pb) return;
    if (pa > pb) {
        int t = pa; pa = pb; pb = t;
    }
    nodes[pb] = pa;
    pop[pa] += pop[pb];
    pop[pb] = 0;
    assert(2 <= pop[pa] && pop[pa] <= cap);
}

int UnionFind::findparent(int a)
{
    assert(0 <= a && a < cap);
    while (a != nodes[a]) {
        a = nodes[a];
        assert(0 <= a && a < cap);
    }
    return a;
}


Shape *copy_and_classify(const Shape &sh)
{
    /* Compute center of mass. */
    double cx = 0, cy = 0;
    for (int i=0; i < (int)sh.pixels.size(); ++i) {
        cx += sh.pixels[i].x;
        cy += sh.pixels[i].y;
    }
    cx = (cx + 0.5) / sh.pixels.size();
    cy = (cy + 0.5) / sh.pixels.size();
    double maxd2 = 0.0;
    for (int i=0; i < (int)sh.pixels.size(); ++i) {
        double dx = (cx - sh.pixels[i].x);
        double dy = (cy - sh.pixels[i].y);
        double d2 = dx*dx + dy*dy;
        if (d2 > maxd2)
          maxd2 = d2;
    }
    const double radius = sqrt(maxd2);
    const double diameter = 2*radius;
    const double area = sh.pixels.size();

    if (area > 3*radius*radius) {
        return new Circle(sh, cx, cy, radius);
    }

    if (area < 6*diameter) {
        /* Since the shape is connected, area must be at least proportional
         * to radius; but if it's consistent with a 6*D rectangle, then
         * we're probably dealing with a line segment. */
        /* Find the farthest pixel from the center. */
        const int npixels = sh.pixels.size();
        assert(npixels > 0);
        double max_d2 = 0.0;
        double max_x = 0.0, max_y = 0.0;
        for (int i=0; i < npixels; ++i) {
            const double dx = sh.pixels[i].x - cx;
            const double dy = sh.pixels[i].y - cy;
            const double d2 = dx*dx + dy*dy;
            if (d2 > max_d2) {
                max_d2 = d2;
                max_x = sh.pixels[i].x;
                max_y = sh.pixels[i].y;
            }
        }
        assert(max_d2 > 0.0);
        /* Find the farthest pixel from the one we just found.
         * It'll be the other end of the line. */
        double diameter2 = 0.0;
        double other_max_x = 0.0, other_max_y = 0.0;
        for (int i=0; i < npixels; ++i) {
            const double dx = sh.pixels[i].x - max_x;
            const double dy = sh.pixels[i].y - max_y;
            const double d2 = dx*dx + dy*dy;
            if (d2 > diameter2) {
                diameter2 = d2;
                other_max_x = sh.pixels[i].x;
                other_max_y = sh.pixels[i].y;
            }
        }
        assert(diameter2 > 0.0);
        /* The line represented by this shape ought to pass through
         * the two pixels we just found. */
        double ang = atan2(max_y - other_max_y, max_x - other_max_x);
        /* Adjust the center, too. */
        cx = (max_x + other_max_x)/2;
        cy = (max_y + other_max_y)/2;
        /* If the segment has a heavier knob on one end (i.e., if one
         * endpoint is significantly closer to the original center of mass
         * than the other endpoint), then we should treat it as an arrow
         * pointing in the heavier (i.e., shorter) direction. */
        if (sqrt(max_d2) > 0.5*sqrt(diameter2) + 2) {
            ang += ((ang > M_PI) ? -M_PI : M_PI);
            return new LineSegment(sh, cx, cy, ang, /*directed=*/true);
        } else if (sqrt(max_d2) < 0.5*sqrt(diameter2) - 2) {
            return new LineSegment(sh, cx, cy, ang, /*directed=*/true);
        } else {
            return new LineSegment(sh, cx, cy, ang, /*directed=*/false);
        }
    }

    return new UnknownShape(sh, cx, cy);
}


std::vector<Shape*> find_shapes_in_image(unsigned char (*im)[3], int w, int h)
{
    UnionFind uf(w*h);
    for (int j=0; j < h-1; ++j) {
        for (int i=0; i < w-1; ++i) {
            if (memcmp(im[j*w+i], im[j*w+(i+1)], 3) == 0)
              uf.merge(j*w+i, j*w+(i+1));
            if (memcmp(im[j*w+i], im[(j+1)*w+i], 3) == 0)
              uf.merge(j*w+i, (j+1)*w+i);
        }
    }
    std::map<int, Shape> shapes;
    for (int j=0; j < h; ++j) {
        for (int i=0; i < w; ++i) {
            int n = j*w+i;
            unsigned char (&color)[3] = im[n];
            if (color[0] == color[1] && color[1] == color[2] && color[0] >= 128) {
                /* It's grayscale and light; ignore it.
                 * This also includes the white background region. */
            } else {
                int pn = uf.findparent(n);
                if (pn == n)
                  shapes[pn] = Shape(color);
                shapes[pn].push_back(i,j);
            }
        }
    }
    /* Convert the map to a simple vector of Shapes. */
    std::vector<Shape*> rv;
    for (std::map<int,Shape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
      rv.push_back(copy_and_classify(it->second));
    /* From here on, use "rv" instead of "shapes". */
    shapes.clear();
    return rv;
}    

