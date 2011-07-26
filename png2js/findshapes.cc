
#include <assert.h>
#include <math.h>
#include <string.h>
#include <map>
#include <vector>
#include "findshapes.h"

Shape::Shape(const unsigned char c[3]): handled(false)
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

#include <stdio.h>
std::vector<Shape> find_shapes_in_image(unsigned char (*im)[3], int w, int h)
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
    std::vector<Shape> rv;
    for (std::map<int,Shape>::const_iterator it = shapes.begin(); it != shapes.end(); ++it)
      rv.push_back(it->second);
    /* From here on, use "rv" instead of "shapes". */
    shapes.clear();
    /* Compute center of mass for each shape. */
    for (int r=0; r < (int)rv.size(); ++r) {
        Shape &sh = rv[r];
        double cx = 0, cy = 0;
        for (int i=0; i < (int)sh.pixels.size(); ++i) {
            cx += sh.pixels[i].x;
            cy += sh.pixels[i].y;
        }
        sh.cx = (cx + 0.5) / sh.pixels.size();
        sh.cy = (cy + 0.5) / sh.pixels.size();
        double maxd2 = 0.0;
        for (int i=0; i < (int)sh.pixels.size(); ++i) {
            double dx = (sh.cx - sh.pixels[i].x);
            double dy = (sh.cy - sh.pixels[i].y);
            double d2 = dx*dx + dy*dy;
            if (d2 > maxd2)
              maxd2 = d2;
        }
        sh.radius = sqrt(maxd2);
    }
    return rv;
}    

