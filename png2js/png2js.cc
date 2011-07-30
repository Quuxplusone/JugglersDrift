
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <string>
#include <vector>
#include "findshapes.h"
#include "readpng.h"

static void do_error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "Error: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
    exit(EXIT_FAILURE);
}

static int BPM = 100;
static int FPS = 20;
static int COUNT = 2;
static double JOINING_SPIN = 0.0;
static bool DO_SPLICE = false;

struct FrameJuggler {
    std::string name;
    double x, y;    /* position */
    double facing;  /* in radians */
    bool is_passing;
    std::string to_whom;
    FrameJuggler(const std::string &s, int cx, int cy):
        name(s), x(cx), y(cy), facing(0.0), is_passing(false) { }

    static bool by_name(const FrameJuggler &a, const FrameJuggler &b) {
        return a.name < b.name;
    }
};

struct Frame {
    std::string filename;
    std::vector<FrameJuggler> jugglers;
    int juggler_radius;
    Frame() { }
    Frame(const char *fname): filename(fname) { }
    FrameJuggler *getJuggler(const std::string &s) {
        for (int i=0; i < (int)jugglers.size(); ++i)
          if (jugglers[i].name == s) return &jugglers[i];
        return NULL;
    }
    int getJugglerIdx(const std::string &s) const {
        for (int i=0; i < (int)jugglers.size(); ++i)
          if (jugglers[i].name == s) return i;
        return -1;
    }
    void center_of_mass(double &cx, double &cy) const;
    void spin_clockwise(double deg, double cx, double cy);
    void slide(double dx, double dy);
    void rotate_jugglers(const std::vector<int> &p);
};

void Frame::center_of_mass(double &cx_, double &cy_) const
{
    double cx = 0, cy = 0;
    const int nJugglers = jugglers.size();
    assert(nJugglers > 0);
    for (int ji = 0; ji < nJugglers; ++ji) {
        cx += jugglers[ji].x;
        cy += jugglers[ji].y;
    }
    cx /= nJugglers;
    cy /= nJugglers;
    cx_ = cx; cy_ = cy;
}

void center_of_mass(const std::vector<Frame> &v, double &cx_, double &cy_)
{
    int n = v.size();
    double cx = 0, cy = 0;
    for (int i=0; i < n; ++i) {
        double dx, dy;
        v[i].center_of_mass(dx, dy);
        cx += dx; cy += dy;
    }
    cx /= n; cy /= n;
    cx_ = cx; cy_ = cy;
}

void Frame::spin_clockwise(double deg, double cx, double cy)
{
    const int nJugglers = jugglers.size();
    for (int ji = 0; ji < nJugglers; ++ji) {
        /* First fix the facing. */
        /* Subtract because we're spinning the pattern clockwise. */
        jugglers[ji].facing -= (deg * M_PI / 180);

        double dx = jugglers[ji].x - cx;
        double dy = jugglers[ji].y - cy;
        double r = sqrt(dx*dx + dy*dy);
        double ang = (r==0.0) ? 0.0 : atan2(dy, dx);
        /* Subtract because we're spinning the pattern clockwise. */
        double new_ang = ang - (deg * M_PI / 180);
        jugglers[ji].x = cx + r*cos(new_ang);
        jugglers[ji].y = cy + r*sin(new_ang);
    }
}

void Frame::slide(double dx, double dy)
{
    const int nJugglers = jugglers.size();
    for (int ji = 0; ji < nJugglers; ++ji) {
        jugglers[ji].x += dx;
        jugglers[ji].y += dy;
    }
}

/* Suppose "p" is {0 2 1} and this frame currently looks like
 * (Alice at (0,0) passing to Bob)
 * (Bob at (10,0) passing to Alice)
 * (Carol at (0,5))
 * Then after this function executes, this frame will look like
 * (Alice at (0,0) passing to Carol)
 * (Bob at (0,5))
 * (Carol at (10,0) passing to Alice)
 */
void Frame::rotate_jugglers(const std::vector<int> &p)
{
    const int nJugglers = jugglers.size();
    assert((int)p.size() == nJugglers);

    std::vector<int> pprime(p.size());
    for (int i=0; i < nJugglers; ++i)
      pprime[p[i]] = i;

    Frame newFrame = *this;
    for (int ji=0; ji < nJugglers; ++ji) {
        /* Get the new position, facing, etc. */
        newFrame.jugglers[ji] = this->jugglers[p[ji]];
        /* ...but keep the old name */
        newFrame.jugglers[ji].name = this->jugglers[ji].name;
        /* Update the name of the person being passed to */
        if (newFrame.jugglers[ji].is_passing) {
            int oi = getJugglerIdx(this->jugglers[p[ji]].to_whom);
            newFrame.jugglers[ji].to_whom = this->jugglers[pprime[oi]].name;
        }
    }
    *this = newFrame;
}

std::vector<Frame> g_Frames;

void printj(const std::vector<Frame> &v)
{
    for (int i=0; i < (int)v.size(); ++i) {
        printf("(Frame %d.)", i);
        const int nJugglers = v[i].jugglers.size();
        for (int j=0; j < nJugglers; ++j) {
            const FrameJuggler &fj = v[i].jugglers[j];
            printf(fj.is_passing ? "  %s (p=%s)" : "  %s", fj.name.c_str(), fj.to_whom.c_str());
        }
        printf("\n");
    }
}

static double mod_360(double x)
{
    while (x < 0.0) x += 360.0;
    while (x >= 360.0) x -= 360.0;
    return x;
}

void process_frame(int t, const char *fname)
{
    unsigned char (*im)[3] = NULL;
    int w, h;
    int rc = ReadPNG(fname, &im, &w, &h);
    if (rc != 0) do_error("Problem reading PNG frame \"%s\"", fname);
    assert(im != NULL);
    assert(w > 0 && h > 0);
    /* Identify the shapes in this frame.  There should be only three
     * different kinds of shapes in any legitimate frame:
     * (1) Jugglers: colored (non-grayscale) disks.
     * (2) Passes: black (#000000) line segments connecting jugglers.
     * (3) Tracery: light grayscale (>#808080) shapes, okay to ignore.
     * The background of the image must be white (#FFFFFF).
     */
    assert(t == (int)g_Frames.size());
    g_Frames.push_back(Frame(fname));
    Frame &this_frame = g_Frames.back();
    /* Find connected regions of the same color.
     * This routine automatically ignores light-gray tracery. */
    std::vector<Shape> all_shapes = find_shapes_in_image(im, w, h);
    free(im);
    /* Classify each remaining shape as a juggler or a pass. */
    double avg_juggler_radius = 0.0;
    for (int i=0; i < (int)all_shapes.size(); ++i) {
        Shape &sh = all_shapes[i];
        if (sh.color[0] || sh.color[1] || sh.color[2]) {
            /* Is this shape non-black? If so, its center of mass is the position
             * of a juggler named <#color>. For sanity's sake, check that the
             * shape is roughly circular, i.e. that its population is roughly
             * pi-r-squared. TODO FIXME BUG HACK */
            char jname[10];
            sprintf(jname, "#%02X%02X%02X", sh.color[0], sh.color[1], sh.color[2]);
            this_frame.jugglers.push_back(FrameJuggler(jname, sh.cx, sh.cy));
            avg_juggler_radius += sh.radius;
            sh.handled = true;
        }
    }
    if (this_frame.jugglers.empty())
      do_error("No jugglers found in frame \"%s\"!", fname);
    avg_juggler_radius /= this_frame.jugglers.size();
    this_frame.juggler_radius = avg_juggler_radius;
    for (int si=0; si < (int)all_shapes.size(); ++si) {
        Shape &sh = all_shapes[si];
        if (!sh.color[0] && !sh.color[1] && !sh.color[2]) {
            /* Is this shape black? If so, it ought to be a line segment roughly
             * joining two jugglers. */
            double ang = 0, ang2 = 0;
            int count = 0;
            for (int i=0; i < (int)sh.pixels.size(); ++i) {
                double dy = sh.pixels[i].y - sh.cy;
                double dx = sh.pixels[i].x - sh.cx;
                if (dx*dx + dy*dy <= sh.radius*sh.radius / 2.0) continue;
                double a = atan2(dy, dx);
                a += 1.0;  // TODO FIXME BUG HACK
                double a2 = a+1.0;
                while (a < 0) a += M_PI;
                while (a > M_PI) a -= M_PI;
                while (a2 < 0) a2 += M_PI;
                while (a2 > M_PI) a2 -= M_PI;
                ang += a;
                ang2 += a2;
                count += 1;
            }
            ang /= count; ang -= 1.0;
            ang2 /= count; ang2 -= 2.0;
            if (fabs(ang - M_PI/2) < 0.1) { fputs("Using ang2.\n", stderr); ang = ang2; }
#if 0
fprintf(stderr, "%s: black line at (%d,%d), ang=%.1f (ang2=%.1f)\n", fname,
    sh.cx, sh.cy, ang*180/M_PI, ang2*180/M_PI);
#endif
            /* Now "ang" should be the angle of the line segment. See which
             * juggler is nearest along each of the rays heading outward from
             * (cx,cy) in the directions given by "ang" and "ang+pi". */
            FrameJuggler *closest_pos_juggler = NULL;
            double min_pos_u = 0.0;
            FrameJuggler *closest_neg_juggler = NULL;
            double min_neg_u = 0.0;
            const double x1 = sh.cx, y1 = sh.cy;
            const double x2 = sh.cx + 100*cos(ang), y2 = sh.cy + 100*sin(ang);
            const double d12sq = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
            for (int i=0; i < (int)this_frame.jugglers.size(); ++i) {
                FrameJuggler &jug = this_frame.jugglers[i];
                const double x3 = jug.x, y3 = jug.y;
                const double u = ((x3 - x1)*(x2 - x1) + (y3 - y1)*(y2 - y1)) / d12sq;
                const double xi = x1 + u*(x2-x1);
                const double yi = y1 + u*(y2-y1);
                /* (xi, yi) is the point along "ang" closest to "jug". */
                const double dj2 = (xi-x3)*(xi-x3) + (yi-y3)*(yi-y3);
                if (dj2 < avg_juggler_radius*avg_juggler_radius) {
                    /* The line passes through this juggler. */
                    if (u > 0 && (closest_pos_juggler == NULL || u < min_pos_u)) {
                        min_pos_u = u;
                        closest_pos_juggler = &jug;
                    } else if (u < 0 && (closest_neg_juggler == NULL || u > min_neg_u)) {
                        min_neg_u = u;
                        closest_neg_juggler = &jug;
                    }
                }
            }
            if (closest_pos_juggler == NULL && closest_neg_juggler == NULL) {
                do_error("One of the black lines in \"%s\" doesn't connect any jugglers!", fname);
            } else if (closest_pos_juggler == NULL) {
                do_error("One of the black lines in \"%s\" connects juggler %s to empty space!",
                    fname, closest_neg_juggler->name.c_str());
            } else if (closest_neg_juggler == NULL) {
                do_error("One of the black lines in \"%s\" connects juggler %s to empty space!",
                    fname, closest_pos_juggler->name.c_str());
            } else if (closest_pos_juggler->is_passing) {
                do_error("In \"%s\", juggler %s is passing to %s and %s simultaneously!",
                    fname, closest_pos_juggler->name.c_str(),
                    closest_pos_juggler->to_whom.c_str(), closest_neg_juggler->name.c_str());
            } else if (closest_neg_juggler->is_passing) {
                do_error("In \"%s\", juggler %s is passing to %s and %s simultaneously!",
                    fname, closest_neg_juggler->name.c_str(),
                    closest_neg_juggler->to_whom.c_str(), closest_pos_juggler->name.c_str());
            } else {
                closest_pos_juggler->is_passing = true;
                closest_pos_juggler->to_whom = closest_neg_juggler->name;
                closest_neg_juggler->is_passing = true;
                closest_neg_juggler->to_whom = closest_pos_juggler->name;
            }
            sh.handled = true;
        }
    }
    /* Lastly, if we didn't figure out what to do with every single shape in
     * this frame, then give an error. */
    int unhandled_count = 0;
    for (int si=0; si < (int)all_shapes.size(); ++si) {
        Shape &sh = all_shapes[si];
        if (!sh.handled) ++unhandled_count;
    }
    if (unhandled_count != 0) {
        do_error("Frame \"%s\" contains %d extraneous shape%s!", fname, unhandled_count,
            &"s"[unhandled_count!=1]);
    }
}

void sort_and_sanity_check()
{
    assert(g_Frames.size() >= 2);
    /* In each frame, sort the jugglers by name; this will make it easier to
     * refer to the same juggler across multiple frames. */
    for (int fi = 0; fi < (int)g_Frames.size(); ++fi) {
        std::sort(g_Frames[fi].jugglers.begin(),
                  g_Frames[fi].jugglers.end(), FrameJuggler::by_name);
    }
    /* Then, make sure we have the same number of jugglers in each frame. */
    const int nJugglers = g_Frames[0].jugglers.size();
    for (int fi = 0; fi < (int)g_Frames.size(); ++fi) {
        const Frame &f = g_Frames[fi];
        if ((int)f.jugglers.size() != nJugglers) {
            do_error("Frame \"%s\" has %d jugglers instead of %d!",
                f.filename.c_str(), (int)f.jugglers.size(), nJugglers);
        }
        for (int ji = 0; ji < nJugglers; ++ji) {
            if (f.jugglers[ji].name != g_Frames[0].jugglers[ji].name) {
                do_error("Frame \"%s\" has no juggler named %s!",
                    f.filename.c_str(), g_Frames[0].jugglers[ji].name.c_str());
            }
        }
        if (abs(g_Frames[0].juggler_radius - f.juggler_radius) > 2) {
            do_error("Frames \"%s\" and \"%s\" are at different scales!",
                g_Frames[0].filename.c_str(), f.filename.c_str());
        }
    }
}


std::vector<int> deduce_permutation(const std::vector<Frame> &v)
{
    /* Spin [0] around its center of mass. Then translate it until
     * its center of mass matches up with the center of mass of [n-1].
     * Then the individual jugglers' positions ought to match up pretty
     * close to exactly, so we can then deduce the permutation in
     * "nearest_jugglers[]" below. */
    Frame nthFrame = v[0];
    double cx, cy;
    nthFrame.center_of_mass(cx, cy);
    nthFrame.spin_clockwise(JOINING_SPIN, cx, cy);
    double new_cx, new_cy;
    v.back().center_of_mass(new_cx, new_cy);
    nthFrame.slide(new_cx - cx, new_cy - cy);

    const int nJugglers = v[0].jugglers.size();
    std::vector<int> perm(nJugglers);
    for (int i=0; i < nJugglers; ++i) {
        const double ox = g_Frames.back().jugglers[i].x;
        const double oy = g_Frames.back().jugglers[i].y;
        int nearest_idx = -1;
        double nearest_d2 = 0.0;
        for (int j=0; j < nJugglers; ++j) {
            const double dx = nthFrame.jugglers[j].x - ox;
            const double dy = nthFrame.jugglers[j].y - oy;
            if (nearest_idx == -1 || (dx*dx+dy*dy < nearest_d2)) {
                nearest_idx = j;
                nearest_d2 = dx*dx+dy*dy;
            }
        }
        assert(nearest_idx != -1);
        perm[i] = nearest_idx;
    }

    for (int i=0; i < nJugglers; ++i) {
        for (int j=i+1; j < nJugglers; ++j) {
            if (perm[i] == perm[j]) {
                do_error("It's ambiguous which of %s and %s should map to %s's original position.",
                    v[0].jugglers[i].name.c_str(), v[0].jugglers[j].name.c_str(),
                    nthFrame.jugglers[perm[i]].name.c_str());
            }
        }
    }

    return perm;
}

void cross(const double a[3], const double b[3], double result[3])
{
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

void intersectLines(const double p1[2], const double v1[2],
                    const double p2[2], const double v2[2],
                    double result[2])
{
    double u = v2[0]*(p1[1]-p2[1]) - v2[1]*(p1[0]-p2[0]);
    u /= (v2[1]*v1[0] - v2[0]*v1[1]);
    result[0] = p1[0] + u*v1[0];
    result[1] = p1[1] + u*v1[1];
}

void deduce_center_of_rotation(const std::vector<Frame> &v,
                               const std::vector<int> &p,
                               double &cx, double &cy)
{
#if 0
    double p1before[3] = { v[0].jugglers[0].x, v[0].jugglers[0].y, 0.0 };
    double p2before[3] = { v[0].jugglers[1].x, v[0].jugglers[1].y, 0.0 };
    double p1after[3] = { v.back().jugglers[p[0]].x, v.back().jugglers[p[0]].y, 0.0 };
    double p2after[3] = { v.back().jugglers[p[1]].x, v.back().jugglers[p[1]].y, 0.0 };

    double v1[3] = { p1after[0]-p1before[0], p1after[1]-p1before[1], 0.0 };
    double v2[3] = { p2after[0]-p2before[0], p2after[1]-p2before[1], 0.0 };
printf("v1=%f %f %f\n", v1[0], v1[1], v1[2]);
printf("v2=%f %f %f\n", v2[0], v2[1], v2[2]);

    if (v1[0] == 0.0 && v1[1] == 0.0) {
        cx = p1before[0];
        cy = p1before[1];
        return;
    } else if (v2[0] == 0.0 && v2[1] == 0.0) {
        cx = p2before[0];
        cy = p2before[1];
        return;
    }

    double vn[3] = { 0.0, 0.0, 1.0 };
    double vx1[3]; cross(v1, vn, vx1);
    double vx2[3]; cross(v2, vn, vx2);
printf("vx1=%f %f %f\n", vx1[0], vx1[1], vx1[2]);
printf("vx2=%f %f %f\n", vx2[0], vx2[1], vx2[2]);
    double result[2];
    intersectLines(p1after, vx1, p2after, vx2, result);
    cx = result[0];
    cy = result[1];
printf("cx=%.1f cy=%.1f\n", cx, cy);
#else
    (void)p;
    v[0].center_of_mass(cx,cy);
#endif
}

void splice_pattern()
{
    const int nJugglers = g_Frames[0].jugglers.size();

    /* The user might have provided us with only half of the pattern, or
     * even only 1/nJugglers'th of the pattern. For each juggler in the
     * final frame, check what's the nearest juggler in the first frame.
     * If everybody matches up, then great, we've got the whole pattern;
     * but if there are any cycles in the nearest-juggler graph, then
     * rotate each of those cycles and splice the rotated pattern onto
     * the end of this one. */

    std::vector<int> nearest_juggler = deduce_permutation(g_Frames);
    assert((int)nearest_juggler.size() == nJugglers);

    /* Okay, now figure out how many times we're going to have to repeat the
     * original sequence in order that all of the cycles in nearest_juggler[]
     * cycle back to the beginning. */
    std::vector<int> oj(nJugglers), nj(nJugglers);
    for (int i=0; i < nJugglers; ++i)
      oj[i] = i;
    int total_rotations = 0;
    while (true) {
        ++total_rotations;
        bool back_to_start = true;
        for (int i=0; i < nJugglers; ++i) {
            nj[i] = oj[nearest_juggler[i]];
            if (nj[i] != i)
              back_to_start = false;
        }
        if (back_to_start) break;
        oj = nj;
    }

    /* We'll need to splice the sequence at least "total_rotations" times,
     * just in order to get the jugglers back to their original roles.
     * It's possible that after "total_rotations" splices, the entire pattern
     * will still be spun relative to its original physical position, so
     * we might need to take *that* and splice it. We'll arbitrarily say that
     * we're only going to spin-and-splice it up to 6 times. */
    int total_spins = -1;
    double final_spin = mod_360(total_rotations * JOINING_SPIN);
    for (int i=1; i <= 6; ++i) {
        for (int j=0; j < i; ++j) {
            double expected = mod_360(j*(360.0/i));
            if (fabs(final_spin - expected) < 2.0) {
                total_spins = i;
                goto done;
            }
        }
    }
  done:
    if (total_spins <= 0)
      do_error("We can only handle up to 5 spin-splices; maybe try a different --join= argument?");

#if 0
printf("total_rotations=%d\n", total_rotations);
printf("total_spins=%d\n", total_spins);
printf("Original sequence:\n");
printj(g_Frames);
#endif

    double cx, cy;
    deduce_center_of_rotation(g_Frames, nearest_juggler, cx, cy);

    /* Rotate-and-splice the pattern "total_rotations" times. */
  {
    std::vector<int> currentPerm = nearest_juggler;
    std::vector<Frame> originalFrames = g_Frames;
    for (int i=1; i < total_rotations; ++i) {
        std::vector<Frame> v = originalFrames;
        for (int t=0; t < (int)originalFrames.size(); ++t) {
            v[t].spin_clockwise(i*JOINING_SPIN, cx, cy);
            v[t].rotate_jugglers(currentPerm);
        }
        g_Frames.insert(g_Frames.end(), v.begin()+1, v.end());
        std::vector<int> newPerm(nJugglers);
        for (int j=0; j < nJugglers; ++j)
          newPerm[j] = nearest_juggler[currentPerm[j]];
        currentPerm = newPerm;
    }
  }

#if 0
printf("Rotated-and-spliced sequence:\n");
printj(g_Frames);
#endif

    /* Now spin-and-splice the pattern "total_spins" times. */
  {
    std::vector<Frame> originalFrames = g_Frames;
    for (int i=1; i < total_spins; ++i) {
        std::vector<Frame> v = originalFrames;
        for (int t=0; t < (int)originalFrames.size(); ++t)
          v[t].spin_clockwise(i*total_rotations*JOINING_SPIN, cx, cy);
        g_Frames.insert(g_Frames.end(), v.begin()+1, v.end());
    }
  }

  /* And remove the last frame, so that the final transition from
   * [n-1] back to [0] works smoothly. */
  g_Frames.resize(g_Frames.size()-1);

#if 0
printf("Spun-and-spliced sequence:\n");
printj(g_Frames);
#endif

}

void infer_facings()
{
    /* Now we can go interpolating facings. For each juggler, we break up
     * his routine into "feeding" sequences, where he's passing in every
     * frame of the sequence, and "repositioning" sequences, where he's
     * not passing.  For each frame of a "feeding" sequence, we aim him at
     * the average of all the feedees' positions.
     * For each frame which is not part of a "feeding" sequence, we aim
     * him by linear interpolation between the start and end facings.
     * TODO FIXME BUG HACK: For now, just aim him where he's passing. */
    const int N = g_Frames.size();
    const int nJugglers = g_Frames[0].jugglers.size();
    for (int ji=0; ji < nJugglers; ++ji) {
        bool ever_passed = false;
        for (int t=0; t < N; ++t) {
            if (g_Frames[t].jugglers[ji].is_passing) {
                const std::string &who = g_Frames[t].jugglers[ji].to_whom;
                const FrameJuggler *target = g_Frames[t].getJuggler(who);
                assert(target != NULL);
                assert(target != &g_Frames[t].jugglers[ji]);
                const double dx = target->x - g_Frames[t].jugglers[ji].x;
                const double dy = target->y - g_Frames[t].jugglers[ji].y;
                g_Frames[t].jugglers[ji].facing = atan2(dy, dx);
                ever_passed = true;
            }
        }
        if (!ever_passed)
          do_error("Juggler %s has no passes!", g_Frames[0].jugglers[ji].name.c_str());
        for (int t=0; t < N; ++t) {
            if (!g_Frames[t].jugglers[ji].is_passing) {
                /* Go backward to find the last time he passed. */
                int a=1;
                while (!g_Frames[(t-a + N) % N].jugglers[ji].is_passing) ++a;
                /* Go forward to find the next time he'll pass. */
                int b=1;
                while (!g_Frames[(t+b) % N].jugglers[ji].is_passing) ++b;
                /* Interpolate. */
                const FrameJuggler &ja = g_Frames[(t-a + N) % N].jugglers[ji];
                const FrameJuggler &jb = g_Frames[(t+b) % N].jugglers[ji];
                double rdiff = (jb.facing - ja.facing);
                while (rdiff < 0) rdiff += 2*M_PI;
                if (rdiff > M_PI) rdiff -= 2*M_PI;
                g_Frames[t].jugglers[ji].facing = ja.facing + (a*rdiff / (a+b));
            }
        }
    }
}

void output()
{
    const int nJugglers = g_Frames[0].jugglers.size();
    const int nBeats = g_Frames.size();

    printf("var PatternLengthInBeats = %d;\n", nBeats*COUNT);
    printf("var JugglerSize = %d;\n", (int)g_Frames[0].juggler_radius);
    printf("var Jugglers = [\n");
    for (int ji=0; ji < nJugglers; ++ji) {
        printf(" { name: '%s',\n", g_Frames[0].jugglers[ji].name.c_str());
        printf("     ts: [");
        for (int t=0; t < nBeats+1; ++t)
          printf("%d, ", t*COUNT);
        printf(" ],\n");
        printf("      x: [");
        for (int t=0; t < nBeats+1; ++t)
          printf("%1.1f, ", g_Frames[t%nBeats].jugglers[ji].x);
        printf(" ],\n");
        printf("      y: [");
        for (int t=0; t < nBeats+1; ++t)
          printf("%1.1f, ", g_Frames[t%nBeats].jugglers[ji].y);
        printf(" ],\n");
        printf("      fa: [");
        for (int t=0; t < nBeats+1; ++t)
          printf("%1.1f, ", g_Frames[t%nBeats].jugglers[ji].facing);
        printf(" ] },\n");
    }
    printf("];\n");
    printf("var Passes = [\n");
    for (int t=0; t < nBeats+1; ++t) {
        for (int ji=0; ji < nJugglers; ++ji) {
            const FrameJuggler &j = g_Frames[t%nBeats].jugglers[ji];
            if (j.is_passing) {
                const int who = g_Frames[t%nBeats].getJugglerIdx(j.to_whom);
                assert(who != -1);
                printf("  { start: %d, end: %f, from: %d, to: %d, hand: '%c' },\n",
                    COUNT*t, COUNT*t+1.3, ji, who,
                    ((COUNT*t % 2) ? 'l' : 'r'));
            } else {
                /* Explicitly insert a self. */
                printf("  { start: %d, end: %f, from: %d, to: %d, hand: '%c' },\n",
                    COUNT*t, COUNT*t+1.3, ji, ji,
                    ((COUNT*t % 2) ? 'l' : 'r'));
            }
        }
    }
    /* Explicitly insert the off-count selfs. */
    for (int t=0; t < COUNT*(nBeats+1); ++t) {
        if (t % COUNT == 0) continue;
        for (int ji=0; ji < nJugglers; ++ji) {
            printf("  { start: %d, end: %f, from: %d, to: %d, hand: '%c' },\n",
                t, t+1.3, ji, ji, ((t % 2) ? 'l' : 'r'));
        }
    }
    printf("];\n");
    printf("BPM=%d;\n", BPM);
    printf("FPS=%d;\n", FPS);
}

void do_help()
{
    puts("Usage: png2js [--options] frame1.png frame2.png [...] > pattern.js");
    puts("Converts a series of PNG frames into a Jugglers' Drift input file.");
    printf("  --count=%-4d Insert <count-1> selfs between each pair of PNG frames.\n", COUNT);
    printf("  --join=%-5.1f Before splicing the ends of the pattern, rotate it <n> degrees.\n", JOINING_SPIN);
    printf("  --bpm=%-6d Set beats-per-minute in the output file.\n", BPM);
    printf("  --fps=%-6d Set frames-per-second in the output file.\n", FPS);
    exit(0);
}

int main(int argc, char **argv)
{
    int i;
    for (i=1; i < argc; ++i) {
        if (argv[i][0] != '-') break;
        if (!strcmp(argv[i], "--")) { ++i; break; }
        if (!strncmp(argv[i], "--bpm=", 6)) {
            BPM = atoi(argv[i]+6);
        } else if (!strncmp(argv[i], "--fps=", 6)) {
            FPS = atoi(argv[i]+6);
        } else if (!strncmp(argv[i], "--count=", 8)) {
            COUNT = atoi(argv[i]+8);
        } else if (!strncmp(argv[i], "--join=", 7)) {
            JOINING_SPIN = mod_360(strtod(argv[i]+7, NULL));
            DO_SPLICE = true;
        } else {
            do_help();
        }
    }
    if (i == argc) {
        do_error("No input images provided!");
    } else if (i == argc-1) {
        do_error("Only one input image provided!");
    }
    for (int t = 0; i+t < argc; ++t) {
        process_frame(t, argv[i+t]);
    }
    sort_and_sanity_check();
    if (DO_SPLICE)
      splice_pattern();
    infer_facings();
    output();
    return 0;
}
