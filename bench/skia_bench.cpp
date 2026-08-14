/*
 * Copyright (c) 2026 the ThorVG project. All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <cstdio>
#include <chrono>
#include "include/core/SkPath.h"
#include "include/core/SkPathBuilder.h"
#include "include/pathops/SkPathOps.h"
#include "scenario.h"

using clk = std::chrono::high_resolution_clock;
static double elapsed(clk::time_point begin) { return std::chrono::duration<double, std::milli>(clk::now() - begin).count(); }

static SkPath build(const scenario::Contour& contour)
{
    SkPathBuilder builder;
    builder.moveTo(contour[0].x0, contour[0].y0);
    for (auto& s : contour) {
        if (s.line) builder.lineTo(s.x3, s.y3);
        else builder.cubicTo(s.x1, s.y1, s.x2, s.y2, s.x3, s.y3);
    }
    builder.close();
    return builder.detach();
}

//the same sampling on both sides, so the areas are comparable
static double area(const SkPath& path)
{
    std::vector<SkPoint> pts;
    double sum = 0.0;

    auto flush = [&]() {
        for (size_t i = 0; i < pts.size(); ++i) {
            auto& a = pts[i];
            auto& b = pts[(i + 1) % pts.size()];
            sum += double(a.fX) * b.fY - double(a.fY) * b.fX;
        }
        pts.clear();
    };

    SkPath::Iter iter(path, false);
    SkPoint p[4];
    SkPath::Verb verb;
    while ((verb = iter.next(p)) != SkPath::kDone_Verb) {
        if (verb == SkPath::kMove_Verb) { flush(); pts.push_back(p[0]); }
        else if (verb == SkPath::kLine_Verb) pts.push_back(p[1]);
        else if (verb == SkPath::kCubic_Verb) {
            for (uint32_t i = 1; i <= 64; ++i) {
                auto t = float(i) / 64.0f, it = 1.0f - t;
                auto x = it * it * it * p[0].fX + 3 * it * it * t * p[1].fX + 3 * it * t * t * p[2].fX + t * t * t * p[3].fX;
                auto y = it * it * it * p[0].fY + 3 * it * it * t * p[1].fY + 3 * it * t * t * p[2].fY + t * t * t * p[3].fY;
                pts.push_back({x, y});
            }
        } else if (verb == SkPath::kQuad_Verb) {
            for (uint32_t i = 1; i <= 64; ++i) {
                auto t = float(i) / 64.0f, it = 1.0f - t;
                pts.push_back({it * it * p[0].fX + 2 * it * t * p[1].fX + t * t * p[2].fX,
                               it * it * p[0].fY + 2 * it * t * p[1].fY + t * t * p[2].fY});
            }
        } else if (verb == SkPath::kClose_Verb) flush();
    }
    flush();

    return fabs(sum * 0.5);
}


static void report(const char* name, const char* op, double ms, const SkPath& out)
{
    uint32_t cubics = 0;
    SkPath::Iter iter(out, false);
    SkPoint pts[4];
    SkPath::Verb verb;
    while ((verb = iter.next(pts)) != SkPath::kDone_Verb) {
        if (verb == SkPath::kCubic_Verb) ++cubics;
    }
    printf("skia|%s|%s|%.5f|%d|%u|%.2f\n", name, op, ms, out.countPoints(), cubics, area(out));
}

int main()
{
    //accumulated union, the stress scene
    for (uint32_t n : {4u, 8u, 12u, 24u, 48u, 96u}) {
        double ms = 0.0;
        SkPath last;
        uint32_t frames = 0;

        for (uint32_t f = 0; f < 16; ++f) {
            auto spin = float(f) / 16.0f * 2.0f * float(M_PI);
            auto contours = scenario::blobs(n, spin);
            std::vector<SkPath> paths;
            for (auto& c : contours) paths.push_back(build(c));

            auto begin = clk::now();
            SkPath acc = paths[0];
            for (uint32_t i = 1; i < n; ++i) {
                SkPath next;
                if (Op(acc, paths[i], kUnion_SkPathOp, &next)) acc = next;
            }
            ms += elapsed(begin);
            last = acc;
            ++frames;
        }
        char name[64];
        snprintf(name, sizeof(name), "union of %u blobs", n);
        report(name, "accumulate", ms / frames, last);
    }

    struct { const char* op; SkPathOp code; } ops[] = {
        {"union", kUnion_SkPathOp}, {"intersect", kIntersect_SkPathOp},
        {"difference", kDifference_SkPathOp}, {"xor", kXOR_SkPathOp}};

    //pairwise, curve against curve at growing size
    for (float r : {90.0f, 360.0f, 1440.0f}) {
        auto a = build(scenario::circle(r * 2.0f, r * 2.0f, r));
        auto b = build(scenario::circle(r * 2.66f, r * 2.0f, r));
        char name[64];
        snprintf(name, sizeof(name), "circle+circle r=%d", int(r));

        for (auto& o : ops) {
            SkPath out;
            auto begin = clk::now();
            for (uint32_t i = 0; i < 200; ++i) { SkPath tmp; Op(a, b, o.code, &tmp); out = tmp; }
            report(name, o.op, elapsed(begin) / 200.0, out);
        }
    }

    //pairwise, lines against curves
    {
        auto a = build(scenario::star(130.0f, 130.0f, 110.0f, 45.0f, 5));
        auto b = build(scenario::circle(220.0f, 220.0f, 90.0f));

        for (auto& o : ops) {
            SkPath out;
            auto begin = clk::now();
            for (uint32_t i = 0; i < 200; ++i) { SkPath tmp; Op(a, b, o.code, &tmp); out = tmp; }
            report("star+circle", o.op, elapsed(begin) / 200.0, out);
        }
    }

    return 0;
}
