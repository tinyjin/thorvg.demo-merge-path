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
#include "../mergepath.h"
#include "scenario.h"

using clk = std::chrono::high_resolution_clock;
static double elapsed(clk::time_point begin) { return std::chrono::duration<double, std::milli>(clk::now() - begin).count(); }

static RenderPath build(const scenario::Contour& contour)
{
    RenderPath path;
    path.moveTo({contour[0].x0, contour[0].y0});
    for (auto& s : contour) {
        if (s.line) path.lineTo({s.x3, s.y3});
        else path.cubicTo({s.x1, s.y1}, {s.x2, s.y2}, {s.x3, s.y3});
    }
    path.close();
    return path;
}

//the same sampling on both sides, so the areas are comparable
static double area(const RenderPath& path)
{
    std::vector<Point> pts;
    auto p = path.pts.data();
    Point start{}, cur{};
    double sum = 0.0;

    auto flush = [&]() {
        for (size_t i = 0; i < pts.size(); ++i) {
            auto& a = pts[i];
            auto& b = pts[(i + 1) % pts.size()];
            sum += double(a.x) * b.y - double(a.y) * b.x;
        }
        pts.clear();
    };

    for (auto cmd : path.cmds) {
        if (cmd == PathCommand::MoveTo) { flush(); start = cur = *p++; pts.push_back(cur); }
        else if (cmd == PathCommand::LineTo) { cur = *p++; pts.push_back(cur); }
        else if (cmd == PathCommand::CubicTo) {
            compat::Bezier bz{cur, p[0], p[1], p[2]};
            for (uint32_t i = 1; i <= 64; ++i) pts.push_back(bz.at(float(i) / 64.0f));
            cur = p[2];
            p += 3;
        } else if (cmd == PathCommand::Close) { flush(); cur = start; }
    }
    flush();

    return fabs(sum * 0.5);
}


static void report(const char* name, const char* op, double ms, const RenderPath& out)
{
    uint32_t cubics = 0;
    for (auto cmd : out.cmds) if (cmd == PathCommand::CubicTo) ++cubics;
    printf("tvg|%s|%s|%.5f|%zu|%u|%.2f\n", name, op, ms, out.pts.size(), cubics, area(out));
}

int main()
{
    //accumulated union, the stress scene
    for (uint32_t n : {4u, 8u, 12u, 24u, 48u, 96u}) {
        double ms = 0.0;
        RenderPath last;
        uint32_t frames = 0;

        for (uint32_t f = 0; f < 16; ++f) {
            auto spin = float(f) / 16.0f * 2.0f * float(M_PI);
            auto contours = scenario::blobs(n, spin);
            std::vector<RenderPath> paths;
            for (auto& c : contours) paths.push_back(build(c));

            auto begin = clk::now();
            RenderPath acc = paths[0];
            for (uint32_t i = 1; i < n; ++i) {
                RenderPath next;
                if (AddMask(acc, paths[i], next)) acc = next;
            }
            ms += elapsed(begin);
            last = acc;
            ++frames;
        }
        char name[64];
        snprintf(name, sizeof(name), "union of %u blobs", n);
        report(name, "accumulate", ms / frames, last);
    }

    //pairwise, curve against curve at growing size
    for (float r : {90.0f, 360.0f, 1440.0f}) {
        auto a = build(scenario::circle(r * 2.0f, r * 2.0f, r));
        auto b = build(scenario::circle(r * 2.66f, r * 2.0f, r));
        char name[64];
        snprintf(name, sizeof(name), "circle+circle r=%d", int(r));

        struct { const char* op; bool (*fn)(const RenderPath&, const RenderPath&, RenderPath&); } ops[] = {
            {"union", AddMask}, {"intersect", IntersectMask}, {"difference", SubtractMask}, {"xor", DifferenceMask}};

        for (auto& o : ops) {
            RenderPath out;
            auto begin = clk::now();
            for (uint32_t i = 0; i < 200; ++i) { RenderPath tmp; o.fn(a, b, tmp); out = tmp; }
            report(name, o.op, elapsed(begin) / 200.0, out);
        }
    }

    //pairwise, lines against curves
    {
        auto a = build(scenario::star(130.0f, 130.0f, 110.0f, 45.0f, 5));
        auto b = build(scenario::circle(220.0f, 220.0f, 90.0f));

        struct { const char* op; bool (*fn)(const RenderPath&, const RenderPath&, RenderPath&); } ops[] = {
            {"union", AddMask}, {"intersect", IntersectMask}, {"difference", SubtractMask}, {"xor", DifferenceMask}};

        for (auto& o : ops) {
            RenderPath out;
            auto begin = clk::now();
            for (uint32_t i = 0; i < 200; ++i) { RenderPath tmp; o.fn(a, b, tmp); out = tmp; }
            report("star+circle", o.op, elapsed(begin) / 200.0, out);
        }
    }

    return 0;
}
