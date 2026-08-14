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

/* The operands of the benchmark, as plain numbers.

   Both harnesses build their own path type out of these, so the two engines get
   the very same geometry down to the last coordinate. A segment is a cubic
   unless it is flagged as a line - a line is handed over as a line, since both
   engines treat it natively. */

#ifndef _BENCH_SCENARIO_H_
#define _BENCH_SCENARIO_H_

#include <vector>
#include <cmath>

namespace scenario
{

struct Segment
{
    float x0, y0, x1, y1, x2, y2, x3, y3;
    bool line;
};

using Contour = std::vector<Segment>;

constexpr float KAPPA = 0.552284f;

//a closed cubic path through the sampled points (catmull-rom to bezier)
inline Contour smooth(const std::vector<std::pair<float, float>>& pts)
{
    Contour out;
    auto cnt = pts.size();
    for (size_t i = 0; i < cnt; ++i) {
        auto& p0 = pts[(i + cnt - 1) % cnt];
        auto& p1 = pts[i];
        auto& p2 = pts[(i + 1) % cnt];
        auto& p3 = pts[(i + 2) % cnt];
        out.push_back({p1.first, p1.second,
                       p1.first + (p2.first - p0.first) / 6.0f, p1.second + (p2.second - p0.second) / 6.0f,
                       p2.first - (p3.first - p1.first) / 6.0f, p2.second - (p3.second - p1.second) / 6.0f,
                       p2.first, p2.second, false});
    }
    return out;
}

inline Contour blob(float cx, float cy, float r, float phase)
{
    std::vector<std::pair<float, float>> pts;
    for (uint32_t i = 0; i < 10; ++i) {
        auto t = float(i) * 2.0f * float(M_PI) / 10.0f;
        auto rr = r * (1.0f + 0.24f * sinf(t * 3.0f + phase) + 0.14f * cosf(t * 2.0f - phase * 1.7f));
        pts.push_back({cx + cosf(t) * rr, cy + sinf(t) * rr});
    }
    return smooth(pts);
}

inline Contour circle(float cx, float cy, float r)
{
    auto k = r * KAPPA;
    return {
        {cx, cy - r, cx + k, cy - r, cx + r, cy - k, cx + r, cy, false},
        {cx + r, cy, cx + r, cy + k, cx + k, cy + r, cx, cy + r, false},
        {cx, cy + r, cx - k, cy + r, cx - r, cy + k, cx - r, cy, false},
        {cx - r, cy, cx - r, cy - k, cx - k, cy - r, cx, cy - r, false},
    };
}

inline Contour star(float cx, float cy, float outer, float inner, uint32_t n)
{
    Contour out;
    std::vector<std::pair<float, float>> pts;
    for (uint32_t i = 0; i < n * 2; ++i) {
        auto r = (i % 2) ? inner : outer;
        auto a = float(i) * float(M_PI) / float(n) - float(M_PI) * 0.5f;
        pts.push_back({cx + cosf(a) * r, cy + sinf(a) * r});
    }
    for (size_t i = 0; i < pts.size(); ++i) {
        auto& p1 = pts[i];
        auto& p2 = pts[(i + 1) % pts.size()];
        out.push_back({p1.first, p1.second, 0.0f, 0.0f, 0.0f, 0.0f, p2.first, p2.second, true});
    }
    return out;
}

//the blobs of one animation frame of the stress scene
inline std::vector<Contour> blobs(uint32_t count, float spin, float canvas = 900.0f)
{
    std::vector<Contour> out;
    auto radius = canvas * 0.5f;
    for (uint32_t i = 0; i < count; ++i) {
        auto t = float(i) * 2.0f * float(M_PI) / float(count) + spin;
        auto orbit = radius * (0.34f + 0.12f * sinf(spin * 2.0f + float(i)));
        out.push_back(blob(radius + cosf(t) * orbit, radius + sinf(t) * orbit, radius * 0.26f, spin * 1.3f + float(i)));
    }
    return out;
}

}

#endif //_BENCH_SCENARIO_H_
