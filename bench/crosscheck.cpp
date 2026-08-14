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

/* Skia is the reference here. Both engines solve the same operands and the areas
   are compared, so a divergence points at a case worth looking into. */

#include <cstdio>
#include "include/core/SkPath.h"
#include "include/core/SkPathBuilder.h"
#include "include/pathops/SkPathOps.h"
#include "../mergepath.h"
#include "scenario.h"

static RenderPath tvgPath(const scenario::Contour& contour)
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

static SkPath skiaPath(const scenario::Contour& contour)
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

//our result is carried into a skia path, so both are judged by the same code
static SkPath toSkia(const RenderPath& path)
{
    SkPathBuilder builder;
    auto p = path.pts.data();
    SkPoint start{}, cur{};

    for (auto cmd : path.cmds) {
        if (cmd == PathCommand::MoveTo) { cur = start = {p->x, p->y}; builder.moveTo(cur); ++p; }
        else if (cmd == PathCommand::LineTo) { cur = {p->x, p->y}; builder.lineTo(cur); ++p; }
        else if (cmd == PathCommand::CubicTo) {
            builder.cubicTo({p[0].x, p[0].y}, {p[1].x, p[1].y}, {p[2].x, p[2].y});
            cur = {p[2].x, p[2].y};
            p += 3;
        } else if (cmd == PathCommand::Close) { builder.close(); cur = start; }
    }
    //the walk emits contours that are consistent under the non zero rule
    builder.setFillType(SkPathFillType::kWinding);

    return builder.detach();
}


static uint32_t failed = 0;
static uint32_t checked = 0;

/* the two paths are sampled on a dense grid and every point is asked to both,
   each with its own fill rule. that is orientation and convention agnostic. */
static void compare(const char* name, const RenderPath& mine, const SkPath& reference)
{
    auto lhs = toSkia(mine);
    ++checked;

    auto box = reference.getBounds();
    box.join(lhs.getBounds());
    box.outset(4.0f, 4.0f);
    if (box.isEmpty()) return;

    constexpr int N = 400;
    uint32_t diff = 0, inside = 0;
    for (int j = 0; j < N; ++j) {
        auto y = box.fTop + (float(j) + 0.5f) * box.height() / N;
        for (int i = 0; i < N; ++i) {
            auto x = box.fLeft + (float(i) + 0.5f) * box.width() / N;
            auto a = lhs.contains(x, y);
            auto b = reference.contains(x, y);
            if (b) ++inside;
            if (a != b) ++diff;
        }
    }
    if (inside == 0 && diff == 0) return;

    auto ratio = inside ? double(diff) / double(inside) * 100.0 : double(diff);
    if (ratio < 0.5) return;

    ++failed;
    printf("  %-42s %6.2f%% of the area differs   (%u of %u samples)\n", name, ratio, diff, inside);
}


int main()
{
    printf("== accumulated union, every frame\n");
    for (uint32_t n : {2u, 3u, 4u, 8u, 12u, 24u}) {
        for (uint32_t f = 0; f < 16; ++f) {
            auto spin = float(f) / 16.0f * 2.0f * float(M_PI);
            auto contours = scenario::blobs(n, spin);

            RenderPath acc = tvgPath(contours[0]);
            SkPath ref = skiaPath(contours[0]);
            for (uint32_t i = 1; i < n; ++i) {
                RenderPath next;
                if (AddMask(acc, tvgPath(contours[i]), next)) acc = next;
                SkPath rnext;
                if (Op(ref, skiaPath(contours[i]), kUnion_SkPathOp, &rnext)) ref = rnext;
            }
            char name[64];
            snprintf(name, sizeof(name), "union of %u blobs, frame %u", n, f);
            compare(name, acc, ref);
        }
    }

    printf("== pairwise\n");
    struct Op4 { const char* name; bool (*tvg)(const RenderPath&, const RenderPath&, RenderPath&); SkPathOp skia; };
    Op4 ops[] = {{"union", AddMask, kUnion_SkPathOp}, {"intersect", IntersectMask, kIntersect_SkPathOp},
                 {"difference", SubtractMask, kDifference_SkPathOp}, {"xor", DifferenceMask, kXOR_SkPathOp}};

    struct Pair { const char* name; scenario::Contour a, b; };
    std::vector<Pair> pairs = {
        {"two circles", scenario::circle(180.0f, 180.0f, 90.0f), scenario::circle(239.0f, 180.0f, 90.0f)},
        {"circle in circle", scenario::circle(180.0f, 180.0f, 90.0f), scenario::circle(180.0f, 180.0f, 40.0f)},
        {"circles apart", scenario::circle(180.0f, 180.0f, 90.0f), scenario::circle(500.0f, 180.0f, 90.0f)},
        {"star + circle", scenario::star(130.0f, 130.0f, 110.0f, 45.0f, 5), scenario::circle(220.0f, 220.0f, 90.0f)},
        {"star + blob", scenario::star(130.0f, 130.0f, 110.0f, 45.0f, 5), scenario::blob(200.0f, 200.0f, 80.0f, 1.0f)},
        {"blob + blob", scenario::blob(180.0f, 180.0f, 90.0f, 0.0f), scenario::blob(240.0f, 200.0f, 90.0f, 2.0f)},
    };

    for (auto& p : pairs) {
        auto ta = tvgPath(p.a), tb = tvgPath(p.b);
        auto sa = skiaPath(p.a), sb = skiaPath(p.b);
        for (auto& o : ops) {
            RenderPath mine;
            SkPath ref;
            o.tvg(ta, tb, mine);
            Op(sa, sb, o.skia, &ref);
            char name[80];
            snprintf(name, sizeof(name), "%s / %s", p.name, o.name);
            compare(name, mine, ref);
        }
    }

    printf("\n%u of %u cases diverge from skia\n", failed, checked);
    return 0;
}
