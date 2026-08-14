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

/* Both results are rasterized by the very same thorvg rasterizer, skia's path is
   only carried over. So whatever shows up is a geometry difference, never a
   rendering one:

     blue    both agree
     red     only ours is filled
     orange  only skia is filled                                             */

#include <cstdio>
#include <cstring>
#include "include/core/SkPath.h"
#include "include/core/SkPathBuilder.h"
#include "include/pathops/SkPathOps.h"
#include "../mergepath.h"
#include "scenario.h"

#ifndef RES_DIR
    #define RES_DIR "res"
#endif

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

//skia's result is carried into our path type, a quad is raised to a cubic
static RenderPath fromSkia(const SkPath& path, FillRule& rule)
{
    RenderPath out;
    SkPath::Iter iter(path, false);
    SkPoint p[4];
    SkPath::Verb verb;

    while ((verb = iter.next(p)) != SkPath::kDone_Verb) {
        if (verb == SkPath::kMove_Verb) out.moveTo({p[0].fX, p[0].fY});
        else if (verb == SkPath::kLine_Verb) out.lineTo({p[1].fX, p[1].fY});
        else if (verb == SkPath::kCubic_Verb) out.cubicTo({p[1].fX, p[1].fY}, {p[2].fX, p[2].fY}, {p[3].fX, p[3].fY});
        else if (verb == SkPath::kQuad_Verb) {
            out.cubicTo({p[0].fX + 2.0f / 3.0f * (p[1].fX - p[0].fX), p[0].fY + 2.0f / 3.0f * (p[1].fY - p[0].fY)},
                        {p[2].fX + 2.0f / 3.0f * (p[1].fX - p[2].fX), p[2].fY + 2.0f / 3.0f * (p[1].fY - p[2].fY)},
                        {p[2].fX, p[2].fY});
        } else if (verb == SkPath::kClose_Verb) out.close();
    }
    auto type = path.getFillType();
    rule = (type == SkPathFillType::kEvenOdd || type == SkPathFillType::kInverseEvenOdd) ? FillRule::EvenOdd : FillRule::NonZero;

    return out;
}

//renders one path into an 8 bit coverage mask
static void rasterize(const RenderPath& path, FillRule rule, const Matrix& m, uint8_t* mask, uint32_t size)
{
    auto buffer = new uint32_t[size * size];
    memset(buffer, 0, size * size * sizeof(uint32_t));

    auto canvas = SwCanvas::gen();
    canvas->target(buffer, size, size, size, ColorSpace::ARGB8888);

    auto shape = Shape::gen();
    shape->appendPath(path.cmds.data(), path.cmds.size(), path.pts.data(), path.pts.size());
    shape->fill(255, 255, 255);
    shape->fillRule(rule);
    shape->transform(m);
    canvas->add(shape);

    canvas->draw(true);
    canvas->sync();

    for (uint32_t i = 0; i < size * size; ++i) mask[i] = uint8_t(buffer[i] >> 24);

    delete[] buffer;
}

struct Case
{
    const char* name;
    RenderPath mine;
    RenderPath reference;
    FillRule rule;
    Point min, max;
};

static void bounds(const RenderPath& path, Point& min, Point& max)
{
    for (auto& pt : path.pts) {
        min.x = fminf(min.x, pt.x); min.y = fminf(min.y, pt.y);
        max.x = fmaxf(max.x, pt.x); max.y = fmaxf(max.y, pt.y);
    }
}

int main()
{
    Initializer::init();

    constexpr uint32_t TILE = 320, COLS = 4;
    std::vector<Case> cases;

    struct Op4 { const char* name; bool (*tvg)(const RenderPath&, const RenderPath&, RenderPath&); SkPathOp skia; };
    Op4 ops[] = {{"union", AddMask, kUnion_SkPathOp}, {"intersect", IntersectMask, kIntersect_SkPathOp},
                 {"difference", SubtractMask, kDifference_SkPathOp}, {"xor", DifferenceMask, kXOR_SkPathOp}};

    struct Pair { const char* name; scenario::Contour a, b; };
    std::vector<Pair> pairs = {
        {"two circles", scenario::circle(180.0f, 180.0f, 90.0f), scenario::circle(239.0f, 180.0f, 90.0f)},
        {"star + circle", scenario::star(130.0f, 130.0f, 110.0f, 45.0f, 5), scenario::circle(220.0f, 220.0f, 90.0f)},
        {"blob + blob", scenario::blob(180.0f, 180.0f, 90.0f, 0.0f), scenario::blob(240.0f, 200.0f, 90.0f, 2.0f)},
        {"circle in circle", scenario::circle(180.0f, 180.0f, 90.0f), scenario::circle(180.0f, 180.0f, 40.0f)},
    };

    for (auto& p : pairs) {
        auto ta = tvgPath(p.a), tb = tvgPath(p.b);
        auto sa = skiaPath(p.a), sb = skiaPath(p.b);
        for (auto& o : ops) {
            Case c;
            c.name = o.name;
            o.tvg(ta, tb, c.mine);
            SkPath ref;
            Op(sa, sb, o.skia, &ref);
            c.reference = fromSkia(ref, c.rule);
            c.min = {FLT_MAX, FLT_MAX};
            c.max = {-FLT_MAX, -FLT_MAX};
            bounds(ta, c.min, c.max);
            bounds(tb, c.min, c.max);
            char* name = new char[64];
            snprintf(name, 64, "%s / %s", p.name, o.name);
            c.name = name;
            cases.push_back(c);
        }
    }

    //the accumulated frames, including the one that diverges
    struct Frame { uint32_t count, frame; };
    for (auto& f : std::vector<Frame>{{12, 11}, {24, 3}, {12, 0}, {24, 8}}) {
        auto spin = float(f.frame) / 16.0f * 2.0f * float(M_PI);
        auto contours = scenario::blobs(f.count, spin);

        Case c;
        c.min = {FLT_MAX, FLT_MAX};
        c.max = {-FLT_MAX, -FLT_MAX};

        RenderPath acc = tvgPath(contours[0]);
        SkPath ref = skiaPath(contours[0]);
        bounds(acc, c.min, c.max);
        for (uint32_t i = 1; i < f.count; ++i) {
            auto next = tvgPath(contours[i]);
            bounds(next, c.min, c.max);
            RenderPath out;
            if (AddMask(acc, next, out)) acc = out;
            SkPath rnext;
            if (Op(ref, skiaPath(contours[i]), kUnion_SkPathOp, &rnext)) ref = rnext;
        }
        c.mine = acc;
        c.reference = fromSkia(ref, c.rule);
        char* name = new char[64];
        snprintf(name, 64, "%u blobs, frame %u", f.count, f.frame);
        c.name = name;
        cases.push_back(c);
    }

    auto rows = (cases.size() + COLS - 1) / COLS;
    auto W = TILE * COLS, H = uint32_t(TILE * rows);
    auto canvasBuf = new uint32_t[W * H];
    for (uint32_t i = 0; i < W * H; ++i) canvasBuf[i] = 0xffffffff;

    auto mineMask = new uint8_t[TILE * TILE];
    auto refMask = new uint8_t[TILE * TILE];
    uint32_t diverged = 0;

    for (size_t i = 0; i < cases.size(); ++i) {
        auto& c = cases[i];
        auto span = fmaxf(c.max.x - c.min.x, c.max.y - c.min.y);
        auto scale = (TILE * 0.78f) / span;
        Matrix m = {scale, 0.0f, TILE * 0.5f - (c.min.x + c.max.x) * 0.5f * scale,
                    0.0f, scale, TILE * 0.46f - (c.min.y + c.max.y) * 0.5f * scale, 0.0f, 0.0f, 1.0f};

        memset(mineMask, 0, TILE * TILE);
        memset(refMask, 0, TILE * TILE);
        if (!c.mine.cmds.empty()) rasterize(c.mine, FillRule::NonZero, m, mineMask, TILE);
        if (!c.reference.cmds.empty()) rasterize(c.reference, c.rule, m, refMask, TILE);

        auto ox = uint32_t(i % COLS) * TILE;
        auto oy = uint32_t(i / COLS) * TILE;
        uint32_t differs = 0;

        for (uint32_t y = 0; y < TILE; ++y) {
            for (uint32_t x = 0; x < TILE; ++x) {
                auto a = mineMask[y * TILE + x];
                auto b = refMask[y * TILE + x];
                uint32_t color = 0xffffffff;
                if (a > 128 && b > 128) color = 0xff3c8cf0;         //both
                else if (a > 128) { color = 0xffe03030; ++differs; } //only ours
                else if (b > 128) { color = 0xfff09030; ++differs; } //only skia
                canvasBuf[(oy + y) * W + ox + x] = color;
            }
        }
        if (differs > TILE * TILE / 2000) ++diverged;
        printf("%-34s %s\n", c.name, differs > TILE * TILE / 2000 ? "DIFFERS" : "same");
    }

    //the frames and the labels go on top of the composed pixels
    auto canvas = SwCanvas::gen();
    canvas->target(canvasBuf, W, W, H, ColorSpace::ARGB8888);
    auto labeled = (Text::load(RES_DIR"/font/PublicSans-Regular.ttf") == Result::Success);

    for (size_t i = 0; i < cases.size(); ++i) {
        auto ox = float(uint32_t(i % COLS) * TILE);
        auto oy = float(uint32_t(i / COLS) * TILE);

        auto frame = Shape::gen();
        frame->appendRect(ox, oy, TILE, TILE);
        frame->strokeWidth(1.0f);
        frame->strokeFill(210, 214, 222);
        canvas->add(frame);

        if (!labeled) continue;
        auto label = Text::gen();
        label->font("PublicSans-Regular");
        label->size(TILE * 0.055f);
        label->text(cases[i].name);
        label->fill(60, 65, 75);
        label->translate(ox + TILE * 0.06f, oy + TILE * 0.87f);
        canvas->add(label);
    }

    canvas->draw(false);
    canvas->sync();

    auto f = fopen("visual.bmp", "wb");
    uint32_t stride = W * 4, size = 54 + stride * H, offset = 54, hdr = 40;
    uint8_t header[54] = {};
    header[0] = 'B'; header[1] = 'M';
    memcpy(header + 2, &size, 4); memcpy(header + 10, &offset, 4); memcpy(header + 14, &hdr, 4);
    int32_t w = int32_t(W), h = -int32_t(H);
    memcpy(header + 18, &w, 4); memcpy(header + 22, &h, 4);
    uint16_t planes = 1, bpp = 32;
    memcpy(header + 26, &planes, 2); memcpy(header + 28, &bpp, 2);
    fwrite(header, 1, 54, f);
    fwrite(canvasBuf, 1, stride * H, f);
    fclose(f);

    printf("\n%u of %zu tiles differ -> visual.bmp\n", diverged, cases.size());

    delete[] mineMask;
    delete[] refMask;
    delete[] canvasBuf;
    Initializer::term();

    return 0;
}
