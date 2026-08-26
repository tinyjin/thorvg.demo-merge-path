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

/* The same scenes as the thorvg demo, on skia alone. Nothing here knows about
   thorvg - skia solves the merges and skia rasterizes them, so the two windows
   can be compared as two whole engines rather than as two solvers behind one
   renderer. The scene geometry comes from the shared scenario. */

#include <cstdio>
#include <cstring>
#include <chrono>
#include <SDL2/SDL.h>
#include "include/core/SkSurface.h"
#include "include/core/SkCanvas.h"
#include "include/core/SkPaint.h"
#include "include/core/SkPath.h"
#include "include/core/SkPathBuilder.h"
#include "include/core/SkFont.h"
#include "include/core/SkFontMgr.h"
#include "include/ports/SkFontMgr_mac_ct.h"
#include "include/pathops/SkPathOps.h"
#include "include/encode/SkPngEncoder.h"
#include "scenario.h"

/* the minimal skia build has the png encoder off, but the serialization path of
   SkSurface still references it. nothing here ever encodes an image. */
namespace SkPngEncoder {
    bool Encode(SkWStream*, const SkPixmap&, const Options&) { return false; }
}

using clk = std::chrono::high_resolution_clock;

static bool traceCost = false;


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


static SkPath rect(float cx, float cy, float w, float h)
{
    SkPathBuilder builder;
    builder.moveTo(cx - w, cy - h);
    builder.lineTo(cx + w, cy - h);
    builder.lineTo(cx + w, cy + h);
    builder.lineTo(cx - w, cy + h);
    builder.close();
    return builder.detach();
}


//the counter wound inner circle is what makes the hole a hole
static SkPath ring(float cx, float cy, float outer, float inner)
{
    auto k = inner * 0.552284f;
    SkPathBuilder builder;
    builder.addPath(build(scenario::circle(cx, cy, outer)));
    builder.moveTo(cx, cy - inner);
    builder.cubicTo({cx - k, cy - inner}, {cx - inner, cy - k}, {cx - inner, cy});
    builder.cubicTo({cx - inner, cy + k}, {cx - k, cy + inner}, {cx, cy + inner});
    builder.cubicTo({cx + k, cy + inner}, {cx + inner, cy + k}, {cx + inner, cy});
    builder.cubicTo({cx + inner, cy - k}, {cx + k, cy - inner}, {cx, cy - inner});
    builder.close();
    builder.setFillType(SkPathFillType::kWinding);
    return builder.detach();
}


static SkPath spun(float cx, float cy, float outer, float inner, int cnt, float spin)
{
    SkPathBuilder builder;
    for (int i = 0; i < cnt * 2; ++i) {
        auto r = (i % 2) ? inner : outer;
        auto a = float(i) * float(M_PI) / float(cnt) - float(M_PI) * 0.5f + spin;
        SkPoint pt = {cx + cosf(a) * r, cy + sinf(a) * r};
        if (i == 0) builder.moveTo(pt); else builder.lineTo(pt);
    }
    builder.close();
    builder.setFillType(SkPathFillType::kWinding);
    return builder.detach();
}


//mm:1 Merge, the operands are concatenated as they are
static SkPath merge(const SkPath& a, const SkPath& b)
{
    SkPath out = a;
    out.addPath(b);
    return out;
}


//the faint operand outlines behind the result, off with -n
static bool g_outline = true;


static void paint(SkCanvas* canvas, const SkPath& path, bool fill)
{
    if (!fill && !g_outline) return;

    SkPaint p;
    p.setAntiAlias(true);
    if (fill) {
        p.setStyle(SkPaint::kFill_Style);
        p.setColor(0xff3c8cf0);
        canvas->drawPath(path, p);
        p.setStyle(SkPaint::kStroke_Style);
        p.setStrokeWidth(1.5f);
        p.setColor(0xff142859);
    } else {
        p.setStyle(SkPaint::kStroke_Style);
        p.setStrokeWidth(1.0f);
        p.setColor(0x6e969ba5);
    }
    canvas->drawPath(path, p);
}


static double tiles(SkCanvas* canvas, uint32_t w, uint32_t h, uint32_t elapsed)
{
    auto tile = std::min(float(w) / 5.0f, float(h) / 2.0f);
    auto progress = float(elapsed % 4000) / 4000.0f;
    auto angle = float(elapsed % 4000) / 4000.0f * 2.0f * float(M_PI);
    auto cx = tile * 0.5f, cy = tile * 0.44f;
    auto radius = tile * 0.28f;

    auto a = build(scenario::star(cx - radius * 0.3f, cy - radius * 0.25f, radius, radius * 0.42f, 5));
    auto b = build(scenario::circle(cx + cosf(angle) * radius * 0.5f, cy + sinf(angle) * radius * 0.5f, radius * 0.62f));
    auto c = rect(cx, cy + cosf(angle) * radius * 0.5f, tile * 0.42f, tile * 0.06f);

    /* the same two limit cases as the thorvg demo, to the coordinate */
    auto gap = (progress < 0.34f) ? tile * 0.06f : (progress < 0.67f ? 0.0f : tile * -0.005f);
    auto bx = tile * 0.25f, by = tile * 0.275f;
    auto bw = tile * 0.25f, bh = tile * 0.30f;
    SkPathBuilder db, eb;
    db.moveTo(bx, by); db.lineTo(bx + bw, by); db.lineTo(bx + bw, by + bh); db.lineTo(bx, by + bh); db.close();
    eb.moveTo(bx + bw + gap, by); eb.lineTo(bx + 2 * bw + gap, by); eb.lineTo(bx + 2 * bw + gap, by + bh); eb.lineTo(bx + bw + gap, by + bh); eb.close();
    auto d = db.detach();
    auto e = eb.detach();

    /* a hole only survives if the counter wound inner circle keeps its direction
       through the solve, and the star both orbits and spins so the ring is cut
       into pieces and rejoined over and over. */
    auto orbit = radius * 0.62f;
    auto f = ring(cx, cy, radius, radius * 0.5f);
    auto g = spun(cx + cosf(angle) * orbit, cy + sinf(angle) * orbit,
                  radius * (0.55f + 0.30f * (0.5f - 0.5f * cosf(angle * 2.0f))), radius * 0.26f, 5, -angle * 2.0f);

    auto begin = clk::now();

    SkPath out[10], tmp;
    out[0] = merge(a, b);
    Op(a, b, kUnion_SkPathOp, &out[1]);
    Op(a, b, kDifference_SkPathOp, &out[2]);
    Op(a, b, kIntersect_SkPathOp, &out[3]);
    Op(a, b, kXOR_SkPathOp, &out[4]);
    //a group accumulates, so a merged result becomes the operand of the next one
    if (Op(a, b, kUnion_SkPathOp, &tmp)) Op(tmp, c, kDifference_SkPathOp, &out[5]);
    Op(d, e, kUnion_SkPathOp, &out[6]);
    Op(f, g, kDifference_SkPathOp, &out[7]);
    //the purest overlap there is - the two boundaries are the very same curve
    Op(b, b, kUnion_SkPathOp, &out[8]);
    Op(b, b, kDifference_SkPathOp, &out[9]);

    auto spent = std::chrono::duration<double, std::milli>(clk::now() - begin).count();

    SkPaint frame;
    frame.setStyle(SkPaint::kStroke_Style);
    frame.setColor(0xffe1e4eb);

    for (uint32_t i = 0; i < 10; ++i) {
        canvas->save();
        canvas->translate(float(i % 5) * tile, float(i / 5) * tile);
        canvas->drawRect({0.0f, 0.0f, tile, tile}, frame);
        paint(canvas, out[i], true);
        if (i == 6) {
            paint(canvas, d, false);
            paint(canvas, e, false);
        } else if (i == 7) {
            paint(canvas, f, false);
            paint(canvas, g, false);
        } else if (i >= 8) {
            paint(canvas, b, false);
        } else {
            paint(canvas, a, false);
            paint(canvas, b, false);
            if (i == 5) paint(canvas, c, false);
        }
        canvas->restore();
    }
    return spent;
}



/* the same comb as the thorvg demo. every tooth meets the same two bar segments,
   so all the crossings pile onto those two. */
static double comb(SkCanvas* canvas, uint32_t w, uint32_t h, uint32_t elapsed, uint32_t teeth)
{
    auto W = float(w), H = float(h);
    auto progress = float(elapsed % 6000) / 6000.0f;
    auto sweep = 0.5f - 0.5f * cosf(progress * 2.0f * float(M_PI));

    SkPathBuilder tb;
    auto left = W * 0.06f, right = W * 0.94f;
    auto base = H * 0.62f, tip = H * 0.22f;
    auto step = (right - left) / float(teeth);
    tb.moveTo(left, base);
    for (uint32_t i = 0; i < teeth; ++i) {
        tb.lineTo(left + (float(i) + 0.5f) * step, tip);
        tb.lineTo(left + (float(i) + 1.0f) * step, base);
    }
    tb.lineTo(right, H * 0.9f);
    tb.lineTo(left, H * 0.9f);
    tb.close();
    auto teethPath = tb.detach();

    auto y = H * (0.24f + 0.32f * sweep), thick = H * 0.035f;
    SkPathBuilder bb;
    bb.moveTo(W * 0.02f, y);
    bb.lineTo(W * 0.98f, y);
    bb.lineTo(W * 0.98f, y + thick);
    bb.lineTo(W * 0.02f, y + thick);
    bb.close();
    auto barPath = bb.detach();

    auto begin = clk::now();

    SkPath merged;
    Op(teethPath, barPath, kUnion_SkPathOp, &merged);

    auto spent = std::chrono::duration<double, std::milli>(clk::now() - begin).count();

    paint(canvas, merged, true);
    paint(canvas, teethPath, false);
    paint(canvas, barPath, false);

    return spent;
}


static double stress(SkCanvas* canvas, uint32_t w, uint32_t h, uint32_t elapsed, uint32_t count, uint32_t& cubics)
{
    auto spin = float(elapsed % 8000) / 8000.0f * 2.0f * float(M_PI);
    auto contours = scenario::blobs(count, spin, float(std::min(w, h)));

    std::vector<SkPath> blobs;
    for (auto& c : contours) blobs.push_back(build(c));

    auto begin = clk::now();

    SkPath acc = blobs[0];
    for (uint32_t i = 1; i < count; ++i) {
        SkPath next;
        if (Op(acc, blobs[i], kUnion_SkPathOp, &next)) acc = next;
    }

    auto spent = std::chrono::duration<double, std::milli>(clk::now() - begin).count();

    cubics = 0;
    SkPath::Iter iter(acc, false);
    SkPoint p[4];
    SkPath::Verb verb;
    while ((verb = iter.next(p)) != SkPath::kDone_Verb) {
        if (verb == SkPath::kCubic_Verb) ++cubics;
    }

    paint(canvas, acc, true);
    for (auto& b : blobs) paint(canvas, b, false);

    return spent;
}



/* The arrangements the thorvg solver does not hold, on skia for reference.
   Same geometry as demo_fragile.h, so the two windows line up tile for tile. The
   readout under each tile is the area it should have against the area it got. */

static constexpr int FRAGILE_TILES = 6;
static constexpr int FRAGILE_COLS = 3;

//the offset the far from origin tile is solved at, in the tile's own units
static constexpr float FRAGILE_FAR = 1.0e6f;

static const char* FRAGILE_LABEL[FRAGILE_TILES] = {
    "single intersection", "partial overlap, sliding edge", "partial overlap, (a + b) - b",
    "far from origin", "self crossing contour", "mixed sibling direction"
};

static const char* FRAGILE_NOTE[FRAGILE_TILES] = {
    "a circle touching another from the inside",
    "a short edge running along a longer one",
    "b unioned in and taken out, against a - b in one step",
    "the same pair solved a million units out and brought back",
    "a bowtie, against the two triangles it is made of",
    "two siblings, one drawn the other way round"
};

//short names for the once a second console line
static const char* FRAGILE_TAG[FRAGILE_TILES] = {
    "tangent", "sliding edge", "(a+b)-b", "far origin", "self crossing", "mixed dir"
};

static uint32_t fragileWrong[FRAGILE_TILES] = {}, fragileFrames[FRAGILE_TILES] = {};

//dir flips the winding by mirroring in x, so the shape is the same circle
static void circleTo(SkPathBuilder& b, float cx, float cy, float r, float dir = 1.0f)
{
    auto k = r * 0.55228474983f;
    auto R = r * dir, C = k * dir;
    b.moveTo(cx, cy - r);
    b.cubicTo(cx + C, cy - r, cx + R, cy - k, cx + R, cy);
    b.cubicTo(cx + R, cy + k, cx + C, cy + r, cx, cy + r);
    b.cubicTo(cx - C, cy + r, cx - R, cy + k, cx - R, cy);
    b.cubicTo(cx - R, cy - k, cx - C, cy - r, cx, cy - r);
    b.close();
}

static SkPath circlePath(float cx, float cy, float r, float dir = 1.0f)
{
    SkPathBuilder b;
    circleTo(b, cx, cy, r, dir);
    return b.detach();
}

static SkPath boxPath(float x, float y, float w, float h)
{
    SkPathBuilder b;
    b.moveTo(x, y); b.lineTo(x + w, y); b.lineTo(x + w, y + h); b.lineTo(x, y + h);
    b.close();
    return b.detach();
}

//the curves are walked rather than jumped over, so a cut curve does not skew the sum
static float pathArea(const SkPath& path)
{
    auto sum = 0.0f;
    SkPoint cur{}, home{};
    auto edge = [&](SkPoint to) { sum += cur.fX * to.fY - to.fX * cur.fY; cur = to; };

    SkPath::Iter iter(path, false);
    SkPoint p[4];
    SkPath::Verb verb;
    while ((verb = iter.next(p)) != SkPath::kDone_Verb) {
        if (verb == SkPath::kMove_Verb) { cur = home = p[0]; }
        else if (verb == SkPath::kLine_Verb) edge(p[1]);
        else if (verb == SkPath::kQuad_Verb) {
            for (int i = 1; i <= 32; ++i) {
                auto t = float(i) / 32.0f, u = 1.0f - t;
                edge({u*u*p[0].fX + 2*u*t*p[1].fX + t*t*p[2].fX, u*u*p[0].fY + 2*u*t*p[1].fY + t*t*p[2].fY});
            }
        }
        else if (verb == SkPath::kCubic_Verb) {
            for (int i = 1; i <= 32; ++i) {
                auto t = float(i) / 32.0f, u = 1.0f - t;
                edge({u*u*u*p[0].fX + 3*u*u*t*p[1].fX + 3*u*t*t*p[2].fX + t*t*t*p[3].fX,
                      u*u*u*p[0].fY + 3*u*u*t*p[1].fY + 3*u*t*t*p[2].fY + t*t*t*p[3].fY});
            }
        }
        else if (verb == SkPath::kClose_Verb) edge(home);
    }
    return fabsf(sum) * 0.5f;
}

static double fragile(SkCanvas* canvas, uint32_t w, uint32_t h, uint32_t elapsed)
{
    auto tileW = float(w) / float(FRAGILE_COLS), tileH = float(h) / float(FRAGILE_TILES / FRAGILE_COLS);
    auto angle = float(elapsed % 6000) / 6000.0f * 2.0f * float(M_PI);
    auto cx = tileW * 0.5f, cy = tileH * 0.42f;

    SkPath ops[FRAGILE_TILES][2], out[FRAGILE_TILES];
    float want[FRAGILE_TILES];

    //the pieces the later tiles need built before the ops are timed
    SkPath farA, farB, bowSplit, mixedSame;

    {
        auto R = tileH * 0.26f, r = R * 0.6f;
        auto dx = cosf(angle) * R * 0.3f, dy = sinf(angle) * R * 0.3f;
        ops[0][0] = circlePath(cx + dx, cy + dy, R);
        ops[0][1] = circlePath(cx + dx, cy + dy - (R - r), r);
        want[0] = float(M_PI) * R * R;
    }
    {
        auto bw = tileW * 0.22f, bh = tileH * 0.34f, sh = bh * 0.46f;
        auto bx = cx - bw, by = cy - bh * 0.5f;
        auto slide = (0.5f - 0.5f * cosf(angle)) * (bh - sh);
        ops[1][0] = boxPath(bx, by, bw, bh);
        ops[1][1] = boxPath(bx + bw, by + slide, bw, sh);
        want[1] = bw * bh + bw * sh;
    }
    {
        auto R = tileH * 0.22f;
        ops[2][0] = circlePath(cx - R * 0.45f, cy, R);
        ops[2][1] = circlePath(cx + cosf(angle) * R * 0.6f, cy + sinf(angle) * R * 0.6f, R * 0.8f);
        //(a + b) - b is a - b, so the same subtract done in one step is the answer
        SkPath direct;
        Op(ops[2][0], ops[2][1], kDifference_SkPathOp, &direct);
        want[2] = pathArea(direct);
    }

    /* the same pair solved twice, once here and once a million units out. the
       answer is brought back before it is measured, so what is left is the solver
       losing the operands' own size in the mantissa. */
    {
        auto R = tileH * 0.22f;
        ops[3][0] = circlePath(cx - R * 0.5f, cy, R);
        ops[3][1] = circlePath(cx + R * 0.5f + cosf(angle) * R * 0.3f, cy + sinf(angle) * R * 0.3f, R * 0.85f);
        farA = ops[3][0]; farA.offset(FRAGILE_FAR, FRAGILE_FAR);
        farB = ops[3][1]; farB.offset(FRAGILE_FAR, FRAGILE_FAR);

        //the same intersect taken around the origin is the answer
        SkPath origin;
        Op(ops[3][0], ops[3][1], kIntersect_SkPathOp, &origin);
        want[3] = pathArea(origin);
    }

    /* a bowtie carries its crossing on its own boundary. the same picture cut into
       the two triangles it is made of is the answer, and the bar sweeps so both an
       over and an under fill show. */
    {
        auto W = tileW * 0.30f, H = tileH * 0.20f;
        SkPathBuilder bow;
        bow.moveTo(cx - W, cy - H);
        bow.lineTo(cx + W, cy + H);
        bow.lineTo(cx + W, cy - H);
        bow.lineTo(cx - W, cy + H);
        bow.close();
        ops[4][0] = bow.detach();
        ops[4][1] = boxPath(cx - W * 1.3f, cy + sinf(angle) * H * 0.7f - H * 0.18f, W * 2.6f, H * 0.36f);

        SkPathBuilder sp;
        sp.moveTo(cx - W, cy - H); sp.lineTo(cx, cy); sp.lineTo(cx - W, cy + H); sp.close();
        sp.moveTo(cx + W, cy + H); sp.lineTo(cx, cy); sp.lineTo(cx + W, cy - H); sp.close();
        bowSplit = sp.detach();

        /* pathArea() reads a self crossing answer low, so on this tile the count is
           what to trust rather than the number. */
        SkPath ref;
        Op(bowSplit, ops[4][1], kIntersect_SkPathOp, &ref);
        want[4] = pathArea(ref);
    }

    /* two contours side by side in one operand, the small one drawn the other way
       round. the bar keeps a slab of constant width through each, however far the
       small one slides. */
    {
        auto R = tileH * 0.20f, r = R * 0.30f;
        auto sx = cx + R * 1.9f + cosf(angle) * R * 0.3f;
        SkPathBuilder mix;
        circleTo(mix, cx - R * 0.55f, cy, R, +1.0f);
        circleTo(mix, sx, cy, r, -1.0f);            //the sibling that disagrees
        ops[5][0] = mix.detach();
        ops[5][1] = boxPath(cx - R * 2.0f, cy - R * 0.18f, R * 4.6f, R * 0.36f);

        //the same pair with both siblings drawn the same way round is the answer
        SkPathBuilder same;
        circleTo(same, cx - R * 0.55f, cy, R, +1.0f);
        circleTo(same, sx, cy, r, +1.0f);
        mixedSame = same.detach();
        SkPath ref;
        Op(mixedSame, ops[5][1], kIntersect_SkPathOp, &ref);
        want[5] = pathArea(ref);
    }

    auto begin = clk::now();

    Op(ops[0][0], ops[0][1], kUnion_SkPathOp, &out[0]);
    Op(ops[1][0], ops[1][1], kUnion_SkPathOp, &out[1]);
    SkPath tmp;
    if (Op(ops[2][0], ops[2][1], kUnion_SkPathOp, &tmp)) Op(tmp, ops[2][1], kDifference_SkPathOp, &out[2]);
    if (Op(farA, farB, kIntersect_SkPathOp, &out[3])) out[3].offset(-FRAGILE_FAR, -FRAGILE_FAR);
    Op(ops[4][0], ops[4][1], kIntersect_SkPathOp, &out[4]);
    Op(ops[5][0], ops[5][1], kIntersect_SkPathOp, &out[5]);

    auto spent = std::chrono::duration<double, std::milli>(clk::now() - begin).count();

    /* skia has no default typeface any more, so one is asked of the coretext
       manager. it is already in the build, skia_use_fonthost_mac defaults on. */
    static auto face = [] {
        auto mgr = SkFontMgr_New_CoreText(nullptr);
        return mgr ? mgr->matchFamilyStyle(nullptr, SkFontStyle::Normal()) : sk_sp<SkTypeface>();
    }();

    SkFont font(face, tileH * 0.040f);
    SkFont small(face, tileH * 0.030f);

    for (int i = 0; i < FRAGILE_TILES; ++i) {
        auto ox = float(i % FRAGILE_COLS) * tileW, oy = float(i / FRAGILE_COLS) * tileH;
        canvas->save();
        SkPaint frame;
        frame.setAntiAlias(true);
        frame.setStyle(SkPaint::kStroke_Style);
        frame.setStrokeWidth(1.0f);
        frame.setColor(0xffe1e4eb);
        canvas->drawRect({ox, oy, ox + tileW, oy + tileH}, frame);
        canvas->translate(ox, oy);

        paint(canvas, out[i], true);
        for (auto& op : ops[i]) paint(canvas, op, false);

        auto got = pathArea(out[i]);
        ++fragileFrames[i];
        if (fabsf(got - want[i]) > want[i] * 0.01f) ++fragileWrong[i];

        int cmds = 0;
        SkPath::Iter it(out[i], false);
        SkPoint p[4];
        while (it.next(p) != SkPath::kDone_Verb) ++cmds;

        SkPaint ink;
        ink.setAntiAlias(true);
        ink.setColor(0xff373737);
        canvas->drawString(FRAGILE_LABEL[i], tileW * 0.06f, tileH * 0.78f, font, ink);
        ink.setColor(0xff828282);
        canvas->drawString(FRAGILE_NOTE[i], tileW * 0.06f, tileH * 0.845f, small, ink);
        char buf[128];
        snprintf(buf, sizeof(buf), "%d cmds,  area %.0f  /  %.0f", cmds, got, want[i]);
        canvas->drawString(buf, tileW * 0.06f, tileH * 0.895f, small, ink);
        canvas->restore();
    }

    return spent;
}


//one frame into a file, so the scene can be checked without a window
static void dump(uint32_t W, uint32_t H, uint32_t elapsed, uint32_t count, bool fragileScene)
{
    auto info = SkImageInfo::Make(int(W), int(H), kBGRA_8888_SkColorType, kPremul_SkAlphaType);
    auto surface = SkSurfaces::Raster(info);
    auto canvas = surface->getCanvas();
    canvas->clear(0xffffffff);

    uint32_t cubics = 0;
    if (fragileScene) fragile(canvas, W, H, elapsed);
    else if (count > 0) stress(canvas, W, H, elapsed, count, cubics);
    else tiles(canvas, W, H, elapsed);

    std::vector<uint32_t> buffer(W * H);
    surface->readPixels(info, buffer.data(), W * 4, 0, 0);

    auto f = fopen("skia_frame.bmp", "wb");
    uint32_t stride = W * 4, size = 54 + stride * H, offset = 54, hdr = 40;
    uint8_t header[54] = {};
    header[0] = 'B'; header[1] = 'M';
    memcpy(header + 2, &size, 4); memcpy(header + 10, &offset, 4); memcpy(header + 14, &hdr, 4);
    int32_t w = int32_t(W), h = -int32_t(H);
    memcpy(header + 18, &w, 4); memcpy(header + 22, &h, 4);
    uint16_t planes = 1, bpp = 32;
    memcpy(header + 26, &planes, 2); memcpy(header + 28, &bpp, 2);
    fwrite(header, 1, 54, f);
    fwrite(buffer.data(), 1, stride * H, f);
    fclose(f);
    printf("skia_frame.bmp\n");
}


int main(int argc, char** argv)
{
    auto count = 0;
    auto teeth = 0;
    auto offscreen = 0;
    auto fragileScene = false;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-t")) traceCost = true;
        else if (!strcmp(argv[i], "-n")) g_outline = false;
        else if (!strcmp(argv[i], "-f")) fragileScene = true;
        else if (!strcmp(argv[i], "-s")) count = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 12;
        else if (!strcmp(argv[i], "-c")) teeth = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 128;
        else if (!strcmp(argv[i], "-o")) offscreen = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 2000;
    }

    uint32_t W = fragileScene ? 1500 : teeth > 0 ? 1200 : (count > 0 ? 900 : 1600);
    uint32_t H = fragileScene ? 900 : teeth > 0 ? 700 : (count > 0 ? 900 : 640);

    if (fragileScene) printf("fragile: the arrangements the thorvg solver does not hold, on skia for reference\n");
    else if (teeth > 0) printf("comb: %d teeth crossed by one bar, up to %d crossings land on a single segment\n", teeth, teeth * 2);
    else if (count > 0) printf("stress: a union of %d curve blobs, accumulated over %d merges\n", count, count - 1);
    else printf("tiles: mm 1~5  /  accumulated | shared edge | ring - star | same shape +/-\n");

    if (offscreen) { dump(W, H, uint32_t(offscreen), uint32_t(count), fragileScene); return 0; }

    SDL_Init(SDL_INIT_VIDEO);
    auto window = SDL_CreateWindow("Skia SkPathOps (Software)", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, int(W), int(H), 0);

    auto begin = SDL_GetTicks();
    auto reported = 0u;
    double cost = 0.0, frameCost = 0.0;
    uint32_t costCnt = 0, cubics = 0;
    auto running = true;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) running = false;
        }

        auto surface = SDL_GetWindowSurface(window);
        if (!surface) break;

        //skia draws straight into the window surface, no thorvg anywhere
        auto info = SkImageInfo::Make(surface->w, surface->h, kBGRA_8888_SkColorType, kPremul_SkAlphaType);
        auto target = SkSurfaces::WrapPixels(info, surface->pixels, size_t(surface->pitch));
        if (!target) break;

        auto canvas = target->getCanvas();
        auto frameBegin = clk::now();
        canvas->clear(0xffffffff);

        auto elapsed = SDL_GetTicks() - begin;
        auto spent = fragileScene ? fragile(canvas, uint32_t(surface->w), uint32_t(surface->h), elapsed)
                   : teeth > 0 ? comb(canvas, uint32_t(surface->w), uint32_t(surface->h), elapsed, uint32_t(teeth))
                   : count > 0 ? stress(canvas, uint32_t(surface->w), uint32_t(surface->h), elapsed, uint32_t(count), cubics)
                               : tiles(canvas, uint32_t(surface->w), uint32_t(surface->h), elapsed);
        cost += spent;
        ++costCnt;
        frameCost += std::chrono::duration<double, std::milli>(clk::now() - frameBegin).count();

        SDL_UpdateWindowSurface(window);

        if ((traceCost || fragileScene) && elapsed / 1000 > reported) {
            reported = elapsed / 1000;
            if (fragileScene) {
                printf("wrong frames per second  ");
                for (int i = 0; i < FRAGILE_TILES; ++i) printf(" %s %u/%u  ", FRAGILE_TAG[i], fragileWrong[i], fragileFrames[i]);
                printf("\n");
                fflush(stdout);   //the line is a second apart, it should not sit in the buffer
                for (int i = 0; i < FRAGILE_TILES; ++i) { fragileWrong[i] = 0; fragileFrames[i] = 0; }
            }
            else if (teeth > 0) printf("comb: merge %.3f ms  |  그리기 포함 %.3f ms  (%d teeth, %d crossings)\n", cost / costCnt, frameCost / costCnt, teeth, teeth * 2);
            else if (count > 0) printf("stress: %.3f ms / frame (%d merges, %u cubics out)\n", cost / costCnt, count - 1, cubics);
            else printf("merge path: %.3f ms / frame\n", cost / costCnt);
            cost = 0.0;
            frameCost = 0.0;
            costCnt = 0;
        }
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
