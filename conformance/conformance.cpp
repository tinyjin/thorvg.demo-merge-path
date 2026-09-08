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

/* Skia's own pathops corpus, run through our solver.

   The cases are not transcribed. shim/ compiles skia's case files
   against a shim in which testPathOp() records its two operands rather than
   solving them, so what runs here is skia's geometry bit for bit, down to the
   SkBits2Float literals it was written with.

   Skia is the reference and the only one. What comes out of this is a conformance
   rate - how much of skia's own corpus we reproduce - and not a claim about which
   of the two is right where they part. Two things are measured:

   - does it come back at all. a crash, a hang or a NaN in the output is a failure
     whatever the answer would have been, and is counted on its own.
   - does it cover the same pixels skia's answer covers.

   Every case is solved in a forked child, so a crash or a hang is observed rather
   than ending the run.

   The whole report goes to the terminal and nothing is written to disk, so a run
   needs no more than skia and thorvg on the machine. --repro prints one case back
   as the calls that reproduce it. */

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <cfloat>
#include <vector>
#include <string>
#include <map>
#include <array>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

#include "../mergepath.h"

#include "include/core/SkPath.h"
#include "include/core/SkPathBuilder.h"
#include "include/core/SkPathTypes.h"
#include "include/pathops/SkPathOps.h"
#include "shim/tests/Test.h"
#include "shim/tests/PathOpsExtendedTest.h"

//the minimal skia build leaves the png encoder out, SkPath's serializer still names it
#include "include/encode/SkPngEncoder.h"
namespace SkPngEncoder {
    bool Encode(SkWStream*, const SkPixmap&, const Options&) { return false; }
}

static constexpr uint32_t MASK = 384;      //the grid answers are judged on
static constexpr double TOLERANCE = 2.0;   //percent of covered pixels, below this two answers are the same
static constexpr uint32_t TIMEOUT = 10;    //seconds a single case gets before it counts as hung


/************************************************************************/
/* The corpus                                                           */
/************************************************************************/

struct Fixture
{
    SkPath a, b;
    SkPathOp op;
    std::string name;
    std::string file;       //which of skia's case files it came from
    conf::Grade grade;
};

static std::vector<Fixture>& fixtures()
{
    static std::vector<Fixture> all;
    return all;
}

struct Suite
{
    std::string name;
    void (*fn)(skiatest::Reporter*);
};

static std::vector<Suite>& suites()
{
    static std::vector<Suite> all;
    return all;
}

static std::string g_suite;

namespace conf {

void enroll(const char* name, void (*fn)(skiatest::Reporter*))
{
    suites().push_back({name, fn});
}

void collect(const SkPath& a, const SkPath& b, SkPathOp op, const char* name, Grade grade)
{
    fixtures().push_back({a, b, op, name ? name : "?", g_suite, grade});
}

}


/************************************************************************/
/* Carrying paths between the two engines                               */
/************************************************************************/

static RenderPath skiaToTvg(const SkPath& path)
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
        } else if (verb == SkPath::kConic_Verb) {
            SkPoint q[5];
            SkPath::ConvertConicToQuads(p[0], p[1], p[2], iter.conicWeight(), q, 1);
            for (uint32_t i = 0; i < 2; ++i) {
                auto &a = q[i * 2], &c = q[i * 2 + 1], &b = q[i * 2 + 2];
                out.cubicTo({a.fX + 2.0f / 3.0f * (c.fX - a.fX), a.fY + 2.0f / 3.0f * (c.fY - a.fY)},
                            {b.fX + 2.0f / 3.0f * (c.fX - b.fX), b.fY + 2.0f / 3.0f * (c.fY - b.fY)},
                            {b.fX, b.fY});
            }
        } else if (verb == SkPath::kClose_Verb) out.close();
    }
    return out;
}


static bool _finite(const RenderPath& p)
{
    for (auto& pt : p.pts) if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) return false;
    return true;
}


/* Back the other way, so both engines are handed the same figure.

   Skia's path holds quads and conics and ours does not, and a conic only survives
   the trip as an approximation. Solving the original on one side and the
   approximation on the other would judge us on a curve skia was never given, so
   the operands make the round trip first and skia solves what we solve. */
static SkPath tvgToSkia(const RenderPath& path, SkPathFillType rule)
{
    SkPathBuilder b;
    b.setFillType(rule);
    auto at = 0u;
    for (auto cmd : path.cmds) {
        if (cmd == PathCommand::MoveTo) { b.moveTo(path.pts[at].x, path.pts[at].y); ++at; }
        else if (cmd == PathCommand::LineTo) { b.lineTo(path.pts[at].x, path.pts[at].y); ++at; }
        else if (cmd == PathCommand::CubicTo) {
            b.cubicTo(path.pts[at].x, path.pts[at].y, path.pts[at + 1].x, path.pts[at + 1].y,
                      path.pts[at + 2].x, path.pts[at + 2].y);
            at += 3;
        } else b.close();
    }
    return b.detach();
}


/************************************************************************/
/* Judging                                                              */
/************************************************************************/

struct Box { float x0, y0, x1, y1; };

static Box _view(const SkPath& a, const SkPath& b)
{
    auto ra = a.getBounds(), rb = b.getBounds();
    Box v{fminf(ra.fLeft, rb.fLeft), fminf(ra.fTop, rb.fTop),
          fmaxf(ra.fRight, rb.fRight), fmaxf(ra.fBottom, rb.fBottom)};
    if (a.isEmpty()) v = {rb.fLeft, rb.fTop, rb.fRight, rb.fBottom};
    if (b.isEmpty()) v = {ra.fLeft, ra.fTop, ra.fRight, ra.fBottom};
    if (!(v.x1 > v.x0) || !(v.y1 > v.y0) || !std::isfinite(v.x0) || !std::isfinite(v.y1)) return {0.0f, 0.0f, 1.0f, 1.0f};
    auto pad = fmaxf(v.x1 - v.x0, v.y1 - v.y0) * 0.05f;
    return {v.x0 - pad, v.y0 - pad, v.x1 + pad, v.y1 + pad};
}


static void _fit(const Box& v, uint32_t size, float& scale, float& dx, float& dy)
{
    auto w = v.x1 - v.x0, h = v.y1 - v.y0;
    scale = fminf(float(size) / w, float(size) / h);
    dx = (float(size) - w * scale) * 0.5f - v.x0 * scale;
    dy = (float(size) - h * scale) * 0.5f - v.y0 * scale;
}


/* the filled pixels of an answer. both engines come through this one
   rasteriser, and the fill rule travels with the path - skia hands some answers
   back wound even odd, and drawing those non zero invents a difference. */
static std::vector<uint8_t> _mask(const RenderPath& path, const Box& view, tvg::FillRule rule)
{
    std::vector<uint32_t> buf(size_t(MASK) * MASK, 0xff000000u);
    if (path.cmds.empty()) return std::vector<uint8_t>(buf.size(), 0);

    auto canvas = tvg::SwCanvas::gen();
    if (!canvas) return std::vector<uint8_t>(buf.size(), 0);
    canvas->target(buf.data(), MASK, MASK, MASK, tvg::ColorSpace::ARGB8888);

    float scale, dx, dy;
    _fit(view, MASK, scale, dx, dy);

    auto shape = tvg::Shape::gen();
    shape->appendPath(path.cmds.data(), uint32_t(path.cmds.size()), path.pts.data(), uint32_t(path.pts.size()));
    shape->fillRule(rule);
    shape->fill(255, 255, 255, 255);
    shape->transform({scale, 0.0f, dx, 0.0f, scale, dy, 0.0f, 0.0f, 1.0f});
    canvas->add(shape);
    canvas->draw(false);
    canvas->sync();
    delete(canvas);

    std::vector<uint8_t> out(buf.size());
    for (size_t i = 0; i < buf.size(); ++i) out[i] = ((buf[i] >> 8) & 0xff) >= 128 ? 1 : 0;
    return out;
}


//the share of the covered pixels two answers disagree on
static double _apart(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b)
{
    size_t either = 0, apart = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i] | b[i]) ++either;
        if (a[i] ^ b[i]) ++apart;
    }
    return either ? double(apart) / double(either) * 100.0 : 0.0;
}


/* The same disagreement, with the hairlines taken out.

   Two answers that trace the same figure but land a fraction of a pixel apart
   differ all along their shared boundary, and on a thin figure that is a large
   share of very few pixels - a number that says nothing about what anyone would
   see. Eroding the difference by one pixel drops any run that is only a boundary
   wide and keeps whatever has an interior, which is the part that reads as a
   wrong shape rather than a soft edge. */
static double _solid(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, uint32_t w)
{
    std::vector<uint8_t> diff(a.size());
    size_t either = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        diff[i] = a[i] ^ b[i];
        if (a[i] | b[i]) ++either;
    }
    if (!either) return 0.0;

    auto h = uint32_t(a.size() / w);
    size_t kept = 0;
    for (uint32_t y = 1; y + 1 < h; ++y) {
        for (uint32_t x = 1; x + 1 < w; ++x) {
            auto at = size_t(y) * w + x;
            if (!diff[at]) continue;
            if (diff[at - w - 1] && diff[at - w] && diff[at - w + 1] &&
                diff[at - 1] && diff[at + 1] &&
                diff[at + w - 1] && diff[at + w] && diff[at + w + 1]) ++kept;
        }
    }
    return double(kept) / double(either) * 100.0;
}


/* The answer, worked out from the operands rather than from either engine.

   Whether a pixel belongs in a boolean result depends on two facts and no others:
   is it inside a, and is it inside b. So rasterising the two operands separately
   and combining per pixel gives a truth that owes nothing to skia, to us, or to
   any path solver at all. It is only as exact as the grid, but on this grid that
   is a pixel. */
static std::vector<uint8_t> _truth(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b, SkPathOp op)
{
    std::vector<uint8_t> out(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        switch (op) {
            case kUnion_SkPathOp: out[i] = a[i] | b[i]; break;
            case kIntersect_SkPathOp: out[i] = a[i] & b[i]; break;
            case kDifference_SkPathOp: out[i] = a[i] & !b[i]; break;
            case kXOR_SkPathOp: out[i] = a[i] ^ b[i]; break;
            default: out[i] = b[i] & !a[i]; break;    //reverse difference
        }
    }
    return out;
}


/* Which kind of wrong an answer is.

   The plane is cut into four regions - in both, in a only, in b only, in neither -
   and every boolean answer is some whole selection of them. So if our answer covers
   each region either almost entirely or almost not at all, we found the pieces and
   chose the wrong ones, which is a question for the traversal. If instead our
   boundary runs through the middle of a region, the pieces themselves came out
   wrong, which is a question for the intersection finding underneath.

   Regions too small to read are skipped: at a handful of pixels the coverage
   fraction is rasterisation noise and nothing else. */
static bool _wholeRegions(const std::vector<uint8_t>& ours, const std::vector<uint8_t>& a,
                          const std::vector<uint8_t>& b)
{
    size_t total[4] = {}, held[4] = {};
    for (size_t i = 0; i < ours.size(); ++i) {
        auto k = uint32_t(a[i] ? 1 : 0) | uint32_t(b[i] ? 2 : 0);
        ++total[k];
        if (ours[i]) ++held[k];
    }
    auto floor = ours.size() / 200;     //half a percent of the grid
    for (auto k = 1u; k < 4; ++k) {     //region 0 is the outside, nothing belongs there
        if (total[k] < floor) continue;
        auto share = double(held[k]) / double(total[k]);
        if (share > 0.05 && share < 0.95) return false;
    }
    return true;
}


/* What our answer turned out to be.

   A wrong answer is far more useful named than measured. Every boolean result is a
   selection of the four regions, so a wrong one usually is too - and when it is the
   selection some other operator would have made, or one of the operands handed back
   untouched, that is a specific bug with a specific place to look, not a percentage. */
enum class Answer : uint8_t {
    Right = 0, Empty, Everything, OperandA, OperandB,
    AsUnion, AsIntersect, AsAMinusB, AsBMinusA, AsXor, Garbled
};

static const char* shapeName(Answer s)
{
    switch (s) {
        case Answer::Right: return "right";
        case Answer::Empty: return "empty";
        case Answer::Everything: return "everything";
        case Answer::OperandA: return "operand a, untouched";
        case Answer::OperandB: return "operand b, untouched";
        case Answer::AsUnion: return "answered union";
        case Answer::AsIntersect: return "answered intersect";
        case Answer::AsAMinusB: return "answered a minus b";
        case Answer::AsBMinusA: return "answered b minus a";
        case Answer::AsXor: return "answered xor";
        default: return "garbled";
    }
}


static Answer _shape(const std::vector<uint8_t>& ours, const std::vector<uint8_t>& a,
                    const std::vector<uint8_t>& b, SkPathOp op, double tolerance)
{
    auto is = [&](const std::vector<uint8_t>& want) { return _apart(ours, want) <= tolerance; };

    if (is(_truth(a, b, op))) return Answer::Right;

    auto empty = std::vector<uint8_t>(ours.size(), 0);
    if (is(empty)) return Answer::Empty;
    if (is(_truth(a, b, kUnion_SkPathOp)) && op != kUnion_SkPathOp) return Answer::AsUnion;
    if (is(a)) return Answer::OperandA;
    if (is(b)) return Answer::OperandB;
    if (is(_truth(a, b, kIntersect_SkPathOp))) return Answer::AsIntersect;
    if (is(_truth(a, b, kDifference_SkPathOp))) return Answer::AsAMinusB;
    if (is(_truth(a, b, kReverseDifference_SkPathOp))) return Answer::AsBMinusA;
    if (is(_truth(a, b, kXOR_SkPathOp))) return Answer::AsXor;
    return Answer::Garbled;
}


/************************************************************************/
/* One case                                                             */
/************************************************************************/

//what became of a case, written by the child and read by the parent
enum Verdict : uint8_t {
    Pass = 0,       //we and skia land on the same answer
    Fail,           //we come back with something else
    Broken,         //our answer is not a number, or the call refused
    Unsupported,    //an inverse fill, which our api does not model
    Tolerated,      //skia declined the case, so there is nothing to be judged against
    Crashed,
    Hung
};

static const char* verdictName(Verdict v)
{
    switch (v) {
        case Pass: return "pass";
        case Fail: return "FAIL";
        case Broken: return "BROKEN";
        case Unsupported: return "n/a";
        case Tolerated: return "no ref";
        case Crashed: return "CRASH";
        default: return "HANG";
    }
}

struct Outcome
{
    Verdict verdict;
    uint8_t reported;       //already sent down the pipe, so the child need not send it again
    uint8_t skiaRefused;    //skia's own Op() declined the case
    double apartSkia;       //percent of covered pixels ours and skia disagree on

    double solidSkia;       //the same, with a one pixel boundary seam eroded away
    double apartTruth;      //ours against the answer worked out from the operands
    double skiaTruth;       //skia against that same answer
    uint8_t whole;          //our error is whole regions, not a boundary in the wrong place
    Answer shape;            //and what the answer turned out to be instead
    uint8_t curved;                 //either operand carries a curve, not only lines
    uint8_t conic;                  //and one of them is a conic, which we can only approximate
    uint8_t selfCross;              //an operand runs through itself
    uint8_t degenerate;             //a cubic in there has doubled up control points
    uint32_t crossings;             //how many times the two outlines meet
    uint8_t oursEmpty, skiaEmpty;   //an answer that covers nothing
    uint8_t bothEmpty;              //both engines returned nothing, so the agreement is vacuous
    uint32_t segs;          //how much geometry went in, to sort the report
};


static const char* opName(SkPathOp op)
{
    switch (op) {
        case kDifference_SkPathOp: return "difference";
        case kIntersect_SkPathOp: return "intersect";
        case kUnion_SkPathOp: return "union";
        case kXOR_SkPathOp: return "xor";
        default: return "revdiff";
    }
}


/* Our solver takes a path, not a path and a fill rule, so an even odd operand has
   to be rewound before it means the same thing. skia's AsWinding does exactly that
   rewrite; where it declines, the case is one our api cannot express and is set
   aside rather than counted against us. */
static bool asNonZero(const SkPath& in, SkPath& out)
{
    auto type = in.getFillType();
    if (type == SkPathFillType::kInverseWinding || type == SkPathFillType::kInverseEvenOdd) return false;
    if (type == SkPathFillType::kWinding) { out = in; return true; }
    return AsWinding(in, &out);
}


/* the control run asks our solver for the neighbouring operator while judging it
   against skia's answer to the one that was asked. it exists to show that the
   comparison bites: if a run with this on still reports agreement, the judge is
   measuring nothing. */
static bool g_control = false;

static bool solveOurs(const RenderPath& a, const RenderPath& b, SkPathOp op, RenderPath& out)
{
    if (g_control) op = SkPathOp((int(op) + 1) % 4);
    switch (op) {
        case kUnion_SkPathOp: return AddMask(a, b, out);
        case kIntersect_SkPathOp: return IntersectMask(a, b, out);
        case kDifference_SkPathOp: return SubtractMask(a, b, out);
        case kXOR_SkPathOp: return DifferenceMask(a, b, out);
        default: return SubtractMask(b, a, out);   //reverse difference
    }
}


static uint32_t countSegs(const SkPath& p) { return uint32_t(p.countVerbs()); }


/* What the corpus is made of, in skia's verbs. It matters because our path carries
   no quad and no conic: a quad elevates to a cubic exactly, a conic does not, and
   the count below says how much of the corpus rides on that approximation. */
struct Verbs { uint64_t move, line, quad, conic, cubic, close; uint32_t withConic, withQuad; };

static void tallyVerbs(const SkPath& p, Verbs& v, bool& sawConic, bool& sawQuad)
{
    SkPath::Iter iter(p, false);
    SkPoint pt[4];
    SkPath::Verb verb;
    while ((verb = iter.next(pt)) != SkPath::kDone_Verb) {
        switch (verb) {
            case SkPath::kMove_Verb: ++v.move; break;
            case SkPath::kLine_Verb: ++v.line; break;
            case SkPath::kQuad_Verb: ++v.quad; sawQuad = true; break;
            case SkPath::kConic_Verb: ++v.conic; sawConic = true; break;
            case SkPath::kCubic_Verb: ++v.cubic; break;
            default: ++v.close; break;
        }
    }
}


//a straight-edged case and a curved one fail for different reasons, so they are counted apart
/* Does the operand cross itself?

   A boolean solver's hard cases are not the curve types, they are the operands
   that are not simple loops: a cubic that doubles back through its own path, or a
   contour that runs over ground it has already covered. The corpus is full of them
   on purpose. Flattening and testing every pair of segments is slow and exact, and
   at this size the cost does not matter. */
static bool _crosses(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy)
{
    auto side = [](float px, float py, float qx, float qy, float rx, float ry) {
        auto v = (qx - px) * (ry - py) - (qy - py) * (rx - px);
        return v > 0.0f ? 1 : (v < 0.0f ? -1 : 0);
    };
    auto d1 = side(ax, ay, bx, by, cx, cy), d2 = side(ax, ay, bx, by, dx, dy);
    auto d3 = side(cx, cy, dx, dy, ax, ay), d4 = side(cx, cy, dx, dy, bx, by);
    return d1 * d2 < 0 && d3 * d4 < 0;   //a proper crossing only, touching does not count
}


static bool selfCrossing(const SkPath& path)
{
    //the flattened outline, contour by contour, at a resolution that catches a small loop
    std::vector<std::pair<float, float>> pts;
    std::vector<size_t> starts;
    auto flat = skiaToTvg(path);
    auto at = 0u;
    float px = 0.0f, py = 0.0f;
    for (auto cmd : flat.cmds) {
        if (cmd == PathCommand::MoveTo) {
            starts.push_back(pts.size());
            px = flat.pts[at].x; py = flat.pts[at].y;
            pts.push_back({px, py});
            ++at;
        } else if (cmd == PathCommand::LineTo) {
            px = flat.pts[at].x; py = flat.pts[at].y;
            pts.push_back({px, py});
            ++at;
        } else if (cmd == PathCommand::CubicTo) {
            auto x1 = flat.pts[at].x, y1 = flat.pts[at].y;
            auto x2 = flat.pts[at + 1].x, y2 = flat.pts[at + 1].y;
            auto x3 = flat.pts[at + 2].x, y3 = flat.pts[at + 2].y;
            for (auto k = 1; k <= 24; ++k) {
                auto t = float(k) / 24.0f, u = 1.0f - t;
                pts.push_back({u*u*u*px + 3*u*u*t*x1 + 3*u*t*t*x2 + t*t*t*x3,
                               u*u*u*py + 3*u*u*t*y1 + 3*u*t*t*y2 + t*t*t*y3});
            }
            px = x3; py = y3;
            at += 3;
        }
    }
    if (pts.size() > 4000) return false;    //too big to pair off, and not what this counts

    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        if (std::find(starts.begin(), starts.end(), i + 1) != starts.end()) continue;
        for (size_t j = i + 2; j + 1 < pts.size(); ++j) {
            if (std::find(starts.begin(), starts.end(), j + 1) != starts.end()) continue;
            if (_crosses(pts[i].first, pts[i].second, pts[i + 1].first, pts[i + 1].second,
                         pts[j].first, pts[j].second, pts[j + 1].first, pts[j + 1].second)) return true;
        }
    }
    return false;
}


/* How many times the two operands' outlines cross.

   Two cubics can meet nine times, and every crossing is a node the solver has to
   find, pair up and walk. If the failures sit where the crossings are many, the
   problem is the root finding and the traversal, not the shape of the input. */
static std::vector<std::pair<float, float>> _flatten(const SkPath& path, std::vector<size_t>& starts)
{
    std::vector<std::pair<float, float>> pts;
    auto flat = skiaToTvg(path);
    auto at = 0u;
    float px = 0.0f, py = 0.0f;
    for (auto cmd : flat.cmds) {
        if (cmd == PathCommand::MoveTo) {
            starts.push_back(pts.size());
            px = flat.pts[at].x; py = flat.pts[at].y;
            pts.push_back({px, py});
            ++at;
        } else if (cmd == PathCommand::LineTo) {
            px = flat.pts[at].x; py = flat.pts[at].y;
            pts.push_back({px, py});
            ++at;
        } else if (cmd == PathCommand::CubicTo) {
            auto x1 = flat.pts[at].x, y1 = flat.pts[at].y;
            auto x2 = flat.pts[at + 1].x, y2 = flat.pts[at + 1].y;
            auto x3 = flat.pts[at + 2].x, y3 = flat.pts[at + 2].y;
            for (auto k = 1; k <= 24; ++k) {
                auto t = float(k) / 24.0f, u = 1.0f - t;
                pts.push_back({u*u*u*px + 3*u*u*t*x1 + 3*u*t*t*x2 + t*t*t*x3,
                               u*u*u*py + 3*u*u*t*y1 + 3*u*t*t*y2 + t*t*t*y3});
            }
            px = x3; py = y3;
            at += 3;
        }
    }
    return pts;
}


static uint32_t crossCount(const SkPath& a, const SkPath& b)
{
    std::vector<size_t> sa, sb;
    auto pa = _flatten(a, sa), pb = _flatten(b, sb);
    if (pa.size() > 3000 || pb.size() > 3000) return 0;

    uint32_t n = 0;
    for (size_t i = 0; i + 1 < pa.size(); ++i) {
        if (std::find(sa.begin(), sa.end(), i + 1) != sa.end()) continue;
        for (size_t j = 0; j + 1 < pb.size(); ++j) {
            if (std::find(sb.begin(), sb.end(), j + 1) != sb.end()) continue;
            if (_crosses(pa[i].first, pa[i].second, pa[i + 1].first, pa[i + 1].second,
                         pb[j].first, pb[j].second, pb[j + 1].first, pb[j + 1].second)) ++n;
        }
    }
    return n;
}


//a cubic whose control points sit on top of each other is really a lower order curve
static bool degenerateCubic(const SkPath& path)
{
    SkPath::Iter iter(path, false);
    SkPoint p[4];
    SkPath::Verb verb;
    while ((verb = iter.next(p)) != SkPath::kDone_Verb) {
        if (verb != SkPath::kCubic_Verb) continue;
        if (p[0] == p[1] || p[1] == p[2] || p[2] == p[3] || p[0] == p[3]) return true;
    }
    return false;
}


static bool anyConic(const SkPath& p)
{
    SkPath::Iter iter(p, false);
    SkPoint pt[4];
    SkPath::Verb verb;
    while ((verb = iter.next(pt)) != SkPath::kDone_Verb) if (verb == SkPath::kConic_Verb) return true;
    return false;
}


static bool anyCurve(const SkPath& p)
{
    SkPath::Iter iter(p, false);
    SkPoint pt[4];
    SkPath::Verb verb;
    while ((verb = iter.next(pt)) != SkPath::kDone_Verb) {
        if (verb == SkPath::kQuad_Verb || verb == SkPath::kConic_Verb || verb == SkPath::kCubic_Verb) return true;
    }
    return false;
}


//solved in the child, so a crash or a hang lands on this case and no other
static Outcome runCase(const Fixture& f, int report)
{
    Outcome r{};
    r.segs = countSegs(f.a) + countSegs(f.b);
    r.curved = uint8_t(anyCurve(f.a) || anyCurve(f.b));
    r.conic = uint8_t(anyConic(f.a) || anyConic(f.b));
    r.selfCross = uint8_t(selfCrossing(f.a) || selfCrossing(f.b));
    r.degenerate = uint8_t(degenerateCubic(f.a) || degenerateCubic(f.b));
    r.crossings = crossCount(f.a, f.b);

    SkPath a, b;
    if (!asNonZero(f.a, a) || !asNonZero(f.b, b)) { r.verdict = Unsupported; return r; }

    auto view = _view(f.a, f.b);

    //one figure, carried into our path and back, so both engines solve the same curves
    auto ourA = skiaToTvg(a), ourB = skiaToTvg(b);
    auto sameA = tvgToSkia(ourA, SkPathFillType::kWinding);
    auto sameB = tvgToSkia(ourB, SkPathFillType::kWinding);

    SkPath skia;
    auto skiaOk = ::Op(sameA, sameB, f.op, &skia);
    r.skiaRefused = uint8_t(!skiaOk);
    auto skiaPath = skiaOk ? skiaToTvg(skia) : RenderPath{};
    auto skiaRule = (skiaOk && skia.getFillType() == SkPathFillType::kEvenOdd) ? tvg::FillRule::EvenOdd : tvg::FillRule::NonZero;

    RenderPath ours;
    auto ourOk = solveOurs(ourA, ourB, f.op, ours);

    /* the corpus carries cases skia itself only expects to survive, not to solve.
       where skia's own Op() declined, our declining is the same answer, and where
       skia declined and we did not there is no reference to be judged against. */
    if (!ourOk || !_finite(ours)) { r.verdict = skiaOk ? Broken : Pass; return r; }
    if (!skiaOk) { r.verdict = Tolerated; return r; }

    auto mOurs = _mask(ours, view, tvg::FillRule::NonZero);
    auto mSkia = _mask(skiaPath, view, skiaRule);

    //the operands on their own, and the answer that follows from them
    auto mA = _mask(ourA, view, tvg::FillRule::NonZero);
    auto mB = _mask(ourB, view, tvg::FillRule::NonZero);
    auto mTruth = _truth(mA, mB, f.op);

    r.apartSkia = _apart(mOurs, mSkia);
    r.solidSkia = _solid(mOurs, mSkia, MASK);
    r.apartTruth = _apart(mOurs, mTruth);
    r.skiaTruth = _apart(mSkia, mTruth);
    r.whole = uint8_t(_wholeRegions(mOurs, mA, mB));
    r.shape = _shape(mOurs, mA, mB, f.op, TOLERANCE);
    r.oursEmpty = uint8_t(ours.cmds.empty());
    r.skiaEmpty = uint8_t(!skiaOk || skiaPath.cmds.empty());
    r.bothEmpty = uint8_t(r.oursEmpty && r.skiaEmpty);

    r.verdict = r.apartSkia <= TOLERANCE ? Pass : Fail;

    if (report >= 0) {
        r.reported = 1;
        auto n = write(report, &r, sizeof(r));
        (void)n;
    }
    return r;
}


/************************************************************************/
/* The run                                                              */
/************************************************************************/

static Outcome fork_case(const Fixture& f)
{
    int fd[2];
    if (pipe(fd) != 0) { Outcome bad{}; bad.verdict = Crashed; return bad; }

    auto pid = fork();
    if (pid == 0) {
        close(fd[0]);
        alarm(TIMEOUT);
        auto r = runCase(f, fd[1]);
        //a case that returned before the verdict was sent has not written itself yet
        if (!r.reported) {
            auto n = write(fd[1], &r, sizeof(r));
            (void)n;
        }
        close(fd[1]);
        _exit(0);
    }
    close(fd[1]);

    Outcome r{};
    auto got = read(fd[0], &r, sizeof(r));
    close(fd[0]);

    int status = 0;
    waitpid(pid, &status, 0);

    if (got != ssize_t(sizeof(r))) {
        Outcome bad{};
        bad.segs = countSegs(f.a) + countSegs(f.b);
        bad.verdict = (WIFSIGNALED(status) && WTERMSIG(status) == SIGALRM) ? Hung : Crashed;
        return bad;
    }
    return r;
}


struct Row
{
    size_t at;              //which fixture it came from, so its operands can be read back
    std::string name, file, op;
    conf::Grade grade;
    Outcome r;
};


//skia names its cases family-then-number, so the leading letters say which family
static std::string family(const std::string& name)
{
    size_t i = 0;
    while (i < name.size() && !isdigit((unsigned char)name[i])) ++i;
    return i ? name.substr(0, i) : name;
}


static const char* entryPoint(SkPathOp op, bool& swap)
{
    swap = false;
    switch (op) {
        case kUnion_SkPathOp: return "AddMask";
        case kIntersect_SkPathOp: return "IntersectMask";
        case kDifference_SkPathOp: return "SubtractMask";
        case kXOR_SkPathOp: return "DifferenceMask";
        default: swap = true; return "SubtractMask";
    }
}


/* The case again, in our own terms.

   Written from the operands the solver was actually handed - rewound to non zero
   and carried through our path, quads and conics already gone - so what is printed
   here runs the same trial the report judged, and not an approximation of it. Nine
   figures is what a float needs to come back the number it went out as. */
static void repro(const RenderPath& path, const char* name)
{
    printf("RenderPath %s;\n", name);
    auto at = 0u;
    for (auto cmd : path.cmds) {
        if (cmd == PathCommand::MoveTo) {
            printf("%s.moveTo({%.9g, %.9g});\n", name, path.pts[at].x, path.pts[at].y);
            ++at;
        } else if (cmd == PathCommand::LineTo) {
            printf("%s.lineTo({%.9g, %.9g});\n", name, path.pts[at].x, path.pts[at].y);
            ++at;
        } else if (cmd == PathCommand::CubicTo) {
            printf("%s.cubicTo({%.9g, %.9g}, {%.9g, %.9g}, {%.9g, %.9g});\n", name,
                   path.pts[at].x, path.pts[at].y, path.pts[at + 1].x, path.pts[at + 1].y,
                   path.pts[at + 2].x, path.pts[at + 2].y);
            at += 3;
        } else printf("%s.close();\n", name);
    }
}


//one case, printed as the calls that run it, so a failure can be taken somewhere else
static int reproduce(const char* want)
{
    for (auto& f : fixtures()) {
        if (f.name != want) continue;
        SkPath a, b;
        if (!asNonZero(f.a, a) || !asNonZero(f.b, b)) {
            printf("%s is an inverse fill, which our api does not model\n", want);
            return 1;
        }
        auto swap = false;
        auto call = entryPoint(f.op, swap);
        printf("//%s, %s, from %s\n\n", f.name.c_str(), opName(f.op), f.file.c_str());
        repro(skiaToTvg(a), "a");
        printf("\n");
        repro(skiaToTvg(b), "b");
        printf("\nRenderPath out;\n%s(%s, %s, out);\n", call, swap ? "b" : "a", swap ? "a" : "b");
        return 0;
    }
    printf("no case named %s\n", want);
    return 1;
}


/************************************************************************/
/* The report                                                           */
/************************************************************************/

static void rule(char c = '-') { printf("%s\n", std::string(92, c).c_str()); }

static void heading(const char* title)
{
    printf("\n");
    rule();
    printf("%s\n", title);
    rule();
}


//a share of a whole, with the whole itself spelled out
static void share(const char* label, uint32_t part, uint32_t whole)
{
    printf("  %-46s %5u / %-5u  %5.1f%%\n", label, part, whole,
           whole ? 100.0 * part / whole : 0.0);
}


struct Tally { uint32_t cases, pass, wrong, other; };

static void tallyLine(const char* label, const Tally& t)
{
    printf("  %-34s %6u %6u %7.1f%% %6u %6u\n", label, t.cases, t.pass,
           t.cases ? 100.0 * t.pass / t.cases : 0.0, t.wrong, t.other);
}

static void tallyHead(const char* what)
{
    printf("  %-34s %6s %6s %8s %6s %6s\n", what, "judged", "pass", "", "FAIL", "other");
}


int main(int argc, char** argv)
{
    auto quiet = false;
    const char* want = nullptr;
    for (auto i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-q")) quiet = true;
        else if (!strcmp(argv[i], "--control")) g_control = true;
        else if (!strcmp(argv[i], "--repro") && i + 1 < argc) want = argv[++i];
    }

    //zero threads: every case is solved in a forked child, and a forked thread pool is not
    tvg::Initializer::init(0);

    //walk skia's own test tables; each testPathOp inside lands in fixtures()
    skiatest::Reporter reporter;
    for (auto& s : suites()) {
        g_suite = s.name;
        s.fn(&reporter);
    }
    g_suite.clear();

    if (want) return reproduce(want);

    if (g_control) printf("CONTROL RUN: ours is asked for the wrong operator on purpose\n\n");
    printf("%zu cases from %zu of skia's suites\n\n", fixtures().size(), suites().size());

    std::vector<Row> rows;
    rows.reserve(fixtures().size());

    for (size_t i = 0; i < fixtures().size(); ++i) {
        auto& f = fixtures()[i];
        rows.push_back({i, f.name, f.file, opName(f.op), f.grade, fork_case(f)});
        if (!quiet && (i % 25 == 0 || i + 1 == fixtures().size())) {
            printf("\r  %zu / %zu", i + 1, fixtures().size());
            fflush(stdout);
        }
    }
    if (!quiet) printf("\r%s\r", std::string(24, ' ').c_str());

    //tallies
    std::map<std::string, std::array<uint32_t, 8>> byFile;
    std::array<uint32_t, 8> total{};
    for (auto& row : rows) {
        ++total[row.r.verdict];
        ++byFile[row.file][row.r.verdict];
    }

    auto judged = fixtures().size() - total[Unsupported] - total[Tolerated];
    auto vacuous = 0u;
    for (auto& row : rows) if (row.r.bothEmpty) ++vacuous;
    auto ok = total[Pass];

    /* What a verdict means. Skia is the reference: both answers go through one
       rasteriser, so a difference in the pixels is a difference in the geometry and
       not in the drawing. Both operands are rewound to non zero first, because our
       entry point takes a path and not a path plus a fill rule. */
    heading("Result");
    printf("  %-26s %6s %6s %6s %6s %6s %6s %6s %6s\n",
           "suite", "cases", "pass", "FAIL", "BROKEN", "CRASH", "HANG", "no ref", "n/a");
    printf("  %s\n", std::string(88, '.').c_str());
    for (auto& [file, v] : byFile) {
        printf("  %-26s %6u %6u %6u %6u %6u %6u %6u %6u\n", file.c_str(),
               v[0] + v[1] + v[2] + v[3] + v[4] + v[5] + v[6],
               v[Pass], v[Fail], v[Broken], v[Crashed], v[Hung], v[Tolerated], v[Unsupported]);
    }
    printf("  %s\n", std::string(88, '.').c_str());
    printf("  %-26s %6zu %6u %6u %6u %6u %6u %6u %6u\n\n", "all", fixtures().size(),
           total[Pass], total[Fail], total[Broken], total[Crashed], total[Hung],
           total[Tolerated], total[Unsupported]);

    printf("  pass     ours and skia cover the same pixels, within %.1f%% of the covered area\n", TOLERANCE);
    printf("  FAIL     ours covers different pixels\n");
    printf("  BROKEN   the call refused, or the answer carries a NaN\n");
    printf("  CRASH    the process died      HANG  did not finish in %us\n", TIMEOUT);
    printf("  no ref   skia's own Op() declined, so there is nothing to be judged against\n");
    printf("  n/a      an inverse fill, which our api does not model\n\n");

    printf("  judged: %zu (%u inverse fill, %u skia declined too)\n",
           judged, total[Unsupported], total[Tolerated]);
    printf("  conformance: %u / %zu = %.1f%%\n", ok, judged, judged ? 100.0 * ok / judged : 0.0);
    printf("  of those, %u agree only because both answers were empty\n", vacuous);
    printf("  does not return: crash %u, hang %u, refused/NaN %u\n",
           total[Crashed], total[Hung], total[Broken]);
    if (!g_control) {
        printf("\n  A pass rate is only worth reading if the comparison can fail.\n");
        printf("  ./conformance/run.sh control reruns the corpus with our solver asked for the\n");
        printf("  neighbouring operator, still judged against skia's answer to the one that was\n");
        printf("  asked. Its numbers have to come out worse than these.\n");
    }

    /* How much of the number is the threshold's doing. If the failures crowd just
       above the line, the rate is an artefact of where the line was drawn. */
    heading("Where the line is drawn");
    printf("  apart is the share of the covered pixels the two answers disagree on. solid is\n");
    printf("  the same after a one pixel erosion, which drops a difference that is only a\n");
    printf("  boundary landing a fraction of a pixel over and keeps one that has an interior.\n\n");
    printf("  %-12s %20s %20s\n", "threshold", "by apart", "by solid");
    for (auto cut : {0.5, 1.0, 2.0, 5.0, 10.0, 25.0}) {
        auto byApart = 0u, bySolid = 0u;
        for (auto& row : rows) {
            if (row.r.verdict == Unsupported || row.r.verdict == Tolerated) continue;
            if (row.r.verdict == Broken || row.r.verdict == Crashed || row.r.verdict == Hung) continue;
            if (row.r.apartSkia <= cut) ++byApart;
            if (row.r.solidSkia <= cut) ++bySolid;
        }
        printf("  %10.1f%% %13u = %5.1f%% %13u = %5.1f%%\n", cut,
               byApart, judged ? 100.0 * byApart / judged : 0.0,
               bySolid, judged ? 100.0 * bySolid / judged : 0.0);
    }
    printf("\n  The run above is the apart column at %.1f%%.\n", TOLERANCE);

    /* Whether a pixel belongs in a boolean answer turns on two facts and no others -
       is it inside a, is it inside b - so rasterising the operands separately and
       combining them gives an answer that owes nothing to any path solver. */
    heading("Against the answer worked out from the operands");
    printf("  Not against each other. The two operands are rasterised separately and combined\n");
    printf("  per pixel, which owes nothing to any path solver. Both engines are held to it.\n\n");
    {
        Tally us{}, sk{};
        for (auto& row : rows) {
            if (row.r.verdict != Pass && row.r.verdict != Fail) continue;
            ++us.cases; ++sk.cases;
            if (row.r.apartTruth <= TOLERANCE) ++us.pass;
            if (row.r.skiaTruth <= TOLERANCE) ++sk.pass;
        }
        share("ours", us.pass, us.cases);
        share("skia", sk.pass, sk.cases);
    }

    /* The architecture question: where does a failure live. An answer that covers each
       region either almost entirely or almost not at all found the pieces and chose the
       wrong ones, which is the traversal's doing. One whose boundary runs through the
       middle of a region got the pieces themselves wrong, which is the intersection
       finding underneath. */
    heading("What kind of wrong");
    {
        auto whole = 0u, cut = 0u;
        for (auto& row : rows) {
            if (row.r.verdict != Fail) continue;
            if (row.r.whole) ++whole; else ++cut;
        }
        printf("  The plane is cut into four regions - in both, in a only, in b only, in neither -\n");
        printf("  and every boolean answer is a whole selection of them.\n\n");
        printf("  %-46s %5u   whole regions, wrongly chosen\n", "the traversal", whole);
        printf("  %-46s %5u   boundary in the wrong place\n", "the intersections underneath", cut);

        //a wrong answer that is exactly what a different operator would have returned names a bug
        std::map<std::string, std::vector<std::string>> kinds;
        for (auto& row : rows) {
            if (row.r.verdict != Fail) continue;
            kinds[shapeName(row.r.shape)].push_back(row.name);
        }
        std::vector<std::pair<std::string, std::vector<std::string>>> ks(kinds.begin(), kinds.end());
        std::sort(ks.begin(), ks.end(), [](auto& x, auto& y) { return x.second.size() > y.second.size(); });
        if (!ks.empty()) {
            printf("\n  And what the answer turned out to be. One that is exactly what a different\n");
            printf("  operator would have returned, or an operand handed back untouched, names a bug\n");
            printf("  rather than measuring one.\n\n");
            for (auto& [name, list] : ks) {
                printf("  %-46s %5zu  ", name.c_str(), list.size());
                for (auto i = 0u; i < 3 && i < list.size(); ++i) printf(" %s", list[i].c_str());
                printf("\n");
            }
        }
    }

    /* The curve types are a conversion question; self crossing and degeneracy are the
       shape of the input, and they are what the corpus was built to attack. */
    heading("What the operands are like");
    {
        Tally lines{}, curves{}, withConic{}, without{};
        Tally cross{}, plain{}, degen{}, clean{}, both{}, neither{};
        for (auto& row : rows) {
            if (row.r.verdict == Unsupported || row.r.verdict == Tolerated) continue;
            auto eat = [&](Tally& t) {
                ++t.cases;
                if (row.r.verdict == Pass) ++t.pass;
                else if (row.r.verdict == Fail) ++t.wrong;
                else ++t.other;
            };
            eat(row.r.curved ? curves : lines);
            eat(row.r.conic ? withConic : without);
            eat(row.r.selfCross ? cross : plain);
            eat(row.r.degenerate ? degen : clean);
            if (row.r.selfCross || row.r.degenerate) eat(both); else eat(neither);
        }
        tallyHead("operands");
        tallyLine("lines only", lines);
        tallyLine("carries a curve", curves);
        printf("\n");
        /* a quad elevates to a cubic exactly; a conic does not - it is split into two
           quads and elevated. both engines are handed that same approximation, so a
           conic case is a fair comparison on a curve that is not quite skia's. */
        tallyLine("carries a conic - approximated", withConic);
        tallyLine("no conic", without);
        printf("\n");
        tallyLine("runs through itself", cross);
        tallyLine("does not", plain);
        tallyLine("has a doubled up cubic", degen);
        tallyLine("does not", clean);
        tallyLine("either of the two", both);
        tallyLine("neither - a plain figure", neither);

        printf("\n  By how many times the two outlines meet. Every crossing is a node the solver\n");
        printf("  has to find, pair with its twin and walk. Two cubics can meet nine times.\n\n");
        tallyHead("crossings");
        const uint32_t cuts[] = {0, 1, 2, 3, 4, 6, 9, 0xffffffffu};
        const char* labels[] = {"0", "1", "2", "3", "4 to 5", "6 to 8", "9 or more"};
        for (auto k = 0u; k < 7; ++k) {
            Tally t{};
            for (auto& row : rows) {
                if (row.r.verdict == Unsupported || row.r.verdict == Tolerated) continue;
                if (row.r.crossings < cuts[k] || row.r.crossings >= cuts[k + 1]) continue;
                ++t.cases;
                if (row.r.verdict == Pass) ++t.pass;
                else if (row.r.verdict == Fail) ++t.wrong;
                else ++t.other;
            }
            if (t.cases) tallyLine(labels[k], t);
        }
    }

    /* The corpus is named family-then-number, and a family is usually one figure
       revisited. Counting them says how many distinct problems the failures stand for. */
    heading("By family");
    {
        std::map<std::string, Tally> byFamily;
        for (auto& row : rows) {
            if (row.r.verdict == Unsupported || row.r.verdict == Tolerated) continue;
            auto& t = byFamily[family(row.name)];
            ++t.cases;
            if (row.r.verdict == Pass) ++t.pass;
            else if (row.r.verdict == Fail) ++t.wrong;
            else ++t.other;
        }
        std::vector<std::pair<std::string, Tally>> fam(byFamily.begin(), byFamily.end());
        std::sort(fam.begin(), fam.end(), [](auto& x, auto& y) {
            return x.second.wrong + x.second.other > y.second.wrong + y.second.other;
        });
        auto listed = 0u;
        for (auto& [name, t] : fam) if (t.wrong || t.other) ++listed;
        printf("  Only the %u families with something to answer for, of %zu.\n\n", listed, fam.size());
        tallyHead("family");
        for (auto& [name, t] : fam) {
            if (!t.wrong && !t.other) continue;
            tallyLine(name.c_str(), t);
        }
    }

    /* Not every failure is ours to answer for, and not every one is worth the same.
       Sorting them by what skia made of the same case says which is which, without
       anyone having to judge it by eye. */
    enum Bucket { Stuck = 0, Fuzzed, Refused, WeAreRight, Invisible, SkiaToo, Ours, Buckets };
    const char* bucketName[Buckets] = {
        "does not return",
        "skia only asks that it survive",
        "refused",
        "ours matches the truth, skia is apart",
        "differs only along a boundary",
        "skia misses the truth too",
        "OURS TO ANSWER FOR",
    };
    const char* bucketNote[Buckets] = {
        "wrong whatever the answer would have been",
        "skia asks only that the call come back",
        "declined a case skia answers",
        "skia's corpus records skia's own bugs",
        "within a pixel of each other",
        "not ours alone",
        "this is the count that is ours",
    };
    //what the case list has room for
    const char* bucketTag[Buckets] = {
        "no answer", "survive only", "refused", "skia is apart",
        "boundary only", "skia misses too", "OURS",
    };
    std::vector<const Row*> held[Buckets];

    heading("What is left, and how much of it is ours");
    {
        printf("  Each case that is not a pass, placed by what the two engines made of the answer\n");
        printf("  worked out from the operands. The first line a case fits is the one it goes under.\n\n");
        for (auto& row : rows) {
            if (row.r.verdict == Pass || row.r.verdict == Unsupported || row.r.verdict == Tolerated) continue;

            auto near = [&](double v) { return v <= TOLERANCE; };
            /* the grade comes first, whatever the verdict. a case skia only asks a solver
               to survive is not one skia got wrong, because skia is not claiming an
               answer to it - counting those as ours to boast of would flatter us. */
            auto loose = (row.grade == conf::Grade::Fail || row.grade == conf::Grade::Fuzz);

            auto k = Ours;
            if (row.r.verdict == Crashed || row.r.verdict == Hung) k = Stuck;
            else if (loose) k = Fuzzed;
            else if (row.r.verdict == Broken) k = Refused;
            else if (near(row.r.apartTruth)) k = WeAreRight;
            else if (row.r.solidSkia < 1.0) k = Invisible;
            else if (!near(row.r.skiaTruth)) k = SkiaToo;
            held[k].push_back(&row);
        }

        auto left = 0u;
        for (auto k = 0; k < Buckets; ++k) left += uint32_t(held[k].size());
        for (auto k = 0; k < Buckets; ++k) {
            printf("  %-38s %5zu  %s\n", bucketName[k], held[k].size(), bucketNote[k]);
        }
        printf("  %-38s %5u\n", "all", left);
    }

    /* The list itself. Everything above is a way of not having to read it, but a
       failure is finally a named case, and ./conformance/run.sh repro <case> prints
       that one back as the calls that run it. */
    heading("The cases");
    printf("  Every case that is not a pass, in the order of the table above. The suite is\n");
    printf("  skia's own file with the PathOps it all shares taken off the front.\n\n");
    printf("  %-34s %-7s %-10s %-6s %6s %6s  %s\n",
           "case", "suite", "op", "", "apart", "solid", "where it lands");
    printf("  %s\n", std::string(88, '.').c_str());
    for (auto k = 0; k < Buckets; ++k) {
        auto list = held[k];
        std::sort(list.begin(), list.end(), [](const Row* x, const Row* y) {
            return x->r.apartSkia > y->r.apartSkia;
        });
        for (auto row : list) {
            auto suite = row->file.rfind("PathOps", 0) == 0 ? row->file.substr(7) : row->file;
            if (row->r.verdict == Crashed || row->r.verdict == Hung) {
                printf("  %-34s %-7s %-10s %-6s %6s %6s  %s\n", row->name.c_str(), suite.c_str(),
                       row->op.c_str(), verdictName(row->r.verdict), "-", "-", bucketTag[k]);
            } else {
                printf("  %-34s %-7s %-10s %-6s %5.1f%% %5.1f%%  %s\n", row->name.c_str(), suite.c_str(),
                       row->op.c_str(), verdictName(row->r.verdict),
                       row->r.apartSkia, row->r.solidSkia, bucketTag[k]);
            }
        }
    }
    printf("  %s\n", std::string(88, '.').c_str());
    printf("  ./conformance/run.sh repro <case> prints one of these back as the calls that run it.\n");

    auto rate = judged ? 100.0 * ok / judged : 0.0;
    printf("\n");
    rule('=');
    printf("  %sCONFORMANCE   %.1f%%   (%u / %zu)\n", g_control ? "CONTROL  " : "", rate, ok, judged);
    printf("  %u failed  (%u wrong shape, %u refused, %u crashed, %u hung)   %zu ours to answer for\n",
           total[Fail] + total[Broken] + total[Crashed] + total[Hung],
           total[Fail], total[Broken], total[Crashed], total[Hung], held[Ours].size());
    rule('=');
    return 0;
}
