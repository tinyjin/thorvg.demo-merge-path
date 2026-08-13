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

#ifndef _MERGE_PATH_H_
#define _MERGE_PATH_H_

#include <vector>
#include <cfloat>
#include <thorvg-1/thorvg.h>

using namespace std;
using namespace tvg;

struct RenderPath
{
    vector<tvg::PathCommand> cmds;
    vector<tvg::Point> pts;

    void moveTo(const Point& pt) { pts.push_back(pt); cmds.push_back(PathCommand::MoveTo); }
    void lineTo(const Point& pt) { pts.push_back(pt); cmds.push_back(PathCommand::LineTo); }
    void cubicTo(const Point& cnt1, const Point& cnt2, const Point& end) { pts.push_back(cnt1); pts.push_back(cnt2); pts.push_back(end); cmds.push_back(PathCommand::CubicTo); }
    void close() { cmds.push_back(PathCommand::Close); }
};


/************************************************************************/
/* tvgMath.h counterparts                                               */
/************************************************************************/

constexpr float PATH_KAPPA = 0.552284f;

static inline Point operator-(const Point& lhs, const Point& rhs) { return {lhs.x - rhs.x, lhs.y - rhs.y}; }
static inline Point operator+(const Point& lhs, const Point& rhs) { return {lhs.x + rhs.x, lhs.y + rhs.y}; }
static inline Point operator*(const Point& lhs, float rhs) { return {lhs.x * rhs, lhs.y * rhs}; }
static inline bool operator==(const Point& lhs, const Point& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
static inline float dot(const Point& lhs, const Point& rhs) { return lhs.x * rhs.x + lhs.y * rhs.y; }
static inline float length2(const Point& a) { return a.x * a.x + a.y * a.y; }
static inline Point lerp(const Point& start, const Point& end, float t) { return start + (end - start) * t; }

//the operands are snapped onto a grid, which keeps this product exact in double
static inline double cross(const Point& lhs, const Point& rhs) { return (double)lhs.x * rhs.y - (double)lhs.y * rhs.x; }

struct Bezier
{
    Point start, ctrl1, ctrl2, end;

    Point at(float t) const
    {
        auto it = 1.0f - t;
        auto a = lerp(start, ctrl1, t), b = lerp(ctrl1, ctrl2, t), c = lerp(ctrl2, end, t);
        a = a * it + b * t;
        b = b * it + c * t;
        return a * it + b * t;
    }

    void split(Bezier& left, Bezier& right) const
    {
        auto c = lerp(ctrl1, ctrl2, 0.5f);
        left.start = start;
        left.ctrl1 = lerp(start, ctrl1, 0.5f);
        left.ctrl2 = lerp(left.ctrl1, c, 0.5f);
        right.end = end;
        right.ctrl2 = lerp(ctrl2, end, 0.5f);
        right.ctrl1 = lerp(c, right.ctrl2, 0.5f);
        left.end = right.start = lerp(left.ctrl2, right.ctrl1, 0.5f);
    }

    bool flatten(float tolerance) const
    {
        auto diff1 = Point{fabsf(ctrl1.x * 3.0f - start.x * 2.0f - end.x), fabsf(ctrl1.y * 3.0f - start.y * 2.0f - end.y)};
        auto diff2 = Point{fabsf(ctrl2.x * 3.0f - end.x * 2.0f - start.x), fabsf(ctrl2.y * 3.0f - end.y * 2.0f - start.y)};
        return (max(diff1.x, diff2.x) + max(diff1.y, diff2.y)) <= tolerance;
    }

    uint32_t segments(float scale = 1.0f) const
    {
        return _segments(*this, 0.5f / scale, 0);
    }

private:
    static uint32_t _segments(const Bezier& bz, float tolerance, uint32_t depth)
    {
        if (depth >= 10 || bz.flatten(tolerance)) return 1;
        Bezier left, right;
        bz.split(left, right);
        return _segments(left, tolerance, depth + 1) + _segments(right, tolerance, depth + 1);
    }
};


/************************************************************************/
/* Internal Class Implementation                                        */
/************************************************************************/

namespace {

//the sw rasterizer quantizes to 1/256 px (tvgSwRle.cpp PIXEL_BITS), match its grid
constexpr float PATHOP_PRECISION = 256.0f;

enum class PathOp : uint8_t { Add = 0, Intersect, Subtract };

struct Edge
{
    Point pt1, pt2;
    bool used = false;

    Point mid() const { return (pt1 + pt2) * 0.5f; }
    Point dir() const { return pt2 - pt1; }
    void reverse() { auto tmp = pt1; pt1 = pt2; pt2 = tmp; }
};

struct Cut
{
    float t;
    Point pt;
};

}


/* snapping every point onto the grid turns the coincidence into a plain equality
   test instead of a tolerance guess. the error is bounded by a half grid cell,
   far below the curve flattening error that is already accepted. */
static Point _snap(const Point& pt, float precision)
{
    return {roundf(pt.x * precision) / precision, roundf(pt.y * precision) / precision};
}


static void _pushEdge(vector<Edge>& edges, const Point& pt1, const Point& pt2)
{
    if (!(pt1 == pt2)) edges.push_back({pt1, pt2});
}


//flattens the path into a set of edges. every contour is implicitly closed.
static void _collect(const RenderPath& path, vector<Edge>& edges, float scale, float precision)
{
    auto pts = path.pts.data();
    Point start{}, cur{};
    auto opened = false;

    edges.reserve(edges.size() + path.pts.size() * 2);

    for (auto cmd : path.cmds) {
        switch (cmd) {
            case PathCommand::MoveTo: {
                if (opened) _pushEdge(edges, cur, start);
                start = cur = _snap(*pts++, precision);
                opened = true;
                break;
            }
            case PathCommand::LineTo: {
                auto to = _snap(*pts++, precision);
                _pushEdge(edges, cur, to);
                cur = to;
                break;
            }
            case PathCommand::CubicTo: {
                Bezier bz{cur, pts[0], pts[1], pts[2]};
                auto cnt = bz.segments(scale);
                auto step = 1.0f / cnt;
                for (uint32_t i = 1; i <= cnt; ++i) {
                    auto to = _snap((i == cnt) ? bz.end : bz.at(step * i), precision);
                    _pushEdge(edges, cur, to);
                    cur = to;
                }
                pts += 3;
                break;
            }
            case PathCommand::Close: {
                if (opened) _pushEdge(edges, cur, start);
                cur = start;
                opened = false;
                break;
            }
        }
    }
    if (opened) _pushEdge(edges, cur, start);
}


static double _area(const vector<Edge>& edges)
{
    auto sum = 0.0;
    for (auto& e : edges) sum += cross(e.pt1, e.pt2);
    return 0.5 * sum;
}


static int32_t _winding(const vector<Edge>& edges, const Point& pt)
{
    int32_t winding = 0;

    for (auto& e : edges) {
        if (e.pt1.y <= pt.y) {
            if (e.pt2.y > pt.y && cross(e.dir(), pt - e.pt1) > 0.0) ++winding;
        } else if (e.pt2.y <= pt.y && cross(e.dir(), pt - e.pt1) < 0.0) --winding;
    }
    return winding;
}


//both operands have to be cut at the very same point, so the evaluation order is
//decided by the edges themselves, not by which side is being split.
static bool _prior(const Edge& lhs, const Edge& rhs)
{
    if (lhs.pt1.x != rhs.pt1.x) return lhs.pt1.x < rhs.pt1.x;
    if (lhs.pt1.y != rhs.pt1.y) return lhs.pt1.y < rhs.pt1.y;
    if (lhs.pt2.x != rhs.pt2.x) return lhs.pt2.x < rhs.pt2.x;
    return lhs.pt2.y < rhs.pt2.y;
}


static bool _crossing(const Edge& lhs, const Edge& rhs, Point& out, float precision)
{
    auto r = lhs.dir();
    auto s = rhs.dir();
    auto denom = cross(r, s);
    if (denom == 0.0) return false;

    auto qp = rhs.pt1 - lhs.pt1;
    auto tn = cross(qp, s);
    auto un = cross(qp, r);
    if (denom < 0.0) {
        denom = -denom;
        tn = -tn;
        un = -un;
    }
    if (tn < 0.0 || tn > denom || un < 0.0 || un > denom) return false;

    auto t = tn / denom;
    out = _snap({lhs.pt1.x + float(r.x * t), lhs.pt1.y + float(r.y * t)}, precision);

    return true;
}


static void _cut(vector<Cut>& cuts, const Edge& edge, const Point& pt)
{
    auto t = dot(pt - edge.pt1, edge.dir()) / length2(edge.dir());
    if (t <= 0.0f || t >= 1.0f) return;

    size_t idx = 0;
    while (idx < cuts.size() && cuts[idx].t < t) ++idx;
    if (idx < cuts.size() && cuts[idx].pt == pt) return;

    cuts.insert(cuts.begin() + idx, {t, pt});
}


//subdivides the edges at every crossing with the counterpart.
static void _split(vector<Edge>& edges, const vector<Edge>& others, float precision)
{
    vector<Edge> out;
    vector<Cut> cuts;

    out.reserve(edges.size());

    for (auto& e : edges) {
        cuts.clear();

        for (auto& o : others) {
            Point pt;
            if (cross(e.dir(), o.dir()) == 0.0) {
                //collinear: cut at the overlapping ends so the shared edges stay identical
                if (cross(o.pt1 - e.pt1, e.dir()) != 0.0) continue;
                _cut(cuts, e, o.pt1);
                _cut(cuts, e, o.pt2);
            } else if (_prior(e, o) ? _crossing(e, o, pt, precision) : _crossing(o, e, pt, precision)) {
                _cut(cuts, e, pt);
            }
        }

        auto prev = e.pt1;
        for (auto& cut : cuts) {
            _pushEdge(out, prev, cut.pt);
            prev = cut.pt;
        }
        _pushEdge(out, prev, e.pt2);
    }

    edges = std::move(out);
}


static Edge* _coincident(vector<Edge>& edges, const Edge& edge)
{
    for (auto& e : edges) {
        if (e.used) continue;
        if (e.pt1 == edge.pt1 && e.pt2 == edge.pt2) return &e;
        if (e.pt1 == edge.pt2 && e.pt2 == edge.pt1) return &e;
    }
    return nullptr;
}


/* an edge lying on the counterpart's boundary has no inside/outside answer, so it
   is resolved by the operation and the relative direction instead of the winding.

                     same direction        opposite direction
        Add          keep a single copy    drop both
        Intersect    keep a single copy    drop both
        Subtract     drop both             keep the lhs copy          */
static void _classify(vector<Edge>& lhs, vector<Edge>& rhs, PathOp op, vector<Edge>& out)
{
    for (auto& e : lhs) {
        if (auto pair = _coincident(rhs, e)) {
            pair->used = true;
            auto sameDir = (e.pt1 == pair->pt1);
            if (op == PathOp::Subtract ? !sameDir : sameDir) out.push_back({e.pt1, e.pt2});
            continue;
        }
        auto inside = (_winding(rhs, e.mid()) != 0);
        if (op == PathOp::Intersect ? inside : !inside) out.push_back({e.pt1, e.pt2});
    }

    for (auto& e : rhs) {
        if (e.used) continue;
        auto inside = (_winding(lhs, e.mid()) != 0);
        if (op == PathOp::Add ? !inside : inside) {
            //the subtracted region is carved out, so its boundary runs backwards
            if (op == PathOp::Subtract) out.push_back({e.pt2, e.pt1});
            else out.push_back({e.pt1, e.pt2});
        }
    }
}


//walks the survived edges back into the contours.
static void _chain(vector<Edge>& edges, RenderPath& out, float precision)
{
    vector<Point> contour;

    out.cmds.reserve(out.cmds.size() + edges.size());
    out.pts.reserve(out.pts.size() + edges.size());

    for (auto& head : edges) {
        if (head.used) continue;

        contour.clear();
        contour.push_back(head.pt1);
        contour.push_back(head.pt2);
        head.used = true;

        auto cur = &head;
        while (true) {
            Edge* next = nullptr;
            auto min = FLT_MAX;
            //at a crossing, the least turning edge continues the contour
            for (auto& e : edges) {
                if (e.used || !(e.pt1 == cur->pt2)) continue;
                auto turn = fabsf(atan2f(cross(cur->dir(), e.dir()), dot(cur->dir(), e.dir())));
                if (turn < min) {
                    min = turn;
                    next = &e;
                }
            }
            if (!next) break;
            next->used = true;
            cur = next;
            contour.push_back(cur->pt2);
            if (cur->pt2 == contour.front()) break;
        }

        if (contour.size() < 4) continue;

        //snapping can leave a sliver thinner than the grid, it is not a contour
        auto area = 0.0;
        for (size_t i = 1; i < contour.size(); ++i) area += cross(contour[i - 1], contour[i]);
        if (fabs(area) * 0.5 * precision < 1.0) continue;

        out.moveTo(contour.front());
        for (size_t i = 1; i < contour.size() - 1; ++i) out.lineTo(contour[i]);
        out.close();
    }
}


static bool _op(const RenderPath& lhs, const RenderPath& rhs, RenderPath& out, PathOp op, float scale = 1.0f)
{
    auto precision = PATHOP_PRECISION * scale;
    vector<Edge> a, b, survived;

    _collect(lhs, a, scale, precision);
    _collect(rhs, b, scale, precision);
    if (a.empty() || b.empty()) return false;

    //the operands must share the winding direction
    if (_area(a) * _area(b) < 0.0) {
        for (auto& e : b) e.reverse();
    }

    //each side is cut against the intact counterpart, otherwise the crossings would
    //be solved from different segments and drift apart
    auto ra = a;
    auto rb = b;
    _split(a, rb, precision);
    _split(b, ra, precision);

    _classify(a, b, op, survived);
    if (survived.empty()) return false;

    _chain(survived, out, precision);

    return true;
}


/************************************************************************/
/* External Class Implementation                                        */
/************************************************************************/

inline bool AddMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    return _op(a, b, out, PathOp::Add);
}

inline bool SubtractMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    return _op(a, b, out, PathOp::Subtract);
}

inline bool IntersectMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    return _op(a, b, out, PathOp::Intersect);
}

inline bool DifferenceMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    //exclude intersections == (a - b) + (b - a)
    auto ret = _op(a, b, out, PathOp::Subtract);
    return _op(b, a, out, PathOp::Subtract) | ret;
}

#endif //_MERGE_PATH_H_
