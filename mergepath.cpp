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

#include "mergepath.h"

#define PATHOP_EPSILON 1e-5f
#define PATHOP_TOLERANCE 1e-5f
#define PATHOP_FLATNESS 2e-4f
#define PATHOP_STRAIGHT 1e-5f
#define PATHOP_DEPTH 24
#define PATHOP_OVERLAP 9


/************************************************************************/
/* Missing members of the thorvg types                                  */
/************************************************************************/

//tvgMath.h
struct BBox : compat::BBox
{
    BBox() {}
    BBox(const Point& min, const Point& max) : compat::BBox{min, max} {}

    //BBox carries no overlap test
    bool intersected(const BBox& rhs) const
    {
        return !(max.x < rhs.min.x || rhs.max.x < min.x ||
                 max.y < rhs.min.y || rhs.max.y < min.y);
    }
};


struct Normalizer
{
    Point offset{0.0f, 0.0f};
    float scale = 1.0f;

    Normalizer() {}

    Normalizer(const BBox& box)
    {
        offset = {(box.min.x + box.max.x) * 0.5f, (box.min.y + box.max.y) * 0.5f};
        auto span = fmaxf(box.max.x - box.min.x, box.max.y - box.min.y);
        if (!(span > 0.0f) || !isfinite(span)) return;
        auto exp = fmaxf(-60.0f, fminf(60.0f, ceilf(log2f(span))));
        scale = exp2f(-exp);
    }

    Point in(const Point& pt) const { return (pt - offset) * scale; }
    Point out(const Point& pt) const { return pt * (1.0f / scale) + offset; }
};


//tvgMath.h
struct Bezier : compat::Bezier
{
    Bezier() {}
    Bezier(const Point& start, const Point& ctrl1, const Point& ctrl2, const Point& end) : compat::Bezier(start, ctrl1, ctrl2, end) {}
    Bezier(const compat::Bezier& rhs) : compat::Bezier(rhs) {}

    //the piece between two parameters
    Bezier sub(float t0, float t1) const
    {
        auto u0 = 1.0f - t0, u1 = 1.0f - t1;
        Bezier out;
        out.start = start * (u0 * u0 * u0) + ctrl1 * (3.0f * t0 * u0 * u0) + ctrl2 * (3.0f * t0 * t0 * u0) + end * (t0 * t0 * t0);
        out.ctrl1 = start * (u0 * u0 * u1) + ctrl1 * (2.0f * t0 * u0 * u1 + u0 * u0 * t1) + ctrl2 * (t0 * t0 * u1 + 2.0f * u0 * t0 * t1) + end * (t0 * t0 * t1);
        out.ctrl2 = start * (u0 * u1 * u1) + ctrl1 * (t0 * u1 * u1 + 2.0f * u0 * u1 * t1) + ctrl2 * (2.0f * t0 * t1 * u1 + u0 * t1 * t1) + end * (t0 * t1 * t1);
        out.end = start * (u1 * u1 * u1) + ctrl1 * (3.0f * t1 * u1 * u1) + ctrl2 * (3.0f * t1 * t1 * u1) + end * (t1 * t1 * t1);
        return out;
    }

    //the subtracted operand is walked backwards
    Bezier reverse() const
    {
        return {end, ctrl2, ctrl1, start};
    }

    //the bezier's bounding box
    BBox bounds() const
    {
        BBox box;
        box.min = {fminf(fminf(start.x, ctrl1.x), fminf(ctrl2.x, end.x)), fminf(fminf(start.y, ctrl1.y), fminf(ctrl2.y, end.y))};
        box.max = {fmaxf(fmaxf(start.x, ctrl1.x), fmaxf(ctrl2.x, end.x)), fmaxf(fmaxf(start.y, ctrl1.y), fmaxf(ctrl2.y, end.y))};
        return box;
    }

    //check if the bezier is a line
    bool line() const
    {
        auto chord = end - start;
        auto leng = length(chord);
        if (leng < 1e-6f) return true;
        return fabsf(cross(chord, ctrl1 - start)) / leng < PATHOP_STRAIGHT && fabsf(cross(chord, ctrl2 - start)) / leng < PATHOP_STRAIGHT;
    }

    //check if the bezier is overlapped, 0 apart, +1 running along, -1 against
    int32_t overlapped(const Bezier& rhs) const
    {
        auto same = [](const Point& lhs, const Point& rhs) {
            return length2(lhs - rhs) < PATHOP_TOLERANCE * PATHOP_TOLERANCE;
        };
        if (same(start, rhs.start) && same(ctrl1, rhs.ctrl1) && same(ctrl2, rhs.ctrl2) && same(end, rhs.end)) return 1;
        if (same(start, rhs.end) && same(ctrl1, rhs.ctrl2) && same(ctrl2, rhs.ctrl1) && same(end, rhs.start)) return -1;
        return 0;
    }

    bool holds(const Point& pt) const
    {
        auto lo = Point{fminf(fminf(start.x, ctrl1.x), fminf(ctrl2.x, end.x)), fminf(fminf(start.y, ctrl1.y), fminf(ctrl2.y, end.y))};
        auto hi = Point{fmaxf(fmaxf(start.x, ctrl1.x), fmaxf(ctrl2.x, end.x)), fmaxf(fmaxf(start.y, ctrl1.y), fmaxf(ctrl2.y, end.y))};
        auto slack = PATHOP_TOLERANCE;
        return pt.x >= lo.x - slack && pt.x <= hi.x + slack && pt.y >= lo.y - slack && pt.y <= hi.y + slack;
    }

    float project(const Point& pt) const
    {
        auto curve = *this;
        auto lo = 0.0f, hi = 1.0f;

        for (uint32_t i = 0; i < PATHOP_DEPTH; ++i) {
            Bezier left, right;
            curve.split(left, right);
            auto l = left.holds(pt), r = right.holds(pt);

            if (l && r) {
                if (length2(left.at(0.5f) - pt) <= length2(right.at(0.5f) - pt)) r = false;
                else l = false;
            }

            auto mid = (lo + hi) * 0.5f;
            if (l) {
                curve = left;
                hi = mid;
            } else if (r) {
                curve = right;
                lo = mid;
            } else return -1.0f;
        }
        return (lo + hi) * 0.5f;
    }

    //convert the bezier from a line
    static Bezier line(const Point& start, const Point& end)
    {
        return {start, lerp(start, end, 1.0f / 3.0f), lerp(start, end, 2.0f / 3.0f), end};
    }
};


//tvgInlist.h
template<typename T>
struct Inlist : compat::Inlist<T>
{
    void insert(T* element, T* at)
    {
        if (!at) {
            this->back(element);
            return;
        }
        element->prev = at->prev;
        element->next = at;
        if (at->prev) at->prev->next = element;
        else this->head = element;
        at->prev = element;
        ++this->count;
    }
};


/************************************************************************/
/* Internal Class Implementation                                        */
/************************************************************************/

namespace {

enum class PathOp : uint8_t { Add = 0, Intersect, Subtract };

struct Segment;
struct Contour;

struct Intersection
{
    INLIST_ITEM(Intersection);

    Segment* segment = nullptr;
    Intersection* pair = nullptr;
    Bezier* prevBezier = nullptr;
    Bezier* nextBezier = nullptr;
    float t = 0.0f;
    bool inside{};
    bool crossing = true;
    bool visited{};

    ~Intersection()
    {
        delete(prevBezier);
        delete(nextBezier);
    }
};

//loop style sorting instead of recursion
static Intersection* _merged(Intersection* lhs, Intersection* rhs)
{
    Intersection* out = nullptr;
    auto tail = &out;

    while (lhs && rhs) {
        if (lhs->t <= rhs->t) {
            *tail = lhs;
            lhs = lhs->next;
        }
        else {
            *tail = rhs;
            rhs = rhs->next;
        }
        tail = &(*tail)->next;
    }
    *tail = lhs ? lhs : rhs;

    return out;
}


static Intersection* _sorted(Intersection* head)
{
    if (!head || !head->next) return head;

    auto slow = head, fast = head->next;
    while (fast && fast->next) {
        slow = slow->next;
        fast = fast->next->next;
    }
    auto rhs = slow->next;
    slow->next = nullptr;

    return _merged(_sorted(head), _sorted(rhs));
}


struct Segment
{
    INLIST_ITEM(Segment);

    Bezier bezier;
    Contour* parent = nullptr;
    Inlist<Intersection> intersections;
    Segment* twin = nullptr;  //the piece of the other path this one runs along
    int32_t coincident{};     //0 if apart, +1 if the twin runs along, -1 if against

    void sort()
    {
        if (intersections.count < 2) return;

        intersections.head = _sorted(intersections.head);

        Intersection* prev = nullptr;
        INLIST_FOREACH(intersections, cur) {
            cur->prev = prev;
            prev = cur;
        }
        intersections.tail = prev;
    }

    //hands each intersection the curve pieces on both sides
    void split()
    {
        INLIST_FOREACH(intersections, cur) {
            auto from = cur->prev ? cur->prev->t : 0.0f;
            auto to = cur->next ? cur->next->t : 1.0f;
            cur->prevBezier = new Bezier(bezier.sub(from, cur->t));
            cur->nextBezier = new Bezier(bezier.sub(cur->t, to));
        }
    }

    //cyclic
    Segment* nextSegment();
    Segment* prevSegment();
};

struct Contour
{
    INLIST_ITEM(Contour);

    Inlist<Segment> segments;
    bool rhs = false;
    bool turn = false;
    uint32_t depth = 0;
    Point probe{};
};

Segment* Segment::nextSegment() { return next ? next : parent->segments.head; }
Segment* Segment::prevSegment() { return prev ? prev : parent->segments.tail; }

struct Root
{
    float t, u; // lhs(t) = rhs(u)
};

struct Hit
{
    Segment* lhs;
    Segment* rhs;
    float t, u;
};

}


/************************************************************************/
/* Curve Math                                                           */
/************************************************************************/

//refines a crossing on the intact curves, the isolation is only a seed
static void _refine(const Bezier& lhs, const Bezier& rhs, Root& root)
{
    for (uint32_t i = 0; i < 8; ++i) {
        auto diff = lhs.at(root.t) - rhs.at(root.u);
        if (length2(diff) < PATHOP_EPSILON * PATHOP_EPSILON) return;

        auto dl = lhs.tangent(root.t);
        auto dr = rhs.tangent(root.u);
        auto det = cross(dl, dr) * -1.0f;
        if (fabsf(det) < PATHOP_EPSILON) return;

        root.t -= (diff.x * -dr.y - diff.y * -dr.x) / det;
        root.u -= (dl.x * diff.y - dl.y * diff.x) / det;
        root.t = fmaxf(0.0f, fminf(1.0f, root.t));
        root.u = fmaxf(0.0f, fminf(1.0f, root.u));
    }
}


//bezier subdivision
static void _isolate(const Bezier& lhs, float lt0, float lt1, const Bezier& rhs, float rt0, float rt1, uint32_t depth, vector<Root>& roots)
{
    auto lbox = lhs.bounds();
    if (!lbox.intersected(rhs.bounds())) return;

    constexpr float flatness = PATHOP_FLATNESS;
    auto lflat = lhs.flatten(flatness);
    auto rflat = rhs.flatten(flatness);

    if (depth >= PATHOP_DEPTH || (lflat && rflat)) {
        auto r = lhs.end - lhs.start;
        auto s = rhs.end - rhs.start;
        auto denom = cross(r, s);
        if (fabsf(denom) < PATHOP_EPSILON * PATHOP_EPSILON) return;

        auto qp = rhs.start - lhs.start;
        auto t = cross(qp, s) / denom;
        auto u = cross(qp, r) / denom;
        if (t < -PATHOP_EPSILON || t > 1.0f + PATHOP_EPSILON) return;
        if (u < -PATHOP_EPSILON || u > 1.0f + PATHOP_EPSILON) return;

        roots.push_back({lt0 + (lt1 - lt0) * fmaxf(0.0f, fminf(1.0f, t)), rt0 + (rt1 - rt0) * fmaxf(0.0f, fminf(1.0f, u))});
        return;
    }

    Bezier ll, lr, rl, rr;
    auto lmid = (lt0 + lt1) * 0.5f;
    auto rmid = (rt0 + rt1) * 0.5f;

    if (lflat) {
        rhs.split(rl, rr);
        _isolate(lhs, lt0, lt1, rl, rt0, rmid, depth + 1, roots);
        _isolate(lhs, lt0, lt1, rr, rmid, rt1, depth + 1, roots);
    } else if (rflat) {
        lhs.split(ll, lr);
        _isolate(ll, lt0, lmid, rhs, rt0, rt1, depth + 1, roots);
        _isolate(lr, lmid, lt1, rhs, rt0, rt1, depth + 1, roots);
    } else {
        lhs.split(ll, lr);
        rhs.split(rl, rr);
        _isolate(ll, lt0, lmid, rl, rt0, rmid, depth + 1, roots);
        _isolate(ll, lt0, lmid, rr, rmid, rt1, depth + 1, roots);
        _isolate(lr, lmid, lt1, rl, rt0, rmid, depth + 1, roots);
        _isolate(lr, lmid, lt1, rr, rmid, rt1, depth + 1, roots);
    }
}


/************************************************************************/
/* Path Build                                                           */
/************************************************************************/

//the signed area of the anchor polygon. only its sign is read, to tell the winding direction apart
static float _area(const RenderPath& path)
{
    auto pts = path.pts.data();
    Point start{}, cur{};
    auto sum = 0.0f;

    for (auto cmd : path.cmds) {
        switch (cmd) {
            case PathCommand::MoveTo: {
                sum += cross(cur, start);
                start = cur = *pts++;
                break;
            }
            case PathCommand::LineTo: {
                sum += cross(cur, *pts);
                cur = *pts++;
                break;
            }
            case PathCommand::CubicTo: {
                sum += cross(cur, pts[2]);
                cur = pts[2]; pts += 3;
                break;
            }
            case PathCommand::Close: {
                sum += cross(cur, start);
                cur = start;
                break;
            }
        }
    }
    return 0.5f * (sum + cross(cur, start));
}


static BBox _bounds(const RenderPath& path)
{
    BBox box{{FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX}};
    for (auto& pt : path.pts) {
        box.min = {fminf(box.min.x, pt.x), fminf(box.min.y, pt.y)};
        box.max = {fmaxf(box.max.x, pt.x), fmaxf(box.max.y, pt.y)};
    }
    return box;
}


//taken as it stands, only the implicit closing the solver would have added is made explicit
static void _copy(const RenderPath& path, RenderPath& out)
{
    out.cmds.insert(out.cmds.end(), path.cmds.begin(), path.cmds.end());
    out.pts.insert(out.pts.end(), path.pts.begin(), path.pts.end());
    if (out.cmds.back() != PathCommand::Close) out.close();
}


static void _append(Contour* contour, const Bezier& bezier)
{
    if (length2(bezier.end - bezier.start) < PATHOP_TOLERANCE * PATHOP_TOLERANCE) return;
    auto segment = new Segment;
    segment->bezier = bezier;
    segment->parent = contour;
    contour->segments.back(segment);
}


static void _contour(const RenderPath& path, bool rhs, const Normalizer& norm, Inlist<Contour>& out)
{
    auto pts = path.pts.data();
    Contour* contour = nullptr;
    Point start{}, cur{};

    for (auto cmd : path.cmds) {
        switch (cmd) {
            case PathCommand::MoveTo: {
                if (contour) _append(contour, Bezier::line(cur, start));
                contour = new Contour;
                contour->rhs = rhs;
                out.back(contour);
                start = cur = norm.in(*pts++);
                break;
            }
            case PathCommand::LineTo: {
                if (contour) _append(contour, Bezier::line(cur, norm.in(*pts)));
                cur = norm.in(*pts++);
                break;
            }
            case PathCommand::CubicTo: {
                if (contour) _append(contour, {cur, norm.in(pts[0]), norm.in(pts[1]), norm.in(pts[2])});
                cur = norm.in(pts[2]);
                pts += 3;
                break;
            }
            case PathCommand::Close: {
                if (contour) _append(contour, Bezier::line(cur, start));
                cur = start;
                break;
            }
        }
    }
    if (contour) _append(contour, Bezier::line(cur, start));

    //drop the contours that carry no area
    {
        INLIST_SAFE_FOREACH(out, empty) {
            if (empty->segments.count < 2) {
                out.remove(empty);
                delete(empty);
            }
        }
    }
}


static void _reverse(Contour* contour)
{
    Inlist<Segment> reversed;
    while (auto segment = contour->segments.front()) {
        segment->bezier = segment->bezier.reverse();
        reversed.front(segment);
    }
    contour->segments = reversed;
    reversed.head = reversed.tail = nullptr;
}


static float _area(const Contour* contour)
{
    auto sum = 0.0f;
    INLIST_FOREACH(contour->segments, segment) sum += cross(segment->bezier.start, segment->bezier.end);
    return 0.5f * sum;
}


//count curve's Extrema points
static uint32_t _turns(const Bezier& bz, float* out)
{
    auto d1 = bz.ctrl1.y - bz.start.y;
    auto d2 = bz.ctrl2.y - bz.ctrl1.y;
    auto d3 = bz.end.y - bz.ctrl2.y;

    //y'(t) = 3(at² + bt + c)
    auto a = d1 - 2.0f * d2 + d3;
    auto b = 2.0f * (d2 - d1);
    auto c = d1;

    auto scale = fmaxf(fabsf(a), fmaxf(fabsf(b), fabsf(c)));
    if (scale < FLT_MIN) return 0;

    uint32_t cnt = 0;
    if (fabsf(a) < scale * PATHOP_EPSILON) {
        if (fabsf(b) >= scale * PATHOP_EPSILON) out[cnt++] = -c / b;
    } else {
        auto disc = b * b - 4.0f * a * c;
        if (disc < 0.0f) return 0;
        disc = sqrtf(disc);
        out[cnt++] = (-b + disc) / (2.0f * a);
        out[cnt++] = (-b - disc) / (2.0f * a);
    }

    //only the ones strictly inside actually cut the curve
    uint32_t n = 0;
    for (uint32_t i = 0; i < cnt; ++i) {
        if (out[i] > 0.0f && out[i] < 1.0f) out[n++] = out[i];
    }
    if (n == 2 && out[0] > out[1]) {
        auto t = out[0];
        out[0] = out[1];
        out[1] = t;
    }
    return n;
}


//ray casting the pt to every piece
static int32_t _winding(const Contour* contour, const Point& pt)
{
    int32_t winding = 0;
    float turns[2];

    INLIST_FOREACH(contour->segments, segment) {
        auto& bz = segment->bezier;
        auto cnt = _turns(bz, turns);
        auto t0 = 0.0f;
        auto y0 = bz.start.y;

        for (uint32_t i = 0; i <= cnt; ++i) {
            auto t1 = (i < cnt) ? turns[i] : 1.0f;
            auto y1 = (i < cnt) ? bz.at(t1).y : bz.end.y;
            auto up = (y0 <= pt.y && pt.y < y1);
            auto down = (y1 <= pt.y && pt.y < y0);

            if (up || down) { //intersection candidate
                //binary search the intersection point
                auto lo = t0, hi = t1;
                for (uint32_t k = 0; k < 30; ++k) {
                    auto mid = (lo + hi) * 0.5f;
                    if ((bz.at(mid).y <= pt.y) == up) lo = mid;
                    else hi = mid;
                }
                if (bz.at((lo + hi) * 0.5f).x > pt.x) winding += up ? 1 : -1;
            }
            t0 = t1;
            y0 = y1;
        }
    }
    return winding;
}


static int32_t _winding(const Inlist<Contour>& path, const Point& pt)
{
    int32_t winding = 0;
    INLIST_FOREACH(path, contour) winding += _winding(contour, pt);
    return winding;
}


//turns every contour to agree with its nesting depth
static void _orient(Inlist<Contour>& path)
{
    INLIST_FOREACH(path, contour) {
        contour->probe = contour->segments.head->bezier.at(0.5f);
        contour->depth = 0;
    }
    INLIST_FOREACH(path, contour) {
        INLIST_FOREACH(path, other) {
            if (other != contour && _winding(other, contour->probe) != 0) ++contour->depth;
        }
    }

    INLIST_FOREACH(path, contour) {
        auto facing = _area(contour);

        if (contour->depth > 0) {
            INLIST_FOREACH(path, other) {
                if (other == contour || other->depth > 0) continue;
                if (_winding(other, contour->probe) == 0) continue;
                facing = _area(other);
                break;
            }
        }
        contour->turn = (facing < 0.0f);
    }

    INLIST_SAFE_FOREACH(path, contour) {
        if (contour->turn) _reverse(contour);
    }
}


/************************************************************************/
/* Intersection                                                         */
/************************************************************************/

static void _pair(Segment* lhs, float lt, Segment* rhs, float rt)
{
    auto a = new Intersection;
    auto b = new Intersection;

    a->segment = lhs;
    a->pair = b;
    a->t = lt;

    b->segment = rhs;
    b->pair = a;
    b->t = rt;

    lhs->intersections.back(a);
    rhs->intersections.back(b);
}


static bool _duplicated(const vector<Root>& roots, const Root& root)
{
    for (auto& r : roots) {
        if (fabsf(r.t - root.t) < 1e-3f && fabsf(r.u - root.u) < 1e-3f) return true;
    }
    return false;
}


static void _cut(Segment* segment, const vector<float>& ts)
{
    auto& list = segment->parent->segments;
    auto from = 0.0f;

    for (auto t : ts) {
        auto piece = new Segment;
        piece->bezier = segment->bezier.sub(from, t);
        piece->parent = segment->parent;
        list.insert(piece, segment);
        from = t;
    }
    segment->bezier = segment->bezier.sub(from, 1.0f);
}


static float _site(Segment* segment, Segment* other)
{
    auto& corner = other->bezier.start;
    if (!segment->bezier.holds(corner)) return -1.0f;

    if (length2(corner - segment->bezier.start) < PATHOP_TOLERANCE * PATHOP_TOLERANCE) return -1.0f;
    if (length2(corner - segment->bezier.end) < PATHOP_TOLERANCE * PATHOP_TOLERANCE) return -1.0f;

    auto t = segment->bezier.project(corner);
    if (t < 0.0f) return -1.0f;

    auto before = other->prevSegment()->bezier.at(0.95f);
    auto after = other->bezier.at(0.05f);
    if (segment->bezier.project(before) < 0.0f && segment->bezier.project(after) < 0.0f) return -1.0f;

    return t;
}


static void _slice(Inlist<Contour>& path, const Inlist<Contour>& other)
{
    vector<float> ts;

    INLIST_FOREACH(path, contour) {
        INLIST_FOREACH(contour->segments, segment) {
            ts.clear();

            INLIST_FOREACH(other, oc) {
                INLIST_FOREACH(oc->segments, os) {
                    auto t = _site(segment, os);
                    if (t >= 0.0f) ts.push_back(t);
                }
            }
            if (ts.empty()) continue;

            sort(ts.begin(), ts.end());
            ts.erase(unique(ts.begin(), ts.end(), [](float lhs, float rhs) { return rhs - lhs < PATHOP_EPSILON; }), ts.end());
            _cut(segment, ts);
        }
    }
}


static uint32_t _intersect(Inlist<Contour>& lhs, Inlist<Contour>& rhs)
{
    vector<Root> roots, merged;
    vector<Hit> pending;
    uint32_t cnt = 0;

    //O(n^2)
    INLIST_FOREACH(lhs, lc) {
        INLIST_FOREACH(lc->segments, ls) {
            if (ls->coincident) continue;
            INLIST_FOREACH(rhs, rc) {
                INLIST_FOREACH(rc->segments, rs) {
                    if (rs->coincident) continue;

                    if (auto dir = ls->bezier.overlapped(rs->bezier)) {
                        ls->coincident = rs->coincident = dir;
                        ls->twin = rs;
                        rs->twin = ls;
                        break;
                    }

                    roots.clear();
                    merged.clear();
                    _isolate(ls->bezier, 0.0f, 1.0f, rs->bezier, 0.0f, 1.0f, 0, roots);

                    for (auto& root : roots) {
                        _refine(ls->bezier, rs->bezier, root);
                        //a hit on a shared vertex is reported by both neighbors
                        auto hit = ls->bezier.at(root.t);
                        auto reach = fmaxf(fabsf(hit.x), fabsf(hit.y)) * 1e-6f;
                        auto vertex = [&](const Point& pt) { return length2(hit - pt) < reach * reach; };

                        //the parameter tells on a long piece, the distance on a short one
                        if (root.t > 1.0f - PATHOP_EPSILON || root.u > 1.0f - PATHOP_EPSILON) continue;
                        if (vertex(ls->bezier.end) || vertex(rs->bezier.end)) continue;
                        if (vertex(ls->bezier.start)) root.t = PATHOP_EPSILON;
                        if (vertex(rs->bezier.start)) root.u = PATHOP_EPSILON;
                        if (root.t < PATHOP_EPSILON) root.t = PATHOP_EPSILON;
                        if (root.u < PATHOP_EPSILON) root.u = PATHOP_EPSILON;

                        if (_duplicated(merged, root)) continue;
                        merged.push_back(root);
                    }

                    //3rd-order bezier curve cannot cross 9+ times unless it is overlapped
                    if (merged.size() > PATHOP_OVERLAP) {
                        ls->coincident = rs->coincident = 1;
                        continue;
                    }

                    for (auto& root : merged) {
                      pending.push_back({ls, rs, root.t, root.u});
                    }
                }
            }
        }
    }

    for (auto& hit : pending) {
        if (hit.lhs->coincident || hit.rhs->coincident) continue;
        _pair(hit.lhs, hit.t, hit.rhs, hit.u);
        ++cnt;
    }

    return cnt;
}


static Segment* _leaves(Contour* contour, const Point& at)
{
    INLIST_FOREACH(contour->segments, segment) {
        if (segment->coincident) continue;
        if (length2(segment->bezier.start - at) < PATHOP_TOLERANCE * PATHOP_TOLERANCE) return segment;
    }
    return nullptr;
}


static bool _noded(const Segment* segment)
{
    INLIST_FOREACH(segment->intersections, hit) {
        if (hit->t <= PATHOP_EPSILON * 2.0f) return true;
    }
    return false;
}


static uint32_t _bridge(Inlist<Contour>& lhs)
{
    uint32_t cnt = 0;

    INLIST_FOREACH(lhs, contour) {
        INLIST_FOREACH(contour->segments, segment) {
            if (!segment->coincident || !segment->twin) continue;

            Point ends[2] = {segment->bezier.start, segment->bezier.end};
            for (auto& at : ends) {
                auto ours = _leaves(contour, at);
                auto theirs = _leaves(segment->twin->parent, at);
                if (!ours || !theirs) continue;
                if (_noded(ours) || _noded(theirs)) continue;
                _pair(ours, PATHOP_EPSILON, theirs, PATHOP_EPSILON);
                ++cnt;
            }
        }
    }
    return cnt;
}


static void _prep(Inlist<Contour>& path)
{
    INLIST_FOREACH(path, contour) {
        INLIST_FOREACH(contour->segments, segment) {
            segment->sort();
            segment->split();
        }
    }
}


static void _mark(Inlist<Contour>& path, const Inlist<Contour>& other)
{
    INLIST_FOREACH(path, contour) {
        Intersection* first = nullptr;
        INLIST_FOREACH(contour->segments, segment) {
            segment->sort();
            segment->split();
            if (!first && !segment->intersections.empty()) first = segment->intersections.head;
        }
        if (!first) continue;

        auto held = (_winding(other, first->nextBezier->at(0.5f)) != 0);
        first->inside = held;

        auto cur = first;
        while (true) {
            auto next = cur->next;
            if (!next) {
                auto segment = cur->segment->nextSegment();
                while (segment->intersections.empty()) segment = segment->nextSegment();
                next = segment->intersections.head;
            }
            if (next == first) break;

            next->inside = (_winding(other, next->nextBezier->at(0.5f)) != 0);
            next->crossing = (next->inside != held);
            held = next->inside;
            cur = next;
        }
    }
}



/************************************************************************/
/* Walk                                                                 */
/************************************************************************/

//who draws a run the two paths share, the left hand operand always carrying it
static bool _onward(const Segment* segment, PathOp op)
{
    return (segment->coincident > 0) ? (op != PathOp::Subtract) : (op == PathOp::Subtract);
}


static bool _owned(const Segment* segment, PathOp op)
{
    return _onward(segment, op) && !segment->parent->rhs;
}


//turns the walk onto the twin at the end it stands at
static Segment* _handover(const Segment* segment, bool& forward, PathOp op)
{
    auto twin = segment->twin;

    if (_owned(twin, op)) {
        if (segment->coincident < 0) forward = !forward;
        return twin;
    }

    if (segment->coincident > 0) forward = !forward;
    return forward ? twin->nextSegment() : twin->prevSegment();
}


static void _emit(RenderPath& out, const Bezier& bezier, bool forward)
{
    auto bz = forward ? bezier : bezier.reverse();
    if (bz.line()) out.lineTo(bz.end); //if so, it can be emitted as a line
    else out.cubicTo(bz.ctrl1, bz.ctrl2, bz.end);
}


//emits the curve pieces
static Intersection* _advance(RenderPath& out, Intersection* from, bool& forward, PathOp op)
{
    _emit(out, forward ? *from->nextBezier : *from->prevBezier, forward);
    if (auto hit = forward ? from->next : from->prev) return hit;

    auto segment = forward ? from->segment->nextSegment() : from->segment->prevSegment();
    while (segment->intersections.empty()) {
        if (segment->twin && !_onward(segment, op)) {
            segment = _handover(segment, forward, op);
            continue;
        }
        _emit(out, segment->bezier, forward);
        segment = forward ? segment->nextSegment() : segment->prevSegment();
    }
    auto hit = forward ? segment->intersections.head : segment->intersections.tail;
    _emit(out, forward ? *hit->prevBezier : *hit->nextBezier, forward);
    return hit;
}


static bool _entry(PathOp op, bool rhs)
{
    return rhs ? (op != PathOp::Add) : (op == PathOp::Intersect);
}


static bool _behind(const Intersection* hit)
{
    if (hit->prev) return hit->prev->inside;

    auto segment = hit->segment->prevSegment();
    while (segment->intersections.empty()) segment = segment->prevSegment();
    return segment->intersections.tail->inside;
}


static void _merge(Inlist<Contour>& lhs, Inlist<Contour>& rhs, PathOp op, RenderPath& out)
{
    Inlist<Contour>* sides[2] = {&lhs, &rhs};

    for (auto side : sides) {
    auto& list = *side;
    auto entry = _entry(op, side == &rhs);

    INLIST_FOREACH(list, contour) {
        INLIST_FOREACH(contour->segments, segment) {
            INLIST_FOREACH(segment->intersections, head) {
                if (head->visited || head->inside != entry) continue;

                out.moveTo(head->segment->bezier.at(head->t));

                auto cur = head;
                auto forward = true;
                while (cur && !cur->visited) {
                    cur->visited = true;
                    auto next = _advance(out, cur, forward, op);
                    if (!next) break;
                    if (next == head) break;
                    if (!next->crossing) { cur = next; continue; }
                    next->visited = true;
                    cur = next->pair;

                    auto want = _entry(op, cur->segment->parent->rhs);
                    if (cur->inside == want) forward = true;
                    else if (_behind(cur) == want) forward = false;
                }
                out.close();
            }
        }
    }
    }
}


static void _copy(const Contour* contour, bool flip, RenderPath& out)
{
    if (flip) {
        out.moveTo(contour->segments.tail->bezier.end);
        for (auto segment = contour->segments.tail; segment; segment = segment->prev) _emit(out, segment->bezier, false);
    } else {
        out.moveTo(contour->segments.head->bezier.start);
        INLIST_FOREACH(contour->segments, segment) _emit(out, segment->bezier, true);
    }
    out.close();
}


//draws a contour that only partly runs along the other path
static void _stitch(Segment* from, PathOp op, RenderPath& out)
{
    auto segment = from;
    auto forward = true;

    out.moveTo(segment->bezier.start);

    do {
        if (segment->twin && !_owned(segment, op)) {
            segment = _handover(segment, forward, op);
            continue;
        }
        _emit(out, segment->bezier, forward);
        segment = forward ? segment->nextSegment() : segment->prevSegment();
    } while (segment != from);
    out.close();
}


static void _uncrossed(Inlist<Contour>& path, const Inlist<Contour>& other, PathOp op, bool lhs, RenderPath& out)
{
    INLIST_FOREACH(path, contour) {
        auto crossed = false;
        auto shared = true;
        auto shares = false;
        INLIST_FOREACH(contour->segments, segment) {
            if (!segment->intersections.empty()) {
                crossed = true;
                break;
            }
            if (segment->coincident) shares = true;
            else shared = false;
        }
        if (crossed) continue;

        if (shared) {
            if (_owned(contour->segments.head, op)) _copy(contour, false, out);
            continue;
        }

        //only a part of it runs along, so it is stitched from the left hand operand
        if (shares) {
            if (lhs) {
                INLIST_FOREACH(contour->segments, segment) {
                    if (segment->twin) continue;
                    _stitch(segment, op, out);
                    break;
                }
            }
            continue;
        }

        auto inside = (_winding(other, contour->segments.head->bezier.at(0.5f)) != 0);
        auto keep = false;
        auto flip = false;

        if (op == PathOp::Add) keep = !inside;
        else if (op == PathOp::Intersect) keep = inside;
        else {
            keep = (lhs != inside);
            flip = !lhs;
        }
        if (keep) _copy(contour, flip, out);
    }
}


static bool _sound(const RenderPath& path)
{
    for (auto& pt : path.pts) {
        if (!isfinite(pt.x) || !isfinite(pt.y)) return false;
    }
    return true;
}


static bool _op(const RenderPath& lhs, const RenderPath& rhs, RenderPath& out, PathOp op)
{
    if (!_sound(lhs) || !_sound(rhs)) return false;

    if (lhs.cmds.empty() || rhs.cmds.empty()) {
        if (op == PathOp::Add) {
            if (!lhs.cmds.empty()) _copy(lhs, out);
            else if (!rhs.cmds.empty()) _copy(rhs, out);
        } else if (op == PathOp::Subtract && !lhs.cmds.empty()) {
            _copy(lhs, out);
        }
        return true;
    }

    auto lbox = _bounds(lhs);
    auto rbox = _bounds(rhs);

    //fast path: apart and winding alike
    if (!lbox.intersected(rbox) && _area(lhs) * _area(rhs) > 0.0f) {
        if (op != PathOp::Intersect) {
            _copy(lhs, out);
            if (op == PathOp::Add) _copy(rhs, out);
        }
        return true;
    }

    Normalizer norm({{fminf(lbox.min.x, rbox.min.x), fminf(lbox.min.y, rbox.min.y)},
                     {fmaxf(lbox.max.x, rbox.max.x), fmaxf(lbox.max.y, rbox.max.y)}});

    Inlist<Contour> a, b;

    _contour(lhs, false, norm, a);
    _contour(rhs, true, norm, b);
    if (a.empty() || b.empty()) return false;

    auto mark = out.pts.size();

    //align the winding directions
    _orient(a);
    _orient(b);

    _slice(a, b);
    _slice(b, a);

    auto crossings = _intersect(a, b);
    crossings += _bridge(a);

    if (crossings > 0) {
        _mark(a, b);
        _mark(b, a);
        _merge(a, b, op, out);
    }

    _uncrossed(a, b, op, true, out);
    _uncrossed(b, a, op, false, out);

    for (auto i = mark; i < out.pts.size(); ++i) out.pts[i] = norm.out(out.pts[i]);

    return true;
}


/************************************************************************/
/* External Class Implementation                                        */
/************************************************************************/

bool AddMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    return _op(a, b, out, PathOp::Add);
}

bool SubtractMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    return _op(a, b, out, PathOp::Subtract);
}

bool IntersectMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    return _op(a, b, out, PathOp::Intersect);
}

bool DifferenceMask(const RenderPath& a, const RenderPath& b, RenderPath& out)
{
    //exclude intersections == (a - b) + (b - a)
    auto ret = _op(a, b, out, PathOp::Subtract);
    return _op(b, a, out, PathOp::Subtract) | ret;
}
