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
        return fabsf(cross(chord, ctrl1 - start)) / leng < 1e-3f && fabsf(cross(chord, ctrl2 - start)) / leng < 1e-3f;
    }

    //check if the bezier is overlapped with another bezier
    bool overlapped(const Bezier& rhs) const
    {
        auto same = [](const Point& lhs, const Point& rhs) {
            return length2(lhs - rhs) < PATHOP_EPSILON;
        };
        return (same(start, rhs.start) && same(ctrl1, rhs.ctrl1) && same(ctrl2, rhs.ctrl2) && same(end, rhs.end)) ||
               (same(start, rhs.end) && same(ctrl1, rhs.ctrl2) && same(ctrl2, rhs.ctrl1) && same(end, rhs.start));
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
    bool coincident{};

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
};

Segment* Segment::nextSegment() { return next ? next : parent->segments.head; }
Segment* Segment::prevSegment() { return prev ? prev : parent->segments.tail; }

struct Root
{
    float t, u; // lhs(t) = rhs(u)
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

    constexpr float flatness = 0.05f;
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
    if (length2(bezier.end - bezier.start) < PATHOP_EPSILON) return;
    auto segment = new Segment;
    segment->bezier = bezier;
    segment->parent = contour;
    contour->segments.back(segment);
}


static void _contour(const RenderPath& path, Inlist<Contour>& out)
{
    auto pts = path.pts.data();
    Contour* contour = nullptr;
    Point start{}, cur{};

    for (auto cmd : path.cmds) {
        switch (cmd) {
            case PathCommand::MoveTo: {
                if (contour) _append(contour, Bezier::line(cur, start));
                contour = new Contour;
                out.back(contour);
                start = cur = *pts++;
                break;
            }
            case PathCommand::LineTo: {
                if (contour) _append(contour, Bezier::line(cur, *pts));
                cur = *pts++;
                break;
            }
            case PathCommand::CubicTo: {
                if (contour) _append(contour, {cur, pts[0], pts[1], pts[2]});
                cur = pts[2];
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


static void _reverse(Inlist<Contour>& path)
{
    INLIST_FOREACH(path, contour) {
        Inlist<Segment> reversed;
        //popping the front and pushing it back to the front flips the order
        while (auto segment = contour->segments.front()) {
            segment->bezier = segment->bezier.reverse();
            reversed.front(segment);
        }
        contour->segments = reversed;
        reversed.head = reversed.tail = nullptr;
    }
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
static int32_t _winding(const Inlist<Contour>& path, const Point& pt)
{
    int32_t winding = 0;
    float turns[2];

    INLIST_FOREACH(path, contour) {
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
    }
    return winding;
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


static uint32_t _intersect(Inlist<Contour>& lhs, Inlist<Contour>& rhs)
{
    vector<Root> roots, merged;
    uint32_t cnt = 0;

    //O(n^2)
    INLIST_FOREACH(lhs, lc) {
        INLIST_FOREACH(lc->segments, ls) {
            INLIST_FOREACH(rhs, rc) {
                INLIST_FOREACH(rc->segments, rs) {
                    roots.clear();
                    merged.clear();
                    _isolate(ls->bezier, 0.0f, 1.0f, rs->bezier, 0.0f, 1.0f, 0, roots);

                    for (auto& root : roots) {
                        _refine(ls->bezier, rs->bezier, root);
                        /* a hit on a shared vertex is reported by both neighbors. the
                           half open range keeps the one that owns it as its start. */
                        if (root.t > 1.0f - PATHOP_EPSILON || root.u > 1.0f - PATHOP_EPSILON) continue;
                        if (root.t < PATHOP_EPSILON) root.t = PATHOP_EPSILON;
                        if (root.u < PATHOP_EPSILON) root.u = PATHOP_EPSILON;
                        if (_duplicated(merged, root)) continue;
                        merged.push_back(root);
                    }

                    //3rd-order bezier curve cannot cross 9+ times unless it is overlapped
                    if (merged.size() > PATHOP_OVERLAP || ls->bezier.overlapped(rs->bezier)) {
                        ls->coincident = rs->coincident = true;
                        continue;
                    }

                    for (auto& root : merged) {
                        _pair(ls, root.t, rs, root.u);
                        ++cnt;
                    }
                }
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

        auto winding = _winding(other, first->nextBezier->at(0.5f));
        first->inside = (winding != 0);

        auto cur = first;
        while (true) {
            auto next = cur->next;
            if (!next) {
                auto segment = cur->segment->nextSegment();
                while (segment->intersections.empty()) segment = segment->nextSegment();
                next = segment->intersections.head;
            }
            if (next == first) break;

            auto turn = cross(next->paired->segment->bezier.tangent(next->paired->t), next->segment->bezier.tangent(next->t));
            if (fabsf(turn) < PATHOP_EPSILON) winding = _winding(other, next->nextBezier->at(0.5f));
            else winding += turn > 0.0f ? 1 : -1;
            next->inside = (winding != 0);
            cur = next;
        }
    }
}



/************************************************************************/
/* Walk                                                                 */
/************************************************************************/

static void _emit(RenderPath& out, const Bezier& bezier, bool forward)
{
    auto bz = forward ? bezier : bezier.reverse();
    if (bz.line()) out.lineTo(bz.end); //if so, it can be emitted as a line
    else out.cubicTo(bz.ctrl1, bz.ctrl2, bz.end);
}


//emits the curve pieces
static Intersection* _advance(RenderPath& out, Intersection* from, bool forward)
{
    _emit(out, forward ? *from->nextBezier : *from->prevBezier, forward);
    if (auto hit = forward ? from->next : from->prev) return hit;

    auto segment = forward ? from->segment->nextSegment() : from->segment->prevSegment();
    while (segment->intersections.empty()) {
        _emit(out, segment->bezier, forward);
        segment = forward ? segment->nextSegment() : segment->prevSegment();
    }
    auto hit = forward ? segment->intersections.head : segment->intersections.tail;
    _emit(out, forward ? *hit->prevBezier : *hit->nextBezier, forward);
    return hit;
}


static void _merge(Inlist<Contour>& lhs, PathOp op, RenderPath& out)
{
    auto entry = (op == PathOp::Intersect); //intersect starts from inner pieces

    INLIST_FOREACH(lhs, contour) {
        INLIST_FOREACH(contour->segments, segment) {
            INLIST_FOREACH(segment->intersections, head) {
                if (head->visited || head->inside != entry) continue;

                out.moveTo(head->segment->bezier.at(head->t));

                auto cur = head;
                auto forward = true;
                while (cur && !cur->visited) {
                    cur->visited = true;
                    auto next = _advance(out, cur, forward);
                    if (!next) break;
                    next->visited = true;
                    cur = next->paired;
                    if (op == PathOp::Subtract) forward = !forward; //if subtract, walk backwards
                }
                out.close();
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


static void _uncrossed(Inlist<Contour>& path, const Inlist<Contour>& other, PathOp op, bool lhs, RenderPath& out)
{
    INLIST_FOREACH(path, contour) {
        auto crossed = false;
        auto shared = true;
        INLIST_FOREACH(contour->segments, segment) {
            if (!segment->intersections.empty()) {
                crossed = true;
                break;
            }
            if (!segment->coincident) shared = false;
        }
        if (crossed) continue;

        //overlapped, copy only one side
        if (shared) {
            if (op != PathOp::Subtract && lhs) _copy(contour, false, out);
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


static bool _op(const RenderPath& lhs, const RenderPath& rhs, RenderPath& out, PathOp op)
{
    if (lhs.cmds.empty() || rhs.cmds.empty()) return false;

    //fast path: apart and winding alike
    if (!_bounds(lhs).intersected(_bounds(rhs)) && _area(lhs) * _area(rhs) > 0.0f) {
        if (op != PathOp::Intersect) {
            _copy(lhs, out);
            if (op == PathOp::Add) _copy(rhs, out);
        }
        return true;
    }

    Inlist<Contour> a, b;

    _contour(lhs, a);
    _contour(rhs, b);
    if (a.empty() || b.empty()) return false;

    //the operands must share the winding direction
    if (_area(lhs) * _area(rhs) < 0.0f) _reverse(b);

    if (_intersect(a, b) > 0) {
        _mark(a, b);
        _prep(b);
        _merge(a, op, out);
    }

    _uncrossed(a, b, op, true, out);
    _uncrossed(b, a, op, false, out);

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
