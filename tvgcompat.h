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

/* Structural copies of thorvg, so the demo can build against the installed
   engine alone. Nothing here is our code and nothing here is worth reviewing.

   The extendable types sit in namespace compat, tvgextend.h derives from them
   to add what thorvg does not have yet. On the way into the engine this file is
   dropped, tvgRender.h / tvgInlist.h / tvgMath.h take over and the members of
   tvgextend.h move into those headers. */

#ifndef _MERGE_PATH_COMPAT_H_
#define _MERGE_PATH_COMPAT_H_

#include <vector>
#include <cfloat>
#include <thorvg-1/thorvg.h>

using namespace std;
using namespace tvg;


/************************************************************************/
/* tvgRender.h                                                          */
/************************************************************************/

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
/* tvgMath.h                                                            */
/************************************************************************/

constexpr float PATH_KAPPA = 0.552284f;

static inline Point operator-(const Point& lhs, const Point& rhs) { return {lhs.x - rhs.x, lhs.y - rhs.y}; }
static inline Point operator+(const Point& lhs, const Point& rhs) { return {lhs.x + rhs.x, lhs.y + rhs.y}; }
static inline Point operator*(const Point& lhs, float rhs) { return {lhs.x * rhs, lhs.y * rhs}; }
static inline bool operator==(const Point& lhs, const Point& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
static inline float dot(const Point& lhs, const Point& rhs) { return lhs.x * rhs.x + lhs.y * rhs.y; }
static inline float cross(const Point& lhs, const Point& rhs) { return lhs.x * rhs.y - lhs.y * rhs.x; }
static inline float length2(const Point& a) { return a.x * a.x + a.y * a.y; }
static inline float length(const Point& a) { return sqrtf(length2(a)); }
static inline Point lerp(const Point& start, const Point& end, float t) { return start + (end - start) * t; }


/************************************************************************/
/* tvgInlist.h                                                          */
/************************************************************************/

#define INLIST_ITEM(T) \
    T* prev; \
    T* next

#define INLIST_FOREACH(inlist, cur) \
    for (auto cur = inlist.head; cur; cur = cur->next)

#define INLIST_SAFE_FOREACH(inlist, cur) \
    auto cur = inlist.head; \
    auto next = cur ? cur->next : nullptr; \
    for (; cur; cur = next, next = (cur ? cur->next : nullptr))


//the types that tvgextend.h derives from
namespace compat
{

struct BBox
{
    Point min, max;
};


struct Bezier
{
    Point start, ctrl1, ctrl2, end;

    Bezier() {}
    Bezier(const Point& start, const Point& ctrl1, const Point& ctrl2, const Point& end) : start(start), ctrl1(ctrl1), ctrl2(ctrl2), end(end) {}

    Point at(float t) const
    {
        auto it = 1.0f - t;
        auto a = lerp(start, ctrl1, t), b = lerp(ctrl1, ctrl2, t), c = lerp(ctrl2, end, t);
        a = a * it + b * t;
        b = b * it + c * t;
        return a * it + b * t;
    }

    Point tangent(float t) const
    {
        auto it = 1.0f - t;
        auto a = (ctrl1 - start) * (3.0f * it * it);
        auto b = (ctrl2 - ctrl1) * (6.0f * it * t);
        auto c = (end - ctrl2) * (3.0f * t * t);
        return a + b + c;
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
        return (fmaxf(diff1.x, diff2.x) + fmaxf(diff1.y, diff2.y)) <= tolerance;
    }
};


template<typename T>
struct Inlist
{
    T* head = nullptr;
    T* tail = nullptr;
    uint32_t count = 0;

    ~Inlist() { free(); }

    void free()
    {
        while (head) {
            auto t = head;
            head = t->next;
            delete(t);
        }
        head = tail = nullptr;
        count = 0;
    }

    void back(T* element)
    {
        if (tail) {
            tail->next = element;
            element->prev = tail;
            element->next = nullptr;
            tail = element;
        } else {
            head = tail = element;
            element->prev = element->next = nullptr;
        }
        ++count;
    }

    void front(T* element)
    {
        if (head) {
            head->prev = element;
            element->prev = nullptr;
            element->next = head;
            head = element;
        } else {
            head = tail = element;
            element->prev = element->next = nullptr;
        }
        ++count;
    }

    T* front()
    {
        if (!head) return nullptr;
        --count;
        auto t = head;
        head = t->next;
        if (!head) tail = nullptr;
        return t;
    }

    void remove(T* element)
    {
        if (element->prev) element->prev->next = element->next;
        if (element->next) element->next->prev = element->prev;
        if (element == head) head = element->next;
        if (element == tail) tail = element->prev;
        element->prev = element->next = nullptr;
        --count;
    }

    bool empty() const { return head ? false : true; }
};

}

#endif //_MERGE_PATH_COMPAT_H_
