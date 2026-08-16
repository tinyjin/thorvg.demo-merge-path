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

/* A curve only stress: N blobs of cubics, merged into one another on every frame. */

#ifndef _DEMO_STRESS_H_
#define _DEMO_STRESS_H_

#include <chrono>
#include "mergepath.h"
#include "template.h"

#ifndef RES_DIR
    #define RES_DIR "res"
#endif

struct StressDemo : tvgdemo::Demo
{
    static constexpr const char* FONT = RES_DIR"/font/PublicSans-Regular.ttf";

    uint32_t count;
    bool trace;
    bool outline;

    Shape* result = nullptr;
    Shape* operands = nullptr;
    Text* label = nullptr;
    Point center{};
    float radius = 0.0f;

    double cost = 0.0;
    uint32_t costCnt = 0;
    uint32_t reported = 0;

    StressDemo(uint32_t count, bool trace, bool outline) : count(count), trace(trace), outline(outline) {}

    //a closed cubic path through the sampled points
    void smooth(RenderPath& path, const vector<Point>& pts)
    {
        auto cnt = pts.size();
        path.moveTo(pts[0]);
        for (size_t i = 0; i < cnt; ++i) {
            auto& p0 = pts[(i + cnt - 1) % cnt];
            auto& p1 = pts[i];
            auto& p2 = pts[(i + 1) % cnt];
            auto& p3 = pts[(i + 2) % cnt];
            path.cubicTo(p1 + (p2 - p0) * (1.0f / 6.0f), p2 - (p3 - p1) * (1.0f / 6.0f), p2);
        }
        path.close();
    }

    void blob(RenderPath& path, const Point& center, float r, float phase)
    {
        vector<Point> pts;
        for (uint32_t i = 0; i < 10; ++i) {
            auto t = float(i) * 2.0f * float(M_PI) / 10.0f;
            auto rr = r * (1.0f + 0.24f * sinf(t * 3.0f + phase) + 0.14f * cosf(t * 2.0f - phase * 1.7f));
            pts.push_back({center.x + cosf(t) * rr, center.y + sinf(t) * rr});
        }
        smooth(path, pts);
    }

    bool content(Canvas* canvas, uint32_t w, uint32_t h) override
    {
        center = {float(w) * 0.5f, float(h) * 0.5f};
        radius = std::min(float(w), float(h)) * 0.5f;

        auto bg = Shape::gen();
        bg->appendRect(0.0f, 0.0f, float(w), float(h));
        bg->fill(255, 255, 255);
        canvas->add(bg);

        result = Shape::gen();
        result->fill(60, 140, 240);
        result->fillRule(FillRule::NonZero);
        result->strokeWidth(1.5f);
        result->strokeFill(20, 40, 90);
        canvas->add(result);

        if (outline) {
            operands = Shape::gen();
            operands->strokeWidth(1.0f);
            operands->strokeFill(150, 155, 165, 90);
            canvas->add(operands);
        }

        if (Text::load(FONT) == Result::Success) {
            label = Text::gen();
            label->font("PublicSans-Regular");
            label->size(radius * 0.038f);
            label->fill(70, 75, 85);
            label->translate(radius * 0.05f, float(h) - radius * 0.1f);
            canvas->add(label);
        }

        return update(canvas, 0);
    }

    bool update(Canvas* canvas, uint32_t elapsed) override
    {
        auto progress = float(elapsed % 8000) / 8000.0f;
        auto spin = progress * 2.0f * float(M_PI);

        vector<RenderPath> blobs(count);
        for (uint32_t i = 0; i < count; ++i) {
            auto t = float(i) * 2.0f * float(M_PI) / float(count) + spin;
            auto orbit = radius * (0.34f + 0.12f * sinf(spin * 2.0f + float(i)));
            blob(blobs[i], {center.x + cosf(t) * orbit, center.y + sinf(t) * orbit}, radius * 0.26f, spin * 1.3f + float(i));
        }

        auto begin = chrono::high_resolution_clock::now();

        //accumulate, the merged result becomes the operand of the next merge
        RenderPath acc = blobs[0];
        for (uint32_t i = 1; i < count; ++i) {
            RenderPath next;
            if (!AddMask(acc, blobs[i], next)) continue;
            acc = next;
        }

        auto spent = chrono::duration<double, milli>(chrono::high_resolution_clock::now() - begin).count();
        cost += spent;
        ++costCnt;

        result->reset();
        result->appendPath(acc.cmds.data(), acc.cmds.size(), acc.pts.data(), acc.pts.size());

        if (operands) {
            operands->reset();
            for (auto& b : blobs) operands->appendPath(b.cmds.data(), b.cmds.size(), b.pts.data(), b.pts.size());
        }

        uint32_t cubics = 0;
        for (auto cmd : acc.cmds) {
            if (cmd == PathCommand::CubicTo) ++cubics;

        }

        if (label) {
            char buf[128];
            snprintf(buf, sizeof(buf), "%u blobs, %u merges  |  %u cubics out  |  %.2f ms", count, count - 1, cubics, spent);
            label->text(buf);
        }

        if (trace && elapsed / 1000 > reported) {
            reported = elapsed / 1000;
            printf("stress: %.3f ms / frame (%u merges, %u cubics out)\n", cost / costCnt, count - 1, cubics);
            cost = 0.0;
            costCnt = 0;
        }

        canvas->update();

        return true;
    }
};

#endif //_DEMO_STRESS_H_
