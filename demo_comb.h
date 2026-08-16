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

/* The worst case for keeping the crossings of one segment in order.

   A comb of N teeth is crossed by a single bar. Every tooth meets the same two
   bar segments, so those two carry 2N crossings each while every other segment
   carries none. The crossings are placed one by one and each placement scans the
   ones already there, so the cost of that alone grows with the square of N.

   The bar sweeps up and down so the count of crossings changes over time. */

#ifndef _DEMO_COMB_H_
#define _DEMO_COMB_H_

#include <chrono>
#include "mergepath.h"
#include "template.h"

#ifndef RES_DIR
    #define RES_DIR "res"
#endif

struct CombDemo : tvgdemo::Demo
{
    static constexpr const char* FONT = RES_DIR"/font/PublicSans-Regular.ttf";

    uint32_t teeth;
    bool trace;
    bool outline;

    Shape* result = nullptr;
    Shape* operands = nullptr;
    Text* label = nullptr;
    float w = 0.0f, h = 0.0f;

    double cost = 0.0;
    double frameCost = 0.0;
    uint32_t costCnt = 0;
    uint32_t reported = 0;

    CombDemo(uint32_t teeth, bool trace, bool outline) : teeth(teeth), trace(trace), outline(outline) {}

    void comb(RenderPath& path)
    {
        auto left = w * 0.06f, right = w * 0.94f;
        auto base = h * 0.62f, tip = h * 0.22f;
        auto step = (right - left) / float(teeth);

        path.moveTo({left, base});
        for (uint32_t i = 0; i < teeth; ++i) {
            path.lineTo({left + (float(i) + 0.5f) * step, tip});
            path.lineTo({left + (float(i) + 1.0f) * step, base});
        }
        path.lineTo({right, h * 0.9f});
        path.lineTo({left, h * 0.9f});
        path.close();
    }

    void bar(RenderPath& path, float y)
    {
        auto thick = h * 0.035f;
        path.moveTo({w * 0.02f, y});
        path.lineTo({w * 0.98f, y});
        path.lineTo({w * 0.98f, y + thick});
        path.lineTo({w * 0.02f, y + thick});
        path.close();
    }

    bool content(Canvas* canvas, uint32_t width, uint32_t height) override
    {
        w = float(width);
        h = float(height);

        if (outline) {
            operands = Shape::gen();
            operands->strokeWidth(1.0f);
            operands->strokeFill(150, 160, 175);
            canvas->add(operands);
        }

        result = Shape::gen();
        result->fill(40, 120, 190, 190);
        result->strokeWidth(1.4f);
        result->strokeFill(20, 80, 140);
        canvas->add(result);

        if (Text::load(FONT) == Result::Success) {
            label = Text::gen();
            label->font("PublicSans-Regular");
            label->size(h * 0.028f);
            label->fill(70, 75, 85);
            label->translate(w * 0.03f, h * 0.94f);
            canvas->add(label);
        }

        return update(canvas, 0);
    }

    bool update(Canvas* canvas, uint32_t elapsed) override
    {
        auto frameBegin = chrono::high_resolution_clock::now();
        auto progress = float(elapsed % 6000) / 6000.0f;
        auto sweep = 0.5f - 0.5f * cosf(progress * 2.0f * float(M_PI));

        RenderPath teethPath, barPath;
        comb(teethPath);
        bar(barPath, h * (0.24f + 0.32f * sweep));

        auto begin = chrono::high_resolution_clock::now();

        RenderPath merged;
        AddMask(teethPath, barPath, merged);

        auto spent = chrono::duration<double, milli>(chrono::high_resolution_clock::now() - begin).count();
        cost += spent;
        ++costCnt;

        result->reset();
        result->appendPath(merged.cmds.data(), merged.cmds.size(), merged.pts.data(), merged.pts.size());

        if (operands) {
            operands->reset();
            operands->appendPath(teethPath.cmds.data(), teethPath.cmds.size(), teethPath.pts.data(), teethPath.pts.size());
            operands->appendPath(barPath.cmds.data(), barPath.cmds.size(), barPath.pts.data(), barPath.pts.size());
        }

        if (label) {
            char buf[160];
            snprintf(buf, sizeof(buf), "%u teeth  |  up to %u crossings on a single bar segment  |  %.3f ms",
                     teeth, teeth * 2, spent);
            label->text(buf);
        }

        canvas->update();

        frameCost += chrono::duration<double, milli>(chrono::high_resolution_clock::now() - frameBegin).count();

        if (trace && elapsed / 1000 > reported) {
            reported = elapsed / 1000;
            printf("comb: merge %.3f ms  |  update() 전체 %.3f ms  (%u teeth, %u crossings)\n",
                   cost / costCnt, frameCost / costCnt, teeth, teeth * 2);
            cost = 0.0;
            frameCost = 0.0;
            costCnt = 0;
        }

        return true;
    }
};

#endif //_DEMO_COMB_H_
