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


//mm:1 Merge, the operands are concatenated as they are
static SkPath merge(const SkPath& a, const SkPath& b)
{
    SkPath out = a;
    out.addPath(b);
    return out;
}


static void paint(SkCanvas* canvas, const SkPath& path, bool fill)
{
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
    auto tile = std::min(float(w) / 3.0f, float(h) / 2.0f);
    auto angle = float(elapsed % 4000) / 4000.0f * 2.0f * float(M_PI);
    auto cx = tile * 0.5f, cy = tile * 0.44f;
    auto radius = tile * 0.28f;

    auto a = build(scenario::star(cx - radius * 0.3f, cy - radius * 0.25f, radius, radius * 0.42f, 5));
    auto b = build(scenario::circle(cx + cosf(angle) * radius * 0.5f, cy + sinf(angle) * radius * 0.5f, radius * 0.62f));
    auto c = rect(cx, cy + cosf(angle) * radius * 0.5f, tile * 0.42f, tile * 0.06f);

    auto begin = clk::now();

    SkPath out[6], tmp;
    out[0] = merge(a, b);
    Op(a, b, kUnion_SkPathOp, &out[1]);
    Op(a, b, kDifference_SkPathOp, &out[2]);
    Op(a, b, kIntersect_SkPathOp, &out[3]);
    Op(a, b, kXOR_SkPathOp, &out[4]);
    //a group accumulates, so a merged result becomes the operand of the next one
    if (Op(a, b, kUnion_SkPathOp, &tmp)) Op(tmp, c, kDifference_SkPathOp, &out[5]);

    auto spent = std::chrono::duration<double, std::milli>(clk::now() - begin).count();

    SkPaint frame;
    frame.setStyle(SkPaint::kStroke_Style);
    frame.setColor(0xffe1e4eb);

    for (uint32_t i = 0; i < 6; ++i) {
        canvas->save();
        canvas->translate(float(i % 3) * tile, float(i / 3) * tile);
        canvas->drawRect({0.0f, 0.0f, tile, tile}, frame);
        paint(canvas, out[i], true);
        paint(canvas, a, false);
        paint(canvas, b, false);
        if (i == 5) paint(canvas, c, false);
        canvas->restore();
    }
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


//one frame into a file, so the scene can be checked without a window
static void dump(uint32_t W, uint32_t H, uint32_t elapsed, uint32_t count)
{
    auto info = SkImageInfo::Make(int(W), int(H), kBGRA_8888_SkColorType, kPremul_SkAlphaType);
    auto surface = SkSurfaces::Raster(info);
    auto canvas = surface->getCanvas();
    canvas->clear(0xffffffff);

    uint32_t cubics = 0;
    if (count > 0) stress(canvas, W, H, elapsed, count, cubics);
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
    auto offscreen = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-t")) traceCost = true;
        else if (!strcmp(argv[i], "-s")) count = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 12;
        else if (!strcmp(argv[i], "-o")) offscreen = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 2000;
    }

    uint32_t W = count > 0 ? 900 : 1200;
    uint32_t H = count > 0 ? 900 : 800;

    if (count > 0) printf("stress: a union of %d curve blobs, accumulated over %d merges\n", count, count - 1);
    else printf("tiles: Merge | Add | Subtract  /  Intersect | Exclude | Add then Subtract\n");

    if (offscreen) { dump(W, H, uint32_t(offscreen), uint32_t(count)); return 0; }

    SDL_Init(SDL_INIT_VIDEO);
    auto window = SDL_CreateWindow("Skia SkPathOps (Software)", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, int(W), int(H), 0);

    auto begin = SDL_GetTicks();
    auto reported = 0u;
    double cost = 0.0;
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
        canvas->clear(0xffffffff);

        auto elapsed = SDL_GetTicks() - begin;
        auto spent = count > 0 ? stress(canvas, uint32_t(surface->w), uint32_t(surface->h), elapsed, uint32_t(count), cubics)
                               : tiles(canvas, uint32_t(surface->w), uint32_t(surface->h), elapsed);
        cost += spent;
        ++costCnt;

        SDL_UpdateWindowSurface(window);

        if (traceCost && elapsed / 1000 > reported) {
            reported = elapsed / 1000;
            if (count > 0) printf("stress: %.3f ms / frame (%d merges, %u cubics out)\n", cost / costCnt, count - 1, cubics);
            else printf("merge path: %.3f ms / frame\n", cost / costCnt);
            cost = 0.0;
            costCnt = 0;
        }
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
