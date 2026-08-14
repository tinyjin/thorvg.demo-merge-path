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

#include <chrono>
#include "mergepath.h"
#include "template.h"

#ifndef RES_DIR
    #define RES_DIR "res"
#endif

static const char* fontFile = RES_DIR"/font/PublicSans-Regular.ttf";
static bool traceCost = false;


/************************************************************************/
/* ThorVG Drawing Contents                                              */
/************************************************************************/

struct UserDemo : tvgdemo::Demo
{
    static constexpr uint32_t COLS = 3;
    static constexpr uint32_t ROWS = 2;
    static constexpr uint32_t TILES = COLS * ROWS;

    //the first five are the lottie merge modes, the last one stacks two of them
    static constexpr const char* LABELS[TILES] = {
        "Merge (mm:1)", "Add (mm:2)", "Subtract (mm:3)",
        "Intersect (mm:4)", "Exclude (mm:5)", "Add, then Subtract"
    };

    struct Tile
    {
        Shape* result = nullptr;
        Shape* operands = nullptr;
        Point offset{};
    };

    Tile tiles[TILES];
    float tileSize = 0.0f;

    //cost tracking, the boolean runs on every frame here
    double cost = 0.0;
    uint32_t costCnt = 0;
    uint32_t reported = 0;

    void star(RenderPath& path, const Point& center, float outer, float inner, uint32_t cnt)
    {
        for (uint32_t i = 0; i < cnt * 2; ++i) {
            auto radius = (i % 2) ? inner : outer;
            auto angle = float(i) * float(M_PI) / float(cnt) - float(M_PI) * 0.5f;
            Point pt = {center.x + cosf(angle) * radius, center.y + sinf(angle) * radius};
            if (i == 0) path.moveTo(pt);
            else path.lineTo(pt);
        }
        path.close();
    }

    void circle(RenderPath& path, const Point& center, float r)
    {
        auto c = r * PATH_KAPPA;
        path.moveTo({center.x, center.y - r});
        path.cubicTo({center.x + c, center.y - r}, {center.x + r, center.y - c}, {center.x + r, center.y});
        path.cubicTo({center.x + r, center.y + c}, {center.x + c, center.y + r}, {center.x, center.y + r});
        path.cubicTo({center.x - c, center.y + r}, {center.x - r, center.y + c}, {center.x - r, center.y});
        path.cubicTo({center.x - r, center.y - c}, {center.x - c, center.y - r}, {center.x, center.y - r});
        path.close();
    }

    void bar(RenderPath& path, const Point& center, float w, float h)
    {
        path.moveTo({center.x - w, center.y - h});
        path.lineTo({center.x + w, center.y - h});
        path.lineTo({center.x + w, center.y + h});
        path.lineTo({center.x - w, center.y + h});
        path.close();
    }

    //mm:1 Merge, the operands are concatenated as they are
    void merge(const RenderPath& a, const RenderPath& b, RenderPath& out)
    {
        out.cmds = a.cmds;
        out.pts = a.pts;
        out.cmds.insert(out.cmds.end(), b.cmds.begin(), b.cmds.end());
        out.pts.insert(out.pts.end(), b.pts.begin(), b.pts.end());
    }

    bool content(Canvas* canvas, uint32_t w, uint32_t h) override
    {
        tileSize = std::min(float(w) / COLS, float(h) / ROWS);

        auto labeled = (Text::load(fontFile) == Result::Success);
        if (!labeled) printf("font not found (%s), the tiles stay unlabeled\n", fontFile);

        auto bg = Shape::gen();
        bg->appendRect(0.0f, 0.0f, float(w), float(h));
        bg->fill(255, 255, 255);
        canvas->add(bg);

        for (uint32_t i = 0; i < TILES; ++i) {
            auto& tile = tiles[i];
            tile.offset = {(i % COLS) * tileSize, (i / COLS) * tileSize};

            auto frame = Shape::gen();
            frame->appendRect(tile.offset.x, tile.offset.y, tileSize, tileSize);
            frame->strokeWidth(1.0f);
            frame->strokeFill(225, 228, 235);
            canvas->add(frame);

            auto result = Shape::gen();
            result->fill(60, 140, 240);
            result->fillRule(FillRule::NonZero);
            result->strokeWidth(1.5f);
            result->strokeFill(20, 40, 90);
            result->translate(tile.offset.x, tile.offset.y);
            tile.result = result;
            canvas->add(result);

            auto operands = Shape::gen();
            operands->strokeWidth(1.0f);
            operands->strokeFill(150, 155, 165, 110);
            operands->translate(tile.offset.x, tile.offset.y);
            tile.operands = operands;
            canvas->add(operands);

            if (!labeled) continue;

            auto label = Text::gen();
            label->font("PublicSans-Regular");
            label->size(tileSize * 0.052f);
            label->text(LABELS[i]);
            label->fill(70, 75, 85);
            label->translate(tile.offset.x + tileSize * 0.07f, tile.offset.y + tileSize * 0.86f);
            canvas->add(label);
        }

        return update(canvas, 0);
    }

    bool update(Canvas* canvas, uint32_t elapsed) override
    {
        auto progress = float(elapsed % 4000) / 4000.0f;
        auto angle = progress * 2.0f * float(M_PI);
        auto center = Point{tileSize * 0.5f, tileSize * 0.44f};
        auto radius = tileSize * 0.28f;

        //the operands keep moving, so the whole solve is redone on every frame
        RenderPath a, b, c;
        star(a, {center.x - radius * 0.3f, center.y - radius * 0.25f}, radius, radius * 0.42f, 5);
        circle(b, {center.x + cosf(angle) * radius * 0.5f, center.y + sinf(angle) * radius * 0.5f}, radius * 0.62f);
        bar(c, {center.x, center.y + cosf(angle) * radius * 0.5f}, tileSize * 0.42f, tileSize * 0.06f);

        auto begin = chrono::high_resolution_clock::now();

        RenderPath out[TILES];
        merge(a, b, out[0]);
        AddMask(a, b, out[1]);
        SubtractMask(a, b, out[2]);
        IntersectMask(a, b, out[3]);
        DifferenceMask(a, b, out[4]);

        //a group accumulates, so a merged result becomes the operand of the next one
        RenderPath tmp;
        if (AddMask(a, b, tmp)) SubtractMask(tmp, c, out[5]);

        auto spent = chrono::duration<double, milli>(chrono::high_resolution_clock::now() - begin).count();
        cost += spent;
        ++costCnt;

        if (traceCost && elapsed / 1000 > reported) {
            reported = elapsed / 1000;
            printf("merge path: %.3f ms / frame\n", cost / costCnt);
            cost = 0.0;
            costCnt = 0;
        }

        for (uint32_t i = 0; i < TILES; ++i) {
            auto& tile = tiles[i];

            tile.result->reset();
            tile.result->appendPath(out[i].cmds.data(), out[i].cmds.size(), out[i].pts.data(), out[i].pts.size());

            tile.operands->reset();
            tile.operands->appendPath(a.cmds.data(), a.cmds.size(), a.pts.data(), a.pts.size());
            tile.operands->appendPath(b.cmds.data(), b.cmds.size(), b.pts.data(), b.pts.size());
        }
        tiles[5].operands->appendPath(c.cmds.data(), c.cmds.size(), c.pts.data(), c.pts.size());

        canvas->update();

        return true;
    }
};


/************************************************************************/
/* Stress Contents                                                      */
/************************************************************************/

/* every operand is built out of cubics only and the merges are accumulated, so a
   frame runs count-1 boolean operations on curves that never stop moving. */
struct StressDemo : tvgdemo::Demo
{
    uint32_t count;

    Shape* result = nullptr;
    Shape* operands = nullptr;
    Text* label = nullptr;
    Point center{};
    float radius = 0.0f;

    double cost = 0.0;
    uint32_t costCnt = 0;
    uint32_t reported = 0;

    StressDemo(uint32_t count) : count(count) {}

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

        operands = Shape::gen();
        operands->strokeWidth(1.0f);
        operands->strokeFill(150, 155, 165, 90);
        canvas->add(operands);

        if (Text::load(fontFile) == Result::Success) {
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

        operands->reset();
        for (auto& b : blobs) operands->appendPath(b.cmds.data(), b.cmds.size(), b.pts.data(), b.pts.size());

        uint32_t cubics = 0;
        for (auto cmd : acc.cmds) {
            if (cmd == PathCommand::CubicTo) ++cubics;

        }

        if (label) {
            char buf[128];
            snprintf(buf, sizeof(buf), "%u blobs, %u merges  |  %u cubics out  |  %.2f ms", count, count - 1, cubics, spent);
            label->text(buf);
        }

        if (traceCost && elapsed / 1000 > reported) {
            reported = elapsed / 1000;
            printf("stress: %.3f ms / frame (%u merges, %u cubics out)\n", cost / costCnt, count - 1, cubics);
            cost = 0.0;
            costCnt = 0;
        }

        canvas->update();

        return true;
    }
};


/************************************************************************/
/* Entry Point                                                          */
/************************************************************************/

int main(int argc, char **argv)
{
    auto stress = 0;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-t")) traceCost = true;
        else if (!strcmp(argv[i], "-s")) stress = (i + 1 < argc && isdigit(argv[i + 1][0])) ? atoi(argv[++i]) : 12;
    }

    if (stress > 0) {
        printf("stress: a union of %d curve blobs, accumulated over %d merges\n", stress, stress - 1);
        return tvgdemo::main(new StressDemo(uint32_t(stress)), argc, argv, true, 900, 900, 0);
    }

    printf("tiles: Merge | Add | Subtract  /  Intersect | Exclude | Add then Subtract\n");

    return tvgdemo::main(new UserDemo, argc, argv, true, 1200, 800, 0);
}
