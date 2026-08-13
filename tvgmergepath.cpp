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

static bool traceCost = false;


/************************************************************************/
/* ThorVG Drawing Contents                                              */
/************************************************************************/

struct UserDemo : tvgdemo::Demo
{
    static constexpr uint32_t COLS = 3;
    static constexpr uint32_t ROWS = 2;
    static constexpr uint32_t TILES = COLS * ROWS;

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
        }

        return update(canvas, 0);
    }

    bool update(Canvas* canvas, uint32_t elapsed) override
    {
        auto progress = float(elapsed % 4000) / 4000.0f;
        auto angle = progress * 2.0f * float(M_PI);
        auto center = Point{tileSize * 0.5f, tileSize * 0.5f};
        auto radius = tileSize * 0.3f;

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
/* Entry Point                                                          */
/************************************************************************/

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-t")) traceCost = true;
    }

    printf("tiles: Merge | Add | Subtract  /  Intersect | Exclude | Add then Subtract\n");

    return tvgdemo::main(new UserDemo, argc, argv, true, 1200, 800, 0);
}
