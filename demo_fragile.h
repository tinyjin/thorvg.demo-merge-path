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

/* The arrangements this solver does not hold.

   Each tile is built so the right answer keeps the same area on every frame while
   the operands move, so any change in what is drawn is the solver changing its
   mind rather than the geometry changing. The tile carries the area it should
   have and the area it got, and the console counts the frames that disagreed.

   The top row is the parameter space tolerance, the bottom row is three failures
   that sit somewhere else entirely - lowering PATHOP_EPSILON moves neither of
   them by a single digit.

   single intersection - a boundary touching another at exactly one point. The
   root there is a double root, and whether it is found rides on rounding.

   partial overlap - a run shared by only part of a segment. overlapped() matches
   whole control point sets so it never sees this, and the swarm guard wants 9
   roots before it fires, which a short run does not reach.

   far from origin - the solver works in the coordinates it is handed. Once the
   operands sit far enough out that their own size is lost in the mantissa, every
   difference taken along the way is cancellation. Solving the same pair around
   the origin and translating the answer back is all it takes.

   self crossing - _intersect() only pairs lhs segments against rhs ones, so a
   contour never meets itself. A shape that crosses its own boundary keeps arcs
   that lie inside it. The zero area operand is the same hole at its limit, a
   contour that is nothing but the overlap of itself.

   mixed sibling direction - _op() picks one reversal for the whole operand from
   a single _area() sign, and _merge() walks forward on both sides off that one
   decision. Nested outer and hole pairs survive it because reversing every
   contour together keeps them relative. Siblings that disagree do not. */

#ifndef _DEMO_FRAGILE_H_
#define _DEMO_FRAGILE_H_

#include "mergepath.h"
#include "template.h"

#ifndef RES_DIR
    #define RES_DIR "res"
#endif

struct FragileDemo : tvgdemo::Demo
{
    static constexpr const char* FONT = RES_DIR"/font/PublicSans-Regular.ttf";

    static constexpr uint32_t COLS = 3;
    static constexpr uint32_t ROWS = 2;
    static constexpr uint32_t TILES = COLS * ROWS;

    //the offset the third row of tiles is solved at, in the tile's own units
    static constexpr float FAR = 1.0e6f;

    static constexpr const char* LABELS[TILES] = {
        "single intersection", "partial overlap, sliding edge", "partial overlap, (a + b) - b",
        "far from origin", "self crossing contour", "mixed sibling direction"
    };

    static constexpr const char* NOTES[TILES] = {
        "a circle touching another from the inside",
        "a short edge running along a longer one",
        "b unioned in and taken out, against a - b in one step",
        "the same pair solved a million units out and brought back",
        "a bowtie, against the two triangles it is made of",
        "two siblings, one drawn the other way round"
    };

    //short names for the once a second console line
    static constexpr const char* TAGS[TILES] = {
        "tangent", "sliding edge", "(a+b)-b", "far origin", "self crossing", "mixed dir"
    };

    bool trace;
    bool outline;

    FragileDemo(bool trace, bool outline) : trace(trace), outline(outline) {}

    struct Tile
    {
        Shape* result = nullptr;
        Shape* operands = nullptr;
        Text* title = nullptr;
        Text* note = nullptr;
        Text* stat = nullptr;
        Point offset{};

        float want = 0.0f;   //the area the answer keeps on every frame
        float got = 0.0f;
        size_t cmds = 0;
        uint32_t wrong = 0;
        uint32_t frames = 0;
    };

    Tile tiles[TILES];
    float tileW = 0.0f, tileH = 0.0f;
    uint32_t reported = 0;

    //dir flips the winding by mirroring in x, so the shape is the same circle
    void circle(RenderPath& path, const Point& center, float r, float dir = 1.0f)
    {
        auto c = r * PATH_KAPPA;
        auto R = r * dir, C = c * dir;
        path.moveTo({center.x, center.y - r});
        path.cubicTo({center.x + C, center.y - r}, {center.x + R, center.y - c}, {center.x + R, center.y});
        path.cubicTo({center.x + R, center.y + c}, {center.x + C, center.y + r}, {center.x, center.y + r});
        path.cubicTo({center.x - C, center.y + r}, {center.x - R, center.y + c}, {center.x - R, center.y});
        path.cubicTo({center.x - R, center.y - c}, {center.x - C, center.y - r}, {center.x, center.y - r});
        path.close();
    }

    void translate(RenderPath& path, float d)
    {
        for (auto& pt : path.pts) { pt.x += d; pt.y += d; }
    }

    void box(RenderPath& path, float x, float y, float w, float h)
    {
        path.moveTo({x, y});
        path.lineTo({x + w, y});
        path.lineTo({x + w, y + h});
        path.lineTo({x, y + h});
        path.close();
    }

    /* the curves are walked rather than jumped over, since the solver cuts them at
       the crossings and an anchor only sum would grow with every cut it makes. */
    float area(const RenderPath& path)
    {
        auto sum = 0.0f;
        Point cur{}, home{};
        auto pt = path.pts.data();
        auto edge = [&](const Point& to) { sum += cross(cur, to); cur = to; };

        for (auto cmd : path.cmds) {
            switch (cmd) {
                case PathCommand::MoveTo: cur = home = *pt++; break;
                case PathCommand::LineTo: edge(*pt++); break;
                case PathCommand::CubicTo: {
                    compat::Bezier bz = {cur, pt[0], pt[1], pt[2]};
                    for (uint32_t i = 1; i <= 32; ++i) edge(bz.at(float(i) / 32.0f));
                    pt += 3;
                    break;
                }
                case PathCommand::Close: edge(home); break;
            }
        }
        return fabsf(sum) * 0.5f;
    }

    bool content(Canvas* canvas, uint32_t w, uint32_t h) override
    {
        tileW = float(w) / float(COLS);
        tileH = float(h) / float(ROWS);

        auto labeled = (Text::load(FONT) == Result::Success);

        auto bg = Shape::gen();
        bg->appendRect(0.0f, 0.0f, float(w), float(h));
        bg->fill(255, 255, 255);
        canvas->add(bg);

        for (uint32_t i = 0; i < TILES; ++i) {
            auto& tile = tiles[i];
            tile.offset = {float(i % COLS) * tileW, float(i / COLS) * tileH};

            auto frame = Shape::gen();
            frame->appendRect(tile.offset.x, tile.offset.y, tileW, tileH);
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

            if (outline) {
                auto operands = Shape::gen();
                operands->strokeWidth(1.0f);
                operands->strokeFill(150, 155, 165, 110);
                operands->translate(tile.offset.x, tile.offset.y);
                tile.operands = operands;
                canvas->add(operands);
            }

            if (!labeled) continue;

            auto line = [&](float size, uint8_t tone, float y) {
                auto text = Text::gen();
                text->font("PublicSans-Regular");
                text->size(tileH * size);
                text->fill(tone, tone, tone);
                text->translate(tile.offset.x + tileW * 0.06f, tile.offset.y + tileH * y);
                canvas->add(text);
                return text;
            };

            tile.title = line(0.040f, 55, 0.78f);
            tile.note = line(0.030f, 130, 0.845f);
            tile.stat = line(0.030f, 130, 0.895f);
            tile.title->text(LABELS[i]);
            tile.note->text(NOTES[i]);
        }

        return update(canvas, 0);
    }

    bool update(Canvas* canvas, uint32_t elapsed) override
    {
        auto angle = float(elapsed % 6000) / 6000.0f * 2.0f * float(M_PI);
        auto cx = tileW * 0.5f, cy = tileH * 0.42f;

        RenderPath out[TILES];
        RenderPath ops[TILES][2];

        /* the small circle touches the big one from the inside, so the union is
           just the big circle. the pair drifts together and stays tangent, which
           leaves the double root at the touch as the only thing that moves. */
        {
            auto R = tileH * 0.26f, r = R * 0.6f;
            auto dx = cosf(angle) * R * 0.3f, dy = sinf(angle) * R * 0.3f;
            circle(ops[0][0], {cx + dx, cy + dy}, R);
            circle(ops[0][1], {cx + dx, cy + dy - (R - r)}, r);
            AddMask(ops[0][0], ops[0][1], out[0]);
            tiles[0].want = float(M_PI) * R * R;
        }

        /* the short box is butted against the tall one and slides along the edge
           they share. the run is a piece of the tall box's edge and the whole of
           the short one's, so the two segments never match as wholes. */
        {
            auto bw = tileW * 0.22f, bh = tileH * 0.34f, sh = bh * 0.46f;
            auto bx = cx - bw, by = cy - bh * 0.5f;
            auto slide = (0.5f - 0.5f * cosf(angle)) * (bh - sh);
            box(ops[1][0], bx, by, bw, bh);
            box(ops[1][1], bx + bw, by + slide, bw, sh);
            AddMask(ops[1][0], ops[1][1], out[1]);
            tiles[1].want = bw * bh + bw * sh;
        }

        /* the tile that first showed this. b is unioned in and taken straight back
           out, so the answer is a on every frame. the union's boundary carries the
           pieces of b that the crossings cut, and the second operation then meets
           its own operand as a fragment rather than a whole segment. */
        {
            auto R = tileH * 0.22f;
            circle(ops[2][0], {cx - R * 0.45f, cy}, R);
            circle(ops[2][1], {cx + cosf(angle) * R * 0.6f, cy + sinf(angle) * R * 0.6f}, R * 0.8f);
            RenderPath tmp;
            if (AddMask(ops[2][0], ops[2][1], tmp)) SubtractMask(tmp, ops[2][1], out[2]);

            //(a + b) - b is a - b, so the same subtract done in one step is the answer
            RenderPath direct;
            SubtractMask(ops[2][0], ops[2][1], direct);
            tiles[2].want = area(direct);
        }

        /* the same pair solved twice, once here and once a million units out. the
           answer is translated back before it is measured, so what is left is the
           solver losing the operands' own size in the mantissa. */
        {
            auto R = tileH * 0.22f;
            circle(ops[3][0], {cx - R * 0.5f, cy}, R);
            circle(ops[3][1], {cx + R * 0.5f + cosf(angle) * R * 0.3f, cy + sinf(angle) * R * 0.3f}, R * 0.85f);

            RenderPath a = ops[3][0], b = ops[3][1];
            translate(a, FAR);
            translate(b, FAR);
            IntersectMask(a, b, out[3]);
            translate(out[3], -FAR);

            //the same intersect taken around the origin is the answer
            RenderPath origin;   //near is a windows.h macro
            IntersectMask(ops[3][0], ops[3][1], origin);
            tiles[3].want = area(origin);
        }

        /* a bowtie carries its crossing on its own boundary, and _intersect() only
           pairs lhs segments against rhs ones, so a contour never meets itself.
           the same picture cut into the two triangles it is made of is the
           answer, and the bar sweeps so both an over and an under fill show. */
        {
            auto W = tileW * 0.30f, H = tileH * 0.20f;
            auto& bow = ops[4][0];
            bow.moveTo({cx - W, cy - H});
            bow.lineTo({cx + W, cy + H});
            bow.lineTo({cx + W, cy - H});
            bow.lineTo({cx - W, cy + H});
            bow.close();
            box(ops[4][1], cx - W * 1.3f, cy + sinf(angle) * H * 0.7f - H * 0.18f, W * 2.6f, H * 0.36f);
            IntersectMask(bow, ops[4][1], out[4]);

            /* the crossing splits the bowtie into two triangles that carry the same
               picture without ever meeting themselves. area() reads a self crossing
               answer low, so the count is what to trust here rather than the number. */
            RenderPath split, want;
            split.moveTo({cx - W, cy - H}); split.lineTo({cx, cy}); split.lineTo({cx - W, cy + H}); split.close();
            split.moveTo({cx + W, cy + H}); split.lineTo({cx, cy}); split.lineTo({cx + W, cy - H}); split.close();
            IntersectMask(split, ops[4][1], want);
            tiles[4].want = area(want);
        }

        /* two contours side by side in one operand, the small one drawn the other
           way round. neither sits inside the other, so there is no reversal of the
           whole operand that lines both of them up with the bar. the bar keeps a
           slab of constant width through each, however far the small one slides. */
        {
            auto R = tileH * 0.20f, r = R * 0.30f;
            auto sx = cx + R * 1.9f + cosf(angle) * R * 0.3f;
            circle(ops[5][0], {cx - R * 0.55f, cy}, R, +1.0f);
            circle(ops[5][0], {sx, cy}, r, -1.0f);          //the sibling that disagrees
            box(ops[5][1], cx - R * 2.0f, cy - R * 0.18f, R * 4.6f, R * 0.36f);
            IntersectMask(ops[5][0], ops[5][1], out[5]);

            //the same pair with both siblings drawn the same way round is the answer
            RenderPath same, want;
            circle(same, {cx - R * 0.55f, cy}, R, +1.0f);
            circle(same, {sx, cy}, r, +1.0f);
            IntersectMask(same, ops[5][1], want);
            tiles[5].want = area(want);
        }

        for (uint32_t i = 0; i < TILES; ++i) {
            auto& tile = tiles[i];

            tile.cmds = out[i].cmds.size();
            tile.got = area(out[i]);
            ++tile.frames;
            if (fabsf(tile.got - tile.want) > tile.want * 0.01f) ++tile.wrong;

            tile.result->reset();
            tile.result->appendPath(out[i].cmds.data(), out[i].cmds.size(), out[i].pts.data(), out[i].pts.size());

            if (tile.operands) {
                tile.operands->reset();
                for (auto& op : ops[i]) tile.operands->appendPath(op.cmds.data(), op.cmds.size(), op.pts.data(), op.pts.size());
            }

            if (tile.stat) {
                char buf[128];
                snprintf(buf, sizeof(buf), "%zu cmds,  area %.0f  /  %.0f", tile.cmds, tile.got, tile.want);
                tile.stat->text(buf);
            }
        }

        if (elapsed / 1000 > reported) {
            reported = elapsed / 1000;
            printf("wrong frames per second  ");
            for (uint32_t i = 0; i < TILES; ++i) printf(" %s %u/%u  ", TAGS[i], tiles[i].wrong, tiles[i].frames);
            printf("\n");
            fflush(stdout);   //the line is a second apart, it should not sit in the buffer
            for (auto& tile : tiles) { tile.wrong = 0; tile.frames = 0; }
        }

        canvas->update();

        return true;
    }
};

#endif //_DEMO_FRAGILE_H_
