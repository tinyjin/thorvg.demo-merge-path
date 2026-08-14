#include <cstdio>
#include <chrono>
#include "include/core/SkPath.h"
#include "include/core/SkPathBuilder.h"
#include "include/pathops/SkPathOps.h"
#include "../mergepath.h"
#include "scenario.h"
using clk = std::chrono::high_resolution_clock;
static double el(clk::time_point b) { return std::chrono::duration<double, std::milli>(clk::now() - b).count(); }

//a 10 sided polygon, so the segment count matches the 10 cubic blob
static scenario::Contour ngon(float cx, float cy, float r, uint32_t n)
{
    scenario::Contour out;
    std::vector<std::pair<float,float>> pts;
    for (uint32_t i = 0; i < n; ++i) {
        auto a = float(i) * 2.0f * float(M_PI) / float(n);
        pts.push_back({cx + cosf(a) * r, cy + sinf(a) * r});
    }
    for (size_t i = 0; i < pts.size(); ++i) {
        auto& p1 = pts[i]; auto& p2 = pts[(i + 1) % pts.size()];
        out.push_back({p1.first, p1.second, 0,0,0,0, p2.first, p2.second, true});
    }
    return out;
}
static RenderPath tp(const scenario::Contour& c)
{
    RenderPath p; p.moveTo({c[0].x0, c[0].y0});
    for (auto& s : c) { if (s.line) p.lineTo({s.x3, s.y3}); else p.cubicTo({s.x1,s.y1},{s.x2,s.y2},{s.x3,s.y3}); }
    p.close(); return p;
}
static SkPath sp(const scenario::Contour& c)
{
    SkPathBuilder b; b.moveTo(c[0].x0, c[0].y0);
    for (auto& s : c) { if (s.line) b.lineTo(s.x3, s.y3); else b.cubicTo(s.x1,s.y1,s.x2,s.y2,s.x3,s.y3); }
    b.close(); return b.detach();
}
int main()
{
    printf("%-34s %10s %10s %8s\n", "10 segments each, union", "tvg ms", "skia ms", "ratio");
    struct C { const char* n; scenario::Contour a, b; };
    C cases[] = {
        {"line x line  (2 decagons)",  ngon(180,180,90,10), ngon(240,200,90,10)},
        {"curve x curve (2 blobs)",    scenario::blob(180,180,90,0.0f), scenario::blob(240,200,90,2.0f)},
        {"line x curve  (decagon+blob)", ngon(180,180,90,10), scenario::blob(240,200,90,2.0f)},
        {"star + circle (the bench case)", scenario::star(130,130,110,45,5), scenario::circle(220,220,90)},
        {"star + blob",                    scenario::star(130,130,110,45,5), scenario::blob(220,220,90,1.0f)},
        {"decagon + circle",               ngon(130,130,110,10), scenario::circle(220,220,90)},
        {"star + circle, far apart",       scenario::star(130,130,110,45,5), scenario::circle(300,300,90)},
        {"convex pentagon + circle",       ngon(130,130,110,5), scenario::circle(220,220,90)},
    };
    for (auto& c : cases) {
        auto ta = tp(c.a), tb = tp(c.b);
        auto sa = sp(c.a), sb = sp(c.b);
        auto t0 = clk::now();
        for (int i = 0; i < 500; ++i) { RenderPath o; AddMask(ta, tb, o); }
        auto tv = el(t0) / 500.0;
        t0 = clk::now();
        for (int i = 0; i < 500; ++i) { SkPath o; Op(sa, sb, kUnion_SkPathOp, &o); }
        auto sk = el(t0) / 500.0;
        RenderPath probe;
        AddMask(ta, tb, probe);
        uint32_t contours = 0;
        for (auto cmd : probe.cmds) if (cmd == PathCommand::MoveTo) ++contours;
        printf("%-34s %10.4f %10.4f %7.2fx   out %zu pts / %u contours\n", c.n, tv, sk, sk / tv, probe.pts.size(), contours);
    }
    return 0;
}
