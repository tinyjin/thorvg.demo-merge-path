#!/usr/bin/env bash
#
# Runs the same boolean scenarios through this solver and through Skia's
# SkPathOps, then prints them side by side.
#
#   ./run.sh build     build both harnesses (needs a built Skia, see below)
#   ./run.sh           run and compare
#
# Skia is not bundled. Point SKIA at a checkout that has been built as:
#
#   python3 tools/git-sync-deps
#   bin/fetch-gn
#   bin/gn gen out/pathops --args='is_official_build=true is_component_build=false
#       skia_enable_ganesh=false skia_enable_graphite=false skia_enable_pdf=false
#       skia_enable_skottie=false skia_enable_svg=false skia_enable_skshaper=false
#       skia_use_expat=false skia_use_freetype=false skia_use_fontconfig=false
#       skia_use_harfbuzz=false skia_use_icu=false skia_use_libjpeg_turbo_decode=false
#       skia_use_libjpeg_turbo_encode=false skia_use_libpng_decode=false
#       skia_use_libpng_encode=false skia_use_libwebp_decode=false
#       skia_use_libwebp_encode=false skia_use_wuffs=false skia_use_zlib=false
#       skia_use_metal=false skia_use_vulkan=false skia_use_dng_sdk=false skia_use_xps=false'
#   ninja -C out/pathops skia

set -e

DIR=$(cd "$(dirname "$0")" && pwd)
THORVG=${THORVG:-/opt/homebrew}
SKIA=${SKIA:-$HOME/Work/LottieFiles/skia}

build() {
    echo "== tvg_bench"
    g++ "$DIR/tvg_bench.cpp" -o "$DIR/tvg_bench" -O2 -std=c++20 \
        -I"$THORVG/include" -L"$THORVG/lib" -lthorvg -Wl,-rpath,"$THORVG/lib"

    if [ -f "$SKIA/out/pathops/libskia.a" ]; then
        echo "== skia_bench"
        g++ "$DIR/skia_bench.cpp" -o "$DIR/skia_bench" -O2 -std=c++20 \
            -I"$SKIA" -I"$DIR" "$SKIA/out/pathops/libskia.a" \
            -framework CoreFoundation -framework CoreGraphics -framework CoreText -framework CoreServices
        echo "== crosscheck"
        g++ "$DIR/crosscheck.cpp" -o "$DIR/crosscheck" -O2 -std=c++20 \
            -I"$SKIA" -I"$DIR" -I"$THORVG/include" "$SKIA/out/pathops/libskia.a" \
            -L"$THORVG/lib" -lthorvg -Wl,-rpath,"$THORVG/lib" \
            -framework CoreFoundation -framework CoreGraphics -framework CoreText -framework CoreServices
    else
        echo "!! $SKIA/out/pathops/libskia.a is missing, only the tvg side will run"
    fi
}

compare() {
    [ -x "$DIR/tvg_bench" ] || { echo "run './run.sh build' first"; exit 1; }

    "$DIR/tvg_bench" > /tmp/bench_tvg.txt
    if [ -x "$DIR/skia_bench" ]; then "$DIR/skia_bench" > /tmp/bench_skia.txt; else : > /tmp/bench_skia.txt; fi

    python3 - <<'PY'
rows = {}
for path, engine in (("/tmp/bench_tvg.txt", "tvg"), ("/tmp/bench_skia.txt", "skia")):
    for line in open(path):
        e, case, op, ms, pts, cubics, area = line.strip().split("|")
        rows.setdefault((case, op), {})[e] = (float(ms), int(pts), int(cubics), float(area))

hdr = f'{"case":<22} {"op":<11} {"tvg ms":>9} {"skia ms":>9} {"ratio":>7} {"tvg pts":>8} {"skia pts":>9}'
print(hdr)
print("-" * len(hdr))
for (case, op), v in rows.items():
    t = v.get("tvg")
    s = v.get("skia")
    if not t:
        continue
    if not s:
        print(f'{case:<22} {op:<11} {t[0]:9.4f} {"-":>9} {"-":>7} {t[1]:8d} {"-":>9}')
        continue
    print(f'{case:<22} {op:<11} {t[0]:9.4f} {s[0]:9.4f} {s[0]/t[0]:6.2f}x {t[1]:8d} {s[1]:9d}')
#the geometry itself is judged by crosscheck, an area alone cannot tell the fill rules apart
PY
}

case "${1:-run}" in
    build) build ;;
    run)
        compare
        if [ -x "$DIR/crosscheck" ]; then
            echo
            echo "== geometry against skia (dense point sampling, fill rule aware)"
            "$DIR/crosscheck"
        fi
        ;;
    check) [ -x "$DIR/crosscheck" ] && "$DIR/crosscheck" || { echo "run './run.sh build' first"; exit 1; } ;;
    *) echo "usage: $0 {build|run|check}"; exit 1 ;;
esac
