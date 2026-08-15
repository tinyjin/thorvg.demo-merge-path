#!/usr/bin/env bash
#
# Runs the same demo against two revisions of the solver, so the flattening
# approach and the curve keeping one can be watched side by side.
#
#   ./ab.sh build          build every variant
#   ./ab.sh flat  -s 24    the flattening solver of an older revision
#   ./ab.sh curve -s 24    the curve keeping solver of the working tree
#   ./ab.sh skia  -s 24    the same scenes on skia alone, skia solves and rasterizes
#   ./ab.sh both  -s 24    flat and curve, side by side
#   ./ab.sh vs    -s 24    curve and skia, side by side
#   ./ab.sh clean
#
# The old solver is taken from the git history. The skia demo is a standalone
# program that never links thorvg, so the two are compared as whole engines.
# Skia needs SKIA to point at a checkout built as described in bench/run.sh.

set -e

REV=${REV:-cbe3971}          # the last commit before the curve rewrite
DIR=$(cd "$(dirname "$0")" && pwd)
WORK=$DIR/.ab
THORVG=${THORVG:-/opt/homebrew}
SKIA=${SKIA:-$HOME/Work/LottieFiles/skia}

build() {
    echo "== extracting the solver of $REV"
    rm -rf "$WORK"
    mkdir -p "$WORK"

    # a quoted include resolves next to the source, so the whole set is copied
    git -C "$DIR" show "$REV:mergepath.h" > "$WORK/mergepath.h"
    cp "$DIR/tvgmergepath.cpp" "$DIR/template.h" "$DIR/demo_tiles.h" "$DIR/demo_stress.h" "$DIR/demo_comb.h" "$WORK/"

    echo "== building tvgmergepath-flat  (solver of $REV)"
    g++ "$WORK/tvgmergepath.cpp" -o "$DIR/tvgmergepath-flat" \
        -O3 -std=c++20 -DRES_DIR=\""$DIR/res"\" -I"$WORK" -I"$THORVG/include" \
        $(sdl2-config --cflags --libs) -L"$THORVG/lib" -lthorvg -Wl,-rpath,"$THORVG/lib"

    echo "== building tvgmergepath-curve (solver of the working tree)"
    g++ "$DIR/tvgmergepath.cpp" "$DIR/mergepath.cpp" -o "$DIR/tvgmergepath-curve" \
        -O3 -std=c++20 -DRES_DIR=\""$DIR/res"\" -I"$DIR" -I"$THORVG/include" \
        $(sdl2-config --cflags --libs) -L"$THORVG/lib" -lthorvg -Wl,-rpath,"$THORVG/lib"

    if [ -f "$SKIA/out/pathops/libskia.a" ]; then
        echo "== building skiamergepath     (skia only, no thorvg)"
        g++ "$DIR/bench/skia_demo.cpp" -o "$DIR/skiamergepath" \
            -O3 -std=c++20 -I"$SKIA" -I"$DIR/bench" -I/opt/homebrew/include \
            $(sdl2-config --cflags --libs) "$SKIA/out/pathops/libskia.a" \
            -framework CoreFoundation -framework CoreGraphics -framework CoreText -framework CoreServices
    else
        echo "-- skipping skiamergepath, $SKIA/out/pathops/libskia.a is missing"
    fi

    echo "== done"
}

need() {
    [ -x "$DIR/$1" ] || { echo "$1 is missing, run './ab.sh build' first"; exit 1; }
}

case "${1:-}" in
    build) build ;;
    flat)  shift; need tvgmergepath-flat;  "$DIR/tvgmergepath-flat" "$@" ;;
    curve) shift; need tvgmergepath-curve; "$DIR/tvgmergepath-curve" "$@" ;;
    skia)  shift; need skiamergepath;      "$DIR/skiamergepath" "$@" ;;
    both|vs)
        MODE=$1
        shift
        if [ "$MODE" = "vs" ]; then
            need tvgmergepath-curve
            need skiamergepath
            echo "== thorvg and skia are running, close either window to stop both"
            "$DIR/tvgmergepath-curve" "$@" &
            FLAT=$!
            "$DIR/skiamergepath" "$@" &
            CURVE=$!
        else
            need tvgmergepath-flat
            need tvgmergepath-curve
            echo "== flat and curve are running, close either window to stop both"
            "$DIR/tvgmergepath-flat" "$@" &
            FLAT=$!
            "$DIR/tvgmergepath-curve" "$@" &
            CURVE=$!
        fi
        trap 'kill $FLAT $CURVE 2>/dev/null' EXIT
        #the stock macOS bash is 3.2 and has no 'wait -n', so poll instead
        while kill -0 $FLAT 2>/dev/null && kill -0 $CURVE 2>/dev/null; do
            sleep 0.3
        done
        ;;
    clean)
        rm -rf "$WORK" "$DIR/tvgmergepath-flat" "$DIR/tvgmergepath-curve" "$DIR/skiamergepath"
        echo "cleaned"
        ;;
    *)
        echo "usage: $0 {build|flat|curve|skia|both|vs|clean} [demo args]"
        echo "  e.g. $0 build && $0 vs -s 24 -t"
        exit 1
        ;;
esac
