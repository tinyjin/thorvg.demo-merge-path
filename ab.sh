#!/usr/bin/env bash
#
# Runs the same demo against two revisions of the solver, so the flattening
# approach and the curve keeping one can be watched side by side.
#
#   ./ab.sh build          build tvgmergepath-flat and tvgmergepath-curve
#   ./ab.sh flat  -s 24    run the flattening solver
#   ./ab.sh curve -s 24    run the curve keeping solver
#   ./ab.sh both  -s 24    run both, side by side
#   ./ab.sh clean
#
# The old solver is taken from the git history, nothing in the tree is touched.

set -e

REV=${REV:-cbe3971}          # the last commit before the curve rewrite
DIR=$(cd "$(dirname "$0")" && pwd)
WORK=$DIR/.ab
THORVG=${THORVG:-/opt/homebrew}

build() {
    echo "== extracting the solver of $REV"
    rm -rf "$WORK"
    mkdir -p "$WORK"

    # a quoted include resolves next to the source, so the whole set is copied
    git -C "$DIR" show "$REV:mergepath.h" > "$WORK/mergepath.h"
    cp "$DIR/tvgmergepath.cpp" "$DIR/template.h" "$WORK/"

    echo "== building tvgmergepath-flat  (solver of $REV)"
    g++ "$WORK/tvgmergepath.cpp" -o "$DIR/tvgmergepath-flat" \
        -O3 -std=c++20 -DRES_DIR=\""$DIR/res"\" -I"$WORK" -I"$THORVG/include" \
        $(sdl2-config --cflags --libs) -L"$THORVG/lib" -lthorvg -Wl,-rpath,"$THORVG/lib"

    echo "== building tvgmergepath-curve (solver of the working tree)"
    g++ "$DIR/tvgmergepath.cpp" -o "$DIR/tvgmergepath-curve" \
        -O3 -std=c++20 -DRES_DIR=\""$DIR/res"\" -I"$DIR" -I"$THORVG/include" \
        $(sdl2-config --cflags --libs) -L"$THORVG/lib" -lthorvg -Wl,-rpath,"$THORVG/lib"

    echo "== done"
}

need() {
    [ -x "$DIR/$1" ] || { echo "$1 is missing, run './ab.sh build' first"; exit 1; }
}

case "${1:-}" in
    build) build ;;
    flat)  shift; need tvgmergepath-flat;  "$DIR/tvgmergepath-flat" "$@" ;;
    curve) shift; need tvgmergepath-curve; "$DIR/tvgmergepath-curve" "$@" ;;
    both)
        shift
        need tvgmergepath-flat
        need tvgmergepath-curve
        echo "== flat and curve are running, close either window to stop both"
        "$DIR/tvgmergepath-flat" "$@" &
        FLAT=$!
        "$DIR/tvgmergepath-curve" "$@" &
        CURVE=$!
        trap 'kill $FLAT $CURVE 2>/dev/null' EXIT
        #the stock macOS bash is 3.2 and has no 'wait -n', so poll instead
        while kill -0 $FLAT 2>/dev/null && kill -0 $CURVE 2>/dev/null; do
            sleep 0.3
        done
        ;;
    clean) rm -rf "$WORK" "$DIR/tvgmergepath-flat" "$DIR/tvgmergepath-curve"; echo "cleaned" ;;
    *)
        echo "usage: $0 {build|flat|curve|both|clean} [demo args]"
        echo "  e.g. $0 build && $0 both -s 24 -t"
        exit 1
        ;;
esac
