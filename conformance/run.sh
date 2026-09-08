#!/usr/bin/env bash
#
# Skia's pathops corpus, run through our solver. The whole report goes to the
# terminal; nothing is written to disk.
#
#   ./conformance/run.sh                build if needed, then run
#   ./conformance/run.sh control        the same corpus with our solver asked for the
#                                       wrong operator, which is the check that the
#                                       judge can fail
#   ./conformance/run.sh repro <case>   print one case back as the calls that run it
#   ./conformance/run.sh build          build only
#
# -q drops the progress counter, for a log.
#
# Skia is not bundled and none of its sources are vendored here. SKIA must point at
# a checkout that has been built the way bench/run.sh documents, and the case files
# are read straight out of it at compile time - so the report reflects whatever
# revision is sitting there, and pulling skia pulls the corpus with it.
#
# What a fresh machine needs:
#
#   - a built skia at $SKIA/out/pathops/libskia.a. bench/run.sh carries the gn args.
#   - thorvg, at /opt/homebrew or wherever THORVG points.
#
#     SKIA=~/skia THORVG=/opt/homebrew ./conformance/run.sh
#
# The shim in shim/tests stands in for skia's own test headers, so it tracks their
# signatures. If a skia revision changes testPathOp() or DEF_TEST, the shim is what
# has to move.

set -e

DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$DIR/.." && pwd)
THORVG=${THORVG:-/opt/homebrew}
SKIA=${SKIA:-$HOME/Work/LottieFiles/skia}

# PathOpsOpTest.cpp is skia's core two-path corpus, PathOpsBattles.cpp is real svg
# figures and PathOpsSkpTest.cpp is paths captured off real web pages. shim/ also
# carries fuzz763, issue3651 and inverse - add a word here to fold them in.
CORPUS="optest battles skp"

build() {
    [ -f "$SKIA/out/pathops/libskia.a" ] || {
        echo "!! $SKIA/out/pathops/libskia.a is missing"
        echo "   point SKIA at a skia checkout built the way bench/run.sh documents"
        exit 1
    }
    [ -f "$THORVG/include/thorvg.h" ] || {
        echo "!! $THORVG/include/thorvg.h is missing"
        echo "   point THORVG at a thorvg install"
        exit 1
    }

    mkdir -p "$DIR/.obj"
    objs=""
    for c in $CORPUS; do
        echo "== corpus_$c"
        # -w: skia's own case files are not ours to keep warning-clean
        g++ -c "$DIR/shim/corpus_$c.cpp" -o "$DIR/.obj/corpus_$c.o" -O1 -std=c++20 \
            -w -I"$DIR/shim" -I"$SKIA" -I"$SKIA/tests"
        objs="$objs $DIR/.obj/corpus_$c.o"
    done

    echo "== support"
    g++ -c "$DIR/shim/support.cpp" -o "$DIR/.obj/support.o" -O2 -std=c++20 -w
    g++ -c "$SKIA/tests/PathOpsTestCommon.cpp" -o "$DIR/.obj/testcommon.o" -O2 -std=c++20 -w -I"$SKIA"
    objs="$objs $DIR/.obj/support.o $DIR/.obj/testcommon.o"

    echo "== conformance"
    # the frameworks are libskia's own, nothing here reaches for them
    g++ "$DIR/conformance.cpp" "$ROOT/mergepath.cpp" $objs \
        -o "$DIR/conformance" -O2 -std=c++20 \
        -I"$THORVG/include" -I"$SKIA" -I"$DIR" \
        -L"$THORVG/lib" -lthorvg -Wl,-rpath,"$THORVG/lib" \
        "$SKIA/out/pathops/libskia.a" \
        -framework CoreFoundation -framework CoreGraphics -framework CoreText -framework CoreServices
}

ready() { [ -x "$DIR/conformance" ] || build; }

# a bare flag is the run, so ./run.sh -q needs no subcommand in front of it
cmd=run
case "${1:-}" in
    build|run|control|repro) cmd=$1; shift ;;
    -*|"") ;;
    *) echo "usage: $0 {build|run|control|repro <case>} [-q]"; exit 1 ;;
esac

case "$cmd" in
    build) build ;;
    run) ready; "$DIR/conformance" "$@" ;;
    control) ready; "$DIR/conformance" --control "$@" ;;
    repro)
        [ -n "${1:-}" ] || { echo "usage: $0 repro <case>"; exit 1; }
        ready; "$DIR/conformance" --repro "$1"
        ;;
esac
