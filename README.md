# Path Demo

Prototype of the boolean path ops behind Lottie's Merge Paths. The four functions in `mergepath.h` are the core of a Path class ThorVG does not have yet.

## Path APIs

Path ops function each (That also aligns to Lottie MergePaths):

```cpp
bool AddMask(const RenderPath& a, const RenderPath& b, RenderPath& out);   // mm:2
bool SubtractMask(const RenderPath& a, const RenderPath& b, RenderPath& out);   // mm:3
bool IntersectMask(const RenderPath& a, const RenderPath& b, RenderPath& out);  // mm:4
bool DifferenceMask(const RenderPath& a, const RenderPath& b, RenderPath& out); // mm:5, exclude intersections
```

`mm:1` (Merge) concatenates the two paths. No boolean, so it is not here.

## Build

ThorVG and SDL2. The makefile defaults to Homebrew (`/opt/homebrew`):

```bash
make
./tvgmergepath
```

Against a local install: `make THORVG=/path/to/install`.

```
./tvgmergepath          # basic
./tvgmergepath -s 12    # stress test: union N cubic blobs every frame
./tvgmergepath -n       # hide operand outlines
```

## Conformance

Skia's own pathops corpus, run through this solver. The case files are not
transcribed: `conformance/shim/` compiles skia's test sources against a shim in
which `testPathOp()` records its two operands instead of solving them, so the
geometry is skia's bit for bit. The whole report goes to the terminal.

Skia is not bundled. Build one first, with pathops and nothing else:

```bash
python3 tools/git-sync-deps && bin/fetch-gn
bin/gn gen out/pathops --args='...'   # bench/run.sh carries the full args
ninja -C out/pathops skia
```

Then, with ThorVG installed:

```bash
SKIA=~/skia THORVG=/opt/homebrew ./conformance/run.sh
```

```
./conformance/run.sh                # build if needed, then run
./conformance/run.sh control        # the same corpus with the wrong operator asked
                                    # for - the check that the judge can fail
./conformance/run.sh repro <case>   # one case, printed back as the calls that run it
```

## Adding a demo

Add a file next to `demo_tiles.h`, then point `tvgmergepath.cpp` at it:

```cpp
return tvgdemo::main(new MyDemo(), argc, argv, true, 1600, 640, 0);
```

