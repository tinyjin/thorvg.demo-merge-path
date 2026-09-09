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

## Adding a demo

Add a file next to `demo_tiles.h`, then point `tvgmergepath.cpp` at it:

```cpp
return tvgdemo::main(new MyDemo(), argc, argv, true, 1600, 640, 0);
```

