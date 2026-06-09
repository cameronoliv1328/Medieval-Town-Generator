# Layout Harness

Standalone renderer for the town layout algorithms. It compiles the actual
generator sources (`OrganicStreetGraph.cpp`, `OrganicStreetGenerator.cpp`,
`OrganicParcelGenerator.cpp`) against a small Unreal-type shim, runs them on a
synthetic terrain + river that mirrors `BuildOrganicRoadNetwork()`, and writes
a top-down image — so street/parcel changes can be iterated and visually
verified in seconds without launching the editor.

## Usage

```bash
Harness/build.sh [seed] [outprefix] # writes <outprefix>.ppm + .png
/tmp/town_harness 1337 /tmp/town debug # extra overlay: occupancy grid + rejects
```

Requires `g++` (C++17) and Python with Pillow for the PNG conversion.

## Layout

- `HarnessMain.cxx` — scene setup, renderer, entry point. The `.cxx`
  extension keeps Unreal Build Tool from compiling it as module code.
- `Shim/` — minimal stand-ins for `CoreMinimal.h` (FVector2D, TArray, FMath,
  FRandomStream, …), `Algo/Reverse.h`, `Engine/DataAsset.h`, and the
  `.generated.h`. The shim directory is not on the module include path, so
  in-engine builds are unaffected.

## Reading the output

- Tan lines: streets (width/brightness by tier); white spans: bridges.
- Red/brown rectangles: buildings (colour by kind); faint outlines: burgage
  plots; small tan boxes in yards: outbuildings.
- Yellow dot: market square centre; red dots: gates; grey ring: wall line.
