# OpenMVS: open Multi-View Stereo reconstruction library

[![Watch reconstruction](https://i.imgur.com/S4L0E6r.jpeg)](https://sketchfab.com/models/3aedcdd22c31447199c18dd9aec5d952/embed)

## Introduction

[OpenMVS (Multi-View Stereo)](http://cdcseacave.github.io/openMVS) is a library for computer-vision scientists and especially targeted to the Multi-View Stereo reconstruction community. While there are mature and complete open-source projects targeting Structure-from-Motion pipelines (like [OpenMVG](https://github.com/openMVG/openMVG)) which recover camera poses and a sparse 3D point-cloud from an input set of images, there are none addressing the last part of the photogrammetry chain-flow. *OpenMVS* aims at filling that gap by providing a complete set of algorithms to recover the full surface of the scene to be reconstructed. The input is a set of camera poses plus the sparse point-cloud and the output is a textured mesh. The main topics covered by this project are:

- **dense point-cloud reconstruction** for obtaining a complete and accurate as possible point-cloud
- **mesh reconstruction** for estimating a mesh surface that explains the best the input point-cloud
- **mesh refinement** for recovering all fine details
- **mesh texturing** for computing a sharp and accurate texture to color the mesh

See the complete [documentation](https://github.com/cdcseacave/openMVS/wiki) on wiki.

## Build

See the [building](https://github.com/cdcseacave/openMVS/wiki/Building) wiki page. Windows, Ubuntu and MacOS x64 continuous integration status [![Continuous Integration](https://github.com/cdcseacave/openMVS/actions/workflows/continuous_integration.yml/badge.svg)](https://github.com/cdcseacave/openMVS/actions/workflows/continuous_integration.yml)
Automatic Windows x64 binary builds can be found for each commit on its Artifacts page.

## Example

See the usage [example](https://github.com/cdcseacave/openMVS/wiki/Usage) wiki page.

### Low-memory dense fusion

Design details (Chinese): [docs/LowMemoryFusionAndSimplification.md](docs/LowMemoryFusionAndSimplification.md).

For large scenes, `DensifyPointCloud` can write the dense-fuse result incrementally
instead of retaining the complete point-cloud and its per-point view and weight
arrays in memory. By default this mode does not downsample the output:

```text
DensifyPointCloud -w depthmaps -i scene.mvs --resolution-level 2 --fusion-mode 0 --fusion-filter 2 --geometric-iters 0 --postprocess-dmaps 0 --stream-fusion 1 --stream-fusion-cache-mb 2048
```

Missing depth maps are estimated first with the normal per-image estimation (each
map is saved and released immediately), then the fusion streams from the depth-map
files; depth maps already in the working folder are reused, so the estimation can
also be run separately beforehand with `--fusion-mode 1`. Views whose estimation
produced no depth map are skipped with a warning. Existing maps are validated
and truncated, resolution-mismatched, or camera-mismatched maps are reported
rather than reconstructed. Changing the resolution level does not
invalidate existing depth maps automatically, so use a separate working folder or
regenerate them. Keep the input images and camera poses unchanged between
runs that share a working folder.

The cache budget is in MiB and accounts for resident depth maps, color images,
active pixel masks, and one decoded 8-bit RGB image. It is not a limit on the
whole process: decoder internals, scene data, library allocations, and other
temporary allocations also require memory. If the active reference image and its
neighbors do not fit, the command fails with a message to increase the budget;
it does not silently drop neighbors. Color images are loaded only when
`--estimate-colors 2` is used.
The pixel-visited masks are stored in a temporary file, and the PLY is written
to a temporary file in the output directory before replacing the destination.
Allow disk space for the output and up to one bit per depth-map pixel (rounded to
machine-word boundaries). A process interrupted forcibly may leave its temporary
`.stream-*` directory behind.

The streaming mode supports `--fusion-mode 0`, `--fusion-filter 2`,
`--geometric-iters 0`, `--postprocess-dmaps 0`, and color/normal estimation
values 0 or 2. Operations requiring an in-memory point-cloud, such as final
color/normal estimation, segmentation, and point-cloud filtering, and
`--normalize-coordinates` are not supported in this mode. The output is a binary
PLY; it does not contain per-point view IDs or weights, only the view count, and
does not create a new `.mvs` scene. ROI cropping and the existing binary PLY
properties are preserved. The physical cache size does not change the streaming
fusion order, although it can affect disk I/O. Results can differ if a
non-streaming run's cache eviction or parallel scheduling changes the order of
fusion or selection of neighbors. ROI cropping can change record order even when
the point sets are identical.

The streaming fusion uses all `--max-threads` threads and its output is
byte-identical for any thread count. Each reference depth-map is split into
row strips that are fused speculatively in parallel without touching the shared
pixel masks (recording which pixels each point would claim and which it skipped);
the strips are then committed strictly in row order, re-checking every speculative
point against the current masks and recomputing serially only the few that conflict.
The PLY is written by a background thread, and the missing depth maps of the next
reference image are loaded in the background while the current one is fused.
A larger `--stream-fusion-cache-mb` reduces reloading from disk.

The existing PLY writer limits the output to 2,147,483,647 vertices; exceeding
this limit fails without replacing the destination. As in the normal mode,
`--remove-dmaps 1` deletes depth maps after a successful fusion.

### Feature-adaptive simplification (opt-in)

Add these options to the **streaming fusion** command above (defaults shown):

```text
--simplify-adaptive 1 --simplify-min-spacing 1.0 --simplify-max-spacing 2.0 --simplify-error 0.5 --simplify-transition 30 --simplify-memory-mb 256
```

Flat, untextured areas are thinned strongly, edges, thin structures and texture
keep their detail, and the density changes smoothly between them
(no cells, no grid, no block pattern). The output is a strict subset of the normal
streaming output (original positions, colors, normals and view counts; nothing
is averaged or moved).

Local GSD: for every fused point, each supporting view projects the surface
tangent plane through the depth-map camera (projection Jacobian); the lower
20th-percentile pixel footprint is the point GSD `g` (scene units), capped at twice
the scene median. Points without a valid scale are always kept. The log reports
dense GSD min/max and approximate p10/p50/p90.

Feature score `s` in [0,1], from the neighbors within `3g` (wide enough that depth
noise on a flat surface stays near 0), is the maximum of:

- surface variation of the local PCA (linear ramp 0.03..0.25): edges, corners, clutter;
- linearity (smoothstep 0.7..0.9): wires, poles, thin strips;
- half of the luminance standard deviation ramp (10..50, if colors are written): texture.

Linear ramps (instead of near-binary steps) avoid density steps when surface noise
or material changes. Sparse areas need no extra term: Poisson-disk sampling never
removes points that have no retained neighbor.

Target spacing `r = g * (min + (max - min) * (1 - s))`, then limited so that `ln r`
grows by at most `ln 2` per `--simplify-transition` median GSD (default 30, about
4 m at 0.14 m GSD). This is an exact distance transform (Dijkstra) over occupied
voxels of one median GSD, so it follows the surface, only ever lowers `r`, and turns
every density change into a gradual ramp; `0` disables it.

Sampling: variable-radius Poisson-disk selection over the original points, visiting
features first (then more views, then a stable hash). A point `p` is removed if an
already retained point `q` is within `min(r_p, r_q)` along `p`'s local PCA tangent
plane and either closer than `min * g`, or within the surface noise band
(`error * g + 2.5 sigma`, at most `r_p`) across the plane with the neighborhood
centroids of `p` and `q` within `error * g` of the same plane. Spacing is therefore
measured along the surface, so a thick noisy layer is not kept as several stacked
layers, while separate layers and small reliefs are still kept.

Out-of-core processing: point metadata is spooled to disk and split into square
xy tiles with a halo covering the spacing transition, the feature radius and the
Poisson conflicts (about 44 median GSD with the defaults). Tiles are processed in a
fixed order; decisions near a tile edge are passed to the neighbor tiles, so there
are no seams. The tile size is derived from `--simplify-memory-mb` (about 224 bytes
per loaded point); it is a target, not
a hard limit, because the densest tile may exceed it. Changing it changes the
processing order and therefore the exact sample set slightly (the guarantees above
do not change). The fusion cache is released before simplification. Allow about
100 bytes of temporary disk space per input point in addition to the PLY and visit
masks. An I/O error aborts without replacing an existing destination PLY.

Each tile is processed with all `--max-threads` threads, with byte-identical output
for any thread count: parallel sorts with total orders, the spacing transition as a
parallel label-correcting shortest path (monotonic rounding makes it converge to the
same values as Dijkstra), and the Poisson-disk selection as a deterministic parallel
greedy (a point only depends on the covering points ranked before it: it is removed
once one of them is retained and retained once all of them are removed).

Reduction depends on how much of the scene is flat and untextured and on point
density relative to GSD; there is no fixed ratio. Validation with the default
settings on a 180-depth-map aerial scene (`--resolution-level 2`, 768 MiB fusion cache):

| | Points | Peak working set | Time |
|---|---:|---:|---:|
| Streaming fusion | 19,082,657 | ~880 MiB | 3m27s |
| + adaptive simplification | 6,525,424 (-65.80%) | ~880 MiB | 7m54s (simplification 2m52s) |

The output was verified to be an exact record subset of the unsimplified output.
For the removed points, the distance to the nearest retained point (in median GSD)
is p50 0.95, p99 1.86, p99.9 2.85 (max 4.57, inside thick noisy layers or where the
local GSD is larger). 16% of the points were classified as features. Maps of the
3D sample spacing show smooth density ramps and no seams at material changes or
across the 16 tile borders. Use a smaller `--simplify-max-spacing` (e.g. 1.5) to keep more points
on flat areas, or a larger one (up to 4) for stronger reduction. These are sample
measurements, not guarantees for other scenes.

When built with `OpenMVS_ENABLE_TESTS`, run `Tests --stream-fusion` for the
small-scene streaming regression test (all color/normal schemas, ROI on/off,
and simplification on/off). `Tests 0` also checks the GSD estimate and, on a
synthetic scene (flat area, 90 degree ridge, thin line, two layers 3 GSD apart,
checker texture, unknown GSD), original-point membership, coverage distance,
flat-area reduction, feature/texture density, single-tile versus multi-tile
consistency and failed-output safety. An optional second argument after the test
selector enables a diagnostic log, for example `Tests 0 simplify-tests.log`.

## License

See the [copyright](https://github.com/cdcseacave/openMVS/blob/master/COPYRIGHT.md) file.

## Contact

openmvs[AT]googlegroups.com
