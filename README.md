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

For large scenes, `DensifyPointCloud` can write the dense-fuse result incrementally
instead of retaining the complete point-cloud and its per-point view and weight
arrays in memory. This mode does not downsample the output:

```text
DensifyPointCloud -w depthmaps -i scene.mvs --resolution-level 2 --fusion-mode 1 --geometric-iters 0 --postprocess-dmaps 0
DensifyPointCloud -w depthmaps -i scene.mvs --resolution-level 2 --fusion-mode 0 --fusion-filter 2 --geometric-iters 0 --postprocess-dmaps 0 --stream-fusion 1 --stream-fusion-cache-mb 2048
```

Use the same working folder, input scene, image-resolution options, and depth-map
generation settings in both commands. Streaming fusion requires existing depth
maps; it reports missing, truncated, resolution-mismatched, or camera-mismatched
maps rather than reconstructing them. Changing the resolution level does not
invalidate existing depth maps automatically, so use a separate working folder or
regenerate them. Keep the input images and camera poses unchanged between the
two runs.

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

The existing PLY writer limits the output to 2,147,483,647 vertices; exceeding
this limit fails without replacing the destination. As in the normal mode,
`--remove-dmaps 1` deletes depth maps after a successful fusion.

When built with `OpenMVS_ENABLE_TESTS`, run `Tests --stream-fusion` for the
small-scene streaming regression test.

## License

See the [copyright](https://github.com/cdcseacave/openMVS/blob/master/COPYRIGHT.md) file.

## Contact

openmvs[AT]googlegroups.com
