#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
"""
Generate per-image ignore masks for aerial (nadir + oblique) imagery, to be used with
DensifyPointCloud --mask-path <out> --ignore-mask-label 0

Each mask is saved as <out>/<image name without extension>.mask.png (same size as the image):
  255 - pixel is used, 0 - pixel is ignored

Modes:
  geo   (default) uses the COLMAP model (cameras.txt/images.txt/points3D.txt): the ground plane is
        fitted to the sparse points and a pixel is masked if its viewing ray is more than
        --max-off-nadir degrees away from the vertical, or if the ray hits the local ground plane
        at a depth (camera z) larger than --max-depth (0 - disabled); nadir images stay intact,
        oblique images lose only their far (upper) part
  fixed   masks the top --ratio fraction of every image
  corners masks only the image corners (see --corner-radius, default 0.85 in this mode)

Additional masks, combined with any mode:
  --corner-radius r  masks pixels farther than r * half-diagonal from the principal point
                     (residual distortion of undistorted fisheye images is largest in the corners);
                     for a 4:3 image the image border is at 0.8 (left/right) and 0.6 (top/bottom),
                     so values in (0.8, 1) only cut the corners, smaller values also cut the borders
  --mask-black       masks the black (invalid) area left by undistortion, connected to the image border

Images found in the image folder but missing from the COLMAP model get the fixed mask (geo mode)
so that DensifyPointCloud finds a mask for every image.

example:
  python GenerateViewMasks.py --mode corners --corner-radius 0.85 --mask-black --images <undistorted>/images --out <undistorted>/masks
  python GenerateViewMasks.py --sparse <undistorted>/sparse --images <undistorted>/images --out <undistorted>/masks --max-off-nadir 60 --corner-radius 0.85
  python GenerateViewMasks.py --mode fixed --ratio 0.5 --images <undistorted>/images --out <undistorted>/masks
"""

import argparse
import os
import sys

import cv2
import numpy as np

IMAGE_EXTS = ('.jpg', '.jpeg', '.png', '.tif', '.tiff', '.bmp')


def read_cameras_txt(path):
    cameras = {}
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            elems = line.split()
            cam_id, model = int(elems[0]), elems[1]
            width, height = int(elems[2]), int(elems[3])
            params = list(map(float, elems[4:]))
            if model in ('SIMPLE_PINHOLE', 'SIMPLE_RADIAL', 'RADIAL'):
                fx = fy = params[0]
                cx, cy = params[1], params[2]
            elif model in ('PINHOLE', 'OPENCV', 'FULL_OPENCV'):
                fx, fy, cx, cy = params[:4]
            else:
                raise ValueError('unsupported camera model: ' + model)
            if model not in ('SIMPLE_PINHOLE', 'PINHOLE'):
                print('warning: camera %d is %s, distortion is ignored' % (cam_id, model))
            cameras[cam_id] = dict(width=width, height=height, K=np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]]))
    return cameras


def qvec2rotmat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * z * x + 2 * w * y],
        [2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x],
        [2 * z * x - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y]])


def read_images_txt(path):
    images = []
    with open(path, 'r') as f:
        lines = [l for l in f if not l.startswith('#')]
    for i in range(0, len(lines) - 1, 2):
        elems = lines[i].split()
        if len(elems) < 10:
            continue
        R = qvec2rotmat(list(map(float, elems[1:5])))
        t = np.array(list(map(float, elems[5:8])))
        pts = lines[i + 1].split()
        ids = np.array(pts[2::3], dtype=np.int64) if pts else np.zeros(0, np.int64)
        images.append(dict(R=R, t=t, C=-R.T @ t, camera_id=int(elems[8]),
                           name=' '.join(elems[9:]), point_ids=ids[ids >= 0]))
    return images


def read_points3d_txt(path):
    ids, xyz = [], []
    with open(path, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            elems = line.split(maxsplit=4)
            ids.append(int(elems[0]))
            xyz.append((float(elems[1]), float(elems[2]), float(elems[3])))
    return dict(zip(ids, range(len(ids)))), np.array(xyz)


def fit_up_vector(points, centers):
    # ground normal = smallest principal direction of the sparse points, oriented towards the cameras
    pts = points
    if len(pts) > 200000:
        pts = pts[np.random.default_rng(0).choice(len(pts), 200000, replace=False)]
    mean = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - mean, full_matrices=False)
    up = vt[2]
    if np.median((centers - mean) @ up) < 0:
        up = -up
    return up / np.linalg.norm(up)


def fixed_mask(width, height, ratio):
    mask = np.full((height, width), 255, np.uint8)
    mask[:int(round(height * ratio)), :] = 0
    return mask


def corner_mask(width, height, radius, cx=None, cy=None):
    # keep pixels within radius * half-diagonal from the principal point (image center by default)
    if cx is None or cy is None:
        cx, cy = width * 0.5, height * 0.5
    u, v = np.meshgrid(np.arange(width, dtype=np.float32) + 0.5, np.arange(height, dtype=np.float32) + 0.5)
    r2 = (u - cx) ** 2 + (v - cy) ** 2
    rmax = radius * 0.5 * np.hypot(width, height)
    return np.where(r2 <= rmax * rmax, 255, 0).astype(np.uint8)


def black_border_mask(img, threshold, margin):
    # mask near-black pixels connected to the image border (invalid area after undistortion)
    gray = img if img.ndim == 2 else img.max(axis=2)
    dark = (gray <= threshold).astype(np.uint8)
    num, labels = cv2.connectedComponents(dark, connectivity=8)
    border = np.unique(np.concatenate([labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]]))
    border = border[border > 0]
    invalid = np.isin(labels, border).astype(np.uint8)
    if margin > 0 and invalid.any():
        invalid = cv2.dilate(invalid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * margin + 1, 2 * margin + 1)))
    return np.where(invalid > 0, 0, 255).astype(np.uint8)


def read_image(image_dir, name, flags=cv2.IMREAD_COLOR):
    img = cv2.imdecode(np.fromfile(os.path.join(image_dir, name), np.uint8), flags)
    if img is None:
        raise IOError('cannot read image ' + name)
    return img


def geo_mask(image, camera, up, ground_h, max_off_nadir, max_depth):
    w, h = camera['width'], camera['height']
    u, v = np.meshgrid(np.arange(w, dtype=np.float64) + 0.5, np.arange(h, dtype=np.float64) + 0.5)
    Kinv = np.linalg.inv(camera['K'])
    # ray directions in world space, scaled so that the camera z component is 1 (t == depth)
    rays_cam = np.stack([Kinv[0, 0] * u + Kinv[0, 2], Kinv[1, 1] * v + Kinv[1, 2], np.ones_like(u)], axis=-1)
    up_cam = image['R'] @ up  # up direction expressed in camera coordinates
    rays_up = rays_cam @ up_cam  # component of the ray along the up direction
    ray_norm = np.linalg.norm(rays_cam, axis=-1)
    down_cos = -rays_up / ray_norm
    valid = down_cos > np.cos(np.deg2rad(max_off_nadir))
    if max_depth > 0:
        cam_h = image['C'] @ up - ground_h
        with np.errstate(divide='ignore', invalid='ignore'):
            depth = np.where(rays_up < 0, cam_h / -rays_up, np.inf)
        valid &= depth <= max_depth
    return np.where(valid, 255, 0).astype(np.uint8)


def write_mask(out_dir, image_name, mask):
    stem = os.path.splitext(os.path.basename(image_name))[0]
    path = os.path.join(out_dir, stem + '.mask.png')
    if not cv2.imwrite(path, mask):
        raise IOError('cannot write ' + path)
    return path


def write_preview(preview_dir, img, image_name, mask):
    if img is None or img.shape[:2] != mask.shape:
        return
    overlay = img.copy()
    overlay[mask == 0] = (0.4 * overlay[mask == 0] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
    scale = 1024.0 / max(overlay.shape[:2])
    overlay = cv2.resize(overlay, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    stem = os.path.splitext(os.path.basename(image_name))[0]
    cv2.imencode('.jpg', overlay)[1].tofile(os.path.join(preview_dir, stem + '.preview.jpg'))


def list_images(image_dir):
    if not image_dir or not os.path.isdir(image_dir):
        return []
    return sorted(f for f in os.listdir(image_dir) if f.lower().endswith(IMAGE_EXTS))


def main():
    parser = argparse.ArgumentParser(description='Generate ignore masks (*.mask.png) for DensifyPointCloud')
    parser.add_argument('--mode', choices=['geo', 'fixed', 'corners'], default='geo')
    parser.add_argument('--sparse', help='COLMAP model folder with cameras.txt, images.txt, points3D.txt (required in geo mode, optional otherwise to use the principal point as corner-mask center)')
    parser.add_argument('--images', help='image folder (needed for fixed/corners modes, --mask-black, previews and images missing from the model)')
    parser.add_argument('--out', required=True, help='output mask folder')
    parser.add_argument('--ratio', type=float, default=0.5, help='fraction of the image height masked from the top (fixed mode / fallback)')
    parser.add_argument('--max-off-nadir', type=float, default=60.0, help='mask rays farther than this angle from the vertical, in degrees (geo mode)')
    parser.add_argument('--max-depth', type=float, default=0.0, help='mask pixels whose ground depth (camera z, scene units) exceeds this value, 0 - disabled (geo mode)')
    parser.add_argument('--corner-radius', type=float, default=None, help='mask pixels farther than this fraction of the half-diagonal from the principal point (0 or unset - disabled; default 0.85 in corners mode)')
    parser.add_argument('--mask-black', action='store_true', help='also mask the black invalid border left by undistortion')
    parser.add_argument('--black-threshold', type=int, default=8, help='max intensity considered black for --mask-black')
    parser.add_argument('--black-margin', type=int, default=5, help='extra pixels masked around the black border for --mask-black')
    parser.add_argument('--preview', type=int, default=10, help='number of preview overlays written to <out>/preview (0 - none)')
    args = parser.parse_args()

    if args.corner_radius is None:
        args.corner_radius = 0.85 if args.mode == 'corners' else 0.0
    if args.mode == 'geo' and not args.sparse:
        parser.error('--sparse is required in geo mode')
    if args.mode != 'geo' and not args.images:
        parser.error('--images is required in %s mode' % args.mode)
    if args.mask_black and not args.images:
        parser.error('--images is required by --mask-black')

    os.makedirs(args.out, exist_ok=True)
    preview_dir = os.path.join(args.out, 'preview')
    if args.preview > 0:
        if not args.images:
            print('warning: --images not given, previews disabled')
            args.preview = 0
        else:
            os.makedirs(preview_dir, exist_ok=True)

    # load the COLMAP model if available
    cameras, images = {}, []
    if args.sparse:
        cameras = read_cameras_txt(os.path.join(args.sparse, 'cameras.txt'))
        images = read_images_txt(os.path.join(args.sparse, 'images.txt'))
    model = {os.path.basename(im['name']).lower(): im for im in images}

    up = heights = id2idx = None
    if args.mode == 'geo':
        id2idx, xyz = read_points3d_txt(os.path.join(args.sparse, 'points3D.txt'))
        centers = np.array([im['C'] for im in images])
        up = fit_up_vector(xyz, centers)
        heights = xyz @ up
        global_ground_h = float(np.median(heights))
        flight_h = float(np.median(centers @ up - global_ground_h))
        print('loaded %d images, %d points; up=%s, median flight height above ground %.2f (scene units)'
              % (len(images), len(xyz), np.round(up, 4), flight_h))

    # list of images to process: model images (geo mode) plus images found in the folder
    names = [im['name'] for im in images] if args.mode == 'geo' else []
    seen = set(os.path.basename(n).lower() for n in names)
    names += [n for n in list_images(args.images) if n.lower() not in seen]
    if not names:
        print('error: no images found')
        return 1

    stats = []
    for n, name in enumerate(names):
        image = model.get(os.path.basename(name).lower())
        camera = cameras[image['camera_id']] if image is not None else None
        need_img = args.mask_black or n < args.preview or camera is None
        img = read_image(args.images, name) if need_img and args.images else None
        if camera is not None:
            w, h = camera['width'], camera['height']
            cx, cy = camera['K'][0, 2], camera['K'][1, 2]
        else:
            h, w = img.shape[:2]
            cx = cy = None

        if args.mode == 'geo' and image is not None:
            idxs = [id2idx[i] for i in image['point_ids'] if i in id2idx]
            ground_h = float(np.median(heights[idxs])) if len(idxs) >= 10 else global_ground_h
            mask = geo_mask(image, camera, up, ground_h, args.max_off_nadir, args.max_depth)
        elif args.mode == 'corners':
            mask = np.full((h, w), 255, np.uint8)
        else:
            if args.mode == 'geo':
                print('warning: %s not in the COLMAP model, fixed mask used' % name)
            mask = fixed_mask(w, h, args.ratio)
        if args.corner_radius > 0:
            mask &= corner_mask(w, h, args.corner_radius, cx, cy)
        if args.mask_black and img is not None:
            if img.shape[:2] != (h, w):
                print('warning: %s size %dx%d differs from the camera %dx%d, black mask skipped' % (name, img.shape[1], img.shape[0], w, h))
            else:
                mask &= black_border_mask(img, args.black_threshold, args.black_margin)

        write_mask(args.out, name, mask)
        if n < args.preview:
            write_preview(preview_dir, img, name, mask)
        ratio = 1.0 - np.count_nonzero(mask) / mask.size
        stats.append(ratio)
        print('[%d/%d] %s masked %.1f%%' % (n + 1, len(names), name, ratio * 100), end='\r')
    print()

    stats = np.array(stats)
    print('%d masks written to %s; masked area: mean %.1f%%, min %.1f%%, max %.1f%%; images fully masked: %d'
          % (len(stats), args.out, stats.mean() * 100, stats.min() * 100, stats.max() * 100, int(np.sum(stats >= 0.999))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
