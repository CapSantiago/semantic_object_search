"""Robust foreground sampling for aligned RGB bounding boxes and depth images."""
from math import isfinite

import numpy as np
import cv2
from scipy.spatial.transform import Rotation


def truncated_box(center_x, center_y, width, height, image_width, image_height, margin=2.):
    """A clipped instance is insufficient for reliable map-object confirmation."""
    if not all(isfinite(v) for v in (center_x,center_y,width,height,image_width,image_height)):
        return True
    return (width<=0 or height<=0 or center_x-width/2<=margin or center_y-height/2<=margin
            or center_x+width/2>=image_width-margin or center_y+height/2>=image_height-margin)


def instance_centroid(image, polygon, k, translation, quaternion, scale=1.0):
    geometry = instance_geometry(image, polygon, k, translation, quaternion, scale)
    return geometry[0] if geometry is not None else None


def compatible_size(category, extent):
    """Conservative bounds for the supported small cups/bottles, in metres.

    These are class-size priors, not fixture coordinates or simulator truth.
    The horizontal AABB diagonal uses conservative room for heading variation;
    it remains a visible-surface estimate, not the object's exact dimensions.
    """
    limits = {'cup': (.25, .30), 'bottle': (.35, .60)}
    if category not in limits:
        return True
    if len(extent)!=3 or not all(isfinite(v) and v>=0 for v in extent):
        return False
    horizontal, height = limits[category]
    return float(np.hypot(extent[0],extent[1]))<=horizontal and extent[2]<=height


def instance_geometry(image, polygon, k, translation, quaternion, scale=1.0):
    """Estimate a map-frame centre from the visible instance's 3-D extent.

    Unlike a single surface sample, this uses both seat/back and side surfaces.
    The near-depth window rejects walls behind mask holes; map-height filtering
    rejects floor pixels. Trimmed bounds reduce isolated segmentation outliers.
    The estimate is still limited by partial occlusion and visible geometry.
    """
    polygon = np.asarray(polygon, dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.isfinite(polygon).all():
        return None
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    if fx <= 0 or fy <= 0:
        return None
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 1)
    depth = image.astype(np.float32) * scale
    v, u = np.nonzero(mask & np.isfinite(depth) & (depth > .05) & (depth < 8.))
    if len(u) < 20:
        return None
    z = depth[v, u]
    near = np.quantile(z, .1)
    keep = (z >= near-.1) & (z <= near+.7)
    u, v, z = u[keep], v[keep], z[keep]
    camera_points = np.column_stack(((u-cx)*z/fx, (v-cy)*z/fy, z))
    points = Rotation.from_quat(quaternion).apply(camera_points) + translation
    points = points[(points[:, 2] > .08) & (points[:, 2] < 2.)]
    if len(points) < 20:
        return None
    bounds=np.quantile(points, [.1, .9], axis=0)
    return (tuple(float(value) for value in bounds.mean(axis=0)),
            tuple(float(value) for value in bounds[1]-bounds[0]))


def foreground_depth(image, center_x, center_y, width, height, scale=1.0, polygon=None):
    """Return a foreground pixel and depth in metres, or None.

    Use the upper/central box to avoid floor between chair legs. A robust near
    quantile and a narrow depth band reject background visible through holes.
    Pixel coordinates come from that band, so the depth and projection ray agree.
    This assumes the target is the foreground within its central box; heavy
    overlapping occluders still need instance masks or a stronger depth model.
    """
    if not all(isfinite(v) for v in (center_x, center_y, width, height)):
        return None
    if width <= 0 or height <= 0:
        return None
    rows, columns = image.shape[:2]
    left = max(0, int(center_x - .25*width))
    right = min(columns, int(center_x + .25*width) + 1)
    top = max(0, int(center_y - .35*height))
    bottom = min(rows, int(center_y + .15*height) + 1)
    if left >= right or top >= bottom:
        return None
    patch = image[top:bottom, left:right].astype(np.float32) * scale
    valid = np.isfinite(patch) & (patch > .05) & (patch < 8.)
    if polygon is not None:
        polygon = np.asarray(polygon, dtype=np.float64)
        if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.isfinite(polygon).all():
            return None
        mask = np.zeros(patch.shape, dtype=np.uint8)
        local = np.rint(polygon - [left, top]).astype(np.int32)
        cv2.fillPoly(mask, [local], 1)
        valid &= mask.astype(bool)
    if np.count_nonzero(valid) < 5:
        return None
    near = np.quantile(patch[valid], .25)
    foreground = valid & (np.abs(patch - near) <= .12)
    if np.count_nonzero(foreground) < 5:
        return None
    v, u = np.nonzero(foreground)
    return (float(np.median(u) + left), float(np.median(v) + top),
            float(np.median(patch[foreground])))
