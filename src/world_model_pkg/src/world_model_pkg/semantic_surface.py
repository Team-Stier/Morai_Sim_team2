"""Bound semantic foreground to the nearest object surface in each image region.

The bounds are development priors, not measured object boxes or free-space evidence.
"""
import cv2
import numpy as np
from .pointpainting import project_points


def surface_mask(xyz, scores, transform, intrinsic, classes, config):
    camera, uv, valid = project_points(xyz, scores.shape[:2], transform, intrinsic)
    selected = np.zeros(len(xyz), dtype=bool)
    if not config['enabled']:
        return valid
    support = int(config['minimum_support_points'])
    gap = float(config['depth_gap_m'])
    if support < 1 or not np.isfinite(gap) or gap <= 0:
        raise ValueError('invalid nearest-surface support/depth gap')
    labels = scores.argmax(2).astype(np.uint8)
    indices = np.flatnonzero(valid)
    pixels = np.floor(uv[valid]).astype(np.int64)
    sampled = labels[pixels[:, 1], pixels[:, 0]]
    for label, name in enumerate(classes):
        if name not in config['class_extents_m']:
            continue
        limits = np.asarray(config['class_extents_m'][name], dtype=np.float64)
        if limits.shape != (3,) or not np.isfinite(limits).all() or (limits <= 0).any():
            raise ValueError('invalid class surface extent: ' + name)
        # Use the whole argmax silhouette for connectivity, including lower-score
        # pixels; the independent 0.7 foreground gate still applies to output.
        _, regions = cv2.connectedComponents((labels == label).astype(np.uint8), connectivity=8)
        members = indices[sampled == label]
        member_pixels = pixels[sampled == label]
        region_ids = regions[member_pixels[:, 1], member_pixels[:, 0]]
        for region in np.unique(region_ids):
            points = members[region_ids == region]
            order = points[np.argsort(camera[points, 2], kind='stable')]
            depths = camera[order, 2]
            # Never skip an unsupported front layer to select a dense rear wall.
            breaks = np.flatnonzero(np.diff(depths) > gap)
            nearest = order[:breaks[0] + 1] if len(breaks) else order
            if len(nearest) < support:
                continue
            front_depth = camera[nearest[0], 2]
            near = nearest[camera[nearest, 2] <= front_depth + limits[2]]
            if len(near) < support:
                continue
            center = np.median(camera[near, :2], axis=0)
            keep = (np.abs(camera[near, :2] - center) <= limits[:2] / 2).all(1)
            if np.count_nonzero(keep) >= support:
                selected[near[keep]] = True
    return selected
