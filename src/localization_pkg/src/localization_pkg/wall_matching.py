"""Two-sided wall-normal matching. Never infer position along a straight tunnel."""
from dataclasses import dataclass
import math
import numpy as np
from .live_estimator import rotation


@dataclass
class WallMatchConfig:
    enabled: bool = True
    max_points: int = 2500
    min_height_m: float = 1.8
    max_height_m: float = 4.0
    max_range_m: float = 40.0
    association_distance_m: float = 1.5
    min_points_per_wall: int = 35
    min_span_m: float = 8.0
    max_heading_error_rad: float = 0.0872664626
    max_width_error_m: float = 0.70
    max_residual_m: float = 0.40
    max_correction_m: float = 1.2
    min_wall_length_m: float = 20.0
    innovation_gate_chi2: float = 16.0

    def __post_init__(self):
        for key, value in vars(self).items():
            if key != 'enabled' and (not math.isfinite(value) or value <= 0):
                raise ValueError('invalid wall matching parameter: '+key)
        if self.max_height_m <= self.min_height_m or self.max_points < 2*self.min_points_per_wall:
            raise ValueError('invalid wall matching bounds')
        if any(not isinstance(getattr(self, k), int) for k in ('max_points', 'min_points_per_wall')):
            raise ValueError('wall sample counts must be integers')


class WallMatcher:
    def __init__(self, lines, stddev, config=None):
        self.config = config or WallMatchConfig()
        self.lines = [np.asarray(line, dtype=float) for line in lines]
        self.stddev = float(stddev)
        if len(self.lines) != 2 or not math.isfinite(self.stddev) or self.stddev <= 0:
            raise ValueError('tunnel matching requires exactly two walls and positive uncertainty')
        for line in self.lines:
            if line.ndim != 2 or line.shape[1] != 3 or len(line) < 2 or not np.isfinite(line).all():
                raise ValueError('invalid wall baseline')
        all_xy = np.concatenate(self.lines)[:, :2]
        _, axes = np.linalg.eigh(np.cov(all_xy.T))
        self.normal = axes[:, 0]
        self.tangent = axes[:, 1]
        for line in self.lines:
            if np.ptp(line[:, :2] @ self.tangent) < self.config.min_wall_length_m:
                raise ValueError('wall too short')
        starts, ends, groups = [], [], []
        for group, line in enumerate(self.lines):
            starts.extend(line[:-1, :2]); ends.extend(line[1:, :2]); groups.extend([group]*(len(line)-1))
        self.starts, self.ends = np.array(starts), np.array(ends)
        self.groups = np.array(groups)
        self.directions = self.ends-self.starts
        self.lengths = np.linalg.norm(self.directions, axis=1)
        if np.any(self.lengths < 1e-4):
            raise ValueError('degenerate wall segment')
        normals = np.column_stack((-self.directions[:, 1], self.directions[:, 0]))/self.lengths[:, None]
        normals *= np.where(normals @ self.normal < 0, -1., 1.)[:, None]
        if np.any(normals @ self.normal < 0.95):
            raise ValueError('wall pair is not a supported straight corridor')
        self.normals = normals

    def match(self, points_lidar, orientation, position, translation):
        c = self.config
        points = np.asarray(points_lidar, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('invalid cloud dimensions')
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) > c.max_points:
            points = points[np.linspace(0, len(points)-1, c.max_points, dtype=int)]
        level = (points+np.asarray(translation)) @ rotation(orientation).T
        mask = ((level[:, 2] >= c.min_height_m) & (level[:, 2] <= c.max_height_m) &
                (np.linalg.norm(level[:, :2], axis=1) <= c.max_range_m))
        xy = level[mask, :2] + np.asarray(position)[:2]
        if len(xy) < 2*c.min_points_per_wall:
            raise ValueError('too few elevated wall returns')
        sides = [float((np.median(line[:, :2], axis=0)-np.asarray(position)[:2]) @ self.normal) for line in self.lines]
        if sides[0]*sides[1] >= 0:
            raise ValueError('predicted pose is not between both walls')
        offsets = xy[:, None, :]-self.starts[None, :, :]
        fractions = np.einsum('ijk,jk->ij', offsets, self.directions)/(self.lengths**2)
        distances = np.einsum('ijk,jk->ij', offsets, self.normals)
        score = np.where((fractions >= 0) & (fractions <= 1), abs(distances), np.inf)
        nearest = score.argmin(axis=1)
        keep = score[np.arange(len(xy)), nearest] <= c.association_distance_m
        xy, nearest = xy[keep], nearest[keep]
        residuals = -np.sum((xy-self.starts[nearest])*self.normals[nearest], axis=1)/(self.normals[nearest] @ self.normal)
        corrections, spreads, counts = [], [], []
        for group in range(2):
            use = self.groups[nearest] == group
            support, residual = xy[use], residuals[use]
            if len(support) < c.min_points_per_wall:
                raise ValueError('both wall sides need enough returns')
            median = float(np.median(residual))
            good = abs(residual-median) <= c.max_residual_m
            support, residual = support[good], residual[good]
            if len(support) < c.min_points_per_wall:
                raise ValueError('wall residual support rejected')
            span = np.quantile(support @ self.tangent, [.05, .95])
            if span[1]-span[0] < c.min_span_m:
                raise ValueError('wall support span too short')
            _, basis = np.linalg.eigh(np.cov(support.T))
            angle = math.acos(float(np.clip(abs(basis[:, 1] @ self.tangent), 0, 1)))
            if angle > c.max_heading_error_rad:
                raise ValueError('wall heading inconsistent with IMU/map')
            corrections.append(float(np.median(residual)))
            spreads.append(float(np.sqrt(np.mean((residual-corrections[-1])**2))))
            counts.append(len(support))
        if abs(corrections[0]-corrections[1]) > c.max_width_error_m:
            raise ValueError('two-wall width mismatch')
        correction = float(np.mean(corrections))
        if abs(correction) > c.max_correction_m or max(spreads) > c.max_residual_m:
            raise ValueError('wall correction or residual too large')
        # Equal weight per wall: dense returns on one wall must not dominate.
        variance = self.stddev**2 + max(spreads)**2 + (0.5*(corrections[0]-corrections[1]))**2
        return dict(normal=self.normal.copy(), residual=correction, variance=variance,
                    support=counts, rms=max(spreads))
