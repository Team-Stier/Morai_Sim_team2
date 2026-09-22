"""Translation of overlapping measured surfaces, without changing output points."""
import numpy as np


def translation(previous, current, max_points=96, iterations=6, trim_fraction=0.7):
    old = np.asarray(previous, dtype=float)[:, :2]
    new = np.asarray(current, dtype=float)[:, :2]
    a = old[np.linspace(0, len(old)-1, min(max_points, len(old)), dtype=int)]
    b = new[np.linspace(0, len(new)-1, min(max_points, len(new)), dtype=int)]
    fits = []
    # Zero handles changing visible portions; centroid initialization retains
    # large displacements such as a fast rear vehicle between scans.
    for seed in (np.zeros(2), new.mean(axis=0)-old.mean(axis=0)):
        shift = seed.copy()
        for _ in range(iterations):
            delta = b[None, :, :]-(a[:, None, :]+shift)
            distance = (delta*delta).sum(axis=2)
            indices = distance.argmin(axis=1)
            nearest = distance[np.arange(len(a)), indices]
            residual = delta[np.arange(len(a)), indices]
            shift += np.median(residual[nearest <= np.quantile(nearest, trim_fraction)], axis=0)
        distance = ((a[:, None, :]+shift-b[None, :, :])**2).sum(axis=2)
        score = (np.mean(np.sort(distance.min(axis=1))[:max(1, int(np.ceil(trim_fraction*len(a))))]) +
                 np.mean(np.sort(distance.min(axis=0))[:max(1, int(np.ceil(trim_fraction*len(b))))]))
        fits.append((score, shift))
    # Exactly ambiguous featureless surfaces use the minimum-motion solution.
    # This is an estimation prior, not proof that the object is stationary.
    best_score = min(score for score, _ in fits)
    shift = min((shift for score, shift in fits if score <= best_score+1e-12),
                key=lambda value: float(value@value))
    return float(shift[0]), float(shift[1]), 0.0
