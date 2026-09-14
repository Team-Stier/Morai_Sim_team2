"""Paired, ground-truth-free diagnostics; these are not driving quality gates."""
import math
import numpy as np


def plane(points):
    center = points.mean(axis=0)
    _, values, vectors = np.linalg.svd(points-center, full_matrices=False)
    if len(values)<3 or values[1]<1e-6:
        raise ValueError('degenerate ground patch')
    normal = vectors[-1]
    if normal[2]<0:
        normal = -normal
    return normal, center


def validate_config(c):
    lo, hi = np.asarray(c['roi_min']), np.asarray(c['roi_max'])
    if lo.shape!=(3,) or hi.shape!=(3,) or not np.all(np.isfinite([lo,hi])) or not np.all(lo<hi):
        raise ValueError('invalid metric ROI')
    for key in ('max_points','ransac_iterations','min_inliers'):
        if not isinstance(c[key],int) or c[key]<3:
            raise ValueError('invalid '+key)
    if c['min_inliers']>c['max_points']:
        raise ValueError('min_inliers exceeds max_points')
    for key in ('distance_threshold_m','min_x_span_m','min_y_span_m'):
        if not math.isfinite(c[key]) or c[key]<=0:
            raise ValueError('invalid '+key)
    if not 0<c['min_inlier_ratio']<=1 or not 0<c['max_plane_tilt_deg']<90:
        raise ValueError('invalid plane acceptance gate')


def ground_candidates(raw, config):
    """Select a patch without consulting the correction or corrected cloud."""
    validate_config(config)
    raw = np.asarray(raw,dtype=float)
    if raw.ndim!=2 or raw.shape[1]!=3:
        raise ValueError('expected Nx3 points')
    raw = raw[np.all(np.isfinite(raw),axis=1)]
    mask = np.all((raw>=config['roi_min']) & (raw<=config['roi_max']),axis=1)
    points = raw[mask]
    rng = np.random.RandomState(config['seed'])
    if len(points)>config['max_points']:
        points = points[np.sort(rng.choice(len(points),config['max_points'],replace=False))]
    if len(points)<config['min_inliers']:
        raise ValueError('insufficient raw ground candidates')
    threshold = config['distance_threshold_m']
    max_tilt_cos = math.cos(math.radians(config['max_plane_tilt_deg']))
    best = np.zeros(len(points),dtype=bool)
    for _ in range(config['ransac_iterations']):
        a,b,c = points[rng.choice(len(points),3,replace=False)]
        normal = np.cross(b-a,c-a)
        norm = np.linalg.norm(normal)
        if norm<1e-9 or abs(normal[2])/norm<max_tilt_cos:
            continue
        normal /= norm
        candidate = np.abs((points-a).dot(normal))<=threshold
        if candidate.sum()>best.sum():
            best = candidate
    if best.sum()<config['min_inliers']:
        raise ValueError('no supported ground plane')
    for _ in range(2):
        normal,center = plane(points[best])
        best = np.abs((points-center).dot(normal))<=threshold
        if best.sum()<config['min_inliers']:
            raise ValueError('insufficient refined ground support')
    patch = points[best]
    normal,_ = plane(patch)
    ratio = len(patch)/len(points)
    spans = np.ptp(patch,axis=0)
    if ratio<config['min_inlier_ratio'] or normal[2]<max_tilt_cos:
        raise ValueError('ambiguous or steep ground candidate')
    if spans[0]<config['min_x_span_m'] or spans[1]<config['min_y_span_m']:
        raise ValueError('ground patch has insufficient spatial extent')
    return patch, {'candidate_count':len(points),'inlier_count':len(patch),
                   'inlier_ratio':ratio,'x_span_m':float(spans[0]),'y_span_m':float(spans[1])}


def plane_metrics(points):
    normal,center = plane(points)
    residual = (points-center).dot(normal)
    return {'tilt_deg':math.degrees(math.atan2(np.linalg.norm(normal[:2]),abs(normal[2]))),
            'forward_slope_deg':math.degrees(math.atan2(-normal[0],normal[2])),
            'lateral_slope_deg':math.degrees(math.atan2(-normal[1],normal[2])),
            'horizontal_height_rmse_m':float(np.std(points[:,2])),
            'orthogonal_plane_rmse_m':float(np.sqrt(np.mean(residual**2)))}


def paired_metrics(raw,rotation,config):
    r = np.asarray(rotation,dtype=float)
    if r.shape!=(3,3) or not np.all(np.isfinite(r)) or not np.allclose(r.T@r,np.eye(3),atol=1e-8,rtol=0) or not np.isclose(np.linalg.det(r),1.,atol=1e-8,rtol=0):
        raise ValueError('audit matrix is not a proper rotation')
    patch,support = ground_candidates(raw,config)
    # Use exactly the same returns, with the matrix recorded by the C++ detector.
    after = patch@r.T
    before_metrics,after_metrics = plane_metrics(patch),plane_metrics(after)
    distance_error = np.linalg.norm(after,axis=1)-np.linalg.norm(patch,axis=1)
    metrics = dict(support)
    metrics.update({'before_'+k:v for k,v in before_metrics.items()})
    metrics.update({'after_'+k:v for k,v in after_metrics.items()})
    metrics['tilt_change_deg'] = after_metrics['tilt_deg']-before_metrics['tilt_deg']
    metrics['range_preservation_rmse_m'] = float(np.sqrt(np.mean(distance_error**2)))
    metrics['round_trip_rmse_m'] = float(np.sqrt(np.mean(np.sum((after@r-patch)**2,axis=1))))
    return metrics,patch,after


def distribution(values):
    values = np.asarray(values,dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return dict(zip(('median','p95','min','max'),map(float,(np.median(values),np.percentile(values,95),values.min(),values.max()))))
