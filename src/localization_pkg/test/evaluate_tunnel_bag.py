#!/usr/bin/env python3
"""Offline, read-only sensor replay. Truth is used ONLY after estimation.

No ROS master, publisher, simulator connection or fitted trajectory alignment.
The optional truth JSONL is an explicitly authorized development evaluation
artifact, never a localization observation or a competition runtime dependency.
"""
import argparse
import bisect
import importlib.util
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import rosbag
import yaml


def load_core(path):
    spec = importlib.util.spec_from_file_location('replay_core', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def replay(bag_path, core_path, config_path, overrides=None):
    module = load_core(core_path)
    values = yaml.safe_load(Path(config_path).read_text())
    values.update(overrides or {})
    fields = vars(module.EstimatorConfig())
    core = module.GpsImuEstimator(module.EstimatorConfig(
        **{k: v for k, v in values.items() if k in fields}))
    contract_root = Path(__file__).resolve().parents[2]/'ros_architecture_pkg/config'
    projection = yaml.safe_load((contract_root/'tf/map_projection.yaml').read_text())
    timing = yaml.safe_load((contract_root/'timestamp/timestamp_contract.yaml').read_text())[
        'development_localization_profile']
    core.config.max_integration_step_sec = timing['max_integration_step_sec']
    projector = module.MapProjector(projection['epsg'], projection['origin_utm_m'])
    imu, gps = [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, m, _ in bag.read_messages(topics=[
                '/molit/sensors/imu/data', '/molit/sensors/gps/fix']):
            t = m.header.stamp.to_sec()
            if topic.endswith('/imu/data'):
                q, a, w = m.orientation, m.linear_acceleration, m.angular_velocity
                imu.append(module.ImuObservation(t, [q.x, q.y, q.z, q.w],
                    [a.x, a.y, a.z], [w.x, w.y, w.z],
                    m.orientation_covariance, m.angular_velocity_covariance))
            elif m.status.status >= 0:
                try:
                    position = projector.project(m.latitude, m.longitude, m.altitude)
                    gps.append(module.GpsObservation(t, position,
                        None if m.position_covariance_type == 0 else m.position_covariance))
                except ValueError:
                    # MORAI may emit zero coordinates while blacked out. Match
                    # the runtime zone/finiteness guard, not merely fix status.
                    continue
    stamps = [m.stamp for m in imu]
    outputs, epoch = [], 0
    for t, kind, observation in sorted([(m.stamp, 0, m) for m in imu] +
                                        [(m.stamp, 1, m) for m in gps], key=lambda x: x[:2]):
        if kind == 0:
            if core.last_imu_stamp is not None and t-core.last_imu_stamp > timing['imu_timeout_sec']:
                core.reset()
                epoch += 1
            if core.process_imu(observation):
                outputs.append(dict(stamp=t, position=core.state[:3].tolist(),
                    velocity=core.state[3:6].tolist(), bias=core.state[6:].tolist(),
                    gps_age=t-core.last_gps_stamp, epoch=epoch))
        else:
            right = bisect.bisect_left(stamps, t)
            if right == 0 or right == len(stamps):
                continue
            a, b = imu[right-1], imu[right]
            if b.stamp-a.stamp > timing['max_integration_step_sec']:
                continue
            f = (t-a.stamp)/(b.stamp-a.stamp)
            def blend(x, y):
                return np.asarray(x)*(1-f)+np.asarray(y)*f
            sample = module.ImuObservation(t,
                module.slerp(a.orientation_xyzw, b.orientation_xyzw, f),
                blend(a.acceleration_mps2, b.acceleration_mps2),
                blend(a.angular_velocity_radps, b.angular_velocity_radps),
                blend(a.orientation_covariance, b.orientation_covariance),
                blend(a.angular_velocity_covariance, b.angular_velocity_covariance))
            core.process_gps(observation, sample)
            if core.gps_reinitialized:
                epoch += 1
    return outputs


def evaluate(outputs, truth_path):
    truth = {}
    yaw = {}
    for line in Path(truth_path).read_text().splitlines():
        d = json.loads(line)
        # Identical repeated physics-frame stamps are not independent samples.
        if d['stamp'] in truth and truth[d['stamp']] != d['xyz']:
            raise ValueError('conflicting truth at the same stamp')
        truth[d['stamp']] = d['xyz']
        if 'yaw_deg' in d:
            yaw[d['stamp']] = np.deg2rad(d['yaw_deg'])
    stamps = sorted(truth)
    records = []
    for out in outputs:
        t = out['stamp']
        j = bisect.bisect_left(stamps, t)
        if not 0 < j < len(stamps):
            continue
        a, b = stamps[j-1:j+1]
        if b-a > .15:
            continue
        p = np.array(truth[a])+(np.array(truth[b])-truth[a])*(t-a)/(b-a)
        error = np.array(out['position'])[:2]-p[:2]
        record = dict(out, error_xy_m=float(np.linalg.norm(error)))
        if a in yaw and b in yaw:
            angle_delta = np.arctan2(np.sin(yaw[b]-yaw[a]), np.cos(yaw[b]-yaw[a]))
            angle = yaw[a]+angle_delta*(t-a)/(b-a)
            record['error_longitudinal_m'] = float(error.dot([np.cos(angle), np.sin(angle)]))
            record['error_lateral_m'] = float(error.dot([-np.sin(angle), np.cos(angle)]))
        records.append(record)
    def metrics(rows):
        e = np.array([r['error_xy_m'] for r in rows])
        if not len(e):
            return None
        result = dict(samples=len(e), rmse_m=float(np.sqrt(np.mean(e*e))),
                      max_m=float(e.max()), p95_m=float(np.percentile(e, 95)))
        for axis in ('longitudinal', 'lateral'):
            values = [r['error_'+axis+'_m'] for r in rows if 'error_'+axis+'_m' in r]
            if len(values) == len(rows):
                result[axis+'_rmse_m'] = float(np.sqrt(np.mean(np.square(values))))
                result[axis+'_max_abs_m'] = float(np.max(np.abs(values)))
        return result
    return dict(evidence='single recorded development run; not general accuracy',
        truth_never_used_as_filter_input=True, all=metrics(records),
        blackout=metrics([r for r in records if r['gps_age'] > .65]), records=records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--truth', required=True)
    parser.add_argument('--core', required=True, help='Trusted source file; imported as Python')
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = evaluate(replay(args.bag, args.core, args.config), args.truth)
    report['input_sha256'] = {key: hashlib.sha256(Path(value).read_bytes()).hexdigest()
        for key, value in vars(args).items() if key != 'output'}
    Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}, indent=2))


if __name__ == '__main__':
    main()
