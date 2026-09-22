#!/usr/bin/env python3
"""Compare the current estimator with an explicitly supplied historical source.

Only load a trusted repository revision: importing the baseline executes Python.
This is an offline synthetic evaluator, not a ROS publisher or a runtime input.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

from test_blackout_drift import BIAS, GpsImuEstimator, drive, stationary, tunnel_stop


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-file', required=True)
    parser.add_argument('--baseline-commit', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location('baseline_drift', args.baseline_file)
    baseline = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = baseline
    spec.loader.exec_module(baseline)
    report = dict(baseline_commit=args.baseline_commit,
                  evidence='synthetic, not MORAI runtime', imu_rate_hz=50,
                  gps_rate_hz=5, gps_training_sec=20, blackout_sec=15,
                  injected_body_bias_mps2=BIAS.tolist(), scenarios={})
    scenarios = [('stationary', stationary, 20), ('drive_brake_stop', tunnel_stop, 22),
                 ('cruise', lambda t: ([4*t, 0., 0.], [0., 0., 0.]), 20)]
    for name, trajectory, reference_time in scenarios:
        case = dict(reference_elapsed_sec=reference_time)
        for label, cls in [('before', baseline.GpsImuEstimator), ('after', GpsImuEstimator)]:
            core = cls()
            records = drive(core, 35., trajectory, 20.)
            case[label] = dict(
                final_position_error_m=float(np.linalg.norm(core.state[:3]-trajectory(35.)[0])),
                map_displacement_from_reference_time_m=float(np.linalg.norm(
                    records[35]['map_position']-records[reference_time]['map_position'])),
                final_velocity_mps=core.state[3:6].tolist(),
                final_horizontal_stddev_m=records[35]['map_position_stddev'])
        report['scenarios'][name] = case
    output = json.dumps(report, indent=2)+'\n'
    Path(args.output).write_text(output)
    print(output, end='')


if __name__ == '__main__':
    main()
