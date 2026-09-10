"""Synthetic fixtures only; not connected to any live ROS graph."""
from types import SimpleNamespace as NS


def stamp(value=10):
    return NS(secs=value, nsecs=0, sec=value, nanosec=0)


def header(frame='', value=10):
    return NS(seq=0, stamp=stamp(value), frame_id=frame)


def vector(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def component():
    return NS(header=header(), component='path_planning_pkg', state=2,
              ready=True, stop_required=False, data_stamp=stamp(9),
              data_age_sec=1., processing_latency_sec=0.01, processed_count=7,
              dropped_count=0, invalid_count=0, reason='synthetic fixture')


def ego():
    covariance = [0.] * 36
    for i in (0, 7, 35):
        covariance[i] = 0.5
    return NS(header=header('map'), child_frame_id='base_link',
              pose=NS(pose=NS(position=vector(1., 2.),
                              orientation=NS(x=0., y=0., z=0., w=1.)),
                      covariance=list(covariance)),
              twist=NS(twist=NS(linear=vector(-2.), angular=vector(z=0.1)),
                       covariance=list(covariance)),
              pose_valid=[True, True, False, False, False, True],
              twist_valid=[True, False, False, False, False, True], reset_id=2)


def localization():
    return NS(header=header(value=11), mode=3, gps_fix_valid=False,
              map_pose_valid=True, local_odometry_valid=True, stop_required=False,
              ego_state_stamp=stamp(), local_odometry_stamp=stamp(), reset_id=2,
              gps_age_sec=4., map_position_stddev_m=1.,
              local_position_stddev_m=0.5, yaw_stddev_rad=0.1, reason='blackout fixture')
