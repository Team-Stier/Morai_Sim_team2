"""Course cruise targets, curvature speed and cyclic braking envelope in SI units."""
import math


def build_speed_profile(points, zones, normal_margin_kph, lateral_accel, deceleration, preview_m):
    count = len(points)
    distances = [math.hypot(points[(i+1) % count][0]-p[0],
                            points[(i+1) % count][1]-p[1]) for i, p in enumerate(points)]
    speeds = []
    for i, p in enumerate(points):
        before, after = points[(i-10) % count], points[(i+10) % count]
        ab = math.hypot(p[0]-before[0], p[1]-before[1])
        bc = math.hypot(after[0]-p[0], after[1]-p[1])
        ac = math.hypot(after[0]-before[0], after[1]-before[1])
        cross = abs((p[0]-before[0])*(after[1]-p[1]) - (p[1]-before[1])*(after[0]-p[0]))
        curvature = 2*cross/(ab*bc*ac)
        cruise = (zones.policy['high_speed']['cruise_kph'] if zones.unlimited(i)
                  else zones.policy['normal_limit_kph']-normal_margin_kph)/3.6
        curve_speed = math.sqrt(lateral_accel/curvature) if curvature > 0 else cruise
        speeds.append(min(cruise, curve_speed))
    # Two backward laps propagate braking through the route seam as well.
    for k in range(2*count-1, -1, -1):
        i = k % count
        speeds[i] = min(speeds[i], math.sqrt(speeds[(i+1) % count]**2 + 2*deceleration*distances[i]))
    result = []
    for i in range(count):
        j, distance = i, 0.0
        while distance < preview_m:
            distance += distances[j]
            j = (j+1) % count
        # Preview braking only: never accelerate before entering the unlimited zone.
        result.append(min(speeds[i], speeds[j]))
    return result
