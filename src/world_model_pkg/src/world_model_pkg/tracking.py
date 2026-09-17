"""Deterministic bounded nearest-neighbour tracking in the map frame."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TrackerConfig:
    association_distance_m: float = 2.0
    maximum_coast_sec: float = 0.6
    minimum_confirmation_hits: int = 2
    minimum_velocity_hits: int = 3
    position_alpha: float = 0.65
    size_alpha: float = 0.5
    velocity_alpha: float = 0.4
    maximum_speed_mps: float = 25.0
    maximum_prediction_sec: float = 0.3
    velocity_stddev_mps: float = 2.0

    def __post_init__(self):
        for value, name in (
            (self.association_distance_m, "association distance"),
            (self.maximum_coast_sec, "maximum coast"),
            (self.maximum_speed_mps, "maximum speed"),
            (self.maximum_prediction_sec, "maximum prediction duration"),
            (self.velocity_stddev_mps, "velocity uncertainty"),
        ):
            if not math.isfinite(float(value)) or value <= 0.0:
                raise ValueError(name + " must be finite and positive")
        for value, name in (
            (self.position_alpha, "position alpha"),
            (self.size_alpha, "size alpha"),
            (self.velocity_alpha, "velocity alpha"),
        ):
            if not math.isfinite(float(value)) or not 0.0 < value <= 1.0:
                raise ValueError(name + " must be inside (0, 1]")
        if self.minimum_confirmation_hits < 1 or self.minimum_velocity_hits < 2:
            raise ValueError("track hit thresholds are invalid")


@dataclass(frozen=True)
class Detection:
    center: tuple
    size: tuple
    source_stamp_ns: int
    source_frame_id: str
    source_local_id: int
    timestamp_provenance: str
    calibration_id: str
    calibration_verified: bool
    semantic_class: int = 0
    confidence: float = -1.0

    def __post_init__(self):
        if len(self.center) != 3 or len(self.size) != 3:
            raise ValueError("detection center and size must have three values")
        if not all(math.isfinite(float(value)) for value in self.center + self.size):
            raise ValueError("detection geometry is non-finite")
        if min(self.size) <= 0.0 or self.source_stamp_ns <= 0:
            raise ValueError("detection size and stamp must be positive")
        if not self.source_frame_id or not self.timestamp_provenance or not self.calibration_id:
            raise ValueError("detection provenance is incomplete")


@dataclass(frozen=True)
class TrackView:
    track_id: int
    center: tuple
    size: tuple
    velocity: tuple
    velocity_valid: bool
    source_stamp_ns: int
    source_frame_id: str
    source_local_id: int
    timestamp_provenance: str
    calibration_id: str
    calibration_verified: bool
    semantic_class: int
    confidence: float
    state: int
    observation_count: int
    age_sec: float
    velocity_stddev_mps: float


@dataclass
class _Track:
    track_id: int
    center: tuple
    size: tuple
    velocity: tuple
    velocity_valid: bool
    state_stamp_ns: int
    last_detection_center: tuple
    detection: Detection
    hits: int
    matched_this_frame: bool = True


class MultiObjectTracker:
    TENTATIVE = 0
    CONFIRMED = 1
    COASTING = 2

    def __init__(self, config=None):
        self.config = config if config is not None else TrackerConfig()
        self._tracks = {}
        self._next_id = 1
        self._last_stamp_ns = None
        self._reset_id = None

    def clear(self, reset_id=None):
        self._tracks.clear()
        self._last_stamp_ns = None
        if reset_id is not None:
            self._reset_id = int(reset_id)

    @staticmethod
    def _blend(first, second, alpha):
        return tuple(alpha * float(a) + (1.0 - alpha) * float(b) for a, b in zip(first, second))

    def _predict(self, track, stamp_ns):
        dt = max(0.0, (stamp_ns - track.state_stamp_ns) * 1.0e-9)
        dt = min(dt, self.config.maximum_prediction_sec)
        if not track.velocity_valid:
            return track.center
        return tuple(track.center[index] + track.velocity[index] * dt for index in range(3))

    def _new_track(self, detection, stamp_ns):
        identifier = self._next_id
        self._next_id += 1
        self._tracks[identifier] = _Track(
            track_id=identifier,
            center=detection.center,
            size=detection.size,
            velocity=(0.0, 0.0, 0.0),
            velocity_valid=False,
            state_stamp_ns=stamp_ns,
            last_detection_center=detection.center,
            detection=detection,
            hits=1,
        )

    def update(self, detections, stamp_ns, reset_id):
        stamp_ns = int(stamp_ns)
        reset_id = int(reset_id)
        if stamp_ns <= 0:
            raise ValueError("tracking stamp must be positive")
        if self._reset_id is None:
            self._reset_id = reset_id
        elif reset_id != self._reset_id:
            self.clear(reset_id)
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            self.clear(reset_id)
            raise ValueError("tracking stamp duplicated or regressed")
        values = tuple(detections)
        for value in values:
            if value.source_stamp_ns != stamp_ns:
                raise ValueError("every detection must belong to the fusion stamp")

        predicted = {identifier: self._predict(track, stamp_ns) for identifier, track in self._tracks.items()}
        candidates = []
        for identifier, point in predicted.items():
            for index, detection in enumerate(values):
                distance = math.hypot(point[0] - detection.center[0], point[1] - detection.center[1])
                if distance <= self.config.association_distance_m:
                    candidates.append((distance, identifier, index))
        matched_tracks = set()
        matched_detections = set()
        for _distance, identifier, index in sorted(candidates):
            if identifier in matched_tracks or index in matched_detections:
                continue
            matched_tracks.add(identifier)
            matched_detections.add(index)
            track = self._tracks[identifier]
            detection = values[index]
            dt = (stamp_ns - track.detection.source_stamp_ns) * 1.0e-9
            raw_velocity = (0.0, 0.0, 0.0)
            raw_valid = dt > 1.0e-6
            if raw_valid:
                raw_velocity = tuple(
                    (detection.center[axis] - track.last_detection_center[axis]) / dt
                    for axis in range(3)
                )
                raw_valid = math.hypot(raw_velocity[0], raw_velocity[1]) <= self.config.maximum_speed_mps
            hits = track.hits + 1
            velocity_valid = raw_valid and hits >= self.config.minimum_velocity_hits
            if velocity_valid:
                velocity = (
                    self._blend(raw_velocity, track.velocity, self.config.velocity_alpha)
                    if track.velocity_valid
                    else raw_velocity
                )
            else:
                velocity = track.velocity if track.velocity_valid else (0.0, 0.0, 0.0)
            track.center = self._blend(detection.center, predicted[identifier], self.config.position_alpha)
            track.size = self._blend(detection.size, track.size, self.config.size_alpha)
            track.velocity = velocity
            track.velocity_valid = velocity_valid or track.velocity_valid
            track.state_stamp_ns = stamp_ns
            track.last_detection_center = detection.center
            track.detection = detection
            track.hits = hits
            track.matched_this_frame = True

        for identifier, track in tuple(self._tracks.items()):
            if identifier in matched_tracks:
                continue
            age = (stamp_ns - track.detection.source_stamp_ns) * 1.0e-9
            if age > self.config.maximum_coast_sec:
                del self._tracks[identifier]
                continue
            track.center = predicted[identifier]
            track.state_stamp_ns = stamp_ns
            track.matched_this_frame = False
        for index, detection in enumerate(values):
            if index not in matched_detections:
                self._new_track(detection, stamp_ns)

        self._last_stamp_ns = stamp_ns
        return self.views(stamp_ns)

    def views(self, fusion_stamp_ns):
        output = []
        for identifier in sorted(self._tracks):
            track = self._tracks[identifier]
            age = max(0.0, (fusion_stamp_ns - track.detection.source_stamp_ns) * 1.0e-9)
            if not track.matched_this_frame:
                state = self.COASTING
            elif track.hits >= self.config.minimum_confirmation_hits:
                state = self.CONFIRMED
            else:
                state = self.TENTATIVE
            source = track.detection
            output.append(TrackView(
                track_id=track.track_id,
                center=track.center,
                size=track.size,
                velocity=track.velocity,
                velocity_valid=track.velocity_valid,
                source_stamp_ns=source.source_stamp_ns,
                source_frame_id=source.source_frame_id,
                source_local_id=source.source_local_id,
                timestamp_provenance=source.timestamp_provenance,
                calibration_id=source.calibration_id,
                calibration_verified=source.calibration_verified,
                semantic_class=source.semantic_class,
                confidence=source.confidence,
                state=state,
                observation_count=track.hits,
                age_sec=age,
                velocity_stddev_mps=(self.config.velocity_stddev_mps if track.velocity_valid else -1.0),
            ))
        return tuple(output)
