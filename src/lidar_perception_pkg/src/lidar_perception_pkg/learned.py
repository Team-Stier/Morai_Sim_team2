"""Single-scan PointPillars adapter. No GT, tracking, TF or motion compensation."""
import hashlib
from pathlib import Path

import numpy as np
from common_msgs_pkg.lidar_validation import MODEL_CLASSES


def read_xyzi(message):
    """Read row-padded FLOAT32 XYZI, rejecting malformed layouts before numpy."""
    if (message.is_bigendian or message.point_step <= 0 or
            message.row_step < message.width * message.point_step or
            len(message.data) != message.row_step * message.height):
        raise ValueError('invalid PointCloud2 layout')
    offsets = []
    for name in ('x', 'y', 'z', 'intensity'):
        fields = [f for f in message.fields if f.name == name]
        if (len(fields) != 1 or fields[0].datatype != 7 or fields[0].count != 1 or
                fields[0].offset < 0 or fields[0].offset + 4 > message.point_step):
            raise ValueError('missing/invalid FLOAT32 ' + name)
        offsets.append(fields[0].offset)
    if len(set(offsets)) != 4 or any(abs(a-b) < 4 for i,a in enumerate(offsets) for b in offsets[i+1:]):
        raise ValueError('overlapping XYZI fields')
    if message.width * message.height == 0:
        raise ValueError('empty raw scan is not evidence of free space')
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity'],
                      'formats': ['<f4'] * 4, 'offsets': offsets,
                      'itemsize': message.point_step})
    view = np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=message.data, strides=(message.row_step, message.point_step))
    points = np.column_stack([view[n].reshape(-1) for n in dtype.names])
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('no finite XYZI points')
    if np.any(points[:, 3] < 0) or np.any(points[:, 3] > 255):
        raise ValueError('expected Velodyne intensity in [0,255]')
    return points


def roi_points(points, bounds):
    return points[np.all((points[:, :3] >= bounds[:3]) & (points[:, :3] <= bounds[3:]), axis=1)]


def observations_from_predictions(points, boxes, scores, labels, classes, bounds, threshold):
    """Return supported detections overlapping the user's ROI, as enclosing AABBs."""
    from common_msgs_pkg.msg import LidarObjectObservation
    if len(boxes) != len(scores) or len(scores) != len(labels):
        raise ValueError('model output lengths disagree')
    result = []
    for box, score, label in zip(boxes, scores, labels):
        if (len(box) < 7 or not np.isfinite(box).all() or
                not np.isfinite(score) or not 0 <= score <= 1 or
                label != int(label) or not 1 <= label <= len(classes) or np.any(box[3:6] <= 0)):
            raise ValueError('invalid model prediction')
        if score < threshold:
            continue
        name = classes[int(label)-1]
        if name not in MODEL_CLASSES:
            raise ValueError('unsupported native model class: ' + name)
        c, s = np.cos(box[6]), np.sin(box[6])
        extent = np.array([abs(c)*box[3]+abs(s)*box[4],
                           abs(s)*box[3]+abs(c)*box[4], box[5]])
        if np.any(box[:3]+extent/2 < bounds[:3]) or np.any(box[:3]-extent/2 > bounds[3:]):
            continue
        delta = points[:, :3] - box[:3]
        local = np.column_stack((c*delta[:,0]+s*delta[:,1],
                                 -s*delta[:,0]+c*delta[:,1], delta[:,2]))
        count = int(np.count_nonzero(np.all(np.abs(local) <= box[3:6]/2, axis=1)))
        if count == 0:
            continue
        obj = LidarObjectObservation(scan_local_id=len(result), point_count=count,
                                     confidence=-1, semantic_class=MODEL_CLASSES[name],
                                     learned_box=True, model_class=name, model_score=float(score))
        obj.center.x, obj.center.y, obj.center.z = map(float, box[:3])
        obj.size.x, obj.size.y, obj.size.z = map(float, extent)
        result.append(obj)
    return result


class PointPillars:
    def __init__(self, repository, checkpoint, checkpoint_sha256):
        import sys
        import logging
        import yaml
        import torch
        from easydict import EasyDict
        self.torch = torch
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA GPU unavailable')
        checkpoint = Path(checkpoint).expanduser().resolve()
        if len(checkpoint_sha256) != 64 or hashlib.sha256(checkpoint.read_bytes()).hexdigest() != checkpoint_sha256:
            raise ValueError('checkpoint SHA256 mismatch')
        root = Path(repository).expanduser().resolve()
        sys.path.insert(0, str(root))
        from pcdet.config import merge_new_config
        from pcdet.datasets import DatasetTemplate
        from pcdet.models import build_network, load_data_to_gpu
        self.load_data_to_gpu = load_data_to_gpu
        raw = yaml.safe_load((root/'tools/cfgs/nuscenes_models/cbgs_pp_multihead.yaml').read_text())
        raw['DATA_CONFIG']['_BASE_CONFIG_'] = str(root/'tools'/raw['DATA_CONFIG']['_BASE_CONFIG_'])
        cfg = merge_new_config(EasyDict(), raw)
        self.classes = cfg.CLASS_NAMES
        # Avoid stochastic test-time shuffling. No historical sweeps are fabricated.
        for processor in cfg.DATA_CONFIG.DATA_PROCESSOR:
            if processor.NAME == 'shuffle_points':
                processor.SHUFFLE_ENABLED['test'] = False
        self.range = np.asarray(cfg.DATA_CONFIG.POINT_CLOUD_RANGE)
        self.dataset = DatasetTemplate(cfg.DATA_CONFIG, self.classes, training=False,
                                       root_path=root, logger=logging.getLogger('pointpillars'))
        torch.set_num_threads(2)
        self.model = build_network(cfg.MODEL, len(self.classes), self.dataset)
        # Official, hash-checked checkpoint only. Strict load prevents partial/random heads.
        state = torch.load(str(checkpoint), map_location='cpu', weights_only=False)['model_state']
        self.model.load_state_dict(state, strict=True)
        self.model.cuda().eval()

    def infer(self, points):
        points = roi_points(points, self.range)
        if not len(points):
            return np.empty((0,7)), np.empty(0), np.empty(0, dtype=np.int64)
        # nuScenes uses raw intensity; 5th feature is sweep age (zero for current scan).
        features = np.column_stack((points, np.zeros(len(points), dtype=np.float32)))
        data = self.dataset.prepare_data({'points': features, 'frame_id': 'live_scan'})
        batch = self.dataset.collate_batch([data])
        self.load_data_to_gpu(batch)
        with self.torch.inference_mode():
            predictions, _ = self.model(batch)
        prediction = predictions[0]
        return tuple(prediction[k].detach().cpu().numpy() for k in ('pred_boxes','pred_scores','pred_labels'))
