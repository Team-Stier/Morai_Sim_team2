"""Read emitted lamp colors inside a detected 3/4-lamp housing, without map IDs."""
import cv2
import numpy as np


def classify_layout_hypotheses(bgr, bbox, config):
    """Observe both bulb layouts; World Model selects by mapped head geometry.

    YOLO's housing class is diagnostic only. Hypotheses share one source box,
    and must not count as multiple temporal confirmations or independent heads.
    """
    hypotheses = []
    for layout in (3, 4):
        mask, valid, reason = classify_lamps(bgr, bbox, layout, config)
        hypotheses.append(dict(layout=layout, lamp_mask=mask, valid=valid, reason=reason))
    return hypotheses


def classify_lamps(bgr, bbox, layout, config):
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = map(float, bbox)
    margin = config['clipping_margin_px']
    if (layout not in (3, 4) or x2-x1 < layout*2 or y2-y1 < 3 or
            x1 <= margin or y1 <= margin or x2 >= w-margin or y2 >= h-margin):
        return 0, False, 'clipped_or_too_small'
    crop = bgr[int(y1):int(np.ceil(y2)), int(x1):int(np.ceil(x2))]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue, sat, value = cv2.split(hsv)
    lit = (sat >= config['minimum_saturation']) & (value >= config['minimum_value'])
    colors = {
        1: lit & ((hue <= config['red_hue_max']) | (hue >= config['red_hue_min_wrap'])),
        2: lit & (hue >= config['yellow_hue_min']) & (hue <= config['yellow_hue_max']),
        4: lit & (hue >= config['green_hue_min']) & (hue <= config['green_hue_max']),
    }
    slots = [(0, 1), (1, 2), (2, 4)] if layout == 3 else [(0, 1), (1, 2), (2, 8), (3, 4)]
    mask = 0
    for slot, bit in slots:
        left, right = int(crop.shape[1]*slot/layout), int(crop.shape[1]*(slot+1)/layout)
        pixels = colors[4 if bit == 8 else bit][:, left:right]
        # Require an actual connected illuminated patch; scattered JPEG noise is insufficient.
        count, _, stats, _ = cv2.connectedComponentsWithStats(pixels.astype(np.uint8), 8)
        area = max(stats[1:, cv2.CC_STAT_AREA], default=0) if count > 1 else 0
        if area >= max(config['minimum_pixels'], pixels.size*config['minimum_slot_fraction']):
            mask |= bit
    if mask & 1 and mask & 4 or mask & 2 and mask & (4 | 8):
        return 0, False, 'conflicting_lamps'
    return mask, bool(mask), 'hsv_lamp_slots' if mask else 'unlit_or_unresolved'
