"""Surface-independent color evidence from original component pixels only."""
import cv2
import numpy as np


def red_pixels(bgr: np.ndarray, cfg) -> np.ndarray:
    """Keep hue bands fixed; accept dim/desaturated red only with RGB support."""
    hsv = cv2.cvtColor(bgr.reshape(-1,1,3),cv2.COLOR_BGR2HSV)[:,0]
    hue = (hsv[:,0] <= cfg.red_hue_low_max) | (hsv[:,0] >= cfg.red_hue_high_min)
    strict = hue & (hsv[:,1] >= cfg.red_min_saturation) & (hsv[:,2] >= cfg.red_min_value)
    if not cfg.red_shadow_enabled:
        return strict
    channels = bgr.reshape(-1,3).astype(np.float32)
    excess = (channels[:,2]-np.maximum(channels[:,0],channels[:,1])) / np.maximum(1,channels.sum(axis=1))
    supported = (hue & (hsv[:,1] >= cfg.red_shadow_min_saturation) &
                 (hsv[:,2] >= cfg.red_shadow_min_value) & (excess >= cfg.red_shadow_min_normalized_excess))
    return strict | supported


def component_red_evidence(frame_crop, component, red_crop, cfg, *, diagnostics=False) -> dict:
    """Full-mask ratio is always reported. Interior may only rescue a red majority.

    Erosion selects color samples, never changes the detection mask or geometry.
    Insufficient interior samples fall back to all exact component pixels.
    """
    total = int(np.count_nonzero(component))
    if total == 0:
        raise ValueError('Color statistics require a nonempty component mask')
    count = int(np.count_nonzero(red_crop & component))
    ratio = count/total
    interior_total, interior_count = 0,0
    interior_ratio = None
    if cfg.red_interior_erosion_pixels:
        size = cfg.red_interior_erosion_pixels*2+1
        interior = cv2.erode(component.astype(np.uint8),np.ones((size,size),np.uint8),
                             borderType=cv2.BORDER_CONSTANT,borderValue=0) != 0
        interior_total = int(np.count_nonzero(interior))
        interior_count = int(np.count_nonzero(red_crop & interior))
        if (interior_total >= cfg.red_interior_min_pixels and
                interior_total >= cfg.red_interior_min_fraction*total):
            interior_ratio = interior_count/interior_total
    full_pass = ratio >= cfg.red_fraction_threshold
    interior_pass = (interior_ratio is not None and ratio >= cfg.red_interior_min_full_ratio and
                     interior_ratio >= cfg.red_fraction_threshold)
    basis = 'full_component' if full_pass else ('supported_component_interior' if interior_pass else 'insufficient_red_evidence')
    reason = (f'red component pixels={count}/{total}, ratio={ratio:.3f}, threshold={cfg.red_fraction_threshold:.3f}; '
              f'interior ratio={"unavailable" if interior_ratio is None else f"{interior_ratio:.3f}"}; basis={basis}')
    info = {'red_pixel_count': count, 'total_component_pixel_count': total, 'red_pixel_ratio': ratio,
            'red_classification_threshold': cfg.red_fraction_threshold,
            'interior_pixel_count': interior_total, 'interior_red_pixel_count': interior_count,
            'interior_red_pixel_ratio': interior_ratio,
            'interior_min_full_ratio': cfg.red_interior_min_full_ratio,
            'classification_basis': basis, 'is_red': bool(full_pass or interior_pass), 'reason': reason}
    if diagnostics:
        pixels = frame_crop[component]
        hsv = cv2.cvtColor(pixels.reshape(-1,1,3),cv2.COLOR_BGR2HSV)[:,0]
        hue = (hsv[:,0] <= cfg.red_hue_low_max) | (hsv[:,0] >= cfg.red_hue_high_min)
        strict_count = int(np.count_nonzero(hue & (hsv[:,1] >= cfg.red_min_saturation) & (hsv[:,2] >= cfg.red_min_value)))
        info.update(mean_hsv=[float(v) for v in hsv.mean(axis=0)],
                    median_hsv=[float(v) for v in np.median(hsv,axis=0)],
                    hue_in_red_range_pixel_count=int(np.count_nonzero(hue)),
                    strict_red_pixel_count=strict_count, supplementary_red_pixel_count=count-strict_count,
                    thresholds={name:getattr(cfg,name) for name in (
                        'red_hue_low_max','red_hue_high_min','red_min_saturation','red_min_value',
                        'red_shadow_enabled','red_shadow_min_saturation','red_shadow_min_value',
                        'red_shadow_min_normalized_excess','red_interior_erosion_pixels',
                        'red_interior_min_pixels','red_interior_min_fraction','red_interior_min_full_ratio')})
    return info
