"""Standard JPEG RGB transport for display consumers; metric sensors stay raw."""
import cv2
import numpy as np


def encode_rgb_jpeg(rgb, quality=75):
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('JPEG input must be an RGB uint8 image')
    if not 1 <= quality <= 100:
        raise ValueError('JPEG quality must be between 1 and 100')
    success, encoded = cv2.imencode('.jpg', rgb[:, :, ::-1],
                                    [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not success:
        raise RuntimeError('JPEG encoding failed')
    return encoded.tobytes()
