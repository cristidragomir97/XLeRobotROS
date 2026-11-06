"""Compressed RGB preserves color order and leaves mapping pixels untouched."""
import unittest
import base64

import cv2
import numpy as np
from xlerobot_sim.camera_streams import encode_rgb_jpeg


class CompressedRGBTests(unittest.TestCase):
    def test_jpeg_interoperability_and_color_order(self):
        rgb = np.zeros((96, 128, 3), dtype=np.uint8)
        rgb[:, :64] = [255, 0, 0]
        rgb[:, 64:] = [0, 0, 255]
        before = rgb.copy()
        encoded = encode_rgb_jpeg(rgb)
        decoded = cv2.cvtColor(cv2.imdecode(np.frombuffer(encoded, np.uint8),
                                          cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        self.assertEqual(decoded.shape, rgb.shape)
        self.assertGreater(decoded[48, 24, 0], 240)
        self.assertGreater(decoded[48, 104, 2], 240)
        self.assertLess(len(encoded), rgb.nbytes / 10)
        np.testing.assert_array_equal(rgb, before)

    def test_dashboard_passes_jpeg_through_without_reencoding(self):
        from sensor_msgs.msg import CompressedImage
        from robocore_viz.serialization import ros2dict
        pixels = np.full((48, 64, 3), [255, 0, 0], dtype=np.uint8)
        encoded = encode_rgb_jpeg(pixels)
        message = CompressedImage()
        message.header.frame_id = 'head_camera_rgb_optical_frame'
        message.format = 'rgb8; jpeg compressed bgr8'
        message.data = encoded
        result = ros2dict(message)
        self.assertEqual(result['_data_jpeg'], base64.b64encode(encoded).decode())
        self.assertEqual(result['data'], [])
        self.assertEqual(result['header']['frame_id'], message.header.frame_id)

    def test_rejects_invalid_format_and_quality(self):
        with self.assertRaises(ValueError):
            encode_rgb_jpeg(np.zeros((8, 8), dtype=np.float32))
        with self.assertRaises(ValueError):
            encode_rgb_jpeg(np.zeros((8, 8, 3), dtype=np.uint8), 101)


if __name__ == '__main__':
    unittest.main()
