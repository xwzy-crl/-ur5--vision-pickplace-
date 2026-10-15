import importlib.util
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from ur5_vision_lab.vision_math import red_region, depth_metres, median_depth, back_project


class VisionMathTests(unittest.TestCase):
    def test_depth_unit_conversion(self):
        self.assertAlmostEqual(depth_metres(np.array([[960]], dtype=np.uint16), '16UC1')[0, 0], .96)
        self.assertAlmostEqual(depth_metres(np.array([[.96]], dtype=np.float32), '32FC1')[0, 0], .96, places=6)
        with self.assertRaises(ValueError):
            depth_metres(np.ones((2, 2)), 'rgb8')

    def test_invalid_depth_and_outlier(self):
        values = np.array([[.96, .96, .96], [.96, .96, .96], [.96, np.nan, 2.9]])
        self.assertEqual(median_depth(values, np.ones_like(values)), .96)
        with self.assertRaises(ValueError):
            median_depth(np.full((3, 3), np.nan), np.ones((3, 3)))

    def test_pixel_projection(self):
        k = [500, 0, 320, 0, 500, 240, 0, 0, 1]
        np.testing.assert_allclose(back_project(320, 240, .96, k), (0, 0, .96))
        np.testing.assert_allclose(back_project(370, 215, 1, k), (.1, -.05, 1))
        with self.assertRaises(ValueError):
            back_project(1, 1, 1, [0]*9)

    def test_camera_geometry_agrees_with_spawn_and_tf(self):
        # R_y(pi/2) world<-camera_link; REP optical rotation camera_link<-optical.
        world_camera = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        camera_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        world_optical = world_camera @ camera_optical
        np.testing.assert_allclose(world_optical.T @ world_optical, np.eye(3))
        self.assertAlmostEqual(np.linalg.det(world_optical), 1)
        surface = np.array([.45, .30, 1.20]) + world_optical @ np.array([0, 0, .96])
        np.testing.assert_allclose(surface, (.45, .30, .24), atol=1e-10)
        center = surface - np.array([0, 0, .02])
        np.testing.assert_allclose(center, (.45, .30, .22), atol=1e-10)
        shifted = world_optical @ np.array([0, -.1, .96])
        self.assertAlmostEqual(shifted[0], .1)

    @unittest.skipUnless(importlib.util.find_spec('cv2'), 'OpenCV unavailable on this host')
    def test_red_detection_and_nonred_rejection(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[30:50, 40:60] = (0, 0, 255)
        result = red_region(image)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0], (49.5, 39.5))
        image[30:50, 40:60] = (255, 0, 0)
        self.assertIsNone(red_region(image))


if __name__ == '__main__':
    unittest.main()
