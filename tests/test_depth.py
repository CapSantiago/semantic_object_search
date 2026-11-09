import unittest

import numpy as np

from semantic_search.depth import foreground_depth, instance_centroid


class DepthTests(unittest.TestCase):
    def test_chair_hole_does_not_project_onto_wall(self):
        image = np.full((100, 100), 3., dtype=np.float32)
        image[15:65, 32:48] = 1.2
        image[15:65, 55:65] = 1.2
        image[45:55, 45:55] = 3.  # Original centre patch sees only the wall.
        image[20, 40] = .1  # A single near outlier must not set the depth.
        u, v, z = foreground_depth(image, 50., 50., 80., 80.)
        self.assertAlmostEqual(z, 1.2, places=5)
        self.assertLess(u, 65)
        self.assertLess(v, 65)

    def test_uint16_millimetres_and_clipped_box(self):
        image = np.full((60, 80), 1500, dtype=np.uint16)
        u, v, z = foreground_depth(image, 2., 20., 40., 40., .001)
        self.assertAlmostEqual(z, 1.5, places=5)
        self.assertGreaterEqual(u, 0)
        self.assertGreaterEqual(v, 0)

    def test_invalid_depth_and_invalid_box_are_rejected(self):
        image = np.full((60, 80), np.nan, dtype=np.float32)
        self.assertIsNone(foreground_depth(image, 40, 30, 20, 20))
        self.assertIsNone(foreground_depth(image, float('nan'), 30, 20, 20))
        self.assertIsNone(foreground_depth(image, 40, 30, 0, 20))
        self.assertIsNone(foreground_depth(image, -100, -100, 20, 20))

    def test_mask_excludes_nearer_occluding_instance(self):
        image = np.full((100, 100), 3., dtype=np.float32)
        image[15:65, 30:50] = .8  # Another instance in the same bounding box.
        image[15:65, 55:70] = 1.6
        polygon = [(55, 15), (69, 15), (69, 64), (55, 64)]
        u, v, z = foreground_depth(image, 50, 50, 80, 80, polygon=polygon)
        self.assertAlmostEqual(z, 1.6, places=5)
        self.assertGreaterEqual(u, 55)
        self.assertIsNone(foreground_depth(image, 50, 50, 80, 80, polygon=[(1, 1)]))

    def test_instance_extent_rejects_far_holes_and_transforms_to_map(self):
        image = np.ones((100, 100), dtype=np.float32)
        image[40:60, 40:60] = 4.0
        polygon = [(20, 20), (80, 20), (80, 80), (20, 80)]
        k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
        point = instance_centroid(image, polygon, k, [1., 2., 0.], [0., 0., 0., 1.])
        np.testing.assert_allclose(point, [1., 2., 1.], atol=.015)


if __name__ == '__main__':
    unittest.main()
