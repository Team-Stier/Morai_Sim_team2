import math
import sys
import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
from world_model_pkg.transforms import transform_aabb


class TransformAabbTest(unittest.TestCase):
    def test_translation_and_quarter_turn_make_map_aligned_box(self):
        half = math.sqrt(0.5)
        result = transform_aabb(
            center=(5.0, 2.0, 0.5),
            size=(4.0, 2.0, 1.0),
            translation=(10.0, 20.0, 1.5),
            quaternion=(0.0, 0.0, half, half),
        )
        self.assertAlmostEqual(result.center[0], 8.0)
        self.assertAlmostEqual(result.center[1], 25.0)
        self.assertAlmostEqual(result.center[2], 2.0)
        self.assertAlmostEqual(result.size[0], 2.0)
        self.assertAlmostEqual(result.size[1], 4.0)
        self.assertAlmostEqual(result.size[2], 1.0)

    def test_rejects_bad_geometry_and_transform(self):
        with self.assertRaises(ValueError):
            transform_aabb((0, 0, 0), (1, -1, 1), (0, 0, 0), (0, 0, 0, 1))
        with self.assertRaises(ValueError):
            transform_aabb((0, 0, 0), (1, 1, 1), (0, 0, 0), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
