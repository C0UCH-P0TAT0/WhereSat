import unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from core.coordinates import eci_to_body, body_to_eci

class TestCoordinateMath(unittest.TestCase):
    def test_round_trip(self):
        # 1. Generate a random ECI star vector (normalized to length 1)
        raw_vector = np.random.rand(3)
        original_eci = raw_vector / np.linalg.norm(raw_vector)
            
        # 2. Generate a random satellite attitude (quaternion)
        random_quat = R.random().as_quat()
            
        # 3. Push it through the pipeline
        body_vector = eci_to_body(original_eci, random_quat)
        recovered_eci = body_to_eci(body_vector, random_quat)
            
        # 4. Prove the math is flawless
        # np.allclose handles the microscopic floating-point rounding errors
        np.testing.assert_allclose(original_eci, recovered_eci, rtol=1e-5, atol=1e-8)
        print("Round-trip math verified.")

if __name__ == '__main__':
        unittest.main()