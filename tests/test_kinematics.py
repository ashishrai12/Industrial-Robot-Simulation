import unittest
import numpy as np
from src.robot_simulation.kinematics import PlanarKinematics2D

class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.l1 = 1.0
        self.l2 = 1.0
        self.kin = PlanarKinematics2D(self.l1, self.l2)

    def test_forward_kinematics_origin(self):
        # theta1=0, theta2=0 -> should be at (2, 0)
        pos = self.kin.forward(0, 0)
        np.testing.assert_allclose(pos, [2.0, 0.0], atol=1e-7)

    def test_forward_kinematics_90_90(self):
        # theta1=90, theta2=90 -> should be at (-1, 1)
        pos = self.kin.forward(np.pi/2, np.pi/2)
        np.testing.assert_allclose(pos, [-1.0, 1.0], atol=1e-7)

    def test_inverse_kinematics(self):
        # Test if IK(FK(theta)) returns something consistent
        t1, t2 = 0.5, 0.8
        pos = self.kin.forward(t1, t2)
        solutions = self.kin.inverse(pos[0], pos[1])
        self.assertIsNotNone(solutions)
        
        # There might be two solutions (elbow up/down), check if our solution reaches the target
        pos_recalc = self.kin.forward(solutions[0], solutions[1])
        np.testing.assert_allclose(pos, pos_recalc, atol=1e-7)

    def test_out_of_reach(self):
        # (5, 5) is definitely out of reach for links 1+1=2
        solutions = self.kin.inverse(5, 5)
        self.assertIsNone(solutions)

if __name__ == '__main__':
    unittest.main()
