import unittest

import motion3d as m3d


class TestEulerAxes(unittest.TestCase):
    def test_euler_axes_from_str(self):
        self.assertEqual(m3d.EulerAxes.FromStr("SXYZ"), m3d.EulerAxes.kSXYZ)
        self.assertEqual(m3d.EulerAxes.FromStr("SXYX"), m3d.EulerAxes.kSXYX)
        self.assertEqual(m3d.EulerAxes.FromStr("SXZY"), m3d.EulerAxes.kSXZY)
        self.assertEqual(m3d.EulerAxes.FromStr("SXZX"), m3d.EulerAxes.kSXZX)
        self.assertEqual(m3d.EulerAxes.FromStr("SYZX"), m3d.EulerAxes.kSYZX)
        self.assertEqual(m3d.EulerAxes.FromStr("SYZY"), m3d.EulerAxes.kSYZY)
        self.assertEqual(m3d.EulerAxes.FromStr("SYXZ"), m3d.EulerAxes.kSYXZ)
        self.assertEqual(m3d.EulerAxes.FromStr("SYXY"), m3d.EulerAxes.kSYXY)
        self.assertEqual(m3d.EulerAxes.FromStr("SZXY"), m3d.EulerAxes.kSZXY)
        self.assertEqual(m3d.EulerAxes.FromStr("SZXZ"), m3d.EulerAxes.kSZXZ)
        self.assertEqual(m3d.EulerAxes.FromStr("SZYX"), m3d.EulerAxes.kSZYX)
        self.assertEqual(m3d.EulerAxes.FromStr("SZYZ"), m3d.EulerAxes.kSZYZ)
        self.assertEqual(m3d.EulerAxes.FromStr("RZYX"), m3d.EulerAxes.kRZYX)
        self.assertEqual(m3d.EulerAxes.FromStr("RXYX"), m3d.EulerAxes.kRXYX)
        self.assertEqual(m3d.EulerAxes.FromStr("RYZX"), m3d.EulerAxes.kRYZX)
        self.assertEqual(m3d.EulerAxes.FromStr("RXZX"), m3d.EulerAxes.kRXZX)
        self.assertEqual(m3d.EulerAxes.FromStr("RXZY"), m3d.EulerAxes.kRXZY)
        self.assertEqual(m3d.EulerAxes.FromStr("RYZY"), m3d.EulerAxes.kRYZY)
        self.assertEqual(m3d.EulerAxes.FromStr("RZXY"), m3d.EulerAxes.kRZXY)
        self.assertEqual(m3d.EulerAxes.FromStr("RYXY"), m3d.EulerAxes.kRYXY)
        self.assertEqual(m3d.EulerAxes.FromStr("RYXZ"), m3d.EulerAxes.kRYXZ)
        self.assertEqual(m3d.EulerAxes.FromStr("RZXZ"), m3d.EulerAxes.kRZXZ)
        self.assertEqual(m3d.EulerAxes.FromStr("RXYZ"), m3d.EulerAxes.kRXYZ)
        self.assertEqual(m3d.EulerAxes.FromStr("RZYZ"), m3d.EulerAxes.kRZYZ)
        self.assertRaises(m3d.InvalidEulerAxesException, lambda: m3d.EulerAxes.FromStr("invalid"))

    def test_euler_axes_to_str(self):
        self.assertEqual(m3d.EulerAxes.kSXYZ.toStr(), "SXYZ")
        self.assertEqual(m3d.EulerAxes.kSXYX.toStr(), "SXYX")
        self.assertEqual(m3d.EulerAxes.kSXZY.toStr(), "SXZY")
        self.assertEqual(m3d.EulerAxes.kSXZX.toStr(), "SXZX")
        self.assertEqual(m3d.EulerAxes.kSYZX.toStr(), "SYZX")
        self.assertEqual(m3d.EulerAxes.kSYZY.toStr(), "SYZY")
        self.assertEqual(m3d.EulerAxes.kSYXZ.toStr(), "SYXZ")
        self.assertEqual(m3d.EulerAxes.kSYXY.toStr(), "SYXY")
        self.assertEqual(m3d.EulerAxes.kSZXY.toStr(), "SZXY")
        self.assertEqual(m3d.EulerAxes.kSZXZ.toStr(), "SZXZ")
        self.assertEqual(m3d.EulerAxes.kSZYX.toStr(), "SZYX")
        self.assertEqual(m3d.EulerAxes.kSZYZ.toStr(), "SZYZ")
        self.assertEqual(m3d.EulerAxes.kRZYX.toStr(), "RZYX")
        self.assertEqual(m3d.EulerAxes.kRXYX.toStr(), "RXYX")
        self.assertEqual(m3d.EulerAxes.kRYZX.toStr(), "RYZX")
        self.assertEqual(m3d.EulerAxes.kRXZX.toStr(), "RXZX")
        self.assertEqual(m3d.EulerAxes.kRXZY.toStr(), "RXZY")
        self.assertEqual(m3d.EulerAxes.kRYZY.toStr(), "RYZY")
        self.assertEqual(m3d.EulerAxes.kRZXY.toStr(), "RZXY")
        self.assertEqual(m3d.EulerAxes.kRYXY.toStr(), "RYXY")
        self.assertEqual(m3d.EulerAxes.kRYXZ.toStr(), "RYXZ")
        self.assertEqual(m3d.EulerAxes.kRZXZ.toStr(), "RZXZ")
        self.assertEqual(m3d.EulerAxes.kRXYZ.toStr(), "RXYZ")
        self.assertEqual(m3d.EulerAxes.kRZYZ.toStr(), "RZYZ")


if __name__ == '__main__':
    unittest.main()
