import datetime
import unittest

import motion3d as m3d


class TestTime(unittest.TestCase):
    def test_constructur(self):
        # default constructor
        time_default = m3d.Time()
        self.assertEqual(time_default.toNSec(), 0)

        # from seconds as float
        sec1 = float(1.0)
        sec2 = float(-1.0)
        time_sec1 = m3d.Time(sec1)
        time_sec2 = m3d.Time(1.0)
        time_sec3 = m3d.Time.FromSec(1.0)
        time_sec4 = m3d.Time(sec2)
        time_sec5 = m3d.Time.FromSec(-1.0)
        self.assertEqual(time_sec1.toNSec(), 1000000000)
        self.assertEqual(time_sec2.toNSec(), 1000000000)
        self.assertEqual(time_sec3.toNSec(), 1000000000)
        self.assertEqual(time_sec4.toNSec(), 0)
        self.assertEqual(time_sec5.toNSec(), 0)

        # from nanoseconds as integer
        nsec = int(100000)
        time_nsec1 = m3d.Time(nsec)
        time_nsec2 = m3d.Time(100000)
        time_nsec3 = m3d.Time.FromNSec(100000)
        self.assertEqual(time_nsec1.toNSec(), 100000)
        self.assertEqual(time_nsec2.toNSec(), 100000)
        self.assertEqual(time_nsec3.toNSec(), 100000)

        # from datetime
        time_datetime = m3d.Time(datetime.datetime.fromtimestamp(1))
        self.assertEqual(time_datetime.toNSec(), 1000000000)

        # from timestamp, separated into seconds and nanoseconds as uint32
        sep_sec = int(200000)
        sep_nsec = int(100000)
        time_sep1 = m3d.Time(sep_sec, sep_nsec)
        time_sep2 = m3d.Time.FromSecNSec(200000, 100000)
        self.assertEqual(time_sep1.toNSec(), 200000000100000)
        self.assertEqual(time_sep2.toNSec(), 200000000100000)

    def test_conversion(self):
        time = m3d.Time.FromSec(2.0)

        # seconds
        self.assertEqual(time.toSec(), 2.0)

        # nanoseconds
        self.assertEqual(time.toNSec(), 2000000000)

        # separated timestamp
        sep_sec, sep_nsec = time.toSecNSec()
        self.assertEqual(sep_sec, 2)
        self.assertEqual(sep_nsec, 0)

    def test_operators(self):
        time1a = m3d.Time.FromSec(1.0)
        time1b = m3d.Time.FromSec(1.0)
        time2 = m3d.Time.FromSec(2.0)

        # equality
        self.assertTrue(time1a == time1b)
        self.assertFalse(time1a == time2)

        self.assertFalse(time1a != time1b)
        self.assertTrue(time1a != time2)

        # comparison
        self.assertFalse(time1a > time1b)
        self.assertFalse(time1a > time2)
        self.assertTrue(time2 > time1a)

        self.assertFalse(time1a < time1b)
        self.assertTrue(time1a < time2)
        self.assertFalse(time2 < time1a)

        self.assertTrue(time1a >= time1b)
        self.assertFalse(time1a >= time2)
        self.assertTrue(time2 >= time1a)

        self.assertTrue(time1a <= time1b)
        self.assertTrue(time1a <= time2)
        self.assertFalse(time2 <= time1a)

    def test_diff(self):
        time1a = m3d.Time.FromSec(10.0)
        time1b = m3d.Time.FromSec(10.0)
        time2 = m3d.Time.FromSec(12.0)

        self.assertEqual(m3d.timeDiffAbs(time1a, time1b).toNSec(), 0)
        self.assertEqual(m3d.timeDiffAbs(time1a, time2).toNSec(), 2000000000)
        self.assertEqual(m3d.timeDiffAbs(time2, time1a).toNSec(), 2000000000)

    def test_descrption(self):
        t = m3d.Time.FromNSec(1234)
        self.assertEqual(str(t), '1234')


if __name__ == '__main__':
    unittest.main()
