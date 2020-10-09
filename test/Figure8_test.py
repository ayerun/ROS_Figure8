#!/usr/bin/env python3

""" Unit testing Figure8 """
import unittest
import homework2.Figure8 as f8
import numpy as np

class MyTestCase(unittest.TestCase):
    '''
    Test all trajectory and turtle command functions in homework2 python pacakge
    All calculations are done at t=0 and t=2
    All calculations assume W=1, H=2, T=3
    '''
    def test_xd(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(tclass.find_xd(t1), 0)
        self.assertEquals(round(tclass.find_xd(t2),3), round(-np.sqrt(3)/4,3))

    def test_yd(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(tclass.find_yd(t1), 0)
        self.assertEquals(round(tclass.find_yd(t2),3), round(np.sqrt(3)/2,3))

    def test_xd_dot(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(round(tclass.find_xd_dot(t1),3), round(np.pi/3,3))
        self.assertEquals(round(tclass.find_xd_dot(t2),3), round(-np.pi/6,3))

    def test_yd_dot(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(round(tclass.find_yd_dot(t1),3), round(4*np.pi/3,3))
        self.assertEquals(round(tclass.find_yd_dot(t2),3), round(-2*np.pi/3,3))

    def test_xd_ddot(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(round(tclass.find_xd_ddot(t1),3), 0)
        self.assertEquals(round(tclass.find_xd_ddot(t2),3), round((np.pi**2*np.sqrt(3))/9,3))

    def test_yd_ddot(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(round(tclass.find_yd_ddot(t1),3), 0)
        self.assertEquals(round(tclass.find_yd_ddot(t2),3), round((-8*np.pi**2*np.sqrt(3))/9,3))

    def test_vd(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(round(tclass.find_vd(t1),3), 4.318)
        self.assertEquals(round(tclass.find_vd(t2),3), 2.159)

    def test_wd(self):
        tclass = f8.figure8([1,2,3,4])
        t1 = 0
        t2 = 2
        self.assertEquals(round(tclass.find_wd(t1),3), 0)
        self.assertEquals(round(tclass.find_wd(t2),3), 2.561)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(homework2, "Figure_8_Tests", MyTestCase)