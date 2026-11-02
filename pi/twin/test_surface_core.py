"""Regression tests: preserve real vertical faces, reject curtains and stale objects."""
import unittest
import numpy as np
from surface_core import DepthSmoother, surface_indices, valid_triangles, rays


class SurfaceTests(unittest.TestCase):
    def test_vertical_face_is_preserved(self):
        p=np.array([[[0,0,0],[.01,0,0]],[[0,0,.01],[.01,0,.01]]],np.float32)
        self.assertEqual(len(valid_triangles(p,np.ones((2,2),bool),surface_indices((2,2)))),6)

    def test_depth_discontinuity_is_not_bridged(self):
        p=np.array([[[0,0,0],[.01,0,0]],[[0,0,.50],[.01,0,.50]]],np.float32)
        self.assertEqual(len(valid_triangles(p,np.ones((2,2),bool),surface_indices((2,2)))),0)

    def test_missing_data_is_not_filled(self):
        p=np.zeros((2,2,3),np.float32)
        self.assertEqual(len(valid_triangles(p,np.array([[1,0],[0,1]],bool),surface_indices((2,2)))),0)

    def test_drop_is_immediate_and_jitter_is_smoothed(self):
        s=DepthSmoother(); s.update(np.array([500,500,500],np.uint16))
        out=s.update(np.array([0,900,504],np.uint16))
        np.testing.assert_allclose(out,[0,900,502.6],atol=.001)

    def test_optical_center_ray(self):
        r=rays(dict(width=3,height=3,ppx=1,ppy=1,fx=2,fy=2))
        np.testing.assert_equal(r[1,1],[0,0,1])


if __name__=='__main__': unittest.main()
