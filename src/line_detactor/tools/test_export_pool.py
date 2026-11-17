"""Pure PyTorch export-lowering checks; no ROS or CUDA build required."""
import unittest
import torch
from torch.nn import functional as F
from export_stop_line_onnx import StaticAdaptivePool


class PoolExportTests(unittest.TestCase):
    def test_all_real_pyramid_bins_match_native_adaptive_pool(self):
        torch.manual_seed(5)
        x=torch.randn(1,128,10,4)
        for size in [(1,1),(2,2),(3,3),(6,4)]:
            with self.subTest(size=size):
                torch.testing.assert_close(StaticAdaptivePool(10,4,*size)(x),F.adaptive_avg_pool2d(x,size),rtol=2e-6,atol=2e-6)

    def test_nonuniform_bins_preserve_edge_impulses(self):
        for y in range(10):
            x=torch.zeros(1,1,10,4);x[0,0,y,3]=1
            torch.testing.assert_close(StaticAdaptivePool(10,4,6,4)(x),F.adaptive_avg_pool2d(x,(6,4)),rtol=0,atol=1e-7)


if __name__=='__main__':unittest.main()
