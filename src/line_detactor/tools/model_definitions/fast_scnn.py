"""Fast-SCNN backbone with two independent BEV lane logits (left, right).

Architecture: https://arxiv.org/abs/1902.04502 (Table 1).
Task adaptations: 2 sigmoid channels instead of softmax, exact-size upsampling,
and GroupNorm in pooled branches to support singleton batches at 1x1 resolution.
"""
from __future__ import annotations

import torch
from torch import nn
from torch.nn import functional as F


class ConvBNReLU(nn.Sequential):
    def __init__(self, inputs, outputs, kernel=3, stride=1, groups=1, dilation=1):
        super().__init__(
            nn.Conv2d(inputs, outputs, kernel, stride, dilation * (kernel // 2),
                      groups=groups, dilation=dilation, bias=False),
            nn.BatchNorm2d(outputs), nn.ReLU(inplace=True))


class DSConv(nn.Sequential):
    def __init__(self, inputs, outputs, stride=1):
        # No intermediate activation between depthwise and pointwise convolution.
        super().__init__(nn.Conv2d(inputs, inputs, 3, stride, 1, groups=inputs, bias=False),
                         nn.BatchNorm2d(inputs), ConvBNReLU(inputs, outputs, kernel=1))


class Bottleneck(nn.Module):
    def __init__(self, inputs, outputs, stride):
        super().__init__()
        hidden = inputs * 6
        self.body = nn.Sequential(ConvBNReLU(inputs, hidden, kernel=1),
                                  ConvBNReLU(hidden, hidden, stride=stride, groups=hidden),
                                  nn.Conv2d(hidden, outputs, 1, bias=False), nn.BatchNorm2d(outputs))
        self.residual = stride == 1 and inputs == outputs

    def forward(self, x):
        out = self.body(x)
        return out + x if self.residual else out


class PyramidPooling(nn.Module):
    def __init__(self, channels=128):
        super().__init__()
        self.bins = (1, 2, 3, 6)
        self.branches = nn.ModuleList([
            nn.Sequential(nn.Conv2d(channels, channels // 4, 1, bias=False),
                          nn.GroupNorm(4, channels // 4), nn.ReLU(inplace=True)) for _ in self.bins])
        self.project = ConvBNReLU(channels * 2, channels, kernel=1)

    def forward(self, x):
        size = x.shape[-2:]
        features = [x]
        for bins, branch in zip(self.bins, self.branches):
            pooled = F.adaptive_avg_pool2d(x, (min(bins, size[0]), min(bins, size[1])))
            features.append(F.interpolate(branch(pooled), size=size, mode="bilinear", align_corners=False))
        return self.project(torch.cat(features, dim=1))


class FeatureFusion(nn.Module):
    def __init__(self):
        super().__init__()
        self.low_depthwise = ConvBNReLU(128, 128, groups=128, dilation=4)
        self.low_project = nn.Sequential(nn.Conv2d(128, 128, 1, bias=False), nn.BatchNorm2d(128))
        self.high_project = nn.Sequential(nn.Conv2d(64, 128, 1, bias=False), nn.BatchNorm2d(128))

    def forward(self, high, low):
        low = F.interpolate(low, size=high.shape[-2:], mode="bilinear", align_corners=False)
        return F.relu(self.high_project(high) + self.low_project(self.low_depthwise(low)), inplace=True)


class FastSCNN(nn.Module):
    """Input Bx3xHxW RGB [0,1]; main/aux outputs Bx2xHxW raw logits."""
    channel_names = ("left", "right")

    def __init__(self, auxiliary=True):
        super().__init__()
        self.downsample = nn.Sequential(ConvBNReLU(3, 32, stride=2), DSConv(32, 48, 2), DSConv(48, 64, 2))
        blocks = []
        inputs = 64
        for outputs, stride in ((64, 2), (96, 2), (128, 1)):
            blocks.extend([Bottleneck(inputs, outputs, stride), Bottleneck(outputs, outputs, 1),
                           Bottleneck(outputs, outputs, 1)])
            inputs = outputs
        self.global_features = nn.Sequential(*blocks, PyramidPooling())
        self.fusion = FeatureFusion()
        self.classifier = nn.Sequential(DSConv(128, 128), DSConv(128, 128),
                                        nn.Dropout2d(0.1), nn.Conv2d(128, 2, 1))
        self.auxiliary = (nn.Sequential(ConvBNReLU(64, 32), nn.Dropout2d(0.1), nn.Conv2d(32, 2, 1))
                          if auxiliary else None)
        for module in self.modules():
            if isinstance(module, nn.Conv2d):
                nn.init.kaiming_normal_(module.weight, mode="fan_out", nonlinearity="relu")
                if module.bias is not None:
                    nn.init.zeros_(module.bias)

    def forward(self, x):
        size = x.shape[-2:]
        high = self.downsample(x)
        fused = self.fusion(high, self.global_features(high))
        result = {"out": F.interpolate(self.classifier(fused), size=size, mode="bilinear", align_corners=False)}
        if self.training and self.auxiliary is not None:
            result["aux"] = F.interpolate(self.auxiliary(high), size=size, mode="bilinear", align_corners=False)
        return result
