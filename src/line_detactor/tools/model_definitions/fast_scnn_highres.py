"""Fast-SCNN with learned 1/4 and 1/2 resolution skip decoding.

The baseline module is reused without modification. No pretrained weights are loaded.
"""
import torch
from torch import nn
from torch.nn import functional as F

from fast_scnn import ConvBNReLU, DSConv, FastSCNN


ARCHITECTURE = "fast_scnn_highres_v1"


def require_highres_checkpoint(checkpoint):
    if checkpoint.get("architecture") != ARCHITECTURE:
        raise ValueError("Expected fast_scnn_highres_v1 weights. Baseline weights cannot resume this model.")


class FastSCNNHighRes(FastSCNN):
    """RGB Bx3xHxW -> independent left/right logits Bx2xHxW."""
    def __init__(self, auxiliary=True):
        super().__init__(auxiliary=auxiliary)
        # Keep the encoder, fusion and auxiliary supervision identical to baseline.
        # Replace (do not retain) its 1/8 classifier with two learned skip stages.
        del self.classifier
        self.decoder_project = ConvBNReLU(128, 64, kernel=1)
        self.decoder_quarter = nn.Sequential(DSConv(64 + 48, 64), DSConv(64, 64))
        self.decoder_half = nn.Sequential(DSConv(64 + 32, 32), DSConv(32, 32))
        self.decoder_classifier = nn.Sequential(nn.Dropout2d(.1), nn.Conv2d(32, 2, 1))
        for block in (self.decoder_project, self.decoder_quarter, self.decoder_half, self.decoder_classifier):
            for module in block.modules():
                if isinstance(module, nn.Conv2d):
                    nn.init.kaiming_normal_(module.weight, mode="fan_out", nonlinearity="relu")
                    if module.bias is not None:
                        nn.init.zeros_(module.bias)

    def forward(self, x):
        size = x.shape[-2:]
        half = self.downsample[0](x)         # 150x60, 32 channels
        quarter = self.downsample[1](half)  # 75x30, 48 channels
        eighth = self.downsample[2](quarter)  # 38x15, 64 channels
        fused = self.fusion(eighth, self.global_features(eighth))
        decoded = F.interpolate(self.decoder_project(fused), size=quarter.shape[-2:],
                                mode="bilinear", align_corners=False)
        decoded = self.decoder_quarter(torch.cat((decoded, quarter), dim=1))
        decoded = F.interpolate(decoded, size=half.shape[-2:], mode="bilinear", align_corners=False)
        decoded = self.decoder_half(torch.cat((decoded, half), dim=1))
        result = {"out": F.interpolate(self.decoder_classifier(decoded), size=size,
                                       mode="bilinear", align_corners=False)}
        if self.training and self.auxiliary is not None:
            result["aux"] = F.interpolate(self.auxiliary(eighth), size=size, mode="bilinear", align_corners=False)
        return result
