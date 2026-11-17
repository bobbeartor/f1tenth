"""Same three-channel architecture, fine-tuned from an already trained stop-line model."""
import torch
from torch import nn

from fast_scnn_highres import FastSCNNHighRes, require_highres_checkpoint

ARCHITECTURE = 'fast_scnn_highres_stop_v1'
CHANNELS = ('left', 'right', 'stop_line')


class StopLineSCNN(FastSCNNHighRes):
    channel_names = CHANNELS

    def __init__(self, auxiliary=True):
        super().__init__(auxiliary)
        self.decoder_classifier[-1] = nn.Conv2d(32, 3, 1)
        if auxiliary:
            self.auxiliary[-1] = nn.Conv2d(32, 3, 1)

    def initialize_lanes(self, checkpoint):
        require_highres_checkpoint(checkpoint)
        state = self.state_dict()
        old = checkpoint['model']
        if set(state) != set(old):
            raise ValueError('Initial checkpoint model/auxiliary keys differ')
        expanded = {'decoder_classifier.1.weight', 'decoder_classifier.1.bias',
                    'auxiliary.2.weight', 'auxiliary.2.bias'}
        for key in state:
            if key in expanded:
                if old[key].shape[0] != 2 or state[key].shape[0] != 3 or old[key].shape[1:] != state[key].shape[1:]:
                    raise ValueError('Unexpected output shape: ' + key)
                state[key][:2].copy_(old[key])
            else:
                if old[key].shape != state[key].shape:
                    raise ValueError('Unexpected backbone shape: ' + key)
                state[key].copy_(old[key])
        self.load_state_dict(state, strict=True)


def require_checkpoint(saved):
    if saved.get('architecture') != ARCHITECTURE or saved.get('channels') != list(CHANNELS):
        raise ValueError('Expected a stop_line_0910 three-channel checkpoint')
