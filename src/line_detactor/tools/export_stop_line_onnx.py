"""Offline PT -> static FP32 ONNX export + CPU numerical checks; never trains.

Run after training. The deployed ROS node uses TensorRT/CUDA only, not this script.
"""
import argparse
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np
import onnx
import onnxruntime as ort
import torch
from torch import nn
from torch.nn import functional as F

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'model_definitions'))
from stop_line_model import StopLineSCNN, require_checkpoint, CHANNELS


class StaticAdaptivePool(nn.Module):
    """Exact adaptive bin boundaries for the fixed 10x4 global feature map.

    Uniform bins use one ordinary AvgPool. Nonuniform bins use separable means,
    avoiding unsupported nondivisible adaptive_avg_pool2d in legacy ONNX export.
    """
    def __init__(self, height, width, output_height, output_width):
        super().__init__()
        self.height, self.width = height, width
        self.oh, self.ow = output_height, output_width
        self.ys = [(i * height // output_height, math.ceil((i + 1) * height / output_height)) for i in range(output_height)]
        self.xs = [(i * width // output_width, math.ceil((i + 1) * width / output_width)) for i in range(output_width)]
        def uniform(bounds):
            lengths = {b-a for a,b in bounds}
            steps = {bounds[i][0]-bounds[i-1][0] for i in range(1,len(bounds))}
            return len(lengths)==1 and len(steps)<=1
        self.uniform = uniform(self.ys) and uniform(self.xs)
        self.kernel = (self.ys[0][1], self.xs[0][1])
        self.stride = (self.ys[1][0] if output_height>1 else height,
                       self.xs[1][0] if output_width>1 else width)

    def forward(self, x):
        if self.uniform:
            return F.avg_pool2d(x, self.kernel, self.stride)
        if self.oh != self.height:
            x = torch.cat([x[:,:,a:b,:].mean(dim=2,keepdim=True) for a,b in self.ys],dim=2)
        if self.ow != self.width:
            x = torch.cat([x[:,:,:,a:b].mean(dim=3,keepdim=True) for a,b in self.xs],dim=3)
        return x


class StaticPyramid(nn.Module):
    def __init__(self, source, height=10, width=4):
        super().__init__()
        self.branches, self.project = source.branches, source.project
        self.size = (height,width)
        self.pools = nn.ModuleList([StaticAdaptivePool(height,width,min(n,height),min(n,width)) for n in source.bins])

    def forward(self, x):
        return self.project(torch.cat([x]+[F.interpolate(branch(pool(x)),size=self.size,mode='bilinear',align_corners=False)
            for pool,branch in zip(self.pools,self.branches)],dim=1))


class LogitsOnly(nn.Module):
    def __init__(self, model):
        super().__init__()
        self.model = model

    def forward(self, rgb):
        return self.model(rgb)['out']


def validation_inputs(saved, data_root, image_folder=None):
    # Never open the final-test manifest or sample final-test frames here.
    rng = np.random.default_rng(42)
    values = [('zero', np.zeros((1,3,300,120),np.float32)),
              ('random', rng.random((1,3,300,120),dtype=np.float32))]
    files=[]
    if image_folder:
        files=sorted(p for p in image_folder.glob('*') if p.suffix.lower() in ('.png','.jpg','.jpeg') and not p.name.startswith('._'))[:8]
        if not files:raise ValueError('No validation images in '+str(image_folder))
    elif 'prepared' in saved:
        manifest=Path(saved['prepared'])/'val.json'
        if manifest.is_file():
            rows=json.loads(manifest.read_text(encoding='utf-8'))['samples']
            selected=[r for r in rows if r.get('source')=='dataset_0910'][:4]+[r for r in rows if r.get('source')!='dataset_0910'][:4]
            files=[data_root/r['image'] for r in selected]
    for path in files:
        image=cv2.imdecode(np.fromfile(path,np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.shape!=(300,120,3):raise ValueError('Expected W120 H300 validation image: '+str(path))
        array=np.ascontiguousarray(image[...,::-1].transpose(2,0,1),dtype=np.float32)[None]/255.
        values.append((str(path),array))
    return values


@torch.inference_mode()
def main():
    package = HERE.parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path,help='Default: latest stop_line_0910/runs/*/best.pt in sibling line_detector')
    parser.add_argument('--output',type=Path,default=package/'models/fast_scnn_stop_line_120x300_batch_1.onnx')
    parser.add_argument('--data-root',type=Path,default=package.parent/'line_detector')
    parser.add_argument('--validation-images',type=Path,help='Optional W120 H300 validation folder; do not use final test')
    args=parser.parse_args()
    candidates=sorted((args.data_root/'training/highres/stop_line_0910/runs').glob('*/best.pt'))
    source=args.checkpoint or (candidates[-1] if candidates else None)
    if source is None:parser.error('No trained checkpoint found; supply --checkpoint after training')
    # Read a coherent snapshot if a checkpoint path is atomically replaced elsewhere.
    checkpoint_bytes=source.read_bytes()
    saved=torch.load(io.BytesIO(checkpoint_bytes),map_location='cpu',weights_only=True)
    require_checkpoint(saved)
    torch.set_num_threads(2);cv2.setNumThreads(0)
    model=StopLineSCNN(auxiliary=saved['config']['aux_weight']>0).eval()
    model.load_state_dict(saved['model'],strict=True)
    reference=LogitsOnly(model).eval()
    lowered=copy.deepcopy(model)
    lowered.global_features[-1]=StaticPyramid(lowered.global_features[-1])
    exported=LogitsOnly(lowered).eval()
    inputs=validation_inputs(saved,args.data_root,args.validation_images)
    output=args.output.resolve();output.parent.mkdir(parents=True,exist_ok=True)
    print('Exporting checkpoint epoch {}: {}'.format(saved['epoch'],source),flush=True)
    # Run on CPU so export/verification does not compete with ongoing GPU training.
    with tempfile.TemporaryDirectory(prefix='onnx_export_',dir=output.parent) as temporary:
        path=Path(temporary)/'model.onnx'
        torch.onnx.export(exported,torch.zeros(1,3,300,120),str(path),input_names=['rgb'],output_names=['logits'],
            opset_version=17,dynamo=False,export_params=True,do_constant_folding=True,external_data=False)
        graph=onnx.load(str(path));onnx.checker.check_model(graph,full_check=True)
        def shape(value):return [int(d.dim_value) for d in value.type.tensor_type.shape.dim]
        if len(graph.graph.input)!=1 or len(graph.graph.output)!=1 or shape(graph.graph.input[0])!=[1,3,300,120] or shape(graph.graph.output[0])!=[1,3,300,120]:
            raise ValueError('Unexpected ONNX I/O shape')
        if any(n.domain not in ('','ai.onnx') for n in graph.graph.node):raise ValueError('Custom ONNX operators are not allowed')
        metadata={p.key:p.value for p in graph.metadata_props}
        metadata.update(channels=json.dumps(list(CHANNELS)),checkpoint_sha256=hashlib.sha256(checkpoint_bytes).hexdigest(),
                        checkpoint_epoch=str(saved['epoch']),input_format='RGB float32 [0,1] NCHW W120 H300',output_format='independent left/right/stop_line logits')
        onnx.helper.set_model_props(graph,metadata);onnx.save(graph,str(path))
        options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
        session=ort.InferenceSession(str(path),sess_options=options,providers=['CPUExecutionProvider'])
        records=[]
        logit_threshold=math.log(saved['config']['threshold']/(1-saved['config']['threshold']))
        for name,array in inputs:
            expected=reference(torch.from_numpy(array)).numpy()
            patched=exported(torch.from_numpy(array)).numpy()
            actual=session.run(['logits'],{'rgb':array})[0]
            np.testing.assert_allclose(patched,expected,rtol=3e-4,atol=5e-4)
            np.testing.assert_allclose(actual,expected,rtol=3e-4,atol=5e-4)
            disagreement=np.count_nonzero((actual>=logit_threshold)!=(expected>=logit_threshold),axis=(0,2,3))
            if disagreement.max()>36:raise ValueError('More than 0.1% mask disagreement in a channel')
            records.append(dict(input=name,max_logit_error=float(np.max(np.abs(actual-expected))),mask_difference_pixels=disagreement.tolist()))
        del session
        report=dict(checkpoint=str(source.resolve()),checkpoint_epoch=saved['epoch'],checkpoint_sha256=metadata['checkpoint_sha256'],
            onnx_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),opset=17,input=[1,3,300,120],output=[1,3,300,120],
            channels=list(CHANNELS),torch=str(torch.__version__),onnx=onnx.__version__,onnxruntime=ort.__version__,
            validation=records,ros_build_run=False,tensorrt_engine_run=False,training_run=False,
            note='Snapshot of specified best.pt; rerun after training finishes to deploy final best. CPU ONNX validation is not Jetson/TensorRT validation.')
        path.replace(output)
        output.with_suffix('.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print('Saved '+str(output))
    print('Verified {} inputs; epoch {}; channels left/right/stop_line'.format(len(records),saved['epoch']))


if __name__=='__main__':main()
