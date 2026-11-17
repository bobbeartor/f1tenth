# Stop-line INT8 PTQ workflow

This workflow is offline and does not build ROS. It keeps calibration data in
the training partition, evaluates on the fixed validation partition, and emits
an explicit Q/DQ ONNX model for TensorRT.

## Frozen setup

- Source model: epoch 59 `fast_scnn_stop_line_120x300_batch_1.onnx`
- Calibration: 512 unique train images, 200 stop-positive and 312 stop-negative
- Quantized operators: Conv with symmetric INT8 activations and per-channel
  symmetric INT8 weights
- Floating-point operators: InstanceNormalization, Resize, pooling, elementwise
  operations, the final half-resolution decoder stage and the three-channel classifier
- Evaluation: all 496 labeled validation images, never used for calibration
- Threshold: fixed at 0.5 for the first A/B comparison

`int8_calibration_manifest.json` freezes sample IDs and decoded-image SHA256
hashes. The tools fail if an image changes or calibration overlaps validation.
The original image files are intentionally not copied into this repository.

## Reproduce on the Windows export PC

Run from the repository root. Adjust only the two data paths when the prepared
run moves.

```powershell
$python = "C:\Users\godld\Desktop\traffic\line_detector\.venv-export\Scripts\python.exe"
$data = "C:\Users\godld\Desktop\traffic\line_detector"
$prepared = "$data\training\highres\stop_line_0910\prepared\run_20260910_155954_960947"

& $python src\line_detactor\tools\prepare_int8_calibration.py `
  --data-root $data `
  --train-manifest "$prepared\train.json" `
  --verify-images

& $python src\line_detactor\tools\quantize_int8_ptq.py --data-root $data

& $python src\line_detactor\tools\evaluate_int8_ptq.py `
  --data-root $data `
  --validation-manifest "$prepared\val.json"

& $python src\line_detactor\tools\render_int8_ptq_gallery.py `
  --data-root $data `
  --validation-manifest "$prepared\val.json" `
  --output "$data\evaluation_int8_ptq_gallery_100"

& $python src\line_detactor\tools\benchmark_int8_ptq.py `
  --data-root $data `
  --gallery-index "$data\evaluation_int8_ptq_gallery_100\index.json"
```

The conversion uses ONNX Runtime's static QDQ quantizer, which is already part
of `requirements-export.txt`. NVIDIA Model Optimizer can later replace this
stage if target-TensorRT profiling shows that different Q/DQ placement is
needed; the frozen calibration manifest remains reusable.

## Current offline result

The TensorRT-compatible candidate with floating-point Conv biases passed every
configured accuracy gate:

| Condition / metric | FP32 | INT8 Q/DQ | INT8 - FP32 |
|---|---:|---:|---:|
| Clean stop-line pixel F1 | 0.790586 | 0.792143 | +0.001557 |
| Clean lane macro pixel F1 | 0.848708 | 0.847980 | -0.000727 |
| Occluded stop-line pixel F1 | 0.742926 | 0.743712 | +0.000786 |
| Occluded lane macro pixel F1 | 0.826870 | 0.824301 | -0.002568 |
| Clean negative stop-frame FP rate | 0.016092 | 0.011494 | -0.004598 |
| Occluded negative stop-frame FP rate | 0.059770 | 0.066667 | +0.006897 |
| Positive stop-frame tolerance-hit rate | 1.000000 | 1.000000 | 0.000000 |

These results measure clean and deterministic synthetic-occlusion validation accuracy.
They are not an unseen-drive test and do not establish Jetson latency.

## Windows CPU reference timing

The benchmark preloads the same 100 gallery images and measures model inference
only: 100 warm-up runs per model followed by 1,000 timed runs per model. File I/O
and preprocessing are excluded. ONNX Runtime uses `CPUExecutionProvider` with
two intra-op threads.

| Model | Mean | Median | P95 |
|---|---:|---:|---:|
| FP32 | 2.2215 ms | 2.1760 ms | 2.4354 ms |
| INT8 Q/DQ | 3.8599 ms | 3.8056 ms | 4.1107 ms |

On this Windows CPU path, INT8 is `0.576x` as fast as FP32, or about `1.74x`
higher in mean latency. This does not predict Jetson TensorRT performance:
execution-provider kernels, graph fusion and Q/DQ handling differ. Use the
target-Jetson TensorRT timing for the deployment decision.

## TensorRT/Jetson trial

Keep the package default at FP32 until the target benchmark is complete. To try
the generated candidate, select both the Q/DQ model and INT8 engine mode:

```bash
ros2 launch line_detactor line_detactor.launch.py \
  model_path:="$(ros2 pkg prefix line_detactor)/share/line_detactor/models/fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx" \
  engine_precision:=int8
```

The engine cache name includes the TensorRT major version and precision. Do not
reuse an FP32 or FP16 cache for INT8. The model I/O remains FP32, while Q/DQ
nodes define the internal INT8 regions. If the INT8 engine fails its target
latency/accuracy trial, use the ordinary FP32 ONNX with `engine_precision:=fp16`.

## Acceptance gates

- stop-line F1 drop no greater than 0.01
- lane macro F1 drop no greater than 0.01
- negative stop-frame FP-rate increase no greater than 0.01
- positive stop-frame tolerance-hit-rate drop no greater than 0.01
- a separate, target-Jetson latency/power gate is still required

The authoritative result files are `int8_ptq_evaluation.json`,
`int8_ptq_cpu_benchmark.json`, and the JSON next to the Q/DQ ONNX. The unlabeled
26-image final-test set remains visual-only until ground-truth labels are created.
