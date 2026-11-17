"""TensorRT 8.6/10.x execution with CUDA Runtime; no PyTorch/PyCUDA required."""
import ctypes as ct
import ctypes.util
import hashlib
import json
import os
from pathlib import Path
import numpy as np


class Cuda:
    def __init__(self):
        candidates = [ctypes.util.find_library('cudart'), 'libcudart.so',
                      '/usr/local/cuda/lib64/libcudart.so']
        for path in filter(None, candidates):
            try:
                self.lib = ct.CDLL(path)
                break
            except OSError:
                pass
        else:
            raise RuntimeError('CUDA Runtime libcudart was not found; use the Jetson JetPack environment')
        signatures = {
            'cudaMalloc': [ct.POINTER(ct.c_void_p), ct.c_size_t],
            'cudaFree': [ct.c_void_p],
            'cudaStreamCreate': [ct.POINTER(ct.c_void_p)],
            'cudaStreamDestroy': [ct.c_void_p],
            'cudaStreamSynchronize': [ct.c_void_p],
            'cudaMemcpyAsync': [ct.c_void_p, ct.c_void_p, ct.c_size_t, ct.c_int, ct.c_void_p],
            'cudaDeviceGetAttribute': [ct.POINTER(ct.c_int), ct.c_int, ct.c_int],
            'cudaRuntimeGetVersion': [ct.POINTER(ct.c_int)],
            'cudaDriverGetVersion': [ct.POINTER(ct.c_int)],
        }
        for name, args in signatures.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, ct.c_int
        self.lib.cudaGetErrorString.argtypes = [ct.c_int]
        self.lib.cudaGetErrorString.restype = ct.c_char_p

    def call(self, name, *args):
        status = getattr(self.lib, name)(*args)
        if status:
            raise RuntimeError(f'{name}: {self.lib.cudaGetErrorString(status).decode()}')

    def signature(self):
        values = []
        for attr in [75, 76, 16]:  # ComputeCapabilityMajor/Minor, MultiProcessorCount
            value = ct.c_int()
            self.call('cudaDeviceGetAttribute', ct.byref(value), attr, 0)
            values.append(value.value)
        for name in ['cudaRuntimeGetVersion', 'cudaDriverGetVersion']:
            value = ct.c_int()
            self.call(name, ct.byref(value))
            values.append(value.value)
        return values


class TensorRTBackend:
    def __init__(self, model_path, cache_dir, expected_sha256=None, progress=None):
        stage = progress or (lambda text: None)
        stage("Initializing CUDA / TensorRT runtime")
        import tensorrt as trt
        self.trt = trt
        version = tuple(int(x) for x in trt.__version__.split('.')[:2])
        if not ((version[0] == 8 and version[1] >= 6) or version[0] == 10):
            raise RuntimeError(f'This runner supports TensorRT 8.6 or 10.x; found {trt.__version__}')
        self.cuda = Cuda()
        self.stream, self.input_device, self.output_device = ct.c_void_p(), ct.c_void_p(), ct.c_void_p()
        self.logger = trt.Logger(trt.Logger.WARNING)
        trt.init_libnvinfer_plugins(self.logger, '')
        self.runtime = trt.Runtime(self.logger)
        stage("Reading ONNX / verifying model hash")
        data = Path(model_path).read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if expected_sha256 is not None and digest != expected_sha256:
            raise RuntimeError('Packaged ONNX hash mismatch')
        identity = {'onnx_sha256': digest, 'tensorrt': trt.__version__,
                    'cuda_device_runtime_driver': self.cuda.signature(), 'builder': 'rgb-point-fp32-v1-no-fp16-no-tf32'}
        identity_hash = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()[:24]
        directory = Path(cache_dir).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.engine_path = directory / (identity_hash + '.engine')
        self.engine = None
        stage("Checking engine cache")
        if self.engine_path.exists():
            stage("Reading / deserializing cached engine")
            try:
                self.engine = self.runtime.deserialize_cuda_engine(self.engine_path.read_bytes())
            except Exception:
                self.engine = None
        if self.engine is None:
            stage("Parsing ONNX / configuring TensorRT builder")
            builder = trt.Builder(self.logger)
            network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
            parser = trt.OnnxParser(network, self.logger)
            if not parser.parse(data):
                raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
            config = builder.create_builder_config()
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 256 * 1024 * 1024)
            config.clear_flag(trt.BuilderFlag.FP16)
            config.clear_flag(trt.BuilderFlag.TF32)
            config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED
            stage("Building TensorRT engine (GPU optimization; may take minutes)")
            plan = builder.build_serialized_network(network, config)
            if plan is None:
                raise RuntimeError('TensorRT engine build failed; see parser/builder log')
            stage("Deserializing newly built engine")
            self.engine = self.runtime.deserialize_cuda_engine(plan)
            if self.engine is None:
                raise RuntimeError('Built engine cannot be deserialized')
            stage("Writing engine cache")
            temporary = self.engine_path.with_suffix(f'.{os.getpid()}.tmp')
            temporary.write_bytes(bytes(plan))
            temporary.replace(self.engine_path)
        stage("Creating execution context / allocating GPU buffers")
        self.context = self.engine.create_execution_context()
        if self.context is None or self.engine.num_io_tensors != 2:
            raise RuntimeError('Invalid TensorRT engine I/O')
        for name, shape, mode in [('features', (1, 17, 32), trt.TensorIOMode.INPUT),
                                  ('logits', (1, 2), trt.TensorIOMode.OUTPUT)]:
            if (tuple(self.engine.get_tensor_shape(name)) != shape or
                    self.engine.get_tensor_dtype(name) != trt.float32 or
                    self.engine.get_tensor_mode(name) != mode):
                raise RuntimeError(f'Invalid {name} contract')
        self.output = np.empty((1, 2), dtype=np.float32)
        try:
            self.cuda.call('cudaStreamCreate', ct.byref(self.stream))
            self.cuda.call('cudaMalloc', ct.byref(self.input_device), 1 * 17 * 32 * 4)
            self.cuda.call('cudaMalloc', ct.byref(self.output_device), self.output.nbytes)
            if not self.context.set_tensor_address('features', self.input_device.value):
                raise RuntimeError('Could not bind features')
            if not self.context.set_tensor_address('logits', self.output_device.value):
                raise RuntimeError('Could not bind logits')
            inspector = self.engine.create_engine_inspector()
            inspector.execution_context = self.context
            self.engine_path.with_suffix('.layers.json').write_text(inspector.get_engine_information(trt.LayerInformationFormat.JSON))
            self.engine_path.with_suffix('.metadata.json').write_text(json.dumps(identity, indent=2))
            stage("Trial inference / CUDA warmup")
            for _ in range(20):
                self.infer(np.zeros((1, 17, 32), dtype=np.float32))
        except Exception:
            self.close()
            raise

    def infer(self, features):
        x = np.ascontiguousarray(features, dtype=np.float32)
        if x.shape != (1, 17, 32) or not np.isfinite(x).all():
            raise ValueError('Expected finite float32[1,17,32]')
        self.cuda.call('cudaMemcpyAsync', self.input_device, x.ctypes.data, x.nbytes, 1, self.stream)
        if not self.context.execute_async_v3(stream_handle=self.stream.value):
            raise RuntimeError('TensorRT execution failed')
        self.cuda.call('cudaMemcpyAsync', self.output.ctypes.data, self.output_device, self.output.nbytes, 2, self.stream)
        self.cuda.call('cudaStreamSynchronize', self.stream)
        if not np.isfinite(self.output).all():
            raise RuntimeError('TensorRT produced non-finite logits')
        return self.output.copy()

    def close(self):
        if self.stream.value:
            self.cuda.call('cudaStreamSynchronize', self.stream)
        # Drop engine/context before releasing buffers at final shutdown.
        self.context = None
        for name in ['input_device', 'output_device']:
            pointer = getattr(self, name)
            if pointer.value:
                self.cuda.call('cudaFree', pointer)
                pointer.value = None
        if self.stream.value:
            self.cuda.call('cudaStreamDestroy', self.stream)
            self.stream.value = None


class OnnxBackend:
    """Explicitly requested offline verification only; never an automatic fallback."""
    def __init__(self, model_path):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model_path), options, providers=['CPUExecutionProvider'])

    def infer(self, features):
        return self.session.run(None, {'features': np.ascontiguousarray(features, dtype=np.float32)})[0]

    def close(self):
        self.session = None
