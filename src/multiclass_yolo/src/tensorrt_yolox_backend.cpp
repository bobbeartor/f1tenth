#include "multiclass_yolo/tensorrt_yolox_backend.hpp"

#include "cuda_yolox_preprocessor.hpp"
#include "yolox_geometry.hpp"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace multiclass_yolo
{

namespace
{

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(const Severity severity, const char * message) noexcept override
  {
    if (severity > Severity::kWARNING || message == nullptr) {
      return;
    }
    std::cerr << "[multiclass_yolo TensorRT] " << message << '\n';
  }
};

template<typename T>
struct TensorRtDeleter
{
  void operator()(T * object) const noexcept
  {
    delete object;
  }
};

template<typename T>
using TensorRtUniquePtr = std::unique_ptr<T, TensorRtDeleter<T>>;

void check_cuda(const cudaError_t status, const char * operation)
{
  if (status == cudaSuccess) {
    return;
  }
  throw std::runtime_error(
          std::string(operation) + " failed: " + cudaGetErrorString(status));
}

class CudaBuffer
{
public:
  CudaBuffer() = default;
  ~CudaBuffer()
  {
    if (data_ != nullptr) {
      static_cast<void>(cudaFree(data_));
    }
  }

  CudaBuffer(const CudaBuffer &) = delete;
  CudaBuffer & operator=(const CudaBuffer &) = delete;

  void allocate(const std::size_t byte_count)
  {
    if (byte_count == 0U) {
      throw std::invalid_argument("CUDA allocation size must be positive");
    }
    if (data_ != nullptr) {
      check_cuda(cudaFree(data_), "cudaFree before resize");
      data_ = nullptr;
      capacity_ = 0U;
    }
    check_cuda(cudaMalloc(&data_, byte_count), "cudaMalloc");
    capacity_ = byte_count;
  }

  void ensure_capacity(const std::size_t byte_count)
  {
    if (byte_count > capacity_) {
      allocate(byte_count);
    }
  }

  void * get() const noexcept
  {
    return data_;
  }

private:
  void * data_{nullptr};
  std::size_t capacity_{0U};
};

class CudaStream
{
public:
  CudaStream()
  {
    check_cuda(
      cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
      "cudaStreamCreateWithFlags");
  }

  ~CudaStream()
  {
    if (stream_ != nullptr) {
      static_cast<void>(cudaStreamDestroy(stream_));
    }
  }

  CudaStream(const CudaStream &) = delete;
  CudaStream & operator=(const CudaStream &) = delete;

  cudaStream_t get() const noexcept
  {
    return stream_;
  }

private:
  cudaStream_t stream_{nullptr};
};

class CudaEvent
{
public:
  CudaEvent()
  {
    check_cuda(cudaEventCreate(&event_), "cudaEventCreate");
  }

  ~CudaEvent()
  {
    if (event_ != nullptr) {
      static_cast<void>(cudaEventDestroy(event_));
    }
  }

  CudaEvent(const CudaEvent &) = delete;
  CudaEvent & operator=(const CudaEvent &) = delete;

  cudaEvent_t get() const noexcept
  {
    return event_;
  }

private:
  cudaEvent_t event_{nullptr};
};

std::size_t dimensions_volume(const nvinfer1::Dims & dimensions)
{
  if (dimensions.nbDims <= 0) {
    throw std::runtime_error("TensorRT returned an empty tensor shape");
  }

  std::size_t volume = 1U;
  for (int index = 0; index < dimensions.nbDims; ++index) {
    const int extent = dimensions.d[index];
    if (extent <= 0) {
      throw std::runtime_error(
              "dynamic TensorRT tensor shapes are not supported");
    }
    const auto unsigned_extent = static_cast<std::size_t>(extent);
    if (volume > std::numeric_limits<std::size_t>::max() / unsigned_extent) {
      throw std::overflow_error("TensorRT tensor shape is too large");
    }
    volume *= unsigned_extent;
  }
  return volume;
}

std::size_t expected_input_elements(
  const int input_width, const int input_height)
{
  if (input_width <= 0 || input_height <= 0) {
    throw std::invalid_argument("TensorRT input dimensions must be positive");
  }
  return
    static_cast<std::size_t>(3) *
    static_cast<std::size_t>(input_width) *
    static_cast<std::size_t>(input_height);
}

std::string dimensions_string(const nvinfer1::Dims & dimensions)
{
  std::ostringstream stream;
  stream << '[';
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (index > 0) {
      stream << ',';
    }
    stream << dimensions.d[index];
  }
  stream << ']';
  return stream.str();
}

std::vector<char> read_binary_file(const std::string & path)
{
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) {
    return {};
  }
  const auto end = input.tellg();
  if (end <= 0) {
    return {};
  }
  const auto byte_count = static_cast<std::size_t>(
    static_cast<std::streamoff>(end));
  std::vector<char> bytes(byte_count);
  input.seekg(0, std::ios::beg);
  input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (!input) {
    return {};
  }
  return bytes;
}

bool cache_is_current(
  const std::string & engine_path, const std::string & model_path)
{
  std::error_code error;
  if (!std::filesystem::is_regular_file(engine_path, error) || error) {
    return false;
  }
  const auto engine_time = std::filesystem::last_write_time(engine_path, error);
  if (error) {
    return false;
  }
  const auto model_time = std::filesystem::last_write_time(model_path, error);
  return !error && engine_time >= model_time;
}

void write_engine_cache(
  const std::string & path, const void * data, const std::size_t byte_count)
{
  const std::filesystem::path cache_path(path);
  std::error_code error;
  if (!cache_path.parent_path().empty()) {
    std::filesystem::create_directories(cache_path.parent_path(), error);
    if (error) {
      std::cerr << "[multiclass_yolo TensorRT] Could not create engine "
                << "cache directory: " << error.message() << '\n';
      return;
    }
  }

  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    std::cerr << "[multiclass_yolo TensorRT] Could not write engine "
              << "cache: " << path << '\n';
    return;
  }
  output.write(
    static_cast<const char *>(data),
    static_cast<std::streamsize>(byte_count));
  if (!output) {
    std::cerr << "[multiclass_yolo TensorRT] Failed while writing "
              << "engine cache: " << path << '\n';
  }
}

std::uint64_t elapsed_cuda_nanoseconds(
  const cudaEvent_t started_at, const cudaEvent_t finished_at)
{
  float milliseconds = 0.0F;
  check_cuda(
    cudaEventElapsedTime(&milliseconds, started_at, finished_at),
    "cudaEventElapsedTime");
  if (!std::isfinite(milliseconds) || milliseconds <= 0.0F) {
    return 0U;
  }
  return static_cast<std::uint64_t>(
    std::llround(static_cast<double>(milliseconds) * 1.0e6));
}

}  // namespace

class TensorRtYoloxBackend::Impl
{
public:
  Impl(
    const std::string & model_path,
    const std::string & requested_engine_cache_path,
    const std::string & engine_precision,
    const int input_width,
    const int input_height,
    const std::size_t workspace_size_bytes,
    std::function<void(const std::string &)> progress)
  : progress_(std::move(progress)),
    model_path_(model_path),
    engine_precision_(engine_precision),
    engine_cache_path_(requested_engine_cache_path.empty() ?
      model_path + ".trt" + std::to_string(NV_TENSORRT_MAJOR) +
      "." + engine_precision + ".engine" : requested_engine_cache_path),
    input_width_(input_width),
    input_height_(input_height),
    expected_input_element_count_(
      expected_input_elements(input_width, input_height))
  {
    if (
      engine_precision_ != "fp32" && engine_precision_ != "fp16" &&
      engine_precision_ != "int8")
    {
      throw std::invalid_argument(
              "TensorRT engine_precision must be fp32, fp16, or int8");
    }
    if (workspace_size_bytes == 0U) {
      throw std::invalid_argument("TensorRT workspace size must be positive");
    }

    if (!initLibNvInferPlugins(&logger_, "")) {
      throw std::runtime_error("TensorRT plugin initialization failed");
    }
    stage("Initializing TensorRT runtime");
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("TensorRT runtime creation failed");
    }

    load_or_build_engine(workspace_size_bytes);
    stage("Creating execution context / allocating GPU buffers");
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("TensorRT execution context creation failed");
    }
    inspect_engine(input_width, input_height);

    input_device_.allocate(expected_input_element_count_ * sizeof(float));
    output_device_.allocate(output_element_count_ * sizeof(float));
    configure_execution_bindings();
  }

  TensorRtInferenceTiming infer(
    const float * input,
    const std::size_t input_element_count,
    float * output,
    const std::size_t output_element_count)
  {
    if (input == nullptr || output == nullptr) {
      throw std::invalid_argument("TensorRT input and output must not be null");
    }
    if (input_element_count != expected_input_element_count_) {
      throw std::invalid_argument("TensorRT input element count mismatch");
    }
    if (output_element_count != output_element_count_) {
      throw std::invalid_argument("TensorRT output element count mismatch");
    }

    check_cuda(
      cudaEventRecord(input_started_.get(), stream_.get()),
      "cudaEventRecord(input start)");
    check_cuda(
      cudaMemcpyAsync(
        input_device_.get(), input,
        input_element_count * sizeof(float),
        cudaMemcpyHostToDevice, stream_.get()),
      "cudaMemcpyAsync(host to device)");
    check_cuda(
      cudaEventRecord(input_finished_.get(), stream_.get()),
      "cudaEventRecord(input finish)");

    return execute_and_copy_output(
      input_finished_.get(), false, output, output_element_count);
  }

  TensorRtInferenceTiming infer_nv12(
    const std::uint8_t * nv12,
    const std::size_t data_size,
    const std::size_t source_stride,
    const int source_width,
    const int source_height,
    const int roi_left,
    const int roi_top,
    const int roi_width,
    const int roi_height,
    float * output,
    const std::size_t output_element_count)
  {
    if (nv12 == nullptr || output == nullptr) {
      throw std::invalid_argument("TensorRT NV12 input and output are required");
    }
    if (
      source_width <= 0 || source_height <= 0 ||
      source_width % 2 != 0 || source_height % 2 != 0 ||
      source_stride < static_cast<std::size_t>(source_width) ||
      roi_left < 0 || roi_top < 0 || roi_width <= 0 || roi_height <= 0 ||
      roi_width > source_width || roi_height > source_height ||
      roi_left > source_width - roi_width ||
      roi_top > source_height - roi_height)
    {
      throw std::invalid_argument(
              "NV12 input and inference ROI geometry are invalid");
    }
    if (output_element_count != output_element_count_) {
      throw std::invalid_argument("TensorRT output element count mismatch");
    }
    const std::size_t nv12_byte_count =
      source_stride * static_cast<std::size_t>(source_height) * 3U / 2U;
    if (data_size < nv12_byte_count) {
      throw std::invalid_argument("NV12 input buffer is undersized");
    }
    // Upload only the ROI, expanded to chroma-aligned edges. Preserve odd
    // ROI coordinates used by the standalone detector (e.g. top=65).
    const int upload_left = roi_left & ~1;
    const int upload_top = roi_top & ~1;
    const int upload_width = ((roi_left + roi_width + 1) & ~1) - upload_left;
    const int upload_height = ((roi_top + roi_height + 1) & ~1) - upload_top;
    const std::size_t upload_y_bytes =
      static_cast<std::size_t>(upload_width) * upload_height;
    nv12_device_.ensure_capacity(upload_y_bytes * 3U / 2U);

    check_cuda(
      cudaEventRecord(input_started_.get(), stream_.get()),
      "cudaEventRecord(NV12 input start)");
    check_cuda(
      cudaMemcpy2DAsync(
        nv12_device_.get(), upload_width,
        nv12 + static_cast<std::size_t>(upload_top) * source_stride + upload_left,
        source_stride, upload_width, upload_height,
        cudaMemcpyHostToDevice, stream_.get()),
      "cudaMemcpy2DAsync(NV12 ROI Y)");
    check_cuda(
      cudaMemcpy2DAsync(
        static_cast<std::uint8_t *>(nv12_device_.get()) + upload_y_bytes, upload_width,
        nv12 + source_stride * source_height +
        static_cast<std::size_t>(upload_top / 2) * source_stride + upload_left,
        source_stride, upload_width, upload_height / 2,
        cudaMemcpyHostToDevice, stream_.get()),
      "cudaMemcpy2DAsync(NV12 ROI UV)");
    check_cuda(
      cudaEventRecord(input_finished_.get(), stream_.get()),
      "cudaEventRecord(NV12 input finish)");

    check_cuda(
      launch_nv12_roi_to_bgr_nchw(
        static_cast<const std::uint8_t *>(nv12_device_.get()),
        upload_width, upload_height, roi_left - upload_left, roi_top - upload_top, roi_width,
        roi_height,
        static_cast<float *>(input_device_.get()), input_width_, input_height_,
        stream_.get()),
      "launch NV12 to BGR NCHW kernel");
    check_cuda(
      cudaEventRecord(preprocessing_finished_.get(), stream_.get()),
      "cudaEventRecord(NV12 preprocess finish)");

    return execute_and_copy_output(
      preprocessing_finished_.get(), true, output, output_element_count);
  }

  int output_row_count() const noexcept
  {
    return output_row_count_;
  }

  int output_column_count() const noexcept
  {
    return output_column_count_;
  }

  const std::string & engine_cache_path() const noexcept
  {
    return engine_cache_path_;
  }

private:
  TensorRtInferenceTiming execute_and_copy_output(
    const cudaEvent_t execution_started,
    const bool preprocessing_enabled,
    float * output,
    const std::size_t output_element_count)
  {
    bool enqueued = false;
#if NV_TENSORRT_MAJOR >= 10
    enqueued = context_->enqueueV3(stream_.get());
#else
    enqueued = context_->enqueueV2(
      bindings_.data(), stream_.get(), nullptr);
#endif
    if (!enqueued) {
      static_cast<void>(cudaStreamSynchronize(stream_.get()));
      throw std::runtime_error("TensorRT inference enqueue failed");
    }
    check_cuda(
      cudaEventRecord(execution_finished_.get(), stream_.get()),
      "cudaEventRecord(execution finish)");
    check_cuda(
      cudaMemcpyAsync(
        output, output_device_.get(),
        output_element_count * sizeof(float),
        cudaMemcpyDeviceToHost, stream_.get()),
      "cudaMemcpyAsync(device to host)");
    check_cuda(
      cudaEventRecord(output_finished_.get(), stream_.get()),
      "cudaEventRecord(output finish)");
    check_cuda(
      cudaEventSynchronize(output_finished_.get()),
      "cudaEventSynchronize(output finish)");

    return TensorRtInferenceTiming{
      elapsed_cuda_nanoseconds(input_started_.get(), input_finished_.get()),
      preprocessing_enabled ? elapsed_cuda_nanoseconds(
        input_finished_.get(), preprocessing_finished_.get()) : 0U,
      elapsed_cuda_nanoseconds(execution_started, execution_finished_.get()),
      elapsed_cuda_nanoseconds(
        execution_finished_.get(), output_finished_.get())};
  }

  void load_or_build_engine(const std::size_t workspace_size_bytes)
  {
    stage("Checking engine cache");
    if (cache_is_current(engine_cache_path_, model_path_)) {
      stage("Reading / deserializing cached engine");
      const auto cached_engine = read_binary_file(engine_cache_path_);
      if (!cached_engine.empty()) {
        engine_.reset(runtime_->deserializeCudaEngine(
            cached_engine.data(), cached_engine.size()));
        if (engine_) {
          std::cerr << "[multiclass_yolo TensorRT] Loaded "
                    << engine_precision_ << " engine "
                    << "cache: " << engine_cache_path_ << '\n';
          return;
        }
        std::cerr << "[multiclass_yolo TensorRT] Engine cache was not "
                  << "compatible; rebuilding from ONNX.\n";
      }
    }

    stage("Parsing ONNX / configuring TensorRT builder");
    auto builder = TensorRtUniquePtr<nvinfer1::IBuilder>(
      nvinfer1::createInferBuilder(logger_));
    if (!builder) {
      throw std::runtime_error("TensorRT builder creation failed");
    }
    constexpr std::uint32_t explicit_batch_flag =
#if NV_TENSORRT_MAJOR >= 10
      0U;
#else
      1U << static_cast<std::uint32_t>(
      nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
#endif
    auto network = TensorRtUniquePtr<nvinfer1::INetworkDefinition>(
      builder->createNetworkV2(explicit_batch_flag));
    if (!network) {
      throw std::runtime_error("TensorRT network creation failed");
    }
    auto parser = TensorRtUniquePtr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, logger_));
    if (!parser) {
      throw std::runtime_error("TensorRT ONNX parser creation failed");
    }

    if (!parser->parseFromFile(
        model_path_.c_str(),
        static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
    {
      std::ostringstream message;
      message << "TensorRT could not parse ONNX model: " << model_path_;
      for (int index = 0; index < parser->getNbErrors(); ++index) {
        const auto * error = parser->getError(index);
        if (error != nullptr) {
          message << "\n- " << error->desc();
        }
      }
      throw std::runtime_error(message.str());
    }

    bool has_quantize = false;
    bool has_dequantize = false;
    for (int index = 0; index < network->getNbLayers(); ++index) {
      const auto type = network->getLayer(index)->getType();
      has_quantize = has_quantize || type == nvinfer1::LayerType::kQUANTIZE;
      has_dequantize = has_dequantize ||
        type == nvinfer1::LayerType::kDEQUANTIZE;
    }
    if (engine_precision_ == "int8" && (!has_quantize || !has_dequantize)) {
      throw std::runtime_error(
              "INT8 mode requires an explicit Q/DQ ONNX model");
    }
    if (engine_precision_ != "int8" && (has_quantize || has_dequantize)) {
      throw std::runtime_error(
              "Explicit Q/DQ ONNX requires engine_precision=int8");
    }

    auto config = TensorRtUniquePtr<nvinfer1::IBuilderConfig>(
      builder->createBuilderConfig());
    if (!config) {
      throw std::runtime_error("TensorRT builder config creation failed");
    }
#if NV_TENSORRT_MAJOR >= 10
    config->setMemoryPoolLimit(
      nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size_bytes);
#else
    config->setMaxWorkspaceSize(workspace_size_bytes);
#endif
    if (engine_precision_ == "fp16") {
      if (!builder->platformHasFastFp16()) {
        throw std::runtime_error(
                "TensorRT device does not report fast FP16 support");
      }
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      config->clearFlag(nvinfer1::BuilderFlag::kINT8);
    } else if (engine_precision_ == "int8") {
      if (!builder->platformHasFastInt8()) {
        throw std::runtime_error(
                "TensorRT device does not report fast INT8 support");
      }
      // Q/DQ nodes define the explicit INT8 precision boundaries. TensorRT
      // 8.x additionally requires the builder INT8 flag.
      config->clearFlag(nvinfer1::BuilderFlag::kFP16);
      config->setFlag(nvinfer1::BuilderFlag::kINT8);
    } else {
      config->clearFlag(nvinfer1::BuilderFlag::kFP16);
      config->clearFlag(nvinfer1::BuilderFlag::kINT8);
    }
    config->clearFlag(nvinfer1::BuilderFlag::kTF32);

    std::cerr << "[multiclass_yolo TensorRT] Building "
              << engine_precision_ << " engine from "
              << model_path_ << ". The first launch can take several minutes.\n";
    stage("Building TensorRT engine (GPU optimization; may take minutes)");
    auto serialized = TensorRtUniquePtr<nvinfer1::IHostMemory>(
      builder->buildSerializedNetwork(*network, *config));
    if (!serialized) {
      throw std::runtime_error(
              "TensorRT " + engine_precision_ + " engine build failed");
    }

    stage("Deserializing newly built engine");
    engine_.reset(runtime_->deserializeCudaEngine(
        serialized->data(), serialized->size()));
    if (!engine_) {
      throw std::runtime_error(
              "TensorRT could not deserialize the newly built " +
              engine_precision_ + " engine");
    }
    stage("Writing engine cache");
    write_engine_cache(
      engine_cache_path_, serialized->data(), serialized->size());
    std::cerr << "[multiclass_yolo TensorRT] " << engine_precision_
              << " engine ready; cache="
              << engine_cache_path_ << '\n';
  }

  void inspect_engine(const int input_width, const int input_height)
  {
    nvinfer1::Dims input_dimensions{};
    nvinfer1::Dims output_dimensions{};
    nvinfer1::DataType input_type{};
    nvinfer1::DataType output_type{};

#if NV_TENSORRT_MAJOR >= 10
    if (engine_->getNbIOTensors() != 2) {
      throw std::runtime_error(
              "YOLOX TensorRT engine must contain one input and one output");
    }
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char * name = engine_->getIOTensorName(index);
      if (name == nullptr) {
        throw std::runtime_error("TensorRT returned an unnamed I/O tensor");
      }
      const auto mode = engine_->getTensorIOMode(name);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
        input_dimensions = engine_->getTensorShape(name);
        input_type = engine_->getTensorDataType(name);
      } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
        output_name_ = name;
        output_dimensions = engine_->getTensorShape(name);
        output_type = engine_->getTensorDataType(name);
      }
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error(
              "TensorRT could not identify the YOLOX input and output");
    }
#else
    if (engine_->getNbBindings() != 2) {
      throw std::runtime_error(
              "YOLOX TensorRT engine must contain one input and one output");
    }
    bindings_.resize(static_cast<std::size_t>(engine_->getNbBindings()));
    for (int index = 0; index < engine_->getNbBindings(); ++index) {
      if (engine_->bindingIsInput(index)) {
        input_binding_index_ = index;
        input_dimensions = engine_->getBindingDimensions(index);
        input_type = engine_->getBindingDataType(index);
      } else {
        output_binding_index_ = index;
        output_dimensions = engine_->getBindingDimensions(index);
        output_type = engine_->getBindingDataType(index);
      }
    }
    if (input_binding_index_ < 0 || output_binding_index_ < 0) {
      throw std::runtime_error(
              "TensorRT could not identify the YOLOX input and output");
    }
#endif

    if (input_type != nvinfer1::DataType::kFLOAT ||
      output_type != nvinfer1::DataType::kFLOAT)
    {
      throw std::runtime_error(
              "TensorRT engine I/O must be FP32; reduced-precision I/O is not "
              "accepted");
    }

    const std::size_t input_volume = dimensions_volume(input_dimensions);
    if (input_volume != expected_input_element_count_ ||
      input_dimensions.nbDims != 4 || input_dimensions.d[0] != 1 ||
      input_dimensions.d[1] != 3 || input_dimensions.d[2] != input_height ||
      input_dimensions.d[3] != input_width)
    {
      throw std::runtime_error(
              "unexpected TensorRT input shape " +
              dimensions_string(input_dimensions) + "; expected [1,3," +
              std::to_string(input_height) + "," +
              std::to_string(input_width) + "]");
    }

    output_element_count_ = dimensions_volume(output_dimensions);
    const auto expected_rows = yolox_output_rows(input_width, input_height);
    if (output_dimensions.nbDims != 3 || output_dimensions.d[0] != 1 ||
      output_dimensions.d[1] != expected_rows || output_dimensions.d[2] != 9 || output_element_count_ % 9U != 0U)
    {
      throw std::runtime_error(
              "unexpected TensorRT output shape " +
              dimensions_string(output_dimensions) +
              "; expected FP32 [1," + std::to_string(expected_rows) + ",9]");
    }
    output_row_count_ = output_dimensions.d[1];
    output_column_count_ = output_dimensions.d[2];
  }

  void configure_execution_bindings()
  {
#if NV_TENSORRT_MAJOR >= 10
    if (!context_->setTensorAddress(input_name_.c_str(), input_device_.get()) ||
      !context_->setTensorAddress(output_name_.c_str(), output_device_.get()))
    {
      throw std::runtime_error("TensorRT could not bind YOLOX device buffers");
    }
#else
    bindings_[static_cast<std::size_t>(input_binding_index_)] =
      input_device_.get();
    bindings_[static_cast<std::size_t>(output_binding_index_)] =
      output_device_.get();
#endif
  }

  std::function<void(const std::string &)> progress_;
  void stage(const std::string & text) {if (progress_) {progress_(text);}}
  std::string model_path_;
  std::string engine_precision_;
  std::string engine_cache_path_;
  int input_width_{0};
  int input_height_{0};
  std::size_t expected_input_element_count_{0U};
  std::size_t output_element_count_{0U};
  int output_row_count_{0};
  int output_column_count_{0};

  TensorRtLogger logger_;
  TensorRtUniquePtr<nvinfer1::IRuntime> runtime_;
  TensorRtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TensorRtUniquePtr<nvinfer1::IExecutionContext> context_;
  CudaStream stream_;
  CudaBuffer input_device_;
  CudaBuffer output_device_;
  CudaBuffer nv12_device_;
  CudaEvent input_started_;
  CudaEvent input_finished_;
  CudaEvent preprocessing_finished_;
  CudaEvent execution_finished_;
  CudaEvent output_finished_;

#if NV_TENSORRT_MAJOR >= 10
  std::string input_name_;
  std::string output_name_;
#else
  int input_binding_index_{-1};
  int output_binding_index_{-1};
  std::vector<void *> bindings_;
#endif
};

TensorRtYoloxBackend::TensorRtYoloxBackend(
  const std::string & model_path,
  const std::string & engine_cache_path,
  const std::string & engine_precision,
  const int input_width,
  const int input_height,
  const std::size_t workspace_size_bytes,
  std::function<void(const std::string &)> progress)
: impl_(std::make_unique<Impl>(
    model_path, engine_cache_path, engine_precision, input_width, input_height,
    workspace_size_bytes, std::move(progress)))
{
}

TensorRtYoloxBackend::~TensorRtYoloxBackend() = default;
TensorRtYoloxBackend::TensorRtYoloxBackend(TensorRtYoloxBackend &&) noexcept =
  default;
TensorRtYoloxBackend & TensorRtYoloxBackend::operator=(
  TensorRtYoloxBackend &&) noexcept = default;

TensorRtInferenceTiming TensorRtYoloxBackend::infer(
  const float * input,
  const std::size_t input_element_count,
  float * output,
  const std::size_t output_element_count)
{
  return impl_->infer(
    input, input_element_count, output, output_element_count);
}

TensorRtInferenceTiming TensorRtYoloxBackend::infer_nv12(
  const std::uint8_t * nv12,
  const std::size_t data_size,
  const std::size_t source_stride,
  const int source_width,
  const int source_height,
  const int roi_left,
  const int roi_top,
  const int roi_width,
  const int roi_height,
  float * output,
  const std::size_t output_element_count)
{
  return impl_->infer_nv12(
    nv12, data_size, source_stride, source_width, source_height, roi_left,
    roi_top, roi_width, roi_height, output, output_element_count);
}

int TensorRtYoloxBackend::output_row_count() const noexcept
{
  return impl_->output_row_count();
}

int TensorRtYoloxBackend::output_column_count() const noexcept
{
  return impl_->output_column_count();
}

const std::string & TensorRtYoloxBackend::engine_cache_path() const noexcept
{
  return impl_->engine_cache_path();
}

}  // namespace multiclass_yolo
