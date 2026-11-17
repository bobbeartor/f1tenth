#include "line_detactor/tensorrt_lane_backend.hpp"

#include "cuda_lane_pipeline.hpp"

#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

namespace line_detactor
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
    std::cerr << "[line_detactor TensorRT] " << message << '\n';
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
  if (status != cudaSuccess) {
    throw std::runtime_error(
            std::string(operation) + " failed: " +
            cudaGetErrorString(status));
  }
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
    check_cuda(cudaMalloc(&data_, byte_count), "cudaMalloc");
  }

  void * get() const noexcept
  {
    return data_;
  }

private:
  void * data_{nullptr};
};

class PinnedHostBuffer
{
public:
  PinnedHostBuffer() = default;
  ~PinnedHostBuffer()
  {
    if (data_ != nullptr) {
      static_cast<void>(cudaFreeHost(data_));
    }
  }
  PinnedHostBuffer(const PinnedHostBuffer &) = delete;
  PinnedHostBuffer & operator=(const PinnedHostBuffer &) = delete;

  void allocate(const std::size_t byte_count)
  {
    if (byte_count == 0U) {
      throw std::invalid_argument("Pinned allocation size must be positive");
    }
    check_cuda(
      cudaHostAlloc(&data_, byte_count, cudaHostAllocDefault),
      "cudaHostAlloc");
  }

  void * get() const noexcept
  {
    return data_;
  }

private:
  void * data_{nullptr};
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
      throw std::runtime_error("Dynamic TensorRT tensor shapes are unsupported");
    }
    const auto unsigned_extent = static_cast<std::size_t>(extent);
    if (volume > std::numeric_limits<std::size_t>::max() / unsigned_extent) {
      throw std::overflow_error("TensorRT tensor shape is too large");
    }
    volume *= unsigned_extent;
  }
  return volume;
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
  std::vector<char> bytes(
    static_cast<std::size_t>(static_cast<std::streamoff>(end)));
  input.seekg(0, std::ios::beg);
  input.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  return input ? bytes : std::vector<char>{};
}

bool cache_is_current(
  const std::string & engine_path,
  const std::string & model_path)
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
  const std::string & path,
  const void * data,
  const std::size_t byte_count)
{
  const std::filesystem::path cache_path(path);
  std::error_code error;
  if (!cache_path.parent_path().empty()) {
    std::filesystem::create_directories(cache_path.parent_path(), error);
    if (error) {
      std::cerr << "[line_detactor TensorRT] Could not create engine cache "
                << "directory: " << error.message() << '\n';
      return;
    }
  }
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    std::cerr << "[line_detactor TensorRT] Could not write engine cache: "
              << path << '\n';
    return;
  }
  output.write(
    static_cast<const char *>(data),
    static_cast<std::streamsize>(byte_count));
  if (!output) {
    std::cerr << "[line_detactor TensorRT] Engine cache write failed: "
              << path << '\n';
  }
}

std::uint64_t elapsed_cuda_nanoseconds(
  const cudaEvent_t started,
  const cudaEvent_t finished)
{
  float milliseconds = 0.0F;
  check_cuda(
    cudaEventElapsedTime(&milliseconds, started, finished),
    "cudaEventElapsedTime");
  if (!std::isfinite(milliseconds) || milliseconds <= 0.0F) {
    return 0U;
  }
  return static_cast<std::uint64_t>(
    std::llround(static_cast<double>(milliseconds) * 1.0e6));
}

}  // namespace

class TensorRtLaneBackend::Impl
{
public:
  Impl(
    const std::string & model_path,
    const std::string & requested_engine_cache_path,
    const std::string & engine_precision,
    const int input_width,
    const int input_height,
    const std::size_t workspace_size_bytes,
    const float mask_threshold,
    const float overlay_alpha,
    const bool export_labels,
    const float stop_line_mask_threshold,
    std::function<void(const std::string &)> progress)
  : progress_(std::move(progress)),
    model_path_(model_path),
    engine_precision_(engine_precision),
    engine_cache_path_(requested_engine_cache_path.empty() ?
      model_path + ".trt" + std::to_string(NV_TENSORRT_MAJOR) +
      "." + engine_precision + ".engine" : requested_engine_cache_path),
    input_width_(input_width),
    input_height_(input_height),
    pixel_count_(
      static_cast<std::size_t>(input_width) *
      static_cast<std::size_t>(input_height)),
    image_byte_count_(pixel_count_ * 3U),
    input_element_count_(pixel_count_ * 3U),
    mask_threshold_(mask_threshold),
    stop_line_mask_threshold_(stop_line_mask_threshold == -1.0F ? mask_threshold : stop_line_mask_threshold),
    overlay_alpha_(overlay_alpha),
    export_labels_(export_labels)
  {
    if (engine_precision_ != "fp32" && engine_precision_ != "fp16" &&
      engine_precision_ != "int8")
    {
      throw std::invalid_argument("TensorRT engine_precision must be fp32, fp16 or int8");
    }
    if (input_width <= 0 || input_height <= 0) {
      throw std::invalid_argument("TensorRT input dimensions must be positive");
    }
    if (workspace_size_bytes == 0U) {
      throw std::invalid_argument("TensorRT workspace size must be positive");
    }
    if (!std::isfinite(mask_threshold) ||
      mask_threshold < 0.0F || mask_threshold > 1.0F)
    {
      throw std::invalid_argument("mask_threshold must be in [0,1]");
    }
    if (!std::isfinite(stop_line_mask_threshold_) ||
      stop_line_mask_threshold_ < 0.0F || stop_line_mask_threshold_ > 1.0F)
    {throw std::invalid_argument("stop_line_mask_threshold must be -1 (inherit) or in [0,1]");}
    if (!std::isfinite(overlay_alpha) ||
      overlay_alpha < 0.0F || overlay_alpha > 1.0F)
    {
      throw std::invalid_argument("overlay_alpha must be in [0,1]");
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
    inspect_engine();

    host_bgr_.allocate(image_byte_count_);
    host_preview_bgr_.allocate(image_byte_count_);
    device_bgr_.allocate(image_byte_count_);
    device_input_.allocate(input_element_count_ * sizeof(float));
    device_logits_.allocate(output_element_count_ * sizeof(float));
    device_preview_bgr_.allocate(image_byte_count_);
    if (export_labels_) {
      host_labels_.allocate(pixel_count_ * 2U);
      device_labels_.allocate(pixel_count_ * 2U);
    }
    configure_execution_bindings();
  }

  LaneInferenceTiming infer_bgr(
    const std::uint8_t * bgr,
    const std::size_t data_size,
    const std::size_t source_stride)
  {
    if (bgr == nullptr) {
      throw std::invalid_argument("BGR input must not be null");
    }
    const std::size_t row_bytes =
      static_cast<std::size_t>(input_width_) * 3U;
    if (source_stride < row_bytes ||
      data_size < source_stride * static_cast<std::size_t>(input_height_))
    {
      throw std::invalid_argument("BGR input buffer is undersized");
    }

    auto * pinned_input = static_cast<std::uint8_t *>(host_bgr_.get());
    if (source_stride == row_bytes) {
      std::memcpy(pinned_input, bgr, image_byte_count_);
    } else {
      for (int row = 0; row < input_height_; ++row) {
        std::memcpy(
          pinned_input + static_cast<std::size_t>(row) * row_bytes,
          bgr + static_cast<std::size_t>(row) * source_stride,
          row_bytes);
      }
    }

    check_cuda(
      cudaEventRecord(preprocessing_started_.get(), stream_.get()),
      "cudaEventRecord(preprocess start)");
    check_cuda(
      cudaMemcpyAsync(
        device_bgr_.get(), host_bgr_.get(), image_byte_count_,
        cudaMemcpyHostToDevice, stream_.get()),
      "cudaMemcpyAsync(BGR host to device)");
    check_cuda(
      launch_bgr_to_rgb_nchw(
        static_cast<const std::uint8_t *>(device_bgr_.get()),
        static_cast<float *>(device_input_.get()),
        input_width_, input_height_, stream_.get()),
      "launch BGR to RGB NCHW kernel");
    check_cuda(
      cudaEventRecord(preprocessing_finished_.get(), stream_.get()),
      "cudaEventRecord(preprocess finish)");

    return finish_inference(
      static_cast<const std::uint8_t *>(device_bgr_.get()));
  }

  LaneInferenceTiming infer_device_bgr(
    const std::uint8_t * device_bgr,
    const std::size_t device_stride)
  {
    const std::size_t row_bytes =
      static_cast<std::size_t>(input_width_) * 3U;
    if (device_bgr == nullptr) {
      throw std::invalid_argument("CUDA BGR input must not be null");
    }
    if (device_stride != row_bytes) {
      throw std::invalid_argument(
              "CUDA BGR input must be tightly packed for direct inference");
    }
    check_cuda(
      cudaEventRecord(preprocessing_started_.get(), stream_.get()),
      "cudaEventRecord(direct preprocess start)");
    check_cuda(
      launch_bgr_to_rgb_nchw(
        device_bgr,
        static_cast<float *>(device_input_.get()),
        input_width_, input_height_, stream_.get()),
      "launch direct BGR to RGB NCHW kernel");
    check_cuda(
      cudaEventRecord(preprocessing_finished_.get(), stream_.get()),
      "cudaEventRecord(direct preprocess finish)");
    return finish_inference(device_bgr);
  }

  const std::uint8_t * preview_bgr_data() const noexcept
  {
    return static_cast<const std::uint8_t *>(host_preview_bgr_.get());
  }

  const std::uint8_t * label_data() const noexcept
  {
    return static_cast<const std::uint8_t *>(host_labels_.get());
  }

  const std::uint8_t * stop_line_mask_data() const noexcept
  {
    return export_labels_ ? static_cast<const std::uint8_t *>(host_labels_.get()) + pixel_count_ : nullptr;
  }

  int input_width() const noexcept
  {
    return input_width_;
  }

  int input_height() const noexcept
  {
    return input_height_;
  }

  const std::string & engine_cache_path() const noexcept
  {
    return engine_cache_path_;
  }

private:
  LaneInferenceTiming finish_inference(const std::uint8_t * device_bgr)
  {
    bool enqueued = false;
#if NV_TENSORRT_MAJOR >= 10
    enqueued = context_->enqueueV3(stream_.get());
#else
    enqueued = context_->enqueueV2(bindings_.data(), stream_.get(), nullptr);
#endif
    if (!enqueued) {
      static_cast<void>(cudaStreamSynchronize(stream_.get()));
      throw std::runtime_error("TensorRT inference enqueue failed");
    }
    check_cuda(
      cudaEventRecord(execution_finished_.get(), stream_.get()),
      "cudaEventRecord(inference finish)");

    if (export_labels_) {
      check_cuda(cudaEventRecord(label_export_started_.get(), stream_.get()), "label export start");
      check_cuda(
        launch_lane_labels(static_cast<const float *>(device_logits_.get()),
          static_cast<std::uint8_t *>(device_labels_.get()), input_width_, input_height_,
          mask_threshold_, stop_line_mask_threshold_, stream_.get()), "extract lane labels");
      check_cuda(cudaMemcpyAsync(host_labels_.get(), device_labels_.get(), pixel_count_ * 2U,
          cudaMemcpyDeviceToHost, stream_.get()), "copy lane labels to host");
      check_cuda(cudaEventRecord(label_export_finished_.get(), stream_.get()), "label export finish");
    }
    check_cuda(
      cudaEventRecord(postprocessing_started_.get(), stream_.get()),
      "cudaEventRecord(postprocess start)");
    // Connected mode renders from CPU labels/results. This raw camera overlay
    // would be unused, including in result-only and headless autonomous driving.
    if (!export_labels_) {
      check_cuda(
        launch_lane_overlay(
          device_bgr,
          static_cast<const float *>(device_logits_.get()),
          static_cast<std::uint8_t *>(device_preview_bgr_.get()),
          input_width_, input_height_, mask_threshold_, stop_line_mask_threshold_, overlay_alpha_,
          stream_.get()),
        "launch lane overlay kernel");
      check_cuda(
        cudaMemcpyAsync(
          host_preview_bgr_.get(), device_preview_bgr_.get(), image_byte_count_,
          cudaMemcpyDeviceToHost, stream_.get()),
        "cudaMemcpyAsync(preview device to host)");
    }
    check_cuda(
      cudaEventRecord(postprocessing_finished_.get(), stream_.get()),
      "cudaEventRecord(postprocess finish)");
    check_cuda(
      cudaEventSynchronize(postprocessing_finished_.get()),
      "cudaEventSynchronize(postprocess finish)");

    return LaneInferenceTiming{
      elapsed_cuda_nanoseconds(
        preprocessing_started_.get(), preprocessing_finished_.get()),
      elapsed_cuda_nanoseconds(
        preprocessing_finished_.get(), execution_finished_.get()),
      0U,
      elapsed_cuda_nanoseconds(
        postprocessing_started_.get(), postprocessing_finished_.get()),
      export_labels_ ? elapsed_cuda_nanoseconds(
        label_export_started_.get(), label_export_finished_.get()) : 0U};
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
          try {
            // Reject stale two-channel caches before binding three-channel buffers.
            inspect_engine();
            std::cerr << "[line_detactor TensorRT] Loaded " << engine_precision_
                      << " engine cache: "
                      << engine_cache_path_ << '\n';
            return;
          } catch (const std::exception & error) {
            std::cerr << "[line_detactor TensorRT] Cache I/O mismatch: " << error.what() << '\n';
            engine_.reset();
          }
        }
        std::cerr << "[line_detactor TensorRT] Incompatible engine cache; "
                  << "rebuilding from ONNX.\n";
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
      has_dequantize = has_dequantize || type == nvinfer1::LayerType::kDEQUANTIZE;
    }
    if (engine_precision_ == "int8" && (!has_quantize || !has_dequantize)) {
      throw std::runtime_error(
              "INT8 mode requires an explicit Q/DQ ONNX model; run "
              "tools/quantize_int8_ptq.py first");
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
        throw std::runtime_error("TensorRT device does not report fast FP16 support");
      }
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      config->clearFlag(nvinfer1::BuilderFlag::kINT8);
    } else if (engine_precision_ == "int8") {
      if (!builder->platformHasFastInt8()) {
        throw std::runtime_error("TensorRT device does not report fast INT8 support");
      }
      // TensorRT 8.x requires kINT8 for explicit Q/DQ. The Q/DQ nodes, not a
      // calibrator or per-layer constraints, define every precision boundary.
      config->clearFlag(nvinfer1::BuilderFlag::kFP16);
      config->setFlag(nvinfer1::BuilderFlag::kINT8);
    } else {
      config->clearFlag(nvinfer1::BuilderFlag::kFP16);
      config->clearFlag(nvinfer1::BuilderFlag::kINT8);
    }
    config->clearFlag(nvinfer1::BuilderFlag::kTF32);

    std::cerr << "[line_detactor TensorRT] Building " << engine_precision_
              << " engine from "
              << model_path_ << ". First launch can take several minutes.\n";
    stage("Building TensorRT engine (GPU optimization; may take minutes)");
    auto serialized = TensorRtUniquePtr<nvinfer1::IHostMemory>(
      builder->buildSerializedNetwork(*network, *config));
    if (!serialized) {
      throw std::runtime_error("TensorRT " + engine_precision_ + " engine build failed");
    }
    stage("Deserializing newly built engine");
    engine_.reset(runtime_->deserializeCudaEngine(
        serialized->data(), serialized->size()));
    if (!engine_) {
      throw std::runtime_error(
              "New TensorRT " + engine_precision_ + " engine deserialization failed");
    }
    stage("Writing engine cache");
    write_engine_cache(
      engine_cache_path_, serialized->data(), serialized->size());
    std::cerr << "[line_detactor TensorRT] " << engine_precision_
              << " engine ready; cache="
              << engine_cache_path_ << '\n';
  }

  void inspect_engine()
  {
#if NV_TENSORRT_MAJOR >= 10
    input_name_.clear();
    output_name_.clear();
#else
    input_binding_index_ = -1;
    output_binding_index_ = -1;
#endif
    nvinfer1::Dims input_dimensions{};
    nvinfer1::Dims output_dimensions{};
    nvinfer1::DataType input_type{};
    nvinfer1::DataType output_type{};

#if NV_TENSORRT_MAJOR >= 10
    if (engine_->getNbIOTensors() != 2) {
      throw std::runtime_error(
              "Lane TensorRT engine must contain one input and one output");
    }
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char * name = engine_->getIOTensorName(index);
      if (name == nullptr) {
        throw std::runtime_error("TensorRT returned an unnamed I/O tensor");
      }
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        input_name_ = name;
        input_dimensions = engine_->getTensorShape(name);
        input_type = engine_->getTensorDataType(name);
      } else {
        output_name_ = name;
        output_dimensions = engine_->getTensorShape(name);
        output_type = engine_->getTensorDataType(name);
      }
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("TensorRT could not identify lane model I/O");
    }
#else
    if (engine_->getNbBindings() != 2) {
      throw std::runtime_error(
              "Lane TensorRT engine must contain one input and one output");
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
      throw std::runtime_error("TensorRT could not identify lane model I/O");
    }
#endif

    if (input_type != nvinfer1::DataType::kFLOAT ||
      output_type != nvinfer1::DataType::kFLOAT)
    {
      throw std::runtime_error("Lane TensorRT engine I/O must be FP32");
    }
    if (input_dimensions.nbDims != 4 ||
      input_dimensions.d[0] != 1 || input_dimensions.d[1] != 3 ||
      input_dimensions.d[2] != input_height_ ||
      input_dimensions.d[3] != input_width_ ||
      dimensions_volume(input_dimensions) != input_element_count_)
    {
      throw std::runtime_error(
              "Unexpected TensorRT input shape " +
              dimensions_string(input_dimensions) + "; expected [1,3," +
              std::to_string(input_height_) + "," +
              std::to_string(input_width_) + "]");
    }
    output_element_count_ = dimensions_volume(output_dimensions);
    if (output_dimensions.nbDims != 4 ||
      output_dimensions.d[0] != 1 || output_dimensions.d[1] != 3 ||
      output_dimensions.d[2] != input_height_ ||
      output_dimensions.d[3] != input_width_ ||
      output_element_count_ != pixel_count_ * 3U)
    {
      throw std::runtime_error(
              "Unexpected TensorRT output shape " +
              dimensions_string(output_dimensions) + "; expected [1,3," +
              std::to_string(input_height_) + "," +
              std::to_string(input_width_) + "]");
    }
  }

  void configure_execution_bindings()
  {
#if NV_TENSORRT_MAJOR >= 10
    if (!context_->setTensorAddress(input_name_.c_str(), device_input_.get()) ||
      !context_->setTensorAddress(output_name_.c_str(), device_logits_.get()))
    {
      throw std::runtime_error("TensorRT could not bind lane device buffers");
    }
#else
    bindings_[static_cast<std::size_t>(input_binding_index_)] =
      device_input_.get();
    bindings_[static_cast<std::size_t>(output_binding_index_)] =
      device_logits_.get();
#endif
  }

  std::function<void(const std::string &)> progress_;
  void stage(const std::string & text) {if (progress_) {progress_(text);}}
  std::string model_path_;
  std::string engine_precision_;
  std::string engine_cache_path_;
  int input_width_{0};
  int input_height_{0};
  std::size_t pixel_count_{0U};
  std::size_t image_byte_count_{0U};
  std::size_t input_element_count_{0U};
  std::size_t output_element_count_{0U};
  float mask_threshold_{0.5F};
  float stop_line_mask_threshold_{0.5F};
  float overlay_alpha_{0.75F};
  bool export_labels_{false};

  TensorRtLogger logger_;
  TensorRtUniquePtr<nvinfer1::IRuntime> runtime_;
  TensorRtUniquePtr<nvinfer1::ICudaEngine> engine_;
  TensorRtUniquePtr<nvinfer1::IExecutionContext> context_;
  CudaStream stream_;
  PinnedHostBuffer host_bgr_;
  PinnedHostBuffer host_preview_bgr_;
  PinnedHostBuffer host_labels_;
  CudaBuffer device_labels_;
  CudaEvent label_export_started_;
  CudaEvent label_export_finished_;
  CudaBuffer device_bgr_;
  CudaBuffer device_input_;
  CudaBuffer device_logits_;
  CudaBuffer device_preview_bgr_;
  CudaEvent preprocessing_started_;
  CudaEvent preprocessing_finished_;
  CudaEvent execution_finished_;
  CudaEvent postprocessing_started_;
  CudaEvent postprocessing_finished_;

#if NV_TENSORRT_MAJOR >= 10
  std::string input_name_;
  std::string output_name_;
#else
  int input_binding_index_{-1};
  int output_binding_index_{-1};
  std::vector<void *> bindings_;
#endif
};

TensorRtLaneBackend::TensorRtLaneBackend(
  const std::string & model_path,
  const std::string & engine_cache_path,
  const std::string & engine_precision,
  const int input_width,
  const int input_height,
  const std::size_t workspace_size_bytes,
  const float mask_threshold,
  const float overlay_alpha,
  const bool export_labels,
  const float stop_line_mask_threshold,
  std::function<void(const std::string &)> progress)
: impl_(std::make_unique<Impl>(
    model_path, engine_cache_path, engine_precision, input_width, input_height,
    workspace_size_bytes, mask_threshold, overlay_alpha, export_labels, stop_line_mask_threshold, std::move(progress)))
{
}

TensorRtLaneBackend::~TensorRtLaneBackend() = default;
TensorRtLaneBackend::TensorRtLaneBackend(TensorRtLaneBackend &&) noexcept =
  default;
TensorRtLaneBackend & TensorRtLaneBackend::operator=(
  TensorRtLaneBackend &&) noexcept = default;

LaneInferenceTiming TensorRtLaneBackend::infer_bgr(
  const std::uint8_t * bgr,
  const std::size_t data_size,
  const std::size_t source_stride)
{
  return impl_->infer_bgr(bgr, data_size, source_stride);
}

LaneInferenceTiming TensorRtLaneBackend::infer_device_bgr(
  const std::uint8_t * device_bgr,
  const std::size_t device_stride)
{
  return impl_->infer_device_bgr(device_bgr, device_stride);
}

const std::uint8_t * TensorRtLaneBackend::preview_bgr_data() const noexcept
{
  return impl_->preview_bgr_data();
}

const std::uint8_t * TensorRtLaneBackend::label_data() const noexcept
{
  return impl_->label_data();
}

const std::uint8_t * TensorRtLaneBackend::stop_line_mask_data() const noexcept
{
  return impl_->stop_line_mask_data();
}

int TensorRtLaneBackend::input_width() const noexcept
{
  return impl_->input_width();
}

int TensorRtLaneBackend::input_height() const noexcept
{
  return impl_->input_height();
}

const std::string & TensorRtLaneBackend::engine_cache_path() const noexcept
{
  return impl_->engine_cache_path();
}

}  // namespace line_detactor
