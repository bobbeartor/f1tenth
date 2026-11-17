#include "multiclass_yolo/tensorrt_yolox_backend.hpp"
#include <stdexcept>
namespace multiclass_yolo {
class TensorRtYoloxBackend::Impl {};
TensorRtYoloxBackend::TensorRtYoloxBackend(const std::string &, const std::string &,
 const std::string &, int, int, std::size_t, std::function<void(const std::string &)>) {
 throw std::runtime_error("Rebuild with -DMULTICLASS_YOLO_TENSORRT=ON to use TensorRT/NV12 CUDA");
}
TensorRtYoloxBackend::~TensorRtYoloxBackend() = default;
TensorRtYoloxBackend::TensorRtYoloxBackend(TensorRtYoloxBackend &&) noexcept = default;
TensorRtYoloxBackend & TensorRtYoloxBackend::operator=(TensorRtYoloxBackend &&) noexcept = default;
TensorRtInferenceTiming TensorRtYoloxBackend::infer(const float *, std::size_t, float *, std::size_t) {throw std::logic_error("TensorRT disabled");}
TensorRtInferenceTiming TensorRtYoloxBackend::infer_nv12(const std::uint8_t *, std::size_t, std::size_t, int, int, int, int, int, int, float *, std::size_t) {throw std::logic_error("TensorRT disabled");}
int TensorRtYoloxBackend::output_row_count() const noexcept { return 0; }
int TensorRtYoloxBackend::output_column_count() const noexcept { return 0; }
const std::string & TensorRtYoloxBackend::engine_cache_path() const noexcept {static const std::string empty; return empty;}
}
