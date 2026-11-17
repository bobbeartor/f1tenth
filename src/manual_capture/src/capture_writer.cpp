#include "manual_capture/capture_writer.hpp"

#include <algorithm>
#include <fstream>
#include <utility>

#include "opencv2/imgcodecs.hpp"

namespace manual_capture
{

CaptureWriter::CaptureWriter(
  std::filesystem::path session_directory, std::string meta_header,
  const std::size_t maximum_queued_events)
: session_directory_(std::move(session_directory)),
  meta_header_(std::move(meta_header)),
  maximum_queued_events_(std::max<std::size_t>(1U, maximum_queued_events))
{
  for (auto & count : written_) {
    count.store(0U);
  }
  std::filesystem::create_directories(session_directory_);
  thread_ = std::thread(&CaptureWriter::run, this);
}

CaptureWriter::~CaptureWriter()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void CaptureWriter::setJpegQuality(const int quality)
{
  jpeg_quality_.store(std::clamp(quality, 10, 100));
}

bool CaptureWriter::enqueue(CaptureEvent event)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= maximum_queued_events_) {
      dropped_.fetch_add(1U);
      return false;
    }
    queue_.push_back(std::move(event));
  }
  wake_.notify_one();
  return true;
}

void CaptureWriter::flush()
{
  std::unique_lock<std::mutex> lock(mutex_);
  idle_.wait(lock, [this]() {return queue_.empty() && !busy_;});
}

std::uint64_t CaptureWriter::writtenTotal() const
{
  std::uint64_t total = 0U;
  for (const auto & count : written_) {
    total += count.load();
  }
  return total;
}

std::size_t CaptureWriter::queued() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

void CaptureWriter::run()
{
  while (true) {
    CaptureEvent event;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this]() {return stop_ || !queue_.empty();});
      if (queue_.empty()) {
        return;  // stop requested and everything written
      }
      event = std::move(queue_.front());
      queue_.pop_front();
      busy_ = true;
    }
    write(event);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      busy_ = false;
    }
    idle_.notify_all();
  }
}

void CaptureWriter::write(const CaptureEvent & event)
{
  const std::vector<int> parameters{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_.load()};
  for (const auto & image : event.images) {
    try {
      const auto directory = session_directory_ / viewKey(image.view);
      std::filesystem::create_directories(directory);
      const auto path = directory / frameFileName(event.index);
      if (!image.image.empty() && cv::imwrite(path.string(), image.image, parameters)) {
        written_[index(image.view)].fetch_add(1U);
      } else {
        failed_.fetch_add(1U);
      }
    } catch (...) {
      failed_.fetch_add(1U);
    }
  }
  if (event.meta_row.empty()) {
    return;
  }
  std::ofstream meta(session_directory_ / "meta.csv", std::ios::app);
  if (!meta_header_written_) {
    meta << meta_header_ << '\n';
    meta_header_written_ = true;
  }
  meta << event.meta_row << '\n';
}

}  // namespace manual_capture
