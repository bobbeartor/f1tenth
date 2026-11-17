#ifndef MANUAL_CAPTURE__CAPTURE_WRITER_HPP_
#define MANUAL_CAPTURE__CAPTURE_WRITER_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "manual_capture/capture_logic.hpp"
#include "opencv2/core.hpp"

namespace manual_capture
{

struct CaptureImage
{
  View view;
  cv::Mat image;  // GRAY8 or BGR8, owned copy
};

struct CaptureEvent
{
  std::uint64_t index{0U};
  std::vector<CaptureImage> images;
  std::string meta_row;  // CSV row without newline, written after the images
};

// Writes JPEG files on a background thread so the GUI never blocks on disk.
// Layout: <session>/<view_key>/<index>.jpg and <session>/meta.csv.
class CaptureWriter
{
public:
  CaptureWriter(std::filesystem::path session_directory, std::string meta_header,
    std::size_t maximum_queued_events);
  ~CaptureWriter();

  CaptureWriter(const CaptureWriter &) = delete;
  CaptureWriter & operator=(const CaptureWriter &) = delete;

  void setJpegQuality(int quality);
  // False when the queue is full; the event is dropped and counted.
  bool enqueue(CaptureEvent event);
  void flush();

  const std::filesystem::path & sessionDirectory() const {return session_directory_;}
  std::uint64_t written(View view) const {return written_[index(view)].load();}
  std::uint64_t writtenTotal() const;
  std::uint64_t dropped() const {return dropped_.load();}
  std::uint64_t failed() const {return failed_.load();}
  std::size_t queued() const;
  std::size_t capacity() const {return maximum_queued_events_;}

private:
  void run();
  void write(const CaptureEvent & event);

  std::filesystem::path session_directory_;
  std::string meta_header_;
  std::size_t maximum_queued_events_;
  std::atomic<int> jpeg_quality_{95};
  bool meta_header_written_{false};

  mutable std::mutex mutex_;
  std::condition_variable wake_;
  std::condition_variable idle_;
  std::deque<CaptureEvent> queue_;
  bool busy_{false};
  bool stop_{false};
  std::thread thread_;

  std::array<std::atomic<std::uint64_t>, kViewCount> written_{};
  std::atomic<std::uint64_t> dropped_{0U};
  std::atomic<std::uint64_t> failed_{0U};
};

}  // namespace manual_capture

#endif  // MANUAL_CAPTURE__CAPTURE_WRITER_HPP_
