#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace
{

namespace fs = std::filesystem;

struct LaneMaskParameters
{
  int work_width{960};
  int blur_kernel{5};

  // White top-hat: retain narrow bright structures and suppress broad bright
  // regions. The kernel should be wider than the expected lane marking.
  int tophat_kernel{31};
  double tophat_threshold{60.0};

  // Lane markings are expected to be surrounded by a sufficiently dark road.
  bool dark_gate_enabled{true};
  double dark_level{70.0};
  double minimum_dark_ratio{0.12};
  int dark_window{25};

  // Connected-component shape filtering.
  int minimum_area{200};
  int blob_area{1200};
  double maximum_blob_aspect{2.5};
  double minimum_blob_fill{0.45};
  bool sliver_filter_enabled{true};
  int maximum_sliver_height{12};
  double sliver_top_ratio{0.35};
};

struct LaneMaskResult
{
  cv::Mat mask;
  int kept_components{0};
  std::int64_t lane_pixels{0};
  double lane_percent{0.0};
};

struct ProgramOptions
{
  fs::path input_path;
  fs::path result_directory;
  int work_width{960};
  bool gui_enabled{true};
};

int oddKernel(const double value, const int minimum)
{
  int result = std::max(static_cast<int>(std::lround(value)), minimum);
  if (result % 2 == 0) {
    ++result;
  }
  return result;
}

cv::Mat readImage(const fs::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("Could not open input image: " + path.string());
  }

  const std::vector<unsigned char> encoded{
    std::istreambuf_iterator<char>(stream),
    std::istreambuf_iterator<char>()};
  if (encoded.empty()) {
    throw std::runtime_error("Input image is empty: " + path.string());
  }

  const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
  if (image.empty()) {
    throw std::runtime_error("Could not decode input image: " + path.string());
  }
  return image;
}

void writeImage(const fs::path & path, const cv::Mat & image)
{
  if (path.has_parent_path()) {
    fs::create_directories(path.parent_path());
  }

  std::vector<unsigned char> encoded;
  if (!cv::imencode(path.extension().string(), image, encoded)) {
    throw std::runtime_error("Could not encode result image: " + path.string());
  }

  std::ofstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("Could not create result image: " + path.string());
  }
  stream.write(
    reinterpret_cast<const char *>(encoded.data()),
    static_cast<std::streamsize>(encoded.size()));
  if (!stream) {
    throw std::runtime_error("Could not write result image: " + path.string());
  }
}

LaneMaskResult detectLaneMask(
  const cv::Mat & input_bgr,
  const LaneMaskParameters & parameters)
{
  if (input_bgr.empty()) {
    throw std::invalid_argument("Input image must not be empty");
  }

  cv::Mat gray;
  if (input_bgr.channels() == 3) {
    cv::cvtColor(input_bgr, gray, cv::COLOR_BGR2GRAY);
  } else if (input_bgr.channels() == 4) {
    cv::cvtColor(input_bgr, gray, cv::COLOR_BGRA2GRAY);
  } else if (input_bgr.channels() == 1) {
    gray = input_bgr;
  } else {
    throw std::invalid_argument("Input image must have 1, 3, or 4 channels");
  }

  cv::Mat work;
  const bool shrink =
    parameters.work_width > 0 && parameters.work_width < gray.cols;
  if (shrink) {
    const double scale =
      static_cast<double>(parameters.work_width) /
      static_cast<double>(gray.cols);
    cv::resize(
      gray, work, cv::Size(), scale, scale, cv::INTER_AREA);
  } else {
    work = gray;
  }

  // Spatial parameters were tuned at 960 pixels wide. Scale them so arbitrary
  // input resolutions retain approximately the same physical behavior.
  const double resolution_scale =
    static_cast<double>(work.cols) / 960.0;
  const int tophat_kernel = oddKernel(
    parameters.tophat_kernel * resolution_scale, 3);
  const int dark_window = oddKernel(
    parameters.dark_window * resolution_scale, 3);
  const int minimum_area = std::max(
    10,
    static_cast<int>(std::lround(
      parameters.minimum_area *
      resolution_scale * resolution_scale)));
  const int blob_area = std::max(
    50,
    static_cast<int>(std::lround(
      parameters.blob_area *
      resolution_scale * resolution_scale)));
  const int maximum_sliver_height = std::max(
    2,
    static_cast<int>(std::lround(
      parameters.maximum_sliver_height * resolution_scale)));

  cv::Mat blurred;
  if (parameters.blur_kernel > 1) {
    const int blur_kernel = oddKernel(parameters.blur_kernel, 1);
    cv::GaussianBlur(
      work, blurred, cv::Size(blur_kernel, blur_kernel), 0.0);
  } else {
    blurred = work;
  }

  const cv::Mat tophat_element = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(tophat_kernel, tophat_kernel));
  cv::Mat tophat;
  cv::morphologyEx(
    blurred, tophat, cv::MORPH_TOPHAT, tophat_element);

  cv::Mat candidate;
  cv::threshold(
    tophat,
    candidate,
    parameters.tophat_threshold,
    255.0,
    cv::THRESH_BINARY);

  if (parameters.dark_gate_enabled) {
    cv::Mat dark;
    cv::threshold(
      blurred,
      dark,
      parameters.dark_level,
      1.0,
      cv::THRESH_BINARY_INV);

    cv::Mat dark_float;
    dark.convertTo(dark_float, CV_32F);
    cv::Mat local_dark_ratio;
    cv::boxFilter(
      dark_float,
      local_dark_ratio,
      CV_32F,
      cv::Size(dark_window, dark_window),
      cv::Point(-1, -1),
      true);

    cv::Mat gate_float;
    cv::threshold(
      local_dark_ratio,
      gate_float,
      parameters.minimum_dark_ratio,
      255.0,
      cv::THRESH_BINARY);
    cv::Mat gate;
    gate_float.convertTo(gate, CV_8U);
    cv::bitwise_and(candidate, gate, candidate);
  }

  cv::Mat labels;
  cv::Mat statistics;
  cv::Mat centroids;
  const int component_count = cv::connectedComponentsWithStats(
    candidate,
    labels,
    statistics,
    centroids,
    8,
    CV_32S);

  // Build a label lookup once, then scan the image once. This avoids the
  // Python implementation's full-frame labels == component scan for every
  // retained component.
  std::vector<unsigned char> keep(
    static_cast<std::size_t>(component_count), 0U);
  int kept_components = 0;
  for (int label = 1; label < component_count; ++label) {
    const int y = statistics.at<int>(label, cv::CC_STAT_TOP);
    const int width = statistics.at<int>(label, cv::CC_STAT_WIDTH);
    const int height = statistics.at<int>(label, cv::CC_STAT_HEIGHT);
    const int area = statistics.at<int>(label, cv::CC_STAT_AREA);
    if (area < minimum_area) {
      continue;
    }

    const double aspect =
      static_cast<double>(std::max(width, height)) /
      static_cast<double>(std::max(1, std::min(width, height)));
    const double fill =
      static_cast<double>(area) /
      static_cast<double>(std::max(1, width * height));
    if (
      area > blob_area &&
      aspect < parameters.maximum_blob_aspect &&
      fill > parameters.minimum_blob_fill)
    {
      continue;
    }

    if (
      parameters.sliver_filter_enabled &&
      width > 3 * height &&
      height <= maximum_sliver_height &&
      static_cast<double>(y) <
      static_cast<double>(candidate.rows) * parameters.sliver_top_ratio)
    {
      continue;
    }

    keep[static_cast<std::size_t>(label)] = 1U;
    ++kept_components;
  }

  cv::Mat filtered = cv::Mat::zeros(candidate.size(), CV_8U);
  for (int row = 0; row < labels.rows; ++row) {
    const int * label_row = labels.ptr<int>(row);
    unsigned char * output_row = filtered.ptr<unsigned char>(row);
    for (int column = 0; column < labels.cols; ++column) {
      const int label = label_row[column];
      output_row[column] =
        keep[static_cast<std::size_t>(label)] != 0U ? 255U : 0U;
    }
  }

  cv::Mat output_mask;
  if (shrink) {
    cv::resize(
      filtered,
      output_mask,
      gray.size(),
      0.0,
      0.0,
      cv::INTER_NEAREST);
  } else {
    output_mask = filtered;
  }

  const std::int64_t lane_pixels = cv::countNonZero(output_mask);
  const double lane_percent =
    100.0 * static_cast<double>(lane_pixels) /
    static_cast<double>(output_mask.total());
  return LaneMaskResult{
    output_mask, kept_components, lane_pixels, lane_percent};
}

cv::Mat createOverlay(const cv::Mat & input_bgr, const cv::Mat & mask)
{
  if (input_bgr.size() != mask.size() || mask.type() != CV_8U) {
    throw std::invalid_argument("Overlay input and mask sizes must match");
  }

  cv::Mat tinted = input_bgr.clone();
  tinted.setTo(cv::Scalar(0, 255, 0), mask);
  cv::Mat overlay;
  cv::addWeighted(input_bgr, 0.5, tinted, 0.5, 0.0, overlay);
  return overlay;
}

cv::Mat fitForDisplay(
  const cv::Mat & image,
  const int maximum_width,
  const int maximum_height)
{
  const double scale = std::min(
    {
      1.0,
      static_cast<double>(maximum_width) /
      static_cast<double>(image.cols),
      static_cast<double>(maximum_height) /
      static_cast<double>(image.rows),
    });
  if (scale >= 1.0) {
    return image;
  }

  cv::Mat resized;
  cv::resize(
    image, resized, cv::Size(), scale, scale, cv::INTER_AREA);
  return resized;
}

void showResults(
  const cv::Mat & input_bgr,
  const cv::Mat & mask,
  const cv::Mat & overlay)
{
  const cv::Mat input_view = fitForDisplay(input_bgr, 1280, 720);

  cv::Mat mask_bgr;
  cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
  const cv::Mat overlay_view = fitForDisplay(overlay, 680, 680);
  const cv::Mat mask_view = fitForDisplay(mask_bgr, 680, 680);
  cv::Mat result_view;
  cv::hconcat(overlay_view, mask_view, result_view);

  cv::namedWindow("Input Sample", cv::WINDOW_NORMAL);
  cv::namedWindow("Lane Mask Result: overlay | mask", cv::WINDOW_NORMAL);
  cv::imshow("Input Sample", input_view);
  cv::imshow("Lane Mask Result: overlay | mask", result_view);

  std::cout << "Press any key in an image window to close.\n";
  cv::waitKey(0);
  cv::destroyAllWindows();
}

void printUsage(const char * executable)
{
  std::cout
    << "Usage:\n  " << executable
    << " <input-image> [--result-dir <directory>]"
    << " [--work-width <pixels>] [--no-gui]\n\n"
    << "Examples:\n  " << executable
    << " sample.jpg\n  " << executable
    << " sample.png --result-dir result --work-width 960\n";
}

ProgramOptions parseArguments(const int argc, char ** argv)
{
  ProgramOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    }
    if (argument == "--no-gui") {
      options.gui_enabled = false;
      continue;
    }
    if (argument == "--result-dir") {
      if (++index >= argc) {
        throw std::invalid_argument("--result-dir requires a directory");
      }
      options.result_directory = fs::u8path(argv[index]);
      continue;
    }
    if (argument == "--work-width") {
      if (++index >= argc) {
        throw std::invalid_argument("--work-width requires a positive integer");
      }
      options.work_width = std::stoi(argv[index]);
      if (options.work_width <= 0) {
        throw std::invalid_argument("--work-width must be positive");
      }
      continue;
    }
    if (!argument.empty() && argument[0] == '-') {
      throw std::invalid_argument("Unknown option: " + argument);
    }
    if (!options.input_path.empty()) {
      throw std::invalid_argument("Only one input image may be specified");
    }
    options.input_path = fs::u8path(argument);
  }

  if (options.input_path.empty()) {
    printUsage(argv[0]);
    throw std::invalid_argument("An input image is required");
  }
  if (options.result_directory.empty()) {
    const fs::path parent = options.input_path.parent_path();
    options.result_directory =
      (parent.empty() ? fs::path(".") : parent) / "result";
  }
  return options;
}

}  // namespace

int main(const int argc, char ** argv)
{
  try {
    const ProgramOptions options = parseArguments(argc, argv);
    LaneMaskParameters parameters;
    parameters.work_width = options.work_width;

    const cv::Mat input = readImage(options.input_path);
    const LaneMaskResult result = detectLaneMask(input, parameters);
    const cv::Mat overlay = createOverlay(input, result.mask);

    const std::string stem = options.input_path.stem().string();
    const fs::path mask_path =
      options.result_directory / (stem + "_mask.png");
    const fs::path overlay_path =
      options.result_directory / (stem + "_overlay.jpg");
    writeImage(mask_path, result.mask);
    writeImage(overlay_path, overlay);

    std::cout
      << "Input: " << options.input_path << " ("
      << input.cols << "x" << input.rows << ")\n"
      << "Kept components: " << result.kept_components << '\n'
      << "Lane pixels: " << result.lane_pixels
      << " (" << result.lane_percent << "%)\n"
      << "Mask: " << mask_path << '\n'
      << "Overlay: " << overlay_path << '\n';

    if (options.gui_enabled) {
      showResults(input, result.mask, overlay);
    }
    return EXIT_SUCCESS;
  } catch (const std::exception & exception) {
    std::cerr << "lane_mask error: " << exception.what() << '\n';
    return EXIT_FAILURE;
  }
}
