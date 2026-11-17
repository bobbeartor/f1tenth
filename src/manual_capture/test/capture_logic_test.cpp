#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "manual_capture/capture_logic.hpp"
#include "manual_capture/capture_writer.hpp"

using namespace manual_capture;

TEST(ViewSelection, StartsWithColorOnly)
{
  ViewSelection selection;
  EXPECT_TRUE(selection.preview(View::RGB));
  EXPECT_FALSE(selection.preview(View::LEFT));
  EXPECT_FALSE(selection.preview(View::CENTER));
  ASSERT_EQ(selection.captureTargets().size(), 1U);
  EXPECT_EQ(selection.captureTargets()[0], View::RGB);
  EXPECT_FALSE(dotProjectorRequired(selection));
}

TEST(ViewSelection, CheckFollowsPreviewAndCanBeCleared)
{
  ViewSelection selection;
  EXPECT_FALSE(selection.setChecked(View::LEFT, true));  // preview off
  selection.setPreview(View::LEFT, true);
  EXPECT_TRUE(selection.checked(View::LEFT));            // auto checked
  EXPECT_TRUE(selection.setChecked(View::LEFT, false));
  EXPECT_EQ(selection.captureTargets().size(), 1U);
  selection.setPreview(View::LEFT, false);
  EXPECT_FALSE(selection.checked(View::LEFT));
}

TEST(ViewSelection, LeftOnlyLocksRightAndCenter)
{
  ViewSelection selection;
  selection.setPreview(View::RIGHT, true);
  selection.setPreview(View::CENTER, true);
  EXPECT_TRUE(dotProjectorRequired(selection));
  EXPECT_FALSE(selection.setPreview(View::LEFT_BEV, true));  // locked in LEFT_RIGHT

  selection.setMode(StereoMode::LEFT_ONLY);
  EXPECT_FALSE(selection.preview(View::RIGHT));
  EXPECT_FALSE(selection.preview(View::CENTER));
  EXPECT_FALSE(selection.setPreview(View::RIGHT, true));
  EXPECT_TRUE(selection.setPreview(View::LEFT_BEV, true));
  EXPECT_FALSE(dotProjectorRequired(selection));
  for (const auto view : selection.captureTargets()) {
    EXPECT_NE(view, View::RIGHT);
    EXPECT_NE(view, View::CENTER);
  }
}

TEST(CaptureController, SingleModeCapturesOnEveryTrigger)
{
  CaptureController controller;
  EXPECT_EQ(controller.trigger(), CaptureAction::CAPTURE_ONCE);
  EXPECT_EQ(controller.trigger(), CaptureAction::CAPTURE_ONCE);
  EXPECT_FALSE(controller.recording());
  EXPECT_TRUE(controller.singleViewCaptureAllowed());
}

TEST(CaptureController, AutoModeToggleForbidsPerViewCapture)
{
  CaptureController controller;
  ASSERT_TRUE(controller.setMode(CaptureMode::AUTO));
  EXPECT_EQ(controller.trigger(), CaptureAction::START_AUTO);
  EXPECT_TRUE(controller.recording());
  EXPECT_FALSE(controller.singleViewCaptureAllowed());
  EXPECT_FALSE(controller.setMode(CaptureMode::SINGLE));  // locked while recording
  EXPECT_EQ(controller.start(), CaptureAction::NONE);
  EXPECT_EQ(controller.trigger(), CaptureAction::STOP_AUTO);
  EXPECT_TRUE(controller.singleViewCaptureAllowed());
  EXPECT_EQ(controller.stop(), CaptureAction::NONE);
}

TEST(ButtonEdge, FiresOnlyOnRisingEdge)
{
  ButtonEdge edge;
  EXPECT_FALSE(edge.update(true));  // held at startup
  EXPECT_FALSE(edge.update(true));
  EXPECT_FALSE(edge.update(false));
  EXPECT_TRUE(edge.update(true));
  EXPECT_FALSE(edge.update(true));
}

TEST(Refit, RejectedWhileMoving)
{
  EXPECT_FALSE(decideRefit(3000, 0.1, 100, 1.0).allowed);
  EXPECT_FALSE(decideRefit(-500, 0.1, 100, 1.0).allowed);
  EXPECT_TRUE(decideRefit(40, 0.1, 100, 1.0).allowed);
  EXPECT_TRUE(decideRefit(std::nullopt, 0.0, 100, 1.0).allowed);  // no telemetry
  EXPECT_TRUE(decideRefit(3000, 5.0, 100, 1.0).allowed);          // stale telemetry
}

TEST(Helpers, ExposureNamesAndScheduler)
{
  EXPECT_EQ(maximumExposureUs(30.0), 33333);
  EXPECT_EQ(frameFileName(42), "000042.jpg");
  EXPECT_EQ(sessionDirectoryName(0).rfind("session_", 0), 0U);

  IntervalScheduler scheduler;
  scheduler.reset(10.0, 5.0);
  EXPECT_TRUE(scheduler.due(10.0));
  EXPECT_FALSE(scheduler.due(10.1));
  EXPECT_TRUE(scheduler.due(10.2));
  EXPECT_TRUE(scheduler.due(15.0));   // after a stall: one capture, no burst
  EXPECT_FALSE(scheduler.due(15.1));
}

TEST(CaptureWriter, WritesJpegsPerViewAndMeta)
{
  const auto session = std::filesystem::temp_directory_path() / "manual_capture_writer_test";
  std::filesystem::remove_all(session);
  {
    CaptureWriter writer(session, "event,views", 4);
    CaptureEvent event;
    event.index = 7;
    event.images.push_back({View::RGB, cv::Mat(8, 8, CV_8UC3, cv::Scalar(10, 20, 30))});
    event.images.push_back({View::LEFT, cv::Mat(8, 8, CV_8UC1, cv::Scalar(99))});
    event.meta_row = "7,rgb;stereo_left";
    ASSERT_TRUE(writer.enqueue(std::move(event)));
    writer.flush();
    EXPECT_EQ(writer.written(View::RGB), 1U);
    EXPECT_EQ(writer.written(View::LEFT), 1U);
    EXPECT_EQ(writer.writtenTotal(), 2U);
  }
  EXPECT_TRUE(std::filesystem::exists(session / "rgb" / "000007.jpg"));
  EXPECT_TRUE(std::filesystem::exists(session / "stereo_left" / "000007.jpg"));
  std::ifstream meta(session / "meta.csv");
  std::string header, row;
  std::getline(meta, header);
  std::getline(meta, row);
  EXPECT_EQ(header, "event,views");
  EXPECT_EQ(row, "7,rgb;stereo_left");
  std::filesystem::remove_all(session);
}
