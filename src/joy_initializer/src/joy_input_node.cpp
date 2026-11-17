/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <SDL.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy_initializer
{

class JoyInputNode final : public rclcpp::Node
{
public:
  JoyInputNode()
  : rclcpp::Node("joy_input_node")
  {
    device_id_ = declare_parameter<int>("device_id", 0);
    device_name_contains_ =
      declare_parameter<std::string>("device_name_contains", "8BitDo");
    deadzone_ = declare_parameter<double>("deadzone", 0.05);
    autorepeat_rate_hz_ = declare_parameter<double>("autorepeat_rate", 50.0);
    sticky_buttons_ = declare_parameter<bool>("sticky_buttons", false);
    poll_interval_ms_ = declare_parameter<int>("coalesce_interval_ms", 1);
    reconnect_interval_sec_ =
      declare_parameter<double>("reconnect_interval_sec", 1.0);

    if (device_id_ < 0) {
      throw std::invalid_argument("device_id must be non-negative");
    }
    if (!std::isfinite(deadzone_) || deadzone_ < 0.0 || deadzone_ > 0.90) {
      throw std::invalid_argument("deadzone must be between 0.0 and 0.90");
    }
    if (
      !std::isfinite(autorepeat_rate_hz_) || autorepeat_rate_hz_ < 0.0 ||
      autorepeat_rate_hz_ > 1000.0)
    {
      throw std::invalid_argument("autorepeat_rate must be between 0 and 1000 Hz");
    }
    if (poll_interval_ms_ < 1 || poll_interval_ms_ > 1000) {
      throw std::invalid_argument("coalesce_interval_ms must be between 1 and 1000");
    }
    if (
      !std::isfinite(reconnect_interval_sec_) ||
      reconnect_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument("reconnect_interval_sec must be positive");
    }

    unscaled_deadzone_ = 32767.0 * deadzone_;
    stick_scale_ = -1.0 / ((1.0 - deadzone_) * 32767.0);
    autorepeat_period_sec_ = autorepeat_rate_hz_ > 0.0 ?
      1.0 / autorepeat_rate_hz_ : 0.0;

    publisher_ = create_publisher<sensor_msgs::msg::Joy>(
      "joy", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    // The GameController subsystem gives Bluetooth D-input devices one stable
    // logical layout. Haptics and force feedback are intentionally not
    // initialized by this input-only node.
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
      throw std::runtime_error(
              "SDL game-controller input initialization failed: " +
              std::string(SDL_GetError()));
    }
    sdl_initialized_ = true;
    SDL_GameControllerEventState(SDL_ENABLE);
    install8BitDoUltimate2Mappings();

    last_publish_time_ = now();
    last_open_attempt_time_ = now();
    tryOpenConnectedController();
    if (controller_ == nullptr) {
      logWaitingForController();
    }

    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_interval_ms_),
      std::bind(&JoyInputNode::pollEvents, this));
  }

  ~JoyInputNode() override
  {
    if (poll_timer_) {
      poll_timer_->cancel();
    }
    closeController();
    if (sdl_initialized_) {
      SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
      sdl_initialized_ = false;
    }
  }

private:
  static constexpr std::size_t kAxisCount =
    static_cast<std::size_t>(SDL_CONTROLLER_AXIS_MAX);
  static constexpr std::size_t kButtonCount =
    static_cast<std::size_t>(SDL_CONTROLLER_BUTTON_MAX);

  void install8BitDoUltimate2Mappings()
  {
    // Bundled from the Linux entries in SDL_GameControllerDB. The Bluetooth
    // GUID begins with bus type 05; the USB/D-input GUID is retained for
    // diagnostics, but the 2.4 GHz receiver must remain unplugged in this
    // vehicle configuration. Extra paddle slots are deliberately omitted.
    constexpr const char * mappings[] = {
      "05000000c82d00001260000001000000,8BitDo Ultimate 2 Bluetooth,"
      "a:b0,b:b1,back:b10,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,"
      "dpup:h0.1,guide:b12,leftshoulder:b6,leftstick:b13,lefttrigger:a5,"
      "leftx:a0,lefty:a1,rightshoulder:b7,rightstick:b14,righttrigger:a4,"
      "rightx:a2,righty:a3,start:b11,x:b3,y:b4,platform:Linux,",
      "03000000c82d00001260000011010000,8BitDo Ultimate 2 D-input,"
      "a:b0,b:b1,back:b10,dpdown:h0.4,dpleft:h0.8,dpright:h0.2,"
      "dpup:h0.1,guide:b12,leftshoulder:b6,leftstick:b13,lefttrigger:a5,"
      "leftx:a0,lefty:a1,rightshoulder:b7,rightstick:b14,righttrigger:a4,"
      "rightx:a2,righty:a3,start:b11,x:b3,y:b4,platform:Linux,",
    };

    for (const char * mapping : mappings) {
      if (SDL_GameControllerAddMapping(mapping) < 0) {
        RCLCPP_WARN(
          get_logger(), "Could not install bundled 8BitDo SDL mapping: %s",
          SDL_GetError());
      }
    }
  }

  void logWaitingForController() const
  {
    if (device_name_contains_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "SDL game controller device_id=%d is not present. Waiting for Bluetooth input.",
        device_id_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "SDL game controller containing '%s' is not present. "
        "Waiting for Bluetooth input; the 2.4 GHz receiver should be unplugged.",
        device_name_contains_.c_str());
    }
  }

  bool deviceNameMatches(const int device_index) const
  {
    if (device_index < 0) {
      return false;
    }
    if (device_name_contains_.empty()) {
      return device_index == device_id_;
    }
    const char * name = SDL_JoystickNameForIndex(device_index);
    return
      name != nullptr &&
      std::string(name).find(device_name_contains_) != std::string::npos;
  }

  bool deviceMatches(const int device_index) const
  {
    return
      deviceNameMatches(device_index) &&
      SDL_IsGameController(device_index) == SDL_TRUE;
  }

  void reportUnmappedController(const int device_index)
  {
    if (unmapped_controller_reported_) {
      return;
    }
    const char * raw_name = SDL_JoystickNameForIndex(device_index);
    char guid_text[33] = {};
    SDL_JoystickGetGUIDString(
      SDL_JoystickGetDeviceGUID(device_index), guid_text, sizeof(guid_text));
    RCLCPP_ERROR(
      get_logger(),
      "Bluetooth joystick is visible but SDL has no GameController mapping: "
      "%s (GUID=%s). Update SDL2 or supply an SDL controller mapping; raw "
      "input is refused so accelerator/brake cannot be silently swapped.",
      raw_name == nullptr ? "unknown" : raw_name, guid_text);
    unmapped_controller_reported_ = true;
  }

  void tryOpenConnectedController()
  {
    last_open_attempt_time_ = now();
    if (controller_ != nullptr) {
      return;
    }
    const int count = SDL_NumJoysticks();
    if (count < 0) {
      RCLCPP_WARN(get_logger(), "SDL could not enumerate controllers: %s", SDL_GetError());
      return;
    }
    for (int index = 0; index < count; ++index) {
      if (
        deviceNameMatches(index) &&
        SDL_IsGameController(index) != SDL_TRUE)
      {
        reportUnmappedController(index);
        continue;
      }
      if (deviceMatches(index)) {
        openController(index);
        return;
      }
    }
  }

  void openController(const int device_index)
  {
    if (controller_ != nullptr || !deviceMatches(device_index)) {
      return;
    }
    last_open_attempt_time_ = now();

    SDL_GameController * opened = SDL_GameControllerOpen(device_index);
    if (opened == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "Unable to open SDL game controller index %d: %s",
        device_index, SDL_GetError());
      return;
    }

    SDL_Joystick * joystick = SDL_GameControllerGetJoystick(opened);
    if (joystick == nullptr) {
      RCLCPP_ERROR(get_logger(), "SDL controller has no joystick handle: %s", SDL_GetError());
      SDL_GameControllerClose(opened);
      return;
    }

    const SDL_JoystickID instance_id = SDL_JoystickInstanceID(joystick);
    if (instance_id < 0) {
      RCLCPP_ERROR(get_logger(), "SDL controller has an invalid instance ID: %s", SDL_GetError());
      SDL_GameControllerClose(opened);
      return;
    }

    controller_ = opened;
    controller_instance_id_ = instance_id;
    joy_message_.axes.assign(kAxisCount, 0.0F);
    joy_message_.buttons.assign(kButtonCount, 0);
    joy_message_.header.frame_id = "joy";
    loadCurrentState();

    const char * opened_name = SDL_GameControllerName(controller_);
    const char * name = opened_name == nullptr ? "unknown" : opened_name;
    if (has_opened_before_) {
      RCLCPP_WARN(
        get_logger(),
        "Reopened Bluetooth game controller: %s "
        "(instance_id=%d, axes=SDL-standard, deadzone=%.3f)",
        name, static_cast<int>(controller_instance_id_), deadzone_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Opened Bluetooth game controller: %s "
        "(instance_id=%d, axes=SDL-standard, deadzone=%.3f)",
        name, static_cast<int>(controller_instance_id_), deadzone_);
    }
    has_opened_before_ = true;
    publishCurrentState();
  }

  // On disconnect, send one released-pedal frame before going silent so the
  // last RT/LT value is not kept by subscribers until their input timeout.
  void publishNeutralState()
  {
    if (controller_ == nullptr || joy_message_.axes.empty()) {
      return;
    }
    std::fill(joy_message_.axes.begin(), joy_message_.axes.end(), 0.0F);
    std::fill(joy_message_.buttons.begin(), joy_message_.buttons.end(), 0);
    publishCurrentState();
  }

  void closeController()
  {
    if (controller_ != nullptr) {
      SDL_GameControllerClose(controller_);
      controller_ = nullptr;
    }
    controller_instance_id_ = -1;
    joy_message_.axes.clear();
    joy_message_.buttons.clear();
  }

  void handleRemoval(const SDL_ControllerDeviceEvent & event)
  {
    if (controller_ == nullptr || event.which != controller_instance_id_) {
      return;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Bluetooth game controller disconnected by SDL (instance_id=%d). "
      "Published released pedals; input publication is stopped until it "
      "reconnects.",
      static_cast<int>(controller_instance_id_));
    publishNeutralState();
    closeController();
  }

  float convertStickAxis(std::int16_t value) const
  {
    if (value == -32768) {
      value = -32767;
    }
    double adjusted = static_cast<double>(value);
    if (adjusted > unscaled_deadzone_) {
      adjusted -= unscaled_deadzone_;
    } else if (adjusted < -unscaled_deadzone_) {
      adjusted += unscaled_deadzone_;
    } else {
      adjusted = 0.0;
    }
    return static_cast<float>(adjusted * stick_scale_);
  }

  static float convertTriggerAxis(const std::int16_t value)
  {
    const int non_negative = std::max(0, static_cast<int>(value));
    return static_cast<float>(non_negative) / 32767.0F;
  }

  static bool isTriggerAxis(const SDL_GameControllerAxis axis)
  {
    return
      axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT ||
      axis == SDL_CONTROLLER_AXIS_TRIGGERRIGHT;
  }

  float convertAxis(const SDL_GameControllerAxis axis, const std::int16_t value) const
  {
    return isTriggerAxis(axis) ? convertTriggerAxis(value) : convertStickAxis(value);
  }

  void loadCurrentState()
  {
    if (controller_ == nullptr) {
      return;
    }
    SDL_GameControllerUpdate();
    for (int axis_index = 0; axis_index < SDL_CONTROLLER_AXIS_MAX; ++axis_index) {
      const auto axis = static_cast<SDL_GameControllerAxis>(axis_index);
      const std::int16_t value = SDL_GameControllerGetAxis(controller_, axis);
      joy_message_.axes.at(static_cast<std::size_t>(axis_index)) =
        convertAxis(axis, value);
    }
    for (int button_index = 0; button_index < SDL_CONTROLLER_BUTTON_MAX; ++button_index) {
      const auto button = static_cast<SDL_GameControllerButton>(button_index);
      joy_message_.buttons.at(static_cast<std::size_t>(button_index)) =
        SDL_GameControllerGetButton(controller_, button) != 0U ? 1 : 0;
    }
  }

  bool handleAxis(const SDL_ControllerAxisEvent & event)
  {
    if (
      controller_ == nullptr || event.which != controller_instance_id_ ||
      event.axis >= static_cast<std::uint8_t>(SDL_CONTROLLER_AXIS_MAX))
    {
      return false;
    }
    const auto axis = static_cast<SDL_GameControllerAxis>(event.axis);
    const auto index = static_cast<std::size_t>(event.axis);
    const float converted = convertAxis(axis, event.value);
    if (joy_message_.axes.at(index) == converted) {
      return false;
    }
    joy_message_.axes.at(index) = converted;
    return true;
  }

  bool handleButton(const SDL_ControllerButtonEvent & event, const bool pressed)
  {
    if (
      controller_ == nullptr || event.which != controller_instance_id_ ||
      event.button >= static_cast<std::uint8_t>(SDL_CONTROLLER_BUTTON_MAX))
    {
      return false;
    }
    const auto index = static_cast<std::size_t>(event.button);
    if (sticky_buttons_) {
      if (pressed) {
        joy_message_.buttons.at(index) = 1 - joy_message_.buttons.at(index);
        return true;
      }
      return false;
    }
    const int value = pressed ? 1 : 0;
    if (joy_message_.buttons.at(index) == value) {
      return false;
    }
    joy_message_.buttons.at(index) = value;
    return true;
  }

  void publishCurrentState()
  {
    if (controller_ == nullptr) {
      return;
    }
    joy_message_.header.stamp = now();
    publisher_->publish(joy_message_);
    last_publish_time_ = joy_message_.header.stamp;
  }

  void pollEvents()
  {
    bool state_changed = false;
    SDL_Event event;
    int processed_events = 0;
    constexpr int kMaximumEventsPerPoll = 256;
    while (
      processed_events < kMaximumEventsPerPoll &&
      SDL_PollEvent(&event) == 1)
    {
      ++processed_events;
      switch (event.type) {
        case SDL_CONTROLLERDEVICEADDED:
          openController(event.cdevice.which);
          break;
        case SDL_CONTROLLERDEVICEREMOVED:
          handleRemoval(event.cdevice);
          break;
        case SDL_CONTROLLERDEVICEREMAPPED:
          if (
            controller_ != nullptr &&
            event.cdevice.which == controller_instance_id_)
          {
            loadCurrentState();
            state_changed = true;
          }
          break;
        case SDL_CONTROLLERAXISMOTION:
          state_changed = handleAxis(event.caxis) || state_changed;
          break;
        case SDL_CONTROLLERBUTTONDOWN:
          state_changed = handleButton(event.cbutton, true) || state_changed;
          break;
        case SDL_CONTROLLERBUTTONUP:
          state_changed = handleButton(event.cbutton, false) || state_changed;
          break;
        default:
          break;
      }
    }

    if (
      controller_ != nullptr &&
      SDL_GameControllerGetAttached(controller_) != SDL_TRUE)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Bluetooth game controller is no longer attached "
        "(instance_id=%d). Published released pedals.",
        static_cast<int>(controller_instance_id_));
      publishNeutralState();
      closeController();
      return;
    }

    if (controller_ == nullptr) {
      if (
        (now() - last_open_attempt_time_).seconds() >=
        reconnect_interval_sec_)
      {
        tryOpenConnectedController();
      }
      return;
    }
    const double elapsed_sec = (now() - last_publish_time_).seconds();
    const bool autorepeat_due =
      autorepeat_period_sec_ > 0.0 && elapsed_sec >= autorepeat_period_sec_;
    if (state_changed || autorepeat_due) {
      publishCurrentState();
    }
  }

  int device_id_{0};
  std::string device_name_contains_;
  double deadzone_{0.05};
  double unscaled_deadzone_{0.0};
  double stick_scale_{0.0};
  double autorepeat_rate_hz_{50.0};
  double autorepeat_period_sec_{0.02};
  bool sticky_buttons_{false};
  int poll_interval_ms_{1};
  double reconnect_interval_sec_{1.0};

  bool sdl_initialized_{false};
  bool has_opened_before_{false};
  bool unmapped_controller_reported_{false};
  SDL_GameController * controller_{nullptr};
  SDL_JoystickID controller_instance_id_{-1};

  sensor_msgs::msg::Joy joy_message_;
  rclcpp::Time last_publish_time_;
  rclcpp::Time last_open_attempt_time_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace joy_initializer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joy_initializer::JoyInputNode>());
  rclcpp::shutdown();
  return 0;
}
