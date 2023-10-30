// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "exposure_control.h"

#include <algorithm>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

#include "logging.h"

using namespace std::placeholders;

namespace exposure_control_ros2
{
ExposureControl::ExposureControl(const rclcpp::NodeOptions & options)
: Node("exposure_control", options)
{
  sub_ = this->create_subscription<ImageMetaData>(
    "~/meta", 1,
    std::bind(&ExposureControl::metaDataCallback, this, std::placeholders::_1));
  pub_ = create_publisher<CameraControl>("~/control", 10);
  brightnessTarget_ = declare_parameter<int>("brightness_target", 120);
  brightnessTolerance_ = declare_parameter<int>("brightness_tolerance", 5);
  currentBrightness_ = brightnessTarget_;
  minFramesSkip_ =
    declare_parameter<int>("min_frames_skip", 10);  // number of frames to wait
  minExposureTime_ =
    declare_parameter<double>("min_exposure_time", 1000);  // usecs
  maxConfigExposureTime_ =
    declare_parameter<double>("max_exposure_time", 1000000);  // usecs
  maxExposureTime_ = maxConfigExposureTime_;
  maxGain_ = declare_parameter<double>("max_gain", 10);  // gain in db
  gainPriority_ = declare_parameter<bool>("gain_priority", false);
  LOG_INFO(
    "using " << (gainPriority_ ? "gain" : "time") << " priority, time range: ["
             << minExposureTime_ << " - " << maxExposureTime_
             << "] max gain: " << maxGain_);
}

ExposureControl::~ExposureControl() {}

bool ExposureControl::changeTime(
  double brightRatio, double minTime, double maxTime, const char * debugMsg)
{
  const double optTime =
    std::min(std::max(calculateExposureTime(brightRatio), minTime), maxTime);
  if (currentExposureTime_ != optTime) {
    currentExposureTime_ = optTime;
#ifdef DEBUG
    LOG_INFO(debugMsg);
#else
    (void)debugMsg;
#endif
    return (true);
  }
  return (false);
}

bool ExposureControl::changeGain(
  double brightRatio, double minGain, double maxGain, const char * debugMsg)
{
  const double optGain =
    std::min(std::max(calculateGain(brightRatio), minGain), maxGain);
  if (optGain != currentGain_) {
    currentGain_ = optGain;
#ifdef DEBUG
    LOG_INFO(debugMsg);
#else
    (void)debugMsg;
#endif
    return (true);
  }
  return (false);
}

bool ExposureControl::updateExposureWithGainPriority(double brightRatio)
{
  if (brightRatio < 1) {  // image is too bright
    if (currentGain_ > 0) {
      // if gain is nonzero, dial it back before touching exposure times
      if (changeGain(brightRatio, 0, maxGain_, "gp: --gain!")) {
        return (true);
      }
    } else {
      // gain is already at zero, reduce exposure time
      if (changeTime(brightRatio, 0, maxExposureTime_, "gp: --time!")) {
        return (true);
      }
    }
  } else {  // image is too dark
    if (currentExposureTime_ < maxExposureTime_) {
      // first try bumping the exposure time
      if (changeTime(brightRatio, 0, maxExposureTime_, "gp: ++time!")) {
        return (true);
      }
    } else {
      // try bumping gain if exposure time is at limit
      if (changeGain(brightRatio, 0, maxGain_, "gp: ++gain!")) {
        return (true);
      }
    }
  }
  return (false);
}

bool ExposureControl::updateExposureWithTimePriority(double brightRatio)
{
  if (brightRatio < 1) {  // image is too bright
    if (currentExposureTime_ > minExposureTime_) {
      // first try to shorten the exposure time
      if (changeTime(
            brightRatio, minExposureTime_, maxExposureTime_, "tp: cut time!")) {
        return (true);
      }
    } else {
      // already have reached minimum exposure, try reducing gain
      if (currentGain_ > 0) {
        if (changeGain(brightRatio, 0, maxGain_, "tp: cut gain")) {
          return (true);
        }
      } else {
        // gain is already at zero, must reduce exposure below min
        if (changeTime(
              brightRatio, 0, maxExposureTime_, "tp: cut time below min!")) {
          return (true);
        }
      }
    }
  } else {  // image is too dark
    // if current exposure time is below minimum, bump it
    if (currentExposureTime_ < minExposureTime_) {
      if (changeTime(
            brightRatio, 0, minExposureTime_, "tp: bump time from min!")) {
        return (true);
      }
    } else {
      // next try bumping the gain
      if (currentGain_ < maxGain_) {
        if (changeGain(brightRatio, 0, maxGain_, "tp: bump gain")) {
          return (true);
        }
      } else {
        // already have max gain, must bump exposure time
        if (changeTime(
              brightRatio, minExposureTime_, maxExposureTime_, "tp: ++time!")) {
          return (true);
        }
      }
    }
  }
  return (false);
}

bool ExposureControl::updateExposure(double b)
{
  const double err_b = (brightnessTarget_ - b);
#if 0
  LOG_INFO(
    "b: " << b << " err_b: " << err_b << " curr: " << currentExposureTime_
          << " " << currentGain_);
#endif
  if (numFramesSkip_ > 0) {
    // Changes in gain or shutter take a few
    // frames to arrive at the camera, so we just skip
    // those.
    numFramesSkip_--;
    return (false);
  }

  // the current gain is higher than it should be, let's
  // set it to zero
  if (currentGain_ > maxGain_) {
    currentGain_ = 0;
    return (true);
  }

  // the current exposure is longer than it should be, let's
  // set it to a good value and retry
  if (currentExposureTime_ > maxExposureTime_) {
    currentExposureTime_ = maxExposureTime_;
    return (true);
  }
  if (fabs(err_b) <= brightnessTolerance_) {
    // no need to change anything!
    return (false);
  }
  // ratio between desired and actual brightness
  const double brightRatio =
    std::max(std::min(brightnessTarget_ / b, 10.0), 0.1);

  if (gainPriority_) {
    return (updateExposureWithGainPriority(brightRatio));
  } else {
    return (updateExposureWithTimePriority(brightRatio));
  }
  return (false);
}

double ExposureControl::calculateGain(double brightRatio) const
{
  // because gain is in db:
  // db(G) = 10 * log_10(G) = 10 * ln(G) / ln(10) = 4.34 * ln(G)
  const double kp = 4.34;
  const double desiredGain = currentGain_ + kp * log(brightRatio);
  const double cappedGain = std::max(std::min(desiredGain, maxGain_), 0.0);
  // below threshold set to zero because it can no longer be set accurately
  // at the camera
  return (cappedGain > 0.5 ? cappedGain : 0);
}

double ExposureControl::calculateExposureTime(double brightRatio) const
{
  const double desiredExposureTime = currentExposureTime_ * brightRatio;
  const double optTime =
    std::max(0.0, std::min(desiredExposureTime, maxExposureTime_));
  return (optTime);
}

void ExposureControl::metaDataCallback(const ImageMetaData::UniquePtr msg)
{
  if (msg->brightness < 0) {
    return;
  }
  if (msg->max_exposure_time != 0) {
    maxDriverAllowedExposureTime_ = static_cast<double>(msg->max_exposure_time);
    maxExposureTime_ =
      std::min(maxDriverAllowedExposureTime_, maxConfigExposureTime_);
  }
  const int16_t b = std::min(msg->brightness, static_cast<int16_t>(255));
  // if the exposure parameters are not set yet,
  // grab the current values from the msg
  if (currentExposureTime_ == 0) {
    currentExposureTime_ = static_cast<double>(msg->exposure_time);
  }
  if (currentGain_ == std::numeric_limits<float>::lowest()) {
    currentGain_ = msg->gain;
  }

#if 0
  LOG_INFO(
    "msg: " << msg->exposure_time << "/" << currentExposureTime_
    << " gain: " << msg->gain << "/" << currentGain_
    << " brightness: " << static_cast<int>(b));
#endif
  // check if the reported exposure and brightness settings
  // match ours. That means the changes have taken effect
  // on the driver side, and the brightness measurement can be
  // used right away.
  if (
    fabs(currentGain_ - msg->gain) <= 0.05 * (currentGain_ + msg->gain) &&
    fabs(currentExposureTime_ - static_cast<double>(msg->exposure_time)) <=
      0.05 * (currentExposureTime_ + static_cast<double>(msg->exposure_time)) &&
    numFramesSkip_ < minFramesSkip_) {
    numFramesSkip_ = 0;  // no skipping anymore!
  }

  const double oldExposureTime = currentExposureTime_;
  const double oldGain = currentGain_;
  if (updateExposure(std::max(static_cast<int16_t>(1), b))) {
    LOG_INFO(
      "bright " << static_cast<int>(b) << " at time/gain: [" << oldExposureTime
                << " " << oldGain << "] new: [" << currentExposureTime_ << " "
                << currentGain_ << "]");
    controlMsg_.header = msg->header;
    controlMsg_.gain = currentGain_;
    controlMsg_.exposure_time = currentExposureTime_;
    numFramesSkip_ = minFramesSkip_ * 2;  // restart frame skipping
    pub_->publish(controlMsg_);
  }
}

}  // namespace exposure_control_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(exposure_control_ros2::ExposureControl)
