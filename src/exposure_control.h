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

#ifndef EXPOSURE_CONTROL_H_
#define EXPOSURE_CONTROL_H_

#include <camera_control_msgs_ros2/msg/camera_control.hpp>
#include <image_meta_msgs_ros2/msg/image_meta_data.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace exposure_control_ros2
{
class ExposureControl : public rclcpp::Node
{
public:
  typedef camera_control_msgs_ros2::msg::CameraControl CameraControl;
  typedef image_meta_msgs_ros2::msg::ImageMetaData ImageMetaData;
  explicit ExposureControl(const rclcpp::NodeOptions & options);
  ~ExposureControl();

private:
  void metaDataCallback(const ImageMetaData::UniquePtr msg);
  bool updateExposure(double b);
  double calculateGain(double brightRatio) const;
  double calculateExposureTime(double brightRatio) const;
  bool updateExposureWithGainPriority(double brightRatio);
  bool updateExposureWithTimePriority(double brightRatio);
  bool changeTime(
    double brightRatio, double minTime, double maxTime, const char * debugMsg);
  bool changeGain(
    double brightRatio, double minGain, double maxGain, const char * debugMsg);

  // ------------ variables ------------
  rclcpp::Publisher<CameraControl>::SharedPtr pub_;
  rclcpp::Subscription<ImageMetaData>::SharedPtr sub_;
  CameraControl controlMsg_;
  int brightnessTarget_;
  int brightnessTolerance_;
  double maxExposureTime_;
  double maxDriverAllowedExposureTime_;
  double maxConfigExposureTime_;
  double minExposureTime_;
  double maxGain_;
  uint8_t currentBrightness_;
  double currentExposureTime_{0};
  double currentGain_{std::numeric_limits<float>::lowest()};
  int numFramesSkip_{0};
  int minFramesSkip_{10};
  bool gainPriority_{false};
};
}  // namespace exposure_control_ros2
#endif  // EXPOSURE_CONTROL_H_
