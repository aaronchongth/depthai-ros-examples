#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include <utility>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <thread>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>

std::pair<std::shared_ptr<dai::node::MonoCamera>,
  std::shared_ptr<dai::node::XLinkOut>>
  create_mono(const std::string& stream_name,
              dai::CameraBoardSocket socket,
              dai::Pipeline& pipeline)
{
  auto mono = pipeline.create<dai::node::MonoCamera>();
  auto out = pipeline.create<dai::node::XLinkOut>();
  out->setStreamName(stream_name);

  mono->setResolution(
    dai::node::MonoCamera::Properties::SensorResolution::THE_480_P);
  mono->setBoardSocket(socket);
  mono->out.link(out->input);

  return std::make_pair(mono, out);
}

std::pair<std::shared_ptr<dai::node::StereoDepth>,
  std::shared_ptr<dai::node::XLinkOut>>
  create_depth(const std::string& stream_name,
               const std::shared_ptr<dai::node::MonoCamera>& left_mono_node,
               const std::shared_ptr<dai::node::XLinkOut>& left_out_node,
               const std::shared_ptr<dai::node::MonoCamera>& right_mono_node,
               const std::shared_ptr<dai::node::XLinkOut>& right_out_node,
               bool lrcheck,
               bool extended,
               bool subpixel,
               int confidence,
               int LRchecktresh,
               dai::Pipeline& pipeline)
{
  auto stereo = pipeline.create<dai::node::StereoDepth>();
  auto out = pipeline.create<dai::node::XLinkOut>();
  out->setStreamName(stream_name);

  stereo->initialConfig.setConfidenceThreshold(confidence);
  stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
  stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
  stereo->setLeftRightCheck(lrcheck);
  stereo->setExtendedDisparity(extended);
  stereo->setSubpixel(subpixel);

  // Link plugins CAM -> STEREO -> XLINK
  left_mono_node->out.link(stereo->left);
  right_mono_node->out.link(stereo->right);
  stereo->syncedLeft.link(left_out_node->input);
  stereo->syncedRight.link(right_out_node->input);

  stereo->depth.link(out->input);
  return std::make_pair(stereo, out);
}

std::pair<std::shared_ptr<dai::node::ColorCamera>,
  std::shared_ptr<dai::node::XLinkOut>>
  create_rgb(const std::string& stream_name,
             dai::Pipeline& pipeline)
{
  auto rgb = pipeline.create<dai::node::ColorCamera>();
  auto out = pipeline.create<dai::node::XLinkOut>();
  out->setStreamName(stream_name);
  
  rgb->setBoardSocket(dai::CameraBoardSocket::RGB);
  rgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  rgb->setInterleaved(false);

  // Link plugins CAM -> XLINK
  rgb->video.link(out->input);

  return std::make_pair(rgb, out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle pnh("~");
  // int timeout = 10;
  
  std::string tfPrefix;
  std::string cameraParamUri;
  int badParams = 0;
  bool lrcheck, extended, subpixel;
  int confidence = 200;
  int LRchecktresh = 5;
  int queue_size = 5;

  // OAK-D-LITE specific
  int monoWidth = 640;
  int monoHeight = 480;

  badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
  badParams += !pnh.getParam("tf_prefix", tfPrefix);
  badParams += !pnh.getParam("lrcheck", lrcheck);
  badParams += !pnh.getParam("extended", extended);
  badParams += !pnh.getParam("subpixel", subpixel);
  badParams += !pnh.getParam("confidence", confidence);
  badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
  badParams += !pnh.getParam("queue_size", queue_size);

  if (badParams > 0)
  {   
    std::cout << " Bad parameters -> " << badParams << std::endl;
    throw std::runtime_error("Couldn't find %d of the parameters");
  }

  dai::Pipeline pipeline;

  auto left_mono = create_mono(
    "left", dai::CameraBoardSocket::LEFT, pipeline);
  auto right_mono = create_mono(
    "right", dai::CameraBoardSocket::RIGHT, pipeline);
  auto depth = create_depth(
    "depth", left_mono.first, left_mono.second, right_mono.first,
    right_mono.second, lrcheck, extended, subpixel, confidence, LRchecktresh,
    pipeline);
  auto rgb = create_rgb("video", pipeline);

  dai::Device device(pipeline);
  auto leftQueue = device.getOutputQueue("left", queue_size, false);
  auto rightQueue = device.getOutputQueue("right", queue_size, false);
  auto stereoQueue = device.getOutputQueue("depth", queue_size, false);
  auto previewQueue = device.getOutputQueue("video", queue_size, false);

  auto calibrationHandler = device.readCalibration();
 
  // left mono
  dai::rosBridge::ImageConverter conv(
    tfPrefix + "_left_camera_optical_frame", true);
  auto leftCameraInfo = conv.calibrationToCameraInfo(
    calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight);
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>
    leftPublish(
      leftQueue,
      pnh,
      std::string("left/image"),
      std::bind(
        &dai::rosBridge::ImageConverter::toRosMsg, &conv,
        std::placeholders::_1, std::placeholders::_2),
      queue_size,
      leftCameraInfo,
      "left");
  leftPublish.addPublisherCallback();

  // right mono
  dai::rosBridge::ImageConverter r_conv(
    tfPrefix + "_right_camera_optical_frame", true);
  auto rightCameraInfo = conv.calibrationToCameraInfo(
    calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>
    rightPublish(
      rightQueue,
      pnh,
      std::string("right/image"),
      std::bind(
        &dai::rosBridge::ImageConverter::toRosMsg, &r_conv,
        std::placeholders::_1, std::placeholders::_2),
      queue_size,
      rightCameraInfo,
      "right");
  rightPublish.addPublisherCallback();

  // depth
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>
    depthPublish(
      stereoQueue,
      pnh,
      std::string("stereo/depth"),
      std::bind(
        &dai::rosBridge::ImageConverter::toRosMsg, &r_conv,
        std::placeholders::_1, std::placeholders::_2),
      queue_size,
      rightCameraInfo,
      "stereo");
  depthPublish.addPublisherCallback();

  // rgb
  dai::rosBridge::ImageConverter rgb_conv(
    tfPrefix + "_rgb_camera_optical_frame", true);
  dai::rosBridge::ImageConverter depthConverter(
    tfPrefix + "_right_camera_optical_frame", true);
  auto rgbCameraInfo = depthConverter.calibrationToCameraInfo(
    calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720); 
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>\
    rgbPublish(
      previewQueue,
      pnh,
      std::string("color/image"),
      std::bind(
        &dai::rosBridge::ImageConverter::toRosMsg, rgb_conv,
        std::placeholders::_1, std::placeholders::_2),
      queue_size,
      rgbCameraInfo,
      "color");
  rgbPublish.addPublisherCallback();

  ros::spin();
  return 0;
}
