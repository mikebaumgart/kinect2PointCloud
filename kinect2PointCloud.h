/*
  Copyright 2016, Can HUANG"
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  @Author 
  Can HUANG, Master Student
  MSE, HUST
  Wuhan
  China
*/

/// [headers]
/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <cstdlib>
#include <string>
#include <iostream>

enum processor{
  CPU, OPENCL, OPENGL
};

class Kinect2PointCloud {
public:
  Kinect2PointCloud(processor p, bool mirror);
  void prepareMake3D(const libfreenect2::Freenect2Device::ColorCameraParams & p);
  libfreenect2::Freenect2Device::IrCameraParams getIrParameters();
  libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();
  cv::Mat getColor(); // get color image 1920 * 1020
  cv::Mat getDepth(); // get big depth image 1920 * 1082
  cv::Mat getPointCloud(); //get point Cloud Mat
  cv::Mat updateColor(); // update and get color image
  cv::Mat updateDepth(); // update and get depth image
  void update();
  void updatePointCloud(); // update point cloud using colorImage&bigDepthImage
  void shutDown();

private:
  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device * dev_ = 0;
  libfreenect2::PacketPipeline * pipeline_ = 0;
  libfreenect2::Registration * registration_ = 0;
  libfreenect2::SyncMultiFrameListener listener_;
  libfreenect2::FrameMap frames_;
  libfreenect2::Frame undistorted_, registered_, big_mat_;
  std::string serial_;
  bool mirror_;
  float colmap[1920];
  float rowmap[1080];
  cv::Mat colorImage;
  cv::Mat depthImage;
  cv::Mat pointCloudMat;

};

Kinect2PointCloud::Kinect2PointCloud(processor p, bool mirror = false) :
    mirror_(mirror),
    listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
    undistorted_(512, 424, 4),
    registered_(512, 424, 4),
    big_mat_(1920, 1082, 4),
    colorImage(1080, 1920, CV_8UC4),
    depthImage(1082, 1920, CV_32FC1),
    pointCloudMat(1080, 1920, CV_32FC3)
{
  if(freenect2_.enumerateDevices() == 0) {
    std::cout << "no kinect2 connected!"<< std::endl;
    exit(-1);
  }
  serial_ = freenect2_.getDefaultDeviceSerialNumber();
  switch(p) {
  case CPU:
    std::cout << "creating CPU processor" << std::endl;
    pipeline_ = new libfreenect2::CpuPacketPipeline();
    break;
  case OPENCL:
    std::cout << "creating OPENCL processor" << std::endl;
    pipeline_ = new libfreenect2::OpenCLPacketPipeline();
    break;
  case OPENGL:
    std::cout << "creating OpenGL processor" << std::endl;
    pipeline_ = new libfreenect2::OpenGLPacketPipeline();
  default:
    std::cout << "creating CPU processor" << std::endl;
    pipeline_ = new libfreenect2::CpuPacketPipeline();
    break;
  }

  dev_ = freenect2_.openDevice(serial_, pipeline_);
  dev_->setColorFrameListener(&listener_);
  dev_->setIrAndDepthFrameListener(&listener_);
  dev_->start();

  registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(),dev_->getColorCameraParams());
  prepareMake3D(dev_->getColorCameraParams());

}

libfreenect2::Freenect2Device::IrCameraParams Kinect2PointCloud::getIrParameters() {
  libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
  return ir;
}
libfreenect2::Freenect2Device::ColorCameraParams Kinect2PointCloud::getRgbParameters() {
  libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
  return rgb;
}
void Kinect2PointCloud::prepareMake3D( const libfreenect2::Freenect2Device::ColorCameraParams &p ) {
  const int w = 1920;
  const int h = 1080;
  for(int i = 0; i < w; i++) {
    this->colmap[i] = (i - p.cx + 0.5) / p.fx;
  }
  for(int i = 0; i < h; i++) {
    this->rowmap[i] = (i - p.cy + 0.5) / p.fy;
  }
}

cv::Mat Kinect2PointCloud::updateColor() {
  listener_.waitForNewFrame(frames_);
  libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
  cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
  if (mirror_ == true) {cv:flip(tmp, colorImage, 1);}
  else {colorImage = tmp.clone();}

  listener_.release(frames_);
  return std::move(colorImage);
}
cv::Mat Kinect2PointCloud::updateDepth() {
  listener_.waitForNewFrame(frames_);
  libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
  libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
  registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_, 0 );
  cv::Mat tmp(big_mat_.height, big_mat_.width, CV_32FC1, big_mat_.data);
  if (mirror_ == true) {
    cv::flip(tmp, depthImage, 1);
  }
  else {depthImage = tmp.clone();}
  listener_.release(frames_);
  return std::move(depthImage);
}
void Kinect2PointCloud::update() {
  listener_.waitForNewFrame(frames_);
  libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
  libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
  registration_->apply(rgb, depth, &undistorted_, &registered_,true, &big_mat_, 0);
  cv::Mat tmpDepth(big_mat_.height, big_mat_.width, CV_32FC1, big_mat_.data);
  cv::Mat tmpColor(rgb->height, rgb->width, CV_8UC4, rgb->data);
  if (mirror_ == true) {
    cv::flip(tmpDepth, depthImage, 1);
    cv::flip(tmpColor, colorImage, 1);
  }
  else {
    depthImage = tmpDepth.clone();
    colorImage = colorImage.clone();
  }
  listener_.release(frames_);
}
void Kinect2PointCloud::updatePointCloud() {
  listener_.waitForNewFrame(frames_);
  libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
  libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
  registration_->apply(rgb, depth, &undistorted_, &registered_,true, &big_mat_, 0);
  cv::Mat tmpDepth(big_mat_.height, big_mat_.width, CV_32FC1, big_mat_.data);
  cv::Mat tmpColor(rgb->height, rgb->width, CV_8UC4, rgb->data);
  if (mirror_ == true) {
    cv::flip(tmpDepth, depthImage, 1);
    cv::flip(tmpColor, colorImage, 1);
  }
  else {
    depthImage = tmpDepth.clone();
    colorImage = colorImage.clone();
  }
  const float * itD0 = (float *)depthImage.ptr();
  cv::Mat tmpPointCloud(1080, 1920, CV_32FC3);
  float * itP = (float *)tmpPointCloud.ptr();
  for(std::size_t y = 0; y < 1080; ++y) {

    const unsigned int offset = 1920 * y;
    const float * itD = itD0 + offset;
    const float dy = rowmap[y];

    for(std::size_t x = 0; x < 1920; ++x) {
      const float depth_value = *(itD++) / 1000.0f; 
      if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
        const float rx = colmap[x] * depth_value;
        const float ry = dy * depth_value;
        *(itP++) = rx;
        *(itP++) = ry;
        *(itP++) = depth_value;
      }
      else {
        *(itP++) = std::numeric_limits<float>::quiet_NaN();
        *(itP++) = std::numeric_limits<float>::quiet_NaN();
        *(itP++) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  pointCloudMat = tmpPointCloud.clone();
  listener_.release(frames_);
}

cv::Mat Kinect2PointCloud::getColor() {
  return std::move(colorImage);
}
cv::Mat Kinect2PointCloud::getDepth() {
  return std::move(depthImage);
}
cv::Mat Kinect2PointCloud::getPointCloud() {
  return std::move(pointCloudMat);
}

void Kinect2PointCloud::shutDown() {
  dev_->stop();
  dev_->close();
}
