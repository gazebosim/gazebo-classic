/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <boost/scoped_array.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/util/ImgParser.hh"

using namespace gazebo;
using namespace common;
using namespace util;

// The destination image is RGB888.
const unsigned int ImgParser::BppDst = 3;

//////////////////////////////////////////////////
HSVClrParams::HSVClrParams(float _hmin, float _smin, float _vmin, float _hmax,
                           float _smax, float _vmax)
{
  // Convert from Gimp format to OpenCV format
  this->hmin = _hmin / 2;
  this->hmax = _hmax / 2;
  this->smin = _smin * 255 / 100;
  this->smax = _smax * 255 / 100;
  this->vmin = _vmin * 255 / 100;
  this->vmax = _vmax * 255 / 100;
}

//////////////////////////////////////////////////
ImgParser::ImgParser(const std::string &_filename, const std::string &_dstDir)
{
  // Sanity check.
  boost::filesystem::path srcPath(_filename);
  if (!boost::filesystem::exists(srcPath) ||
      !boost::filesystem::is_regular_file(srcPath))
    gzthrow("Invalid log file [" + _filename + "]. Does not exist.");

  boost::filesystem::path dstPath(_dstDir);
  if (!boost::filesystem::exists(dstPath) ||
      !boost::filesystem::is_directory(dstPath))
    gzthrow("Invalid destination directory [" + _dstDir + "]. Does not exist.");

  this->logFilename = _filename;
  this->imgDstDir = _dstDir;

  // Create the color filter parameters (H:0-360, S:0-100, V:0-100)
  this->green = new HSVClrParams(130, 50, 0, 210, 100, 80);
  this->orange = new HSVClrParams(0, 0, 0, 0, 0, 0);
  this->yellow = new HSVClrParams(44, 36, 13, 124, 100, 93);
  this->blue = new HSVClrParams(0, 0, 0, 0, 0, 0);
  this->white = new HSVClrParams(119, 0, 22, 199, 80, 100);
}

//////////////////////////////////////////////////
ImgParser::~ImgParser()
{
}

//////////////////////////////////////////////////
void ImgParser::Parse()
{
  // Source image.
  int srcSize = this->GetWidth() * this->GetHeight() * this->GetBpp();
  boost::scoped_array<unsigned char> srcImage(new unsigned char[srcSize]);

  // Destination image.
  int dstSize = this->GetWidth() * this->GetHeight() * this->BppDst;
  boost::scoped_array<unsigned char> dstImage(new unsigned char[dstSize]);

  // Filtered image
  cv::Mat filteredImage;

  // Image counter
  int counter = 0;

  // Open the file
  this->logFile.open(this->logFilename.c_str());

  // Read the log description
  std::string line;
  getline(this->logFile, line);
  getline(this->logFile, line);

  // Read the images
  while (this->logFile)
  {
    this->GetNextImage(srcImage.get());
    this->ToRGB(srcImage.get(), dstImage.get());
    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << counter;
    std::string imgName = this->imgDstDir + ss.str() + ".ppm";
    this->SaveImage(imgName.c_str(), dstImage.get());
    std::string imgProcessedName = this->imgDstDir + ss.str() + "_p.ppm";
    ProcessImage(*this->green, *this->orange, *this->yellow, *this->blue,
                 *this->white, imgName.c_str(), imgProcessedName.c_str());
    ++counter;
  }

  this->logFile.close();
}

//////////////////////////////////////////////////
void ImgParser::SaveImage(const std::string &_filename,
                          const unsigned char *_img)
{
  std::ofstream myFile(_filename.c_str(), std::ios::out | std::ios::binary);

  std::string width = boost::lexical_cast<std::string>(this->GetWidth());
  std::string height = boost::lexical_cast<std::string>(this->GetHeight());
  std::string header = "P6 " + width + " " + height + " 255\n";

  // Write a PPM header
  myFile.write(header.c_str(), header.size());

  // Write the data
  uint32_t dataSize = this->GetWidth() * this->GetHeight() * this->BppDst;
  myFile.write(reinterpret_cast<const char *>(_img), dataSize);

  myFile.close();
}

//////////////////////////////////////////////////
void ImgParser::ProcessImage(const HSVClrParams &_green,
        const HSVClrParams &_orange, const HSVClrParams &_yellow,
        const HSVClrParams &_blue, const HSVClrParams &_white,
        const std::string &_filename, const std::string &_filenameDst)
{
  //RNG rng(12345);

  //Color filter using opencv
  cv::Mat result = cv::imread( _filename, 1 );
  cv::Mat hsvImage, greenCh, orangeCh, yellowCh, blueCh, whiteCh;

  cv::cvtColor(result, hsvImage, CV_BGR2HSV);

  inRange(hsvImage,
    cv::Scalar(_green.hmin, _green.smin, _green.vmin),
    cv::Scalar(_green.hmax, _green.smax, _green.vmax),
    greenCh);
  cv::Mat green_image(result.size(), CV_8UC3, cv::Scalar(0, 255, 0));
  green_image.copyTo(result, greenCh);

  inRange(hsvImage,
    cv::Scalar(_orange.hmin, _orange.smin, _orange.vmin),
    cv::Scalar(_orange.hmax, _orange.smax, _orange.vmax),
    orangeCh);
  cv::Mat orange_image(result.size(), CV_8UC3, cv::Scalar(0, 168, 255));
  orange_image.copyTo(result, orangeCh);

  inRange(hsvImage,
    cv::Scalar(_yellow.hmin, _yellow.smin, _yellow.vmin),
    cv::Scalar(_yellow.hmax, _yellow.smax, _yellow.vmax),
    yellowCh);
  cv::Mat yellow_image(result.size(), CV_8UC3, cv::Scalar(0, 255, 255));
  yellow_image.copyTo(result, yellowCh);

  inRange(hsvImage,
    cv::Scalar(_blue.hmin, _blue.smin, _blue.vmin),
    cv::Scalar(_blue.hmax, _blue.smax, _blue.vmax),
    blueCh);
  cv::Mat blue_image(result.size(), CV_8UC3, cv::Scalar(255, 0, 0));
  blue_image.copyTo(result, blueCh);

  inRange(hsvImage,
    cv::Scalar(_white.hmin, _white.smin, _white.vmin),
    cv::Scalar(_white.hmax, _white.smax, _white.vmax),
    whiteCh);
  cv::Mat white_image(result.size(), CV_8UC3, cv::Scalar(255, 255, 255));
  white_image.copyTo(result, whiteCh);

  // Color segmentation
  /*std::vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(result, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
  }


  imwrite(_filenameDst, result);*/
}
