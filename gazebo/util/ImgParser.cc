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

#include <boost/scoped_array.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <iomanip>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/util/ImgParser.hh"

using namespace gazebo;
using namespace util;

// The destination image is RGB888.
const unsigned int ImgParser::BppDst = 3;

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

  // Image counter
  int counter = 0;

  this->logFile.open(this->logFilename.c_str(),
      std::ios::in | std::ios::binary);

  while (this->logFile)
  {
    this->GetNextImage(srcImage.get());
    this->ToRGB(srcImage.get(), dstImage.get());
    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << counter;
    std::string imgName = this->imgDstDir + ss.str() + ".ppm";
    this->SaveImage(imgName.c_str(), dstImage.get());
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
