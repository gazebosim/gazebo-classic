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

#ifndef _LOGPARSER_HH_
#define _LOGPARSER_HH_

#include <fstream>
#include <string>

class LogParser
{
  /// \brief Constructor
  /// \param[in] _filename the path to the image.
  public: LogParser(const std::string &_filename="");

  /// \brief Destructor
  public: virtual ~LogParser();

  /// \brief Get the pixel of an image in RGB888.
  /// \return An OpenCV image in RGB888 format.
  public: void GetNextImage();

  /// \brief Save the current image into disk.
  /// \param[in] _filename the path to the image to be saved.
  public: void SaveCurrentImage(const std::string &_filename);

  private: void ToRGB(unsigned char *_src, unsigned char *_dst);

  private: void Yuv2rgb(unsigned char _y, unsigned char _u, unsigned char _v,
                       unsigned char &_r, unsigned char &_g, unsigned char &_b);

  /// \brief File containing the log dataset.
  private: std::ifstream logFile;

  private: unsigned char rgbImage[320 * 240 * 3];

};

#endif