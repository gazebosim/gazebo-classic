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

#ifndef _ImgParser_HH_
#define _ImgParser_HH_

#include <fstream>
#include <string>

namespace gazebo
{
  namespace util
  {
    /// addtogroup gazebo_util
    /// \{

    /// \class ImgParser ImgParser.hh util/util.hh
    /// \brief Handles loading and parsing a dataset with images.
    ///
    /// The ImgParser class loads a log file with a set of images. Using the
    /// method Parse(), all the images will be converted into RGB888 and saved
    /// into a destination directory.
    ///
    /// This is an abstract class, you should create your own derived class and
    /// implement the virtual methods in order to specify the width, height,
    /// bytes per pixel and image format.
    ///
    class ImgParser
    {
      /// \brief Bytes per pixel used for the destination images.
      public: static const unsigned int BppDst;

      /// \brief Constructor.
      /// \param[in] _filename the path to the image.
      /// \param[in] _dstDir Directory where the RGB images will be saved.
      public: ImgParser(const std::string &_filename,
                        const std::string &_dstDir);

      /// \brief Destructor.
      public: virtual ~ImgParser();

      /// \brief Get the size of every pixel in bytes.
      /// \return The size of every pixel in bytes.
      public: virtual unsigned char GetBpp() = 0;

      /// \brief Get the height of every image in bytes.
      /// \return The height of every image in bytes.
      public: virtual int GetHeight() = 0;

      /// \brief Get the width of every image in bytes.
      /// \return The width of every image in bytes.
      public: virtual int GetWidth() = 0;

      /// \brief Get the next image.
      /// \param[out] _img Next image.
      public: virtual void GetNextImage(unsigned char *_img) = 0;

      /// \brief Convert an image to RGB888.
      /// \param[in] _src Source image.
      /// \param[out] _dst Destination image converted to RGB888.
      private: virtual void ToRGB(const unsigned char *_src,
                                  unsigned char *_dst) = 0;

      /// \brief Parse the image dataset and save the RGB images on disk.
      public: void Parse();

      /// \brief Save an image into disk.
      /// \param[in] _filename the path to the image to be saved.
      /// \param[in] _img Image data.
      public: void SaveImage(const std::string &_filename,
                             const unsigned char *_img);

      /// \brief File containing the log dataset.
      protected: std::ifstream logFile;

      /// \brief Path to the image log file.
      private: std::string logFilename;

      /// \brief Destination directory where the RGB images will be saved.
      private: std::string imgDstDir;
    };
    /// \}
  }
}
#endif
