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

#ifndef _ZADEATIMGPARSER_HH_
#define _ZADEATIMGPARSER_HH_

#include <string>

#include "gazebo/util/ImgParser.hh"

namespace gazebo
{
  namespace util
  {
    /// addtogroup gazebo_util
    /// \{

    /// \class ZadeatImgParser ZadeatImgParser.hh util/util.hh
    /// \brief Handles loading and parsing a dataset with images.
    ///
    /// The ZadeatImgParser class is a derived class from the ImgParser abstract
    /// class. It loads and parses YUV422 images with 320x240 pixels of
    /// resolution. Before every image, there is an 8 byte timespamp.
    /// The image datasets are available at http://nao-gt.ist.tugraz.at/
    ///
    class ZadeatImgParser : public ImgParser
    {
      /// \brief Image's width in pixels.
      public: static const int Width;

      /// \brief Image's height in pixels.
      public: static const int Height;

      /// \brief Bytes per pixel used for the source images.
      public: static const unsigned char Bpp;

      /// \brief Size of the timestamp in bytes.
      public: static const unsigned char TimestampLength;

      // Documentation inherited.
      public: ZadeatImgParser(const std::string &_filename,
                              const std::string &_dstDir);

      // Documentation inherited.
      public: virtual unsigned char GetBpp();

      // Documentation inherited.
      public: int GetHeight();

      // Documentation inherited.
      public: int GetWidth();

      // Documentation inherited.
      public: void GetNextImage(unsigned char *_img);

      // Documentation inherited.
      private: void ToRGB(const unsigned char *_src, unsigned char *_dst);

      /// \brief Convert a pixel from YUV422 to RGB888.
      /// \param[in] _y Y component.
      /// \param[in] _u U component.
      /// \param[in] _v V component.
      /// \param[out] _r R component.
      /// \param[out] _g G component.
      /// \param[out] _b B component.
      private: void Yuv2rgb(unsigned char _y, unsigned char _u,
                            unsigned char _v, unsigned char &_r,
                            unsigned char &_g, unsigned char &_b);
    };
    /// \}
  }
}
#endif
