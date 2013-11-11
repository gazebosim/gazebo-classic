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
#include <stdint.h>
#include <algorithm>
#include <string>

#include "gazebo/util/ZadeatImgParser.hh"

using namespace gazebo;
using namespace util;

const int ZadeatImgParser::Width = 320;
const int ZadeatImgParser::Height = 240;
const unsigned char ZadeatImgParser::Bpp = 2;
const unsigned char ZadeatImgParser::TimestampLength = 8;

//////////////////////////////////////////////////
ZadeatImgParser::ZadeatImgParser(const std::string &_filename,
                                 const std::string &_dstDir)
    : ImgParser(_filename, _dstDir)
{
}

//////////////////////////////////////////////////
unsigned char ZadeatImgParser::GetBpp()
{
  return this->Bpp;
}

//////////////////////////////////////////////////
int ZadeatImgParser::GetHeight()
{
  return this->Height;
}

//////////////////////////////////////////////////
int ZadeatImgParser::GetWidth()
{
  return this->Width;
}

//////////////////////////////////////////////////
void ZadeatImgParser::GetNextImage(unsigned char *_img)
{
  // Timestamp included in every image.
  unsigned char imgTimestamp[this->TimestampLength];

  this->logFile.read(reinterpret_cast<char *>(imgTimestamp),
      this->TimestampLength);
  uint32_t size = this->GetWidth() * this->GetHeight() * this->GetBpp();
  this->logFile.read(reinterpret_cast<char *>(_img), size);
}

//////////////////////////////////////////////////
void ZadeatImgParser::ToRGB(unsigned char *_src, unsigned char *_dst)
{
  for (int i = 0; i < GetHeight(); ++i)
    {
      for (int j = 0; j < this->GetWidth(); ++j)
      {
        int pixel = i * this->Width + j;
        int index = pixel * this->GetBpp();
        int dstIndex = pixel * 3;
        unsigned char r, g, b;
        if (pixel % 2 == 0)
        {
          this->Yuv2rgb(_src[index], _src[index + 1], _src[index + 3], r, g, b);
        }
        else
        {
          this->Yuv2rgb(_src[index], _src[index - 1], _src[index + 1], r, g, b);
        }
        _dst[dstIndex] = r;
        _dst[dstIndex + 1] = g;
        _dst[dstIndex + 2] = b;
      }
    }
}

//////////////////////////////////////////////////
void ZadeatImgParser::Yuv2rgb(unsigned char _y, unsigned char _u,
                              unsigned char _v, unsigned char &_r,
                              unsigned char &_g, unsigned char &_b)
{
  int r = _y + ((1436 * (_v - 128)) >> 10);
  int g = _y - ((354 * (_u - 128) + 732 * (_v - 128)) >> 10);
  int b = _y + ((1814 * (_u - 128)) >> 10);

  r = std::min(255, std::max(0, r));
  g = std::min(255, std::max(0, g));
  b = std::min(255, std::max(0, b));

  _r = static_cast<unsigned char>(r);
  _g = static_cast<unsigned char>(g);
  _b = static_cast<unsigned char>(b);
}
