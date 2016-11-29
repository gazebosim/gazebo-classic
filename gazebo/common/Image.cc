/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Desc: Image class
 * Author: Nate Koenig
 * Date: 14 July 2008
 */

#include <boost/filesystem.hpp>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Image.hh"

using namespace gazebo;
using namespace common;


int Image::count = 0;

//////////////////////////////////////////////////
Image::Image(const std::string &_filename)
{
  if (count == 0)
    FreeImage_Initialise();

  count++;

  this->bitmap = nullptr;
  if (!_filename.empty())
  {
    std::string filename = common::find_file(_filename);
    if (!filename.empty())
      this->Load(filename);
    else
      gzerr << "Unable to find image[" << _filename << "]\n";
  }
}

//////////////////////////////////////////////////
Image::~Image()
{
  count--;

  if (this->bitmap)
    FreeImage_Unload(this->bitmap);
  this->bitmap = nullptr;

  if (count == 0)
    FreeImage_DeInitialise();
}

//////////////////////////////////////////////////
int Image::Load(const std::string &_filename)
{
  this->fullName = _filename;
  if (!boost::filesystem::exists(boost::filesystem::path(this->fullName)))
    this->fullName = common::find_file(_filename);

  if (boost::filesystem::exists(boost::filesystem::path(this->fullName)))
  {
    FREE_IMAGE_FORMAT fifmt =
      FreeImage_GetFIFFromFilename(this->fullName.c_str());

    if (this->bitmap)
      FreeImage_Unload(this->bitmap);
    this->bitmap = nullptr;

    if (fifmt == FIF_PNG)
      this->bitmap = FreeImage_Load(fifmt, this->fullName.c_str(), PNG_DEFAULT);
    else if (fifmt == FIF_JPEG)
      this->bitmap =
        FreeImage_Load(fifmt, this->fullName.c_str(), JPEG_DEFAULT);
    else if (fifmt == FIF_BMP)
      this->bitmap = FreeImage_Load(fifmt, this->fullName.c_str(), BMP_DEFAULT);
    else
    {
      gzerr << "Unknown image format[" << this->fullName << "]\n";
      return -1;
    }

    return 0;
  }

  gzerr << "Unable to open image file[" << this->fullName
        << "], check your GAZEBO_RESOURCE_PATH settings.\n";
  return -1;
}

//////////////////////////////////////////////////
void Image::SavePNG(const std::string &_filename)
{
  FreeImage_Save(FIF_PNG, this->bitmap, _filename.c_str(), 0);
}

//////////////////////////////////////////////////
void Image::SetFromData(const unsigned char *_data, unsigned int _width,
    unsigned int _height, PixelFormat _format)
{
  if (this->bitmap)
    FreeImage_Unload(this->bitmap);
  this->bitmap = nullptr;

  // int redmask = FI_RGBA_RED_MASK;
  int redmask = 0x0000ff;

  // int greenmask = FI_RGBA_GREEN_MASK;
  int greenmask = 0x00ff00;

  // int bluemask = FI_RGBA_BLUE_MASK;
  int bluemask = 0xff0000;

  unsigned int bpp;
  int scanlineBytes;

  if (_format == L_INT8)
  {
    bpp = 8;
    scanlineBytes = _width;
  }
  else if (_format == RGB_INT8)
  {
    bpp = 24;
    redmask = 0xff0000;
    greenmask = 0x00ff00;
    bluemask = 0x0000ff;
    scanlineBytes = _width * 3;
  }
  else if (_format == RGBA_INT8)
  {
    bpp = 32;
    redmask = 0xff000000;
    greenmask = 0x00ff0000;
    bluemask = 0x0000ff00;
    scanlineBytes = _width * 4;
  }
  else if (_format == BGR_INT8)
  {
    bpp = 24;
    redmask = 0x0000ff;
    greenmask = 0x00ff00;
    bluemask = 0xff0000;
    scanlineBytes = _width * 3;
  }
  else
  {
    gzerr << "Unable to handle format[" << _format << "]\n";
    return;
  }

  this->bitmap = FreeImage_ConvertFromRawBits(const_cast<BYTE*>(_data),
      _width, _height, scanlineBytes, bpp, redmask, greenmask, bluemask, true);
}

//////////////////////////////////////////////////
int Image::GetPitch() const
{
  return FreeImage_GetLine(this->bitmap);
}

//////////////////////////////////////////////////
void Image::GetRGBData(unsigned char **_data, unsigned int &_count) const
{
  FIBITMAP *tmp = FreeImage_ConvertTo24Bits(this->bitmap);
  this->GetDataImpl(_data, _count, tmp);
  FreeImage_Unload(tmp);
}

//////////////////////////////////////////////////
void Image::GetData(unsigned char **_data, unsigned int &_count) const
{
  this->GetDataImpl(_data, _count, this->bitmap);
}

//////////////////////////////////////////////////
void Image::GetDataImpl(unsigned char **_data, unsigned int &_count,
                        FIBITMAP *_img) const
{
  int redmask = FI_RGBA_RED_MASK;
  // int bluemask = 0x00ff0000;

  int greenmask = FI_RGBA_GREEN_MASK;
  // int greenmask = 0x0000ff00;

  int bluemask = FI_RGBA_BLUE_MASK;
  // int redmask = 0x000000ff;

  int scanWidth = FreeImage_GetLine(_img);

  if (*_data)
    delete [] *_data;

  _count = scanWidth * FreeImage_GetHeight(_img);
  *_data = new unsigned char[_count];

  FreeImage_ConvertToRawBits(reinterpret_cast<BYTE*>(*_data), _img,
      scanWidth, FreeImage_GetBPP(_img), redmask, greenmask, bluemask, true);

#ifdef FREEIMAGE_COLORORDER
  if (FREEIMAGE_COLORORDER != FREEIMAGE_COLORORDER_RGB)
  {
#else
#ifdef FREEIMAGE_BIGENDIAN
  if (false)
  {
#else
  {
#endif
#endif
/*  FIXME:  why shift by 2 pixels?  this breaks heighmaps by wrapping artificially
    int i = 0;
    for (unsigned int y = 0; y < this->GetHeight(); ++y)
    {
      for (unsigned int x = 0; x < this->GetWidth(); ++x)
      {
        std::swap((*_data)[i], (*_data)[i+2]);
        unsigned int d = FreeImage_GetBPP(this->bitmap)/8;
        i += d;
      }
    }
*/
  }
}

//////////////////////////////////////////////////
unsigned int Image::GetWidth() const
{
  if (!this->Valid())
    return 0;

  return FreeImage_GetWidth(this->bitmap);
}

//////////////////////////////////////////////////
unsigned int Image::GetHeight() const
{
  if (!this->Valid())
    return 0;

  return FreeImage_GetHeight(this->bitmap);
}

//////////////////////////////////////////////////
unsigned int Image::GetBPP() const
{
  if (!this->Valid())
    return 0;

  return FreeImage_GetBPP(this->bitmap);
}

//////////////////////////////////////////////////
Color Image::GetPixel(unsigned int _x, unsigned int _y) const
{
  Color clr;

  if (!this->Valid())
    return clr;

  FREE_IMAGE_COLOR_TYPE type = FreeImage_GetColorType(this->bitmap);

  if (type == FIC_RGB || type == FIC_RGBALPHA)
  {
    RGBQUAD firgb;

    if (FreeImage_GetPixelColor(this->bitmap, _x, _y, &firgb) == FALSE)
    {
      gzerr << "Image: Coordinates out of range["
        << _x << " " << _y << "] \n";
      return clr;
    }

#ifdef FREEIMAGE_COLORORDER
    if (FREEIMAGE_COLORORDER == FREEIMAGE_COLORORDER_RGB)
      clr.Set(firgb.rgbRed, firgb.rgbGreen, firgb.rgbBlue);
    else
      clr.Set(firgb.rgbBlue, firgb.rgbGreen, firgb.rgbRed);
#else
#ifdef FREEIMAGE_BIGENDIAN
    clr.Set(firgb.rgbRed, firgb.rgbGreen, firgb.rgbBlue);
#else
    clr.Set(firgb.rgbBlue, firgb.rgbGreen, firgb.rgbRed);
#endif
#endif
  }
  else
  {
    BYTE byteValue;
    if (FreeImage_GetPixelIndex(this->bitmap, _x, _y, &byteValue) == FALSE)
    {
      gzerr << "Image: Coordinates out of range ["
        << _x << " " << _y << "] \n";
      return clr;
    }

    clr.Set(byteValue, byteValue, byteValue);
  }

  return clr;
}

//////////////////////////////////////////////////
Color Image::GetAvgColor()
{
  unsigned int x, y;
  double rsum, gsum, bsum;
  common::Color pixel;

  rsum = gsum = bsum = 0.0;
  for (y = 0; y < this->GetHeight(); ++y)
  {
    for (x = 0; x < this->GetWidth(); ++x)
    {
      pixel = this->GetPixel(x, y);
      rsum += pixel.r;
      gsum += pixel.g;
      bsum += pixel.b;
    }
  }

  rsum /= (this->GetWidth() * this->GetHeight());
  gsum /= (this->GetWidth() * this->GetHeight());
  bsum /= (this->GetWidth() * this->GetHeight());

  return Color(rsum, gsum, bsum);
}

//////////////////////////////////////////////////
Color Image::GetMaxColor() const
{
  unsigned int x, y;
  Color clr;
  Color maxClr;

  maxClr.Set(0, 0, 0, 0);

  for (y = 0; y < this->GetHeight(); y++)
  {
    for (x = 0; x < this->GetWidth(); x++)
    {
      clr = this->GetPixel(x, y);

      if (clr.r + clr.g + clr.b > maxClr.r + maxClr.g + maxClr.b)
      {
        maxClr = clr;
      }
    }
  }

  return maxClr;
}

//////////////////////////////////////////////////
void Image::Rescale(int _width, int _height)
{
#ifndef _WIN32
  this->bitmap = FreeImage_Rescale(this->bitmap, _width, _height,
      FILTER_LANCZOS3);
#else
  gzerr << "Image::Rescale is not implemented on Windows.\n";
#endif
}

//////////////////////////////////////////////////
bool Image::Valid() const
{
  return this->bitmap != nullptr;
}

//////////////////////////////////////////////////
std::string Image::GetFilename() const
{
  return this->fullName;
}

//////////////////////////////////////////////////
Image::PixelFormat Image::GetPixelFormat() const
{
  Image::PixelFormat fmt = UNKNOWN_PIXEL_FORMAT;
  FREE_IMAGE_TYPE type = FreeImage_GetImageType(this->bitmap);

  unsigned int redMask = FreeImage_GetRedMask(this->bitmap);
  unsigned int bpp = this->GetBPP();

  if (type == FIT_BITMAP)
  {
    if (bpp == 8)
      fmt = L_INT8;
    else if (bpp == 16)
      fmt = L_INT16;
    else if (bpp == 24)
      redMask == 0xff0000 ? fmt = RGB_INT8 : fmt = BGR_INT8;
    else if (bpp == 32)
    {
      redMask == 0xff0000 || redMask == 0xff000000 ?
        fmt = RGBA_INT8 : fmt = BGRA_INT8;
    }
  }
  else if (type == FIT_RGB16)
    fmt = RGB_INT16;
  else if (type == FIT_RGBF)
    fmt = RGB_FLOAT32;
  else if (type == FIT_UINT16 || type == FIT_INT16)
    fmt = L_INT16;

  return fmt;
}

/////////////////////////////////////////////////
Image::PixelFormat Image::ConvertPixelFormat(const std::string &_format)
{
  // Handle old format strings
  if (_format == "L8" || _format == "L_INT8")
    return L_INT8;
  else if (_format == "R8G8B8" || _format == "RGB_INT8")
    return RGB_INT8;

  for (unsigned int i = 0; i < PIXEL_FORMAT_COUNT; ++i)
    if (PixelFormatNames[i] == _format)
      return static_cast<PixelFormat>(i);

  return UNKNOWN_PIXEL_FORMAT;
}
