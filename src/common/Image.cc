/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>

#include <iostream>

#include "math/Vector3.hh"
#include "common/Console.hh"
#include "common/Image.hh"
#include "common/SystemPaths.hh"

using namespace gazebo;
using namespace common;


int Image::count = 0;

//////////////////////////////////////////////////
Image::Image(const std::string &_filename)
{
  if (count == 0)
    FreeImage_Initialise();

  count++;

  this->bitmap = NULL;
  if (!_filename.empty())
    this->Load(_filename);
}

//////////////////////////////////////////////////
Image::~Image()
{
  count--;

  if (this->bitmap)
    FreeImage_Unload(this->bitmap);
  this->bitmap = NULL;

  if (count == 0)
    FreeImage_DeInitialise();
}

//////////////////////////////////////////////////
int Image::Load(const std::string &_filename)
{
  struct stat st;

  // @todo: fix path search. for now repeat similar path search in simulator
  std::list<std::string> gazeboPaths =
    SystemPaths::Instance()->GetGazeboPaths();

  for (std::list<std::string>::iterator iter = gazeboPaths.begin();
      iter!= gazeboPaths.end(); ++iter)
  {
    std::vector<std::string> pathNames;
    pathNames.push_back((*iter)+"/Media");
    pathNames.push_back((*iter)+"/Media/fonts");
    pathNames.push_back((*iter)+"/Media/materials/programs");
    pathNames.push_back((*iter)+"/Media/materials/scripts");
    pathNames.push_back((*iter)+"/Media/materials/textures");
    pathNames.push_back((*iter)+"/Media/models");
    pathNames.push_back((*iter)+"/Media/sets");
    pathNames.push_back((*iter)+"/Media/maps");

    for (std::vector<std::string>::iterator piter = pathNames.begin();
         piter!= pathNames.end(); ++piter)
    {
      std::string path(*piter);
      DIR *dir = opendir(path.c_str());

      // if directory exist
      if (dir != NULL)
      {
        closedir(dir);

        this->fullName = (((*piter)+"/")+_filename);
        // if file exist
        if (stat(this->fullName.c_str(), &st) == 0)
        {
          FREE_IMAGE_FORMAT fifmt =
            FreeImage_GetFIFFromFilename(this->fullName.c_str());

          if (this->bitmap)
            FreeImage_Unload(this->bitmap);
          this->bitmap = NULL;

          if (fifmt == FIF_PNG)
            this->bitmap =
              FreeImage_Load(fifmt, this->fullName.c_str(), PNG_DEFAULT);
          else if (fifmt == FIF_JPEG)
            this->bitmap =
              FreeImage_Load(fifmt, this->fullName.c_str(), JPEG_DEFAULT);
          else if (fifmt == FIF_BMP)
            this->bitmap = FreeImage_Load(fifmt, this->fullName.c_str(),
                BMP_DEFAULT);
          else
          {
            gzerr << "Unknown image format[" << this->fullName << "]\n";
            return -1;
          }

          return 0;
        }
      }
    }
  }

  gzerr << "Unable to open image file[" << _filename << "]\n";
  return -1;
}

//////////////////////////////////////////////////
void Image::SetFromData(const unsigned char *data, unsigned int width,
    unsigned int height, int scanline_bytes, unsigned int bpp)
{
  // int redmask = FI_RGBA_RED_MASK;
  int redmask = 0xff0000;

  // int greenmask = FI_RGBA_GREEN_MASK;
  int greenmask = 0x00ff00;

  // int bluemask = FI_RGBA_BLUE_MASK;
  int bluemask = 0x0000ff;

  if (this->bitmap)
  {
    FreeImage_Unload(this->bitmap);
  }
  this->bitmap = NULL;

  this->bitmap = FreeImage_ConvertFromRawBits(const_cast<BYTE*>(data),
      width, height, scanline_bytes, bpp, redmask, greenmask, bluemask);
}

//////////////////////////////////////////////////
void Image::GetData(unsigned char **_data, unsigned int &_count)
{
  int redmask = FI_RGBA_RED_MASK;
  // int bluemask = 0x00ff0000;

  int greenmask = FI_RGBA_GREEN_MASK;
  // int greenmask = 0x0000ff00;

  int bluemask = FI_RGBA_BLUE_MASK;
  // int redmask = 0x000000ff;

  int scan_width = FreeImage_GetPitch(this->bitmap);

  if (*_data)
    delete [] *_data;

  _count = scan_width * this->GetHeight();
  *_data = new unsigned char[_count];

  FreeImage_ConvertToRawBits(reinterpret_cast<BYTE*>(*_data), this->bitmap,
      scan_width, this->GetBPP(), redmask, greenmask, bluemask, true);

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

    int i = 0;
    for (unsigned int y = 0; y < this->GetHeight(); ++y)
    {
      for (unsigned int x = 0; x < this->GetWidth(); ++x)
      {
        std::swap((*_data)[i], (*_data)[i+2]);
        i += 3;
      }
    }
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
Color Image::GetPixel(unsigned int _x, unsigned int _y)
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
      rsum += pixel.R();
      gsum += pixel.G();
      bsum += pixel.B();
    }
  }

  rsum /= (this->GetWidth() * this->GetHeight());
  gsum /= (this->GetWidth() * this->GetHeight());
  bsum /= (this->GetWidth() * this->GetHeight());

  return Color(rsum, gsum, bsum);
}

//////////////////////////////////////////////////
Color Image::GetMaxColor()
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

      if (clr.R() + clr.G() + clr.B() > maxClr.R() + maxClr.G() + maxClr.B())
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
  this->bitmap = FreeImage_Rescale(this->bitmap, _width, _height,
      FILTER_BICUBIC);
}

//////////////////////////////////////////////////
/*void Image::RenderOpengl(float destW, float destH)
  {
  if (!this->Valid())
  {
  printf("Invalid\n");
  return;
  }

  FIBITMAP *resizedBitmap = FreeImage_Rescale(this->bitmap, destW, destH,
  FILTER_LANCZOS3);

  math::Vector3d pt;
  Color clr;

  unsigned int x, y;
  unsigned int w = this->GetWidth();
  unsigned int h = this->GetHeight();

// float xRes = destW / w;
// float yRes = destH / h;

pt.z = 0;

glBegin(GL_TRIANGLE_STRIP);
for (y = h-1; y > 0; y--)
{
for (x = 0; x < w; x++)
{
pt.x = x;// * xRes;
pt.y = (h-y);// * yRes;

clr = this->GetPixel(x, y);
glColor3ub(clr.r, clr.g, clr.b);
glVertex3f(pt.x, pt.y, pt.z);

pt.y = (h-y-1);// * yRes;
clr = this->GetPixel(x, y-1);
glColor3ub(clr.r, clr.g, clr.b);
glVertex3f(pt.x, pt.y, pt.z);
}
glVertex3f(pt.x, pt.y, pt.z);
glVertex3f(pt.x, pt.y, pt.z);
}
glEnd();

FreeImage_Unload(resizedBitmap);
}*/

//////////////////////////////////////////////////
bool Image::Valid() const
{
  return this->bitmap != NULL;
}

//////////////////////////////////////////////////
std::string Image::GetFilename() const
{
  return this->fullName;
}
