/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Image class
 * Author: Nate Koenig
 * Date: 14 July 2008
 * SVN: $Id$
 */

#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <string.h>

//#include <GL/gl.h>

#include "common/Vector3.hh"
#include "common/GazeboMessage.hh"
#include "common/Image.hh"
#include "common/GazeboConfig.hh"

using namespace gazebo;
using namespace common;


int Image::count = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Image::Image()
{
  if (count == 0)
    FreeImage_Initialise();

  count++;

  this->bitmap = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Image::~Image()
{
  count--;

  if (this->bitmap)
    FreeImage_Unload(this->bitmap);
  this->bitmap = NULL;

  if (count == 0)
    FreeImage_DeInitialise();

}

////////////////////////////////////////////////////////////////////////////////
/// Load
int Image::Load(const std::string &filename)
{
  struct stat st;

  // @todo: fix path search. for now repeat similar path search in simulator 
  std::list<std::string> gazeboPaths=GazeboConfig::Instance()->GetGazeboPaths();
 
  for (std::list<std::string>::iterator iter=gazeboPaths.begin(); 
       iter!=gazeboPaths.end(); ++iter)
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

    for (std::vector<std::string>::iterator piter = pathNames.begin();piter!=pathNames.end();piter++)
    {
      std::string path(*piter);
      DIR *dir=opendir(path.c_str()); 

      //std::cout << "searching path[" << path << "]\n";
      // if directory exist
      if (dir != NULL)
      {
        closedir(dir);

        this->fullName = (((*piter)+"/")+filename);
        // if file exist
        if (stat(this->fullName.c_str(), &st) == 0)
        {
          FREE_IMAGE_FORMAT fifmt = FreeImage_GetFIFFromFilename(this->fullName.c_str());

          if (this->bitmap)
            FreeImage_Unload(this->bitmap);
          this->bitmap = NULL;

          if (fifmt == FIF_PNG)
            this->bitmap = FreeImage_Load(fifmt, this->fullName.c_str(), PNG_DEFAULT);
          else if (fifmt == FIF_JPEG)
            this->bitmap = FreeImage_Load(fifmt, this->fullName.c_str(), JPEG_DEFAULT);
          else if (fifmt == FIF_BMP)
            this->bitmap = FreeImage_Load(fifmt, this->fullName.c_str(), BMP_DEFAULT);
          else
          {
            gzerr(5) << "Unknown image format[" << this->fullName << "]\n";
            return -1;
          }

          return 0;
        }
      }
      
    }
  }

  gzerr(5) << "Unable to open image file[" << filename << "]\n";
  return -1;

}

////////////////////////////////////////////////////////////////////////////////
/// Set the image from raw data (R8G8B8)
void Image::SetFromData( const unsigned char *data, unsigned int width,
    unsigned int height, int scanline_bytes, unsigned int bpp)
{
  //int redmask = FI_RGBA_RED_MASK;
  int redmask = 0xff0000;

  //int greenmask = FI_RGBA_GREEN_MASK;
  int greenmask = 0x00ff00;

  //int bluemask = FI_RGBA_BLUE_MASK;
  int bluemask = 0x0000ff;

  if (this->bitmap)
  {
    FreeImage_Unload(this->bitmap);
  }
  this->bitmap = NULL;

  this->bitmap = FreeImage_ConvertFromRawBits((BYTE*)data, width, height, scanline_bytes, bpp, redmask, greenmask, bluemask);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the image as a data array
void Image::GetData( unsigned char **data, unsigned int &count )
{
  int redmask = FI_RGBA_RED_MASK;
  //int bluemask = 0x00ff0000;

  int greenmask = FI_RGBA_GREEN_MASK;
  //int greenmask = 0x0000ff00;

  int bluemask = FI_RGBA_BLUE_MASK;
  //int redmask = 0x000000ff;

  int scan_width = FreeImage_GetPitch(this->bitmap);

  if (*data)
    delete [] *data;

  count = scan_width * this->GetHeight();
  *data = new unsigned char[count];

  FreeImage_ConvertToRawBits( (BYTE*)(*data), this->bitmap, 
      scan_width, this->GetBPP(),redmask, greenmask, bluemask, true);

#ifdef FREEIMAGE_COLORORDER
    if (FREEIMAGE_COLORORDER == FREEIMAGE_COLORORDER_RGB)
      return;
#else
#ifdef FREEIMAGE_BIGENDIAN
      return;
#endif
#endif

  int i=0;
  for (unsigned int y=0; y<this->GetHeight(); y++)
  {
    for (unsigned int x=0; x < this->GetWidth(); x++)
    {
      std::swap( (*data)[i], (*data)[i+2]); 
      i += 4;
    }
  }
      
}

////////////////////////////////////////////////////////////////////////////////
/// Get the width
unsigned int Image::GetWidth() const
{
  if (!this->Valid())
    return 0;

  return FreeImage_GetWidth( this->bitmap );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the height
unsigned int Image::GetHeight() const
{
  if (!this->Valid())
    return 0;

  return FreeImage_GetHeight( this->bitmap );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the size of one pixel in bits
unsigned int Image::GetBPP() const
{
  if (!this->Valid())
    return 0;

  return FreeImage_GetBPP( this->bitmap );
}

////////////////////////////////////////////////////////////////////////////////
/// Get a pixel color value
Color Image::GetPixel(unsigned int x, unsigned int y)
{
  Color clr;

  if (!this->Valid())
    return clr;

  FREE_IMAGE_COLOR_TYPE type = FreeImage_GetColorType(this->bitmap);

  if (type == FIC_RGB || type == FIC_RGBALPHA)
  {
    RGBQUAD firgb;

    if (FreeImage_GetPixelColor( this->bitmap, x, y, &firgb ) == FALSE)
    {
      std::cerr << "Image: Coordinates out of range[" 
                << x << " " << y << "] \n";
      return clr;
    }

#ifdef FREEIMAGE_COLORORDER
    if (FREEIMAGE_COLORORDER == FREEIMAGE_COLORORDER_RGB)
      clr.Set(  firgb.rgbRed, firgb.rgbGreen, firgb.rgbBlue);
    else
      clr.Set( firgb.rgbBlue, firgb.rgbGreen, firgb.rgbRed);
#else
#ifdef FREEIMAGE_BIGENDIAN
    clr.Set(  firgb.rgbRed, firgb.rgbGreen, firgb.rgbBlue);
#else
    clr.Set( firgb.rgbBlue, firgb.rgbGreen, firgb.rgbRed);
#endif
#endif

  }
  else
  {
    BYTE byteValue;
    if (FreeImage_GetPixelIndex(this->bitmap, x, y, &byteValue) == FALSE)
    {
      std::cerr << "Image: Coordinates out of range   [" 
        << x << " " << y << "] \n";
      return clr;
    }

    clr.Set( byteValue, byteValue, byteValue);
  }

  return clr;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the average color
Color Image::GetAvgColor()
{
  unsigned int x, y;
  Color clrSum;

  for (y = 0; y < this->GetHeight(); y++)
  {
    for (x = 0; x < this->GetWidth(); x++)
    {
      clrSum += this->GetPixel(x,y);
    }
  }

  return clrSum / (this->GetWidth() * this->GetHeight());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the max color
Color Image::GetMaxColor()
{
  unsigned int x, y;
  Color clr;
  Color maxClr;

  maxClr.Set(0,0,0,0);

  for (y = 0; y < this->GetHeight(); y++)
  {
    for (x = 0; x < this->GetWidth(); x++)
    {
      clr = this->GetPixel(x,y);

      if ( clr.R() + clr.G() + clr.B() > maxClr.R() + maxClr.G() + maxClr.B())
      {
        maxClr = clr;
      }
    }
  }

  return maxClr;
}

////////////////////////////////////////////////////////////////////////////////
/// Rescale the image
void Image::Rescale(int width, int height)
{
  this->bitmap = FreeImage_Rescale(this->bitmap, width, height, FILTER_BICUBIC);
}

////////////////////////////////////////////////////////////////////////////////
/// Render this image using opengl
/*void Image::RenderOpengl(float destW, float destH)
{
  if (!this->Valid())
  {
    printf("Invalid\n");
    return;
  }

  FIBITMAP *resizedBitmap = FreeImage_Rescale(this->bitmap, destW, destH, 
                                              FILTER_LANCZOS3);

  Vector3d pt;
  Color clr;

  unsigned int x,y;
  unsigned int w = this->GetWidth();
  unsigned int h = this->GetHeight();

  //float xRes = destW / w;
  //float yRes = destH / h;

  pt.z = 0;

  glBegin(GL_TRIANGLE_STRIP);
  for (y = h-1; y > 0; y--)
  {
    for (x=0; x < w; x++)
    {
      pt.x = x;// * xRes;
      pt.y = (h-y);// * yRes;

      clr = this->GetPixel(x,y);
      glColor3ub( clr.r, clr.g, clr.b );
      glVertex3f(pt.x, pt.y, pt.z);

      pt.y = (h-y-1);// * yRes;
      clr = this->GetPixel(x,y-1);
      glColor3ub( clr.r, clr.g, clr.b );
      glVertex3f(pt.x, pt.y, pt.z);
    }
    glVertex3f(pt.x, pt.y, pt.z);
    glVertex3f(pt.x, pt.y, pt.z);
  }
  glEnd();
  
  FreeImage_Unload(resizedBitmap);
}*/

////////////////////////////////////////////////////////////////////////////////
/// Returns whether this is a valid image
bool Image::Valid() const
{
  return this->bitmap != NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the full filename of the image
std::string Image::GetFilename() const
{
  return this->fullName;
}


