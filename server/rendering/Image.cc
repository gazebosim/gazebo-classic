/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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

//#include <GL/gl.h>

#include "Vector3.hh"
#include "GazeboMessage.hh"
#include "Image.hh"

using namespace gazebo;

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

  if (stat(filename.c_str(), &st) != 0)
  {
    gzerr(5) << "Unable to open image file[" << filename << "]\n";
    return -1;
  }

  FREE_IMAGE_FORMAT fifmt = FreeImage_GetFIFFromFilename(filename.c_str());

  if (this->bitmap)
    FreeImage_Unload(this->bitmap);
  this->bitmap = NULL;

  if (fifmt == FIF_PNG)
    this->bitmap = FreeImage_Load(fifmt, filename.c_str(), PNG_DEFAULT);
  else if (fifmt == FIF_JPEG)
    this->bitmap = FreeImage_Load(fifmt, filename.c_str(), JPEG_DEFAULT);
  else if (fifmt == FIF_BMP)
    this->bitmap = FreeImage_Load(fifmt, filename.c_str(), BMP_DEFAULT);
  else
  {
    std::cerr << "Unknown image format[" << filename << "]\n";
    return -1;
  }

  return 0;
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

  unsigned int colorsUsed = FreeImage_GetImageType(this->bitmap);

  if (colorsUsed == 0)
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
      std::cerr << "Image: Coordinates out of range[" 
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
