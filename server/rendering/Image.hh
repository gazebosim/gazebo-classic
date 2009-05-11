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

#ifndef IMAGE_HH
#define IMAGE_HH

#include <string>
#include <FreeImage.h>

#include "Color.hh"

namespace gazebo
{
  class Image
  {
    /// \brief Constructor
    public: Image();

    /// \brief Destructor
    public: virtual ~Image();

    /// \brief Load an image. Return 0 on success
    public: int Load(const std::string &filename);

    /// \brief Set the image from raw data (R8G8B8)
    /// \param data Pointer to the raw image data
    /// \param width Width in pixels
    /// \param height Height in pixels
    /// \param scanline_bytes Size of a image row in bytes
    /// \param bpp Bits per pixels, aka depth
    public: void SetFromData( const unsigned char *data, unsigned int width,
                     unsigned int height, int scanline_bytes, unsigned int bpp);

    /// \brief Get the width
    public: unsigned int GetWidth() const;

    /// \brief Get the height
    public: unsigned int GetHeight() const;

    /// \brief Get the size of one pixel in bits
    public: unsigned int GetBPP() const;

    /// \brief Get a pixel color value
    public: Color GetPixel(unsigned int x, unsigned int y);

    /// \brief Get the average color
    public: Color GetAvgColor();

    /// \brief Get the max color
    public: Color GetMaxColor();

    /// \brief Rescale the image
    public: void Rescale(int width, int height);

    /// \brief Render this image using opengl
    //public: void RenderOpengl(float destW, float destH);

    /// \brief Returns whether this is a valid image
    public: bool Valid() const;

    /// Count the number of images created. Used for initialising free image
    private: static int count;

    private: FIBITMAP *bitmap;
  };
}

#endif
