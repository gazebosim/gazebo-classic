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
 */

#ifndef __IMAGE_HH__
#define __IMAGE_HH__

#ifdef BOOL
#undef BOOL
#endif
#include <FreeImage.h>
#include <string>

#include "common/CommonTypes.hh"
#include "common/Color.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief Encapsulates an image
    class Image
    {
      public: enum PixelFormat {
                UNKNOWN,
                L_INT8,
                L_INT16,
                RGB_INT8,
                RGBA_INT8,
                BGRA_INT8,
                RGB_INT16,
                RGB_INT32,
                BGR_INT8,
                BGR_INT16,
                BGR_INT32,
                R_FLOAT16,
                RGB_FLOAT16,
                R_FLOAT32,
                RGB_FLOAT32,
                BAYER_RGGB8,
                BAYER_RGGR8,
                BAYER_GBRG8,
                BAYER_GRBG8
              };

      /// \brief Constructor
      public: Image(const std::string &_filename="");

      /// \brief Destructor
      public: virtual ~Image();

      /// \brief Load an image. Return 0 on success
      public: int Load(const std::string &_filename);

      /// \brief Set the image from raw data (R8G8B8)
      /// \param data Pointer to the raw image data
      /// \param width Width in pixels
      /// \param height Height in pixels
      /// \param scanline_bytes Size of a image row in bytes
      /// \param bpp Bits per pixels, aka depth
      public: void SetFromData(const unsigned char *data, unsigned int width,
                  unsigned int height, int scanline_bytes, unsigned int bpp)
              GAZEBO_DEPRECATED;

      /// \brief Set the image from raw data
      /// \param _data Pointer to the raw image data
      /// \param _width Width in pixels
      /// \param _height Height in pixels
      /// \param _format Pixel format of the provided data
      public: void SetFromData(const unsigned char *_data, unsigned int _width,
                  unsigned int _height, Image::PixelFormat _format);

      /// \brief Get the image as a data array
      /// \param _data Pointer to a NULL array of char.
      /// \param _count The resulting data array size
      public: void GetData(unsigned char **_data, unsigned int &_count) const;

      /// \brief Get only the RGB data from the image. This will drop the
      /// alpha channel if one is present.
      /// \param _data Pointer to a NULL array of char.
      /// \param _count The resulting data array size
      public: void GetRGBData(unsigned char **_data,
                              unsigned int &_count) const;

      /// \brief Get the width
      /// \return The image width
      public: unsigned int GetWidth() const;

      /// \brief Get the height
      /// \return The image height
      public: unsigned int GetHeight() const;

      /// \brief Get the size of one pixel in bits
      /// \return The BPP of the image
      public: unsigned int GetBPP() const;

      // \brief Get the size of a row of pixel
      /// \return The pitch of the image
      public: int GetPitch() const;

      /// \brief Get the full filename of the image
      /// \return The filename used to load the image
      public: std::string GetFilename() const;

      /// \brief Get the pixel format
      /// \return PixelFormat
      public: PixelFormat GetPixelFormat() const;

      /// \brief Get a pixel color value
      /// \param _x Column location in the image
      /// \param _y Row location in the image
      public: Color GetPixel(unsigned int _x, unsigned int _y);

      /// \brief Get the average color
      /// \return The average color
      public: Color GetAvgColor();

      /// \brief Get the max color
      /// \return The max color
      public: Color GetMaxColor();

      /// \brief Rescale the image
      /// \param _width New image width
      /// \param _height New image height
      public: void Rescale(int _width, int _height);

      /// \brief Returns whether this is a valid image
      public: bool Valid() const;

      /// \brief Implementation of GetData
      private: void GetDataImpl(unsigned char **_data, unsigned int &_count,
                        FIBITMAP *_img) const;

      /// Count the number of images created. Used for initialising free image
      private: static int count;

      private: FIBITMAP *bitmap;

      private: std::string fullName;
    };
    /// \}
  }
}
#endif
