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

#ifndef IMAGE_HH
#define IMAGE_HH

#ifdef BOOL
#undef BOOL
#endif
#include <FreeImage.h>
#include <string>

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
      public: void GetData(unsigned char **_data, unsigned int &_count);

      /// \brief Get the width
      public: unsigned int GetWidth() const;

      /// \brief Get the height
      public: unsigned int GetHeight() const;

      /// \brief Get the size of one pixel in bits
      public: unsigned int GetBPP() const;

      // \brief Get the size of a row of pixel
      public: int GetPitch() const;

      /// \brief Get the full filename of the image
      public: std::string GetFilename() const;

      /// \brief Get the pixel format
      public: PixelFormat GetPixelFormat() const;


      /// \brief Get a pixel color value
      public: Color GetPixel(unsigned int _x, unsigned int _y);

      /// \brief Get the average color
      public: Color GetAvgColor();

      /// \brief Get the max color
      public: Color GetMaxColor();

      /// \brief Rescale the image
      public: void Rescale(int _width, int _height);

      /// \brief Render this image using opengl
      // public: void RenderOpengl(float destW, float destH);

      /// \brief Returns whether this is a valid image
      public: bool Valid() const;

      /// Count the number of images created. Used for initialising free image
      private: static int count;

      private: FIBITMAP *bitmap;

      private: std::string fullName;
    };
    /// \}
  }
}
#endif
