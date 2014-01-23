/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _STLLOADER_HH_
#define _STLLOADER_HH_

#include <stdint.h>
#include <string>

#include "gazebo/common/MeshLoader.hh"

#define LINE_MAX_LEN 256
#define COR3_MAX 200000
#define ORDER_MAX 10
#define FACE_MAX 200000

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class STLLoader STLLoader.hh common/common.hh
    /// \brief Class used to load STL mesh files
    class STLLoader : public MeshLoader
    {
      /// \brief Constructor
      public: STLLoader();

      /// \brief Destructor
      public: virtual ~STLLoader();

      /// \brief Creates a new mesh and loads the data from a file
      /// \param[in] _filename the mesh file
      public: virtual Mesh *Load(const std::string &_filename);

      /// \brief Reads an ASCII STL (stereolithography) file.
      /// \param[in] _filein the file pointer
      /// \param[out] _mesh the mesh where to load the data
      /// \return true if read was successful
      private: bool ReadAscii(FILE *_filein, Mesh *_mesh);

      /// \brief Reads a binary STL (stereolithography) file.
      /// \param[in] _filein the file pointer
      /// \param[out] the mesh where to load the data
      /// \return true if read was successful
      private: bool ReadBinary(FILE *_filein, Mesh *_mesh);

      /// \brief Compares two strings for equality, disregarding case.
      /// \param[in] _string1 the first string
      /// \param[in] _string2 the seconf string
      /// \return true if the strings are equal (same content)
      private: bool Leqi(char* _string1, char* _string2);

      /// \brief Finds if a vector occurs in a table. This is done using
      /// floating point comparison with the default tolerance of 1e-6
      /// \param[in] _a the vector data
      /// \param[in] _m the number of columns in the table
      /// \param[in] _n the number of rows in the table
      /// \return The column index of the vector
      private: int RcolFind(float _a[][COR3_MAX], int _m, int _n, float _r[]);

      /// \brief Reads a long int from a binary file.
      /// \param[in] _filein the file pointer
      /// \return the value
      private: uint32_t LongIntRead(FILE *_filein);

      /// \brief Reads a short int from a binary file.
      /// \param[in] _filein the file pointer
      /// \param[out] _value the value read
      /// \return true
      private: bool ShortIntRead(FILE *_filein, uint16_t &_value);

      /// \brief Read 1 double precision float from a binary file.
      /// \param[in] _filein the file pointer
      /// \param[out] the value
      /// \return true
      private: bool FloatRead(FILE *_filein, double &_value);
    };
    /// \}
  }
}
#endif

