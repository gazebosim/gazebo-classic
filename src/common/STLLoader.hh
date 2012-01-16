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
#ifndef STLLOADER_HH
#define STLLOADER_HH

#include <stdint.h>

#include "common/MeshLoader.hh"

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
    /// \brief Class used to load STL mesh files
    class STLLoader : public MeshLoader
    {
      /// \brief Constructor
      public: STLLoader();

      /// \brief Destructor
      public: virtual ~STLLoader();

      /// \brief Load a mesh
      public: virtual Mesh *Load(const std::string &filename);

      /// Reads an ASCII STL (stereolithography) file.
      private: void ReadAscii(FILE *_filein, Mesh *_mesh);

      /// Reads a binary STL (stereolithography) file.
      private: void ReadBinary (FILE *_filein, Mesh *_mesh);

      /// Compares two strings for equality, disregarding case.
      private: bool Leqi (char* _string1, char* _string2);

      /// Finds if a vector occurs in a table.
      private: int RcolFind (float _a[][COR3_MAX], int _m, int _n, float _r[]);

      /// Reads a long int from a binary file.
      private: uint32_t LongIntRead (FILE *_filein);

      /// Reads a short int from a binary file.
      private: uint16_t ShortIntRead (FILE *_filein);

      /// Read 1 float from a binary file.
      private: float FloatRead (FILE *_filein);

    };
    /// \}
  }
}
#endif

