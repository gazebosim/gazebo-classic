/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <zlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/FBXLoader.hh"

using namespace gazebo;
using namespace common;

union SinglePropertyValue
{
  float flt;
  double dbl;
  int32_t int32;
  int64_t int64;
  int16_t int16;
  bool b;
};

class FBXProperty
{
  public: bool Parse(std::ifstream &_in)
  {
    char type;
    _in.read(&type, sizeof(type));
    switch (type)
    {
      case 'F':
        _in.read(reinterpret_cast<char*>(&this->value.flt), 4);
        std::cout << "F[" << this->value.flt << "]\n";
        break;
      case 'D':
        _in.read(reinterpret_cast<char*>(&this->value.dbl), 8);
        std::cout << "D[" << this->value.dbl << "]\n";
        break;
      case 'L':
        _in.read(reinterpret_cast<char*>(&this->value.int64), 8);
        std::cout << "L[" << this->value.int64 << "]\n";
        break;
      case 'I':
        _in.read(reinterpret_cast<char*>(&this->value.int32), 4);
        std::cout << "I[" << this->value.int32 << "]\n";
        break;
      case 'Y':
        _in.read(reinterpret_cast<char*>(&this->value.int16), 2);
        std::cout << "Y[" << this->value.int16 << "]\n";
        break;
      case 'C':
        int8_t d;
        _in.read(reinterpret_cast<char*>(&d), 1);
        this->value.b = d & 1;
        std::cout << "C[" << this->value.b << "]\n";
        break;
      // 'S': String and 'R': raw bytes.
      // For now, we are shoving this data into an std::string.
      case 'S':
      case 'R':
        {
          uint32_t strLen;
          _in.read(reinterpret_cast<char*>(&strLen), sizeof(strLen));
          std::string str;
          str.resize(strLen);
          _in.read(&str[0], strLen);
          std::cout << "S/R[" << str << "]\n";
          break;
        }
      // 'f', 'd', 'l', 'i', and 'b' represent value arrays of their
      // corresponding captialized type. For example, 'f' is an array
      // of floats.
      case 'f':
      case 'd':
      case 'l':
      case 'i':
      case 'b':
        {
          uint32_t arrayLen;
          uint32_t encoding;
          uint32_t compressedLen;

          // Read in the length of the array
          _in.read(reinterpret_cast<char*>(&arrayLen), sizeof(arrayLen));

          // Read encoding (0: not compressed, 1: compressed)
          _in.read(reinterpret_cast<char*>(&encoding), sizeof(encoding));

          // Read compressed length (number of bytes?)
          _in.read(reinterpret_cast<char*>(&compressedLen),
                   sizeof(compressedLen));

          std::cout << "Array len[" << arrayLen << "] Encoding["
                    << encoding << "] COmpressedLen[" << compressedLen << "]\n";

          if (encoding == 0)
          {
            switch (type)
            {
              case 'f':
                {
                  float *data = new float[arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'd':
                {
                  double *data = new double[arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'l':
                {
                  int64_t *data = new int64_t[arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'i':
                {
                  int32_t *data = new int32_t[arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'b':
                {
                  int8_t *data = new int8_t[arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              default:
                {
                  gzerr << "Unknown array type[" << type << "]\n";
                }
            }
          }
          else
          {
            z_stream zstream;
            zstream.opaque = Z_NULL;
            zstream.zalloc = Z_NULL;
            zstream.zfree  = Z_NULL;
            zstream.data_type = Z_BINARY;

            if (Z_OK != inflateInit(&zstream))
            {
              gzerr << "zlib initialization failed\n";
              return false;
            }

            uint32_t stride = 0;
            switch (type)
            {
              case 'f':
              case 'i':
                stride = 4;
                break;
              case 'd':
              case 'l':
                stride = 8;
                break;
              default:
                return false;
            }

            uint32_t uncompressedLen = arrayLen * stride;
            char *inData = new char[compressedLen];
            char *outData = new char[uncompressedLen];

            _in.read(&inData[0], compressedLen);

            // Bytef is defined by zlib. It should be a typedef of unsigned
            // char
            zstream.next_in = reinterpret_cast<Bytef*>(
                const_cast<char*>(inData));

            zstream.avail_in = compressedLen;

            // uInt is defined by zlib. It should be a typedef of unsigned
            // int
            zstream.avail_out = uncompressedLen;
            zstream.next_out = reinterpret_cast<Bytef*>(outData);
            const int ret = inflate(&zstream, Z_FINISH);

            if (ret != Z_STREAM_END && ret != Z_OK) {
              gzerr << "zlib failed to decompress array data\n";
              return false;
            }

            // end zlib
            inflateEnd(&zstream);
          }
          break;
        }
      default:
        {
          gzerr << "Unknown property type[" << type << "]\n";
          return false;
        }
    }

    return true;
  }

  public: SinglePropertyValue value;
};

class FBXNode
{
  public: bool Parse(std::ifstream &_in)
  {
    // \todo: With version >= 7500, these are uint64_t values.
    uint32_t endOffset = 0;
    uint32_t propertyCount = 0;
    uint32_t propertyListLen = 0;
    uint8_t nameLen = 0;
    std::string name = "";

    _in.read(reinterpret_cast<char*>(&endOffset), sizeof(endOffset));
    _in.read(reinterpret_cast<char*>(&propertyCount), sizeof(propertyCount));
    _in.read(reinterpret_cast<char*>(&propertyListLen),
        sizeof(propertyListLen));

    _in.read(reinterpret_cast<char*>(&nameLen), sizeof(nameLen));

    name.resize(nameLen);
    _in.read(reinterpret_cast<char*>(&name[0]), nameLen);

    std::cout << "Off[" << endOffset << "] PC["
              << propertyCount << "] PLL[" << propertyListLen << "] NameLen["
              << static_cast<int>(nameLen) << "] Name[" << name << "]\n";

    if (endOffset == 0)
    {
      std::cout << "This is the NULL record.\n";
      return false;
    }

    // Read the properties
    for (uint32_t i = 0; i < propertyCount; ++i)
    {
      FBXProperty *property = new FBXProperty;
      if (property->Parse(_in))
      {
        if (i == 0)
          this->id = property->int32;

        this->properties.push_back(property);
      }
      else
      {
        delete property;
        break;
      }
    }

    while (endOffset > _in.tellg())
    {
      std::cout << "Has children. EndOffset[" << endOffset << "] Pos["
                << _in.tellg() << "}\n";
      FBXNode node;
      if (!node.Parse(_in))
        return true;
    }
    return true;
  }

  public: std::vector<FBXProperty*> properties;
  public: std::vectr<FBXN attributeType;
};

/////////////////////////////////////////////////
FBXLoader::FBXLoader()
{
}

/////////////////////////////////////////////////
FBXLoader::~FBXLoader()
{
}

/////////////////////////////////////////////////
Mesh *FBXLoader::Load(const std::string &_filename)
{
  gzdbg << "Loading FBX[" << _filename << "]\n";
  std::ifstream file(_filename, std::ios::in | std::ios::binary);
  if (!file.is_open())
    gzerr << "Unable to open file[" << _filename << "]\n";

  // Ignore the first 23 bytes
  file.ignore(23);

  uint32_t version;
  file.read(reinterpret_cast<char*>(&version), sizeof(version));
  if (version < 7100)
  {
    gzwarn << "FBX file[" << _filename << " with version[" << version
          << "] is unsupported. The version should be >= 7100."
          << "Attempting to read.\n";
  }

  struct stat stat_buf;
  stat(_filename.c_str(), &stat_buf);
  int fileSize = stat_buf.st_size;
  std::cout << "File Size[" << fileSize << "]\n";

  while (true)
  {
    std::cout << "-----------------------------\n";
    FBXNode node;
    node.Parse(file);

    if (static_cast<int>(file.tellg()) + 160 + 16 >= fileSize)
      break;
  }


  Mesh *mesh = new Mesh();
  return mesh;
}
