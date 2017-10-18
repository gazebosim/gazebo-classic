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
  public: enum class Type
          {
            Float = 0,
            Double,
            Int64,
            Int32,
            Int16,
            Bool,
            String,
            FloatArray,
            DoubleArray,
            Int64Array,
            Int32Array,
            BoolArray,
            Undefined
          };

  public: bool Parse(std::ifstream &_in)
  {
    char ctype;
    _in.read(&ctype, sizeof(ctype));
    switch (ctype)
    {
      case 'F':
        this->type = Type::Float;
        _in.read(reinterpret_cast<char*>(&this->value.flt), 4);
        // std::cout << "F[" << this->value.flt << "]\n";
        break;
      case 'D':
        this->type = Type::Double;
        _in.read(reinterpret_cast<char*>(&this->value.dbl), 8);
        // std::cout << "D[" << this->value.dbl << "]\n";
        break;
      case 'L':
        this->type = Type::Int64;
        _in.read(reinterpret_cast<char*>(&this->value.int64), 8);
        // std::cout << "L[" << this->value.int64 << "]\n";
        break;
      case 'I':
        this->type = Type::Int32;
        _in.read(reinterpret_cast<char*>(&this->value.int32), 4);
        // std::cout << "I[" << this->value.int32 << "]\n";
        break;
      case 'Y':
        this->type = Type::Int16;
        _in.read(reinterpret_cast<char*>(&this->value.int16), 2);
        // std::cout << "Y[" << this->value.int16 << "]\n";
        break;
      case 'C':
        this->type = Type::Bool;
        int8_t d;
        _in.read(reinterpret_cast<char*>(&d), 1);
        this->value.b = d & 1;
        // std::cout << "C[" << this->value.b << "]\n";
        break;
      // 'S': String and 'R': raw bytes.
      // For now, we are shoving this data into an std::string.
      case 'S':
      case 'R':
        {
          this->type = Type::String;
          uint32_t strLen;
          _in.read(reinterpret_cast<char*>(&strLen), sizeof(strLen));
          this->stringValue;
          this->stringValue.resize(strLen);
          _in.read(&this->stringValue[0], strLen);
          // std::cout << "S/R[" << str << "]\n";
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
          uint32_t encoding;
          uint32_t compressedLen;

          // Read in the length of the array
          _in.read(reinterpret_cast<char*>(&this->arrayLen), sizeof(this->arrayLen));

          // Read encoding (0: not compressed, 1: compressed)
          _in.read(reinterpret_cast<char*>(&encoding), sizeof(encoding));

          // Read compressed length (number of bytes?)
          _in.read(reinterpret_cast<char*>(&compressedLen),
                   sizeof(compressedLen));

          // std::cout << "Array len[" << this->arrayLen << "] Encoding["
          //       << encoding << "] COmpressedLen[" << compressedLen << "]\n";

          if (encoding == 0)
          {
            switch (ctype)
            {
              case 'f':
                {
                  this->type = Type::FloatArray;
                  float *data = new float[this->arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      this->arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'd':
                {
                  this->type = Type::DoubleArray;
                  double *data = new double[this->arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      this->arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'l':
                {
                  this->type = Type::Int64Array;
                  int64_t *data = new int64_t[this->arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      this->arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'i':
                {
                  this->type = Type::Int32Array;
                  int32_t *data = new int32_t[this->arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      this->arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              case 'b':
                {
                  this->type = Type::BoolArray;
                  int8_t *data = new int8_t[this->arrayLen];
                  _in.read(reinterpret_cast<char*>(&data),
                      this->arrayLen * sizeof(data[0]));
                  delete [] data;
                  break;
                }
              default:
                {
                  gzerr << "Unknown array type[" << ctype << "]\n";
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

            uint32_t stride = 1;
            switch (ctype)
            {
              case 'f':
                std::cout << "Parsing a float array\n";
                this->type = Type::FloatArray;
                stride = 4;
                break;
              case 'i':
                std::cout << "Parsing an int32 array\n";
                this->type = Type::Int32Array;
                stride = 4;
                break;
              case 'd':
                std::cout << "Parsing a double array\n";
                this->type = Type::DoubleArray;
                stride = 8;
                break;
              case 'l':
                std::cout << "Parsing a int64 array\n";
                this->type = Type::Int64Array;
                stride = 8;
                break;
              case 'b':
                std::cout << "Parsing a bool array\n";
                this->type = Type::BoolArray;
                stride = 1;
                break;
              default:
                return false;
            }

            uint32_t uncompressedLen = this->arrayLen * stride;
            char *inData = new char[compressedLen];
            if (this->arrayData)
              delete [] this->arrayData;
            this->arrayData = new char[uncompressedLen];

            _in.read(&inData[0], compressedLen);

            // Bytef is defined by zlib. It should be a typedef of unsigned
            // char
            zstream.next_in = reinterpret_cast<Bytef*>(
                const_cast<char*>(inData));

            zstream.avail_in = compressedLen;

            // uInt is defined by zlib. It should be a typedef of unsigned
            // int
            zstream.avail_out = uncompressedLen;
            zstream.next_out = reinterpret_cast<Bytef*>(this->arrayData);
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
          gzerr << "Unknown property type[" << ctype << "]\n";
          return false;
        }
    }

    return true;
  }

  public: void Print(const std::string &_prefix)
  {
    switch (this->type)
    {
      case Type::Float:
        std::cout << _prefix << "Flt: " << this->value.flt << std::endl;
        break;
      case Type::Double:
        std::cout << _prefix << "Dbl: " << this->value.dbl << std::endl;
        break;
      case Type::Int32:
        std::cout << _prefix << "I32: " << this->value.int32 << std::endl;
        break;
      case Type::Int64:
        std::cout << _prefix << "I64: " << this->value.int64 << std::endl;
        break;
      case Type::Bool:
        std::cout << _prefix << "Bln: " << this->value.b << std::endl;
        break;
      case Type::String:
        std::cout << _prefix << "Str: " << this->stringValue << std::endl;
        break;
      case Type::FloatArray:
        {
          std::cout << "Float Array[" << this->arrayLen << "]\n";
          const float *f = reinterpret_cast<const float*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; ++i)
            std::cout << "F[" << f[i] << "] ";
          std::cout << std::endl;
          break;
        }
      case Type::DoubleArray:
        {
          std::cout << "Double Array[" << this->arrayLen << "]\n";
          const double *f = reinterpret_cast<const double*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; ++i)
            std::cout << "D[" << f[i] << "] ";
          std::cout << std::endl;
          break;
        }
      case Type::Int32Array:
        {
          std::cout << "Int32 Array[" << this->arrayLen << "]\n";
          const int32_t *f = reinterpret_cast<const int32_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; ++i)
            std::cout << "I[" << f[i] << "] ";
          std::cout << std::endl;
          break;
        }
      case Type::Int64Array:
        {
          std::cout << "Int64 Array[" << this->arrayLen << "]\n";
          const int64_t *f = reinterpret_cast<const int64_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; ++i)
            std::cout << "L[" << f[i] << "] ";
          std::cout << std::endl;
          break;
        }
      case Type::BoolArray:
        {
          const int8_t *f = reinterpret_cast<const int8_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; ++i)
            std::cout << "B[" << static_cast<int>(f[i]) << "] ";
          std::cout << std::endl;
          break;
        }
      default:
        std::cout << "Unknown type!!!\n";
        break;
    }
  }

  public: Type type = Type::Undefined;
  public: SinglePropertyValue value;
  public: std::string stringValue = "";
  public: char *arrayData = nullptr;
  public: uint32_t arrayLen = 0;
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

    _in.read(reinterpret_cast<char*>(&endOffset), sizeof(endOffset));
    _in.read(reinterpret_cast<char*>(&propertyCount), sizeof(propertyCount));
    _in.read(reinterpret_cast<char*>(&propertyListLen),
        sizeof(propertyListLen));

    _in.read(reinterpret_cast<char*>(&nameLen), sizeof(nameLen));

    this->name.resize(nameLen);
    _in.read(reinterpret_cast<char*>(&this->name[0]), nameLen);

    // std::cout << "Off[" << endOffset << "] PC["
    //           << propertyCount << "] PLL[" << propertyListLen << "] NameLen["
    //           << static_cast<int>(nameLen) << "] Name[" << this->name << "]\n";

    if (endOffset == 0)
    {
      // std::cout << "This is the NULL record.\n";
      return true;
    }

    // Read the properties
    for (uint32_t i = 0; i < propertyCount; ++i)
    {
      FBXProperty *property = new FBXProperty;
      std::cout << "Parsing prop\n";
      if (property->Parse(_in))
      {
        this->properties.push_back(property);
      }
      else
      {
        std::cout << "Deleteing property\n";
        delete property;
        // break;
      }
    }

    while (endOffset > _in.tellg())
    {
      // std::cout << "Has children. EndOffset[" << endOffset << "] Pos["
      //           << _in.tellg() << "}\n";
      FBXNode *node = new FBXNode;
      if (node->Parse(_in))
      {
        this->children.push_back(node);
      }
      else
      {
        std::cout << "Deleting node\n";
        delete node;
        // return true;
      }
    }
    return true;
  }

  public: void Print(const std::string _prefix)
  {
    std::cout << _prefix << "Node Name: " << this->name << "\n";
    for (const auto p : this->properties)
    {
      p->Print(_prefix + "  ");
    }

    for (const auto n : this->children)
    {
      n->Print(_prefix + "  ");
    }
  }

  public: std::string name;
  public: std::vector<FBXProperty*> properties;
  public: std::vector<FBXNode*> children;
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

  std::vector<FBXNode*> nodes;
  while (true)
  {
    FBXNode *node = new FBXNode;
    if (node->Parse(file))
      nodes.push_back(node);
    else
      delete node;

    if (static_cast<int>(file.tellg()) + 160 + 16 >= fileSize)
      break;
  }

  for (const auto n : nodes)
    n->Print("");


  Mesh *mesh = new Mesh();
  return mesh;
}
