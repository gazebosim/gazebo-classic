/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <ignition/math/Angle.hh>
#include <ignition/math/Matrix4.hh>
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/common/FBXLoader.hh"

using namespace gazebo;
using namespace common;

// https://code.blender.org/2013/08/fbx-binary-file-format-specification/
// https://wiki.blender.org/index.php/User:Mont29/Foundation/FBX_File_Structure

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
        std::cout << "F[" << this->value.flt << "]\n";
        break;
      case 'D':
        this->type = Type::Double;
        _in.read(reinterpret_cast<char*>(&this->value.dbl), 8);
        std::cout << "D[" << this->value.dbl << "]\n";
        break;
      case 'L':
        this->type = Type::Int64;
        _in.read(reinterpret_cast<char*>(&this->value.int64), 8);
        std::cout << "L[" << this->value.int64 << "]\n";
        break;
      case 'I':
        this->type = Type::Int32;
        _in.read(reinterpret_cast<char*>(&this->value.int32), 4);
        std::cout << "I[" << this->value.int32 << "]\n";
        break;
      case 'Y':
        this->type = Type::Int16;
        _in.read(reinterpret_cast<char*>(&this->value.int16), 2);
        std::cout << "Y[" << this->value.int16 << "]\n";
        break;
      case 'C':
        this->type = Type::Bool;
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
          this->type = Type::String;
          uint32_t strLen;
          _in.read(reinterpret_cast<char*>(&strLen), sizeof(strLen));
          this->stringValue;
          this->stringValue.resize(strLen);
          _in.read(&this->stringValue[0], strLen);
          std::cout << "S/R[" << this->stringValue << "]\n";
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

          std::cout << "Array len[" << this->arrayLen << "] Encoding["
                << encoding << "] COmpressedLen[" << compressedLen << "]\n";

          if (encoding == 0)
          {
            uint32_t stride = 1;
            switch (ctype)
            {
              case 'f':
                this->type = Type::FloatArray;
                stride = 4;
                break;
              case 'd':
                this->type = Type::DoubleArray;
                stride = 8;
                break;
              case 'l':
                this->type = Type::Int64Array;
                stride = 8;
                break;
              case 'i':
                this->type = Type::Int32Array;
                stride = 4;
                break;
              case 'b':
                this->type = Type::BoolArray;
                stride = 1;
                break;
              default:
                gzerr << "Unknown array type[" << ctype << "]\n";
                break;
            }

            this->arrayData = new char[this->arrayLen * stride];
            _in.read(this->arrayData, this->arrayLen * stride);
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
                this->type = Type::FloatArray;
                stride = 4;
                break;
              case 'i':
                this->type = Type::Int32Array;
                stride = 4;
                break;
              case 'd':
                this->type = Type::DoubleArray;
                stride = 8;
                break;
              case 'l':
                this->type = Type::Int64Array;
                stride = 8;
                break;
              case 'b':
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

  public: int64_t Int() const
  {
    if (this->type == Type::Int32)
      return this->value.int32;
    else if (this->type == Type::Int64)
      return this->value.int64;
    return -1;
  }

  public: double Dbl() const
  {
    if (this->type == Type::Float)
      return this->value.flt;
    else if (this->type == Type::Double)
      return this->value.dbl;
    return 0.0;
  }

  public: float Flt() const
  {
    if (this->type == Type::Float)
      return this->value.flt;
    else if (this->type == Type::Double)
      return this->value.dbl;
    return 0.0;
  }

  public: std::string String() const
  {
    if (this->type == Type::String)
      return this->stringValue;
    return "";
  }

  public: bool FillNormals(std::vector<ignition::math::Vector3d> &_n)
  {
    switch (this->type)
    {
      case Type::FloatArray:
        {
          const float *f = reinterpret_cast<const float*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
          {
            // std::cout << "Adding Normal[" << f[i] << " " << f[i+1] << " " << f[i+2]  << "\n";
            _n.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          }
          break;
        }
      case Type::DoubleArray:
        {
          const double *f = reinterpret_cast<const double*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
          {
            // std::cout << "Adding Normal[" << f[i] << " " << f[i+1] << " " << f[i+2]  << "\n";
            _n.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          }
          break;
        }
      case Type::Int32Array:
        {
          const int32_t *f = reinterpret_cast<const int32_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
            _n.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          break;
        }
      case Type::Int64Array:
        {
          const int64_t *f = reinterpret_cast<const int64_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
            _n.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          break;
        }
      default:
        std::cout << "Invalid type!!!\n";
        return false;
    }
    return true;
  }
  public: bool FillVertices(std::vector<ignition::math::Vector3d> &_v)
  {
    switch (this->type)
    {
      case Type::FloatArray:
        {
          const float *f = reinterpret_cast<const float*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
          {
            //std::cout << "Adding Vertex[" << f[i] << " " << f[i+1] << " " << f[i+2]  << "\n";
            _v.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          }
          break;
        }
      case Type::DoubleArray:
        {
          const double *f = reinterpret_cast<const double*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
          {
            //std::cout << "Adding Vertex[" << f[i] << " " << f[i+1] << " " << f[i+2]  << "\n";
            _v.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          }
          break;
        }
      case Type::Int32Array:
        {
          const int32_t *f = reinterpret_cast<const int32_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
            _v.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          break;
        }
      case Type::Int64Array:
        {
          const int64_t *f = reinterpret_cast<const int64_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; i+=3)
            _v.push_back(ignition::math::Vector3d(f[i], f[i+1], f[i+2]));
          break;
        }
      default:
        std::cout << "Invalid type!!!\n";
        return false;
    }
    return true;
  }

  public: bool FillIndices(std::vector<uint64_t> &_i)
  {
    switch (this->type)
    {
      case Type::Int32Array:
        {
          const int32_t *f = reinterpret_cast<const int32_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; ++i)
          {
            // std::cout << "Adding Index[" << f[i] << "]\n";
            int32_t index = f[i];
            if (index < 0)
              index = index ^ -1;
            _i.push_back(index);
          }
          break;
        }
      case Type::Int64Array:
        {
          const int64_t *f = reinterpret_cast<const int64_t*>(this->arrayData);
          for (uint32_t i = 0; i < this->arrayLen; ++i)
          {
            // std::cout << "Adding Index[" << f[i] << "]\n";
            int64_t index = f[i];
            if (index < 0)
              index = index ^ -1;

            _i.push_back(index);
          }
          break;
        }
      default:
        std::cout << "Invalid type!!!\n";
        return false;
    }
    return true;
  }

  public: Type type = Type::Undefined;
  public: SinglePropertyValue value;
  public: std::string stringValue = "";
  public: char *arrayData = nullptr;
  public: uint32_t arrayLen = 0;
};

class FBXNode
{
  public: FBXNode *ChildWithProperty(const std::string &_prop)
  {
    for (const auto p : this->properties)
    {
      if (p->String() == _prop)
        return this;
    }

    for (const auto n : this->children)
    {
      FBXNode *result = n->ChildWithProperty(_prop);
      if (result)
        return result;
    }

    return nullptr;
  }

  public: bool Parse(std::ifstream &_in, const uint32_t _version)
  {
    // \todo: With version >= 7500, these are uint64_t values.
    uint64_t endOffset = 0;
    uint64_t propertyCount = 0;
    uint64_t propertyListLen = 0;
    uint8_t nameLen = 0;

    if (_version >= 7500)
    {
      _in.read(reinterpret_cast<char*>(&endOffset), sizeof(endOffset));
      _in.read(reinterpret_cast<char*>(&propertyCount), sizeof(propertyCount));
      _in.read(reinterpret_cast<char*>(&propertyListLen),
          sizeof(propertyListLen));
    }
    else
    {
      uint32_t eo = 0;
      uint32_t pc = 0;
      uint32_t pll = 0;
      _in.read(reinterpret_cast<char*>(&eo), sizeof(eo));
      _in.read(reinterpret_cast<char*>(&pc), sizeof(pc));
      _in.read(reinterpret_cast<char*>(&pll), sizeof(pll));
      endOffset = eo;
      propertyCount = pc;
      propertyListLen = pll;
    }

    _in.read(reinterpret_cast<char*>(&nameLen), sizeof(nameLen));

    this->name.resize(nameLen);
    _in.read(reinterpret_cast<char*>(&this->name[0]), nameLen);

    std::cout << "Off[" << endOffset << "] PC["
         << propertyCount << "] PLL[" << propertyListLen << "] NameLen["
         << static_cast<int>(nameLen) << "] Name[" << this->name << "]\n";

    if (endOffset == 0)
    {
      std::cout << "This is the NULL record.\n";
      return true;
    }

    // Read the properties
    for (uint32_t i = 0; i < propertyCount; ++i)
    {
      FBXProperty *property = new FBXProperty;
      if (property->Parse(_in))
      {
        this->properties.push_back(property);
      }
      else
      {
        delete property;
        break;
      }
    }

    while (_in.tellg() > 0 && endOffset > static_cast<uint64_t>(_in.tellg()))
    {
       std::cout << "Has children. EndOffset[" << endOffset << "] Pos["
                 << _in.tellg() << "]\n";
      FBXNode *node = new FBXNode;
      if (node->Parse(_in, _version))
      {
        this->children.push_back(node);
      }
      else
      {
        delete node;
        break;
      }
    }
    return true;
  }

  public: void GenerateModels(Mesh *_mesh,
              std::map<int64_t, SubMesh*> &_subMeshMap,
              std::map<int64_t, Material*> &_materialMap,
              std::vector<std::tuple<int64_t, int64_t, std::string>> &_connections)
  {
    if (this->name == "Model")
    {
      int64_t id = this->properties[0]->Int();
      int64_t other = 0;
      FBXNode *scaleNode = this->ChildWithProperty("Lcl Scaling");
      FBXNode *rotNode = this->ChildWithProperty("Lcl Rotation");
      FBXNode *transNode = this->ChildWithProperty("Lcl Translation");
      FBXNode *preRotNode = this->ChildWithProperty("PreRotation");
      FBXNode *geomTransNode = this->ChildWithProperty("GeometricTranslation");
      FBXNode *geomRotNode = this->ChildWithProperty("GeometricRotation");

      ignition::math::Vector3d scale(1, 1, 1);
      ignition::math::Vector3d rot(0, 0, 0);
      ignition::math::Vector3d preRot(0, 0, 0);
      ignition::math::Vector3d trans(0, 0, 0);
      ignition::math::Vector3d geomTrans(0, 0, 0);
      ignition::math::Vector3d geomRot(0, 0, 0);

      if (scaleNode)
      {
        scale.Set(scaleNode->properties[4]->Dbl(),
                  scaleNode->properties[5]->Dbl(),
                  scaleNode->properties[6]->Dbl());
      }
      if (rotNode)
      {
        rot.Set(IGN_DTOR(rotNode->properties[4]->Dbl()),
                IGN_DTOR(rotNode->properties[5]->Dbl()),
                IGN_DTOR(rotNode->properties[6]->Dbl()));
      }
      if (preRotNode)
      {
        preRot.Set(IGN_DTOR(preRotNode->properties[4]->Dbl()),
                   IGN_DTOR(preRotNode->properties[5]->Dbl()),
                   IGN_DTOR(preRotNode->properties[6]->Dbl()));
      }

      if (transNode)
      {
        trans.Set(transNode->properties[4]->Dbl(),
                  transNode->properties[5]->Dbl(),
                  transNode->properties[6]->Dbl());
      }
      if (geomTransNode)
      {
        std::cerr << " -============!!! geometric translation " << std::endl;
        geomTrans.Set(geomTransNode->properties[4]->Dbl(),
                      geomTransNode->properties[5]->Dbl(),
                      geomTransNode->properties[6]->Dbl());
      }
      if (geomRotNode)
      {
        geomRot.Set(geomRotNode->properties[4]->Dbl(),
                      geomRotNode->properties[5]->Dbl(),
                      geomRotNode->properties[6]->Dbl());
      }

      // local transform
      ignition::math::Quaterniond q(rot);
      ignition::math::Quaterniond pq(preRot);
      ignition::math::Matrix4d rotMat(q);
      ignition::math::Matrix4d preRotMat(pq);

      ignition::math::Matrix4d transMat(ignition::math::Matrix4d::Identity);
      transMat.Translate(trans);

      ignition::math::Matrix4d scaleMat(ignition::math::Matrix4d::Identity);
      scaleMat.Scale(scale);

      // geom transform
      ignition::math::Quaterniond gq(geomRot);
      ignition::math::Matrix4d geomMat(geomRot);
      geomMat.Translate(geomTrans);

      // final transform
      ignition::math::Matrix4d finalMat(ignition::math::Matrix4d::Identity);
      finalMat = transMat * preRotMat * rotMat * scaleMat * geomMat;
      // Translation * RotationOffset * RotationPivot * PreRotation
      // * Rotation * PostRotation * RotationPivotInverse
      // * ScalingOffset * ScalingPivot * Scaling * Scaling PivotInverse
      // * GeometricTranslation * GeometricRotation * GeometricScaling.

      for (const auto c : _connections)
      {
        int64_t cfirst = std::get<0>(c);
        int64_t csecond = std::get<1>(c);
        other = 0;
        if (cfirst == id)
          other = csecond;
        else if (csecond == id)
          other = cfirst;

        // Attempt to add a submesh
        if (_subMeshMap.find(other) != _subMeshMap.end())
        {
//          SubMesh *sub = new SubMesh(_subMeshMap[other]);
          SubMesh *sub = _subMeshMap[other];
          sub->Transform(finalMat);

          for (const auto c2 : _connections)
          {
            int64_t c2first = std::get<0>(c2);
            int64_t c2second = std::get<1>(c2);

            int64_t other2 = 0;
            if (c2first == id)
              other2 = c2second;
            else if (c2second == id)
              other2 = c2first;

            if (_materialMap.find(other) != _materialMap.end())
            {
              int matId = _mesh->AddMaterial(_materialMap[other2]);
              std::cout << "Adding material. Model: " << id << " Mat: " << other2
                << std::endl;
              sub->SetMaterialIndex(matId);
            }
          }

          _mesh->AddSubMesh(sub);
        }
      }
    }

    for (const auto n : this->children)
    {
      n->GenerateModels(_mesh, _subMeshMap, _materialMap, _connections);
    }
  }

  public: void GenerateConnections(
              std::vector<std::tuple<int64_t, int64_t, std::string>> &_connections)
  {
    if (this->name == "Connections")
    {
      for (const auto n : this->children)
      {
        if (n->name == "C")
        {
          int64_t from = n->properties[1]->Int();
          int64_t to = n->properties[2]->Int();
          std::string relationship;
          if (n->properties.size() >= 4u)
            relationship = n->properties[3]->String();
          _connections.push_back(std::make_tuple(from, to, relationship));
//          std::cerr << "===============!!!connections: " << n->properties.size() << std::endl;
        }
      }
    }

    for (const auto n : this->children)
    {
      n->GenerateConnections(_connections);
    }
  }

  public: void GenerateMaterials(std::map<int64_t, Material*> &_materialMap)
  {
    if (this->name == "Material")
    {
      int64_t id = this->properties[0]->Int();
      Material *material = new Material();
      _materialMap[id] = material;
      std::cout << "Material ID: " << id << std::endl;
      for (const auto n : this->children)
      {
        if (n->name == "Properties70")
        {
          FBXNode *propNode = n->children[0];
          for (const auto pChild : propNode->children)
          {
            if (pChild->name == "P")
            {
              if (pChild->properties[0]->String() == "AmbientColor")
              {
                Color ambient(pChild->properties[4]->Flt(),
                              pChild->properties[5]->Flt(),
                              pChild->properties[6]->Flt());
                std::cout << "Ambient: " << ambient << std::endl;
                material->SetAmbient(ambient);
              }
              if (pChild->properties[0]->String() == "DiffuseColor")
              {
                Color diffuse(pChild->properties[4]->Flt(),
                              pChild->properties[5]->Flt(),
                              pChild->properties[6]->Flt());
                std::cout << "Diffuse: " << diffuse << std::endl;
                material->SetDiffuse(diffuse);
              }
              if (pChild->properties[0]->String() == "SpecularColor")
              {
                Color spec(pChild->properties[4]->Flt(),
                           pChild->properties[5]->Flt(),
                           pChild->properties[6]->Flt());
                std::cout << "Spec: " << spec << std::endl;
                material->SetSpecular(spec);
              }
              if (pChild->properties[0]->String() == "TransparencyFactor")
              {
                double factor = pChild->properties[4]->Dbl();
                std::cout << "Trans: " << factor << std::endl;
                material->SetTransparency(1-factor);
              }
            }
          }
        }
      }
    }

    for (const auto n : this->children)
    {
      n->GenerateMaterials(_materialMap);
    }
  }

  public: void GenerateGeometry(Mesh *_mesh,
                                std::map<int64_t, SubMesh*> &_subMeshMap)
  {
    if (this->name == "Geometry")
    {
      int64_t id = this->properties[0]->Int();
      SubMesh *subMesh = new SubMesh;
      subMesh->SetName(this->properties[1]->String());
      std::cout << "Generating submesh[" << subMesh->GetName() << "]\n";
      std::vector<ignition::math::Vector3d> vertices;
      std::vector<ignition::math::Vector3d> normals;
      std::vector<uint64_t> indices;
      for (const auto n : this->children)
      {
        if (n->name == "Vertices")
        {
          n->properties[0]->FillVertices(vertices);
        }
        else if (n->name == "PolygonVertexIndex")
        {
          n->properties[0]->FillIndices(indices);
        }
        else if (n->name == "LayerElementNormal")
        {
          for (const auto n2 : n->children)
          {
            if (n2->name == "Normals")
            {
              n2->properties[0]->FillNormals(normals);
            }
          }
        }
      }

      // We are assuming the normals and indices have the same number of
      // values, and that the mapping of normal to vertex is "direct"
      for (uint32_t i = 0; i < indices.size(); ++i)
      {
        subMesh->AddVertex(vertices[indices[i]]);
        subMesh->AddNormal(normals[i]);
        subMesh->AddIndex(i);
      }
      std::cout <<" Adding submesh[" << id << "]\n";
      _subMeshMap[id] = subMesh;
      // _mesh->AddSubMesh(subMesh);
    }

    for (const auto n : this->children)
    {
      n->GenerateGeometry(_mesh, _subMeshMap);
    }
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

  std::cout << "Version[" << version << "]\n";

  struct stat stat_buf;
  stat(_filename.c_str(), &stat_buf);
  int fileSize = stat_buf.st_size;
  std::cout << "File Size[" << fileSize << "]\n";

  std::vector<FBXNode*> nodes;
  while (true)
  {
    FBXNode *node = new FBXNode;
    if (node->Parse(file, version))
      nodes.push_back(node);
    else
      delete node;

    if (file.tellg() < 0)
      break;
    else if (fileSize % 16 == 0)
    {
      if (((static_cast<int>(file.tellg()) + 160 + 16) & ~0xf) >= fileSize)
        break;
    }
    else if (static_cast<int>(file.tellg()) + 160 + 16 >= fileSize)
    {
      break;
    }
  }

  for (const auto n : nodes)
    n->Print("");

  Mesh *mesh = new Mesh();
  std::map<int64_t, SubMesh*> subMeshMap;
  std::map<int64_t, Material*> materialMap;

  std::vector<std::tuple<int64_t, int64_t, std::string>> connections;

  for (const auto n : nodes)
    n->GenerateConnections(connections);

  for (const auto n : nodes)
    n->GenerateGeometry(mesh, subMeshMap);

  for (const auto n : nodes)
    n->GenerateMaterials(materialMap);

  for (const auto n : nodes)
    n->GenerateModels(mesh, subMeshMap, materialMap, connections);


  return mesh;
}
