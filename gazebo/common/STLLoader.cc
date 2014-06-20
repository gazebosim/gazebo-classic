/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include "gazebo/math/Helpers.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/STLLoader.hh"

using namespace gazebo;
using namespace common;


//////////////////////////////////////////////////
  STLLoader::STLLoader()
: MeshLoader()
{
}

//////////////////////////////////////////////////
STLLoader::~STLLoader()
{
}

//////////////////////////////////////////////////
Mesh *STLLoader::Load(const std::string &_filename)
{
  FILE *file = fopen(_filename.c_str(), "r");

  if (!file)
  {
    gzerr << "Unable to open file[" << _filename << "]\n";
    return NULL;
  }

  Mesh *mesh = new Mesh();

  // Try to read ASCII first. If that fails, try binary
  if (!this->ReadAscii(file, mesh))
  {
    fclose(file);
    file = fopen(_filename.c_str(), "r");
    if (!this->ReadBinary(file, mesh))
      gzerr << "Unable to read STL[" << _filename << "]\n";
    fclose(file);
  }

  return mesh;
}

//////////////////////////////////////////////////
bool STLLoader::ReadAscii(FILE *_filein, Mesh *_mesh)
{
  int count;
  char *next;
  float r1;
  float r2;
  float r3;
  float r4;
  char token[LINE_MAX_LEN];
  int width;
  char input[LINE_MAX_LEN];
  bool result = true;

  SubMesh *subMesh = new SubMesh();

  // Read the next line of the file into INPUT.
  while (fgets (input, LINE_MAX_LEN, _filein) != NULL)
  {
    // Advance to the first nonspace character in INPUT.
    for (next = input; *next != '\0' && isspace(*next); next++);

    // Skip blank lines and comments.
    if (*next == '\0' || *next == '#' || *next == '!' || *next == '$')
      continue;

    // Extract the first word in this line.
    sscanf(next, "%s%n", token, &width);

    // Set NEXT to point to just after this token.
    next = next + width;

    // FACET
    if (this->Leqi(token, const_cast<char*>("facet")))
    {
      math::Vector3 normal;

      // Get the XYZ coordinates of the normal vector to the face.
      sscanf(next, "%*s %e %e %e", &r1, &r2, &r3);

      normal.x = r1;
      normal.y = r2;
      normal.z = r3;

      if (fgets (input, LINE_MAX_LEN, _filein) == NULL)
      {
        result = false;
        break;
      }

      for (; result; )
      {
        math::Vector3 vertex;
        if (fgets (input, LINE_MAX_LEN, _filein) == NULL)
        {
          result = false;
          break;
        }

        count = sscanf(input, "%*s %e %e %e", &r1, &r2, &r3);

        if (count != 3)
          break;

        vertex.x = r1;
        vertex.y = r2;
        vertex.z = r3;

        subMesh->AddVertex(vertex);
        subMesh->AddNormal(normal);
        subMesh->AddIndex(subMesh->GetVertexIndex(vertex));
      }

      if (fgets (input, LINE_MAX_LEN, _filein) == NULL)
      {
        result = false;
        break;
      }
    }
    // COLOR
    else if (this->Leqi (token, const_cast<char*>("color")))
    {
      sscanf(next, "%*s %f %f %f %f", &r1, &r2, &r3, &r4);
    }
    // SOLID
    else if (this->Leqi (token, const_cast<char*>("solid")))
    {
    }
    // ENDSOLID
    else if (this->Leqi (token, const_cast<char*>("endsolid")))
    {
    }
    // Unexpected or unrecognized.
    else
    {
      /*printf("\n");
      printf("stl - Fatal error!\n");
      printf(" Unrecognized first word on line.\n");
      */
      result = false;
      break;
    }
  }

  result = subMesh->GetVertexCount() > 0;

  if (result)
    _mesh->AddSubMesh(subMesh);
  else
    delete subMesh;

  return result;
}

//////////////////////////////////////////////////
bool STLLoader::ReadBinary(FILE *_filein, Mesh *_mesh)
{
  int i;
  int iface;
  int face_num;

  SubMesh *subMesh = new SubMesh();

  // 80 byte Header.
  for (i = 0; i < 80; i++)
    static_cast<char>(fgetc(_filein));

  // Number of faces.
  face_num = this->LongIntRead(_filein);

  math::Vector3 normal;
  math::Vector3 vertex;

  // For each (triangular) face,
  // components of normal vector,
  // coordinates of three vertices,
  // 2 byte "attribute".
  for (iface = 0; iface < face_num; iface++)
  {
    if (!this->FloatRead(_filein, normal.x))
      return false;
    if (!this->FloatRead(_filein, normal.y))
      return false;
    if (!this->FloatRead(_filein, normal.z))
      return false;

    if (!this->FloatRead(_filein, vertex.x))
      return false;
    if (!this->FloatRead(_filein, vertex.y))
      return false;
    if (!this->FloatRead(_filein, vertex.z))
      return false;

    subMesh->AddVertex(vertex);
    subMesh->AddNormal(normal);
    subMesh->AddIndex(subMesh->GetVertexCount()-1);

    if (!this->FloatRead(_filein, vertex.x))
      return false;
    if (!this->FloatRead(_filein, vertex.y))
      return false;
    if (!this->FloatRead(_filein, vertex.z))
      return false;
    subMesh->AddVertex(vertex);
    subMesh->AddNormal(normal);
    subMesh->AddIndex(subMesh->GetVertexCount()-1);

    if (!this->FloatRead(_filein, vertex.x))
      return false;
    if (!this->FloatRead(_filein, vertex.y))
      return false;
    if (!this->FloatRead(_filein, vertex.z))
      return false;
    subMesh->AddVertex(vertex);
    subMesh->AddNormal(normal);
    subMesh->AddIndex(subMesh->GetVertexCount()-1);

    uint16_t shortTmp;
    if (!ShortIntRead(_filein, shortTmp))
      return false;
  }

  _mesh->AddSubMesh(subMesh);
  return true;
}

//////////////////////////////////////////////////
bool STLLoader::Leqi(char* _string1, char* _string2)
{
  int i;
  int nchar;
  int nchar1;
  int nchar2;

  nchar1 = strlen(_string1);
  nchar2 = strlen(_string2);

  if (nchar1 < nchar2)
    nchar = nchar1;
  else
    nchar = nchar2;

  // The strings are not equal if they differ over their common length.
  for (i = 0; i < nchar; i++)
    if (toupper (_string1[i]) != toupper (_string2[i]))
      return false;

  // The strings are not equal if the longer one includes nonblanks in the tail.
  if (nchar1 > nchar)
  {
    for (i = nchar; i < nchar1; i++)
      if (_string1[i] != ' ')
        return false;
  }
  else if (nchar2 > nchar)
  {
    for (i = nchar; i < nchar2; i++)
      if (_string2[i] != ' ')
        return false;
  }

  return true;
}

//////////////////////////////////////////////////
int STLLoader::RcolFind(float _a[][COR3_MAX], int _m, int _n, float _r[])
{
  int i;
  int icol;
  int j;

  icol = -1;

  for (j = 0; j < _n; j++)
  {
    for (i = 0; i < _m; i++)
    {
      if (!math::equal(_a[i][j], _r[i]))
        break;
      if (i == _m-1)
        return j;
    }
  }

  return icol;
}

//////////////////////////////////////////////////
bool STLLoader::FloatRead(FILE *_filein, double &_value)
{
  float v;
  if (fread (&v, sizeof(v), 1, _filein) == 0)
    return false;

  _value = v;
  return true;
}

//////////////////////////////////////////////////
uint32_t STLLoader::LongIntRead(FILE *_filein)
{
  union
  {
    uint32_t yint;
    char ychar[4];
  } y;

  y.ychar[0] = fgetc(_filein);
  y.ychar[1] = fgetc(_filein);
  y.ychar[2] = fgetc(_filein);
  y.ychar[3] = fgetc(_filein);

  return y.yint;
}

//////////////////////////////////////////////////
bool STLLoader::ShortIntRead(FILE *_filein, uint16_t &_value)
{
  uint8_t c1;
  uint8_t c2;

  c1 = fgetc(_filein);
  c2 = fgetc(_filein);

  _value = c1 | (c2 << 8);

  return true;
}


