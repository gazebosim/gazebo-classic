/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "common/Console.hh"
#include "common/Mesh.hh"
#include "common/STLLoader.hh"

using namespace gazebo;
using namespace common;


//////////////////////////////////////////////////
/// Constructor
  STLLoader::STLLoader()
: MeshLoader()
{
}

//////////////////////////////////////////////////
/// Destructor
STLLoader::~STLLoader()
{
}

//////////////////////////////////////////////////
/// Load a mesh
Mesh *STLLoader::Load(const std::string &filename)
{
  Mesh *mesh = new Mesh();

  FILE *file = fopen(filename.c_str(), "r");

  /*
     std::string extension;
     extension = filename.substr(filename.rfind(".")+1, filename.size());
     std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
     if (extension == "stl" || extension == "stla")
     this->ReadAscii(file, mesh);
     else if (extension == "stlb")
     this->ReadBinary(file, mesh);
     */

  this->ReadBinary(file, mesh);

  fclose(file);

  return mesh;
}

//////////////////////////////////////////////////
/// Reads an ASCII STL (stereolithography) file.
void STLLoader::ReadAscii(FILE *_filein, Mesh *_mesh)
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

  SubMesh *subMesh = new SubMesh();
  _mesh->AddSubMesh(subMesh);

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
    if (this->Leqi(token, static_cast<char*>("facet")))
    {
      math::Vector3 normal;

      // Get the XYZ coordinates of the normal vector to the face.
      sscanf(next, "%*s %e %e %e", &r1, &r2, &r3);

      normal.x = r1;
      normal.y = r2;
      normal.z = r3;

      if (fgets (input, LINE_MAX_LEN, _filein) == NULL)
        gzerr << "Error..\n";

      for (;;)
      {
        math::Vector3 vertex;
        if (fgets (input, LINE_MAX_LEN, _filein) == NULL)
          gzerr << "Error...\n";

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
        printf("Error...\n");
    }
    // COLOR
    else if (this->Leqi (token, static_cast<char*>("color")))
    {
      sscanf(next, "%*s %f %f %f %f", &r1, &r2, &r3, &r4);
    }
    // SOLID
    else if (this->Leqi (token, static_cast<char*>("solid")))
    {
    }
    // ENDSOLID
    else if (this->Leqi (token, static_cast<char*>("endsolid")))
    {
    }
    // Unexpected or unrecognized.
    else
    {
      printf("\n");
      printf("stl - Fatal error!\n");
      printf(" Unrecognized first word on line.\n");
    }
  }
}

//////////////////////////////////////////////////
/// Reads a binary STL (stereolithography) file.
void STLLoader::ReadBinary(FILE *_filein, Mesh *_mesh)
{
  int i;
  int iface;
  int face_num;

  SubMesh *subMesh = new SubMesh();
  _mesh->AddSubMesh(subMesh);

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
    normal.x = this->FloatRead(_filein);
    normal.y = this->FloatRead(_filein);
    normal.z = this->FloatRead(_filein);

    vertex.x = this->FloatRead(_filein);
    vertex.y = this->FloatRead(_filein);
    vertex.z = this->FloatRead(_filein);
    subMesh->AddVertex(vertex);
    subMesh->AddNormal(normal);
    subMesh->AddIndex(subMesh->GetVertexCount()-1);

    vertex.x = this->FloatRead(_filein);
    vertex.y = this->FloatRead(_filein);
    vertex.z = this->FloatRead(_filein);
    subMesh->AddVertex(vertex);
    subMesh->AddNormal(normal);
    subMesh->AddIndex(subMesh->GetVertexCount()-1);

    vertex.x = this->FloatRead(_filein);
    vertex.y = this->FloatRead(_filein);
    vertex.z = this->FloatRead(_filein);
    subMesh->AddVertex(vertex);
    subMesh->AddNormal(normal);
    subMesh->AddIndex(subMesh->GetVertexCount()-1);

    ShortIntRead(_filein);
  }
}

//////////////////////////////////////////////////
/// Compares two strings for equality, disregarding case.
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
/// Finds if a vector occurs in a table.
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
      if (_a[i][j] != _r[i])
        break;
      if (i == _m-1)
        return j;
    }
  }

  return icol;
}

//////////////////////////////////////////////////
/// Read 1 float from a binary file.
float STLLoader::FloatRead(FILE *_filein)
{
  float rval;
  if (fread (&rval, sizeof(rval), 1, _filein) == 0)
    printf("Error...\n");

  return rval;
}

//////////////////////////////////////////////////
/// Reads a long int from a binary file.
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
/// Reads a short int from a binary file.
uint16_t STLLoader::ShortIntRead(FILE *_filein)
{
  uint8_t c1;
  uint8_t c2;
  uint16_t ival;

  c1 = fgetc(_filein);
  c2 = fgetc(_filein);

  ival = c1 | (c2 << 8);

  return ival;
}

