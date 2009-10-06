#include <algorithm>
#include <iostream>
#include <float.h>
#include <sys/stat.h>
#include <list>

#include "Simulator.hh"
#include "GazeboConfig.hh"
#include "MeshLoader.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Load a mesh from file
void MeshLoader::Load(const std::string &filename)
{
  struct stat st;
  std::string fullname;

  std::list<std::string> gazeboPaths;

  gazeboPaths = Simulator::Instance()->GetGazeboConfig()->GetGazeboPaths();

  printf("Num Paths[%d]\n",gazeboPaths.size()); 
  for (std::list<std::string>::iterator iter=gazeboPaths.begin(); 
       iter!=gazeboPaths.end(); ++iter)
  {
    fullname = (*iter)+"/Media/models/"+filename;
    std::cout << "Fullname[" << fullname << "]\n";
    if (stat(fullname.c_str(), &st) == 0)
    {
      printf("Loading mesh from file[%s]\n",fullname.c_str());

      std::string extension = fullname.substr(fullname.rfind(".")+1, 
          fullname.size());
      if (extension == "mesh")
        this->LoadOgre(fullname);
      else
        this->ivcon.Load(fullname, this->vertices, this->indices);
      break;
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Get the maximun X,Y,Z values
Vector3 MeshLoader::GetMax() const
{
  Vector3 max;
  std::vector<Vector3>::const_iterator iter;

  max.x = -FLT_MAX;
  max.y = -FLT_MAX;
  max.z = -FLT_MAX;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); iter++)
  {
    max.x = std::max(max.x, (*iter).x);
    max.y = std::max(max.y, (*iter).y);
    max.z = std::max(max.z, (*iter).z);
  }

  return max;
}

////////////////////////////////////////////////////////////////////////////////
// Get the minimum X,Y,Z values
Vector3 MeshLoader::GetMin() const
{
  Vector3 min;
  std::vector<Vector3>::const_iterator iter;

  min.x = FLT_MAX;
  min.y = FLT_MAX;
  min.z = FLT_MAX;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); iter++)
  {
    min.x = std::min(min.x, (*iter).x);
    min.y = std::min(min.y, (*iter).y);
    min.z = std::min(min.z, (*iter).z);
  }

  return min;
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of vertices
unsigned int MeshLoader::GetNumVertices() const
{
  return this->vertices.size();
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of indicies
unsigned int MeshLoader::GetNumIndices() const
{
  return this->indices.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Fill vertex and index arrays
void MeshLoader::FillArrays(float **varray, unsigned int **iarray)
{
  if (*varray == NULL)
    *varray = new float[this->vertices.size() * 3];
  if (*iarray == NULL)
    *iarray = new unsigned int[this->indices.size()];

  for (unsigned int i=0; i < this->vertices.size(); i++)
  {
    (*varray)[i*3+0] = this->vertices[i].x;
    (*varray)[i*3+1] = this->vertices[i].y;
    (*varray)[i*3+2] = this->vertices[i].z;
  }

  for (unsigned int i=0; i < this->indices.size(); i++)
    (*iarray)[i] = this->indices[i];
}

////////////////////////////////////////////////////////////////////////////////
// Load an ogre mesh file
void MeshLoader::LoadOgre(const std::string &filename)
{
  FILE *file = fopen(filename.c_str(), "rb");

  if (this->ReadFileHeader(file))
  {
    MeshChunk chunk;
    chunk.Read(file);

    if (chunk.id == 0x3000)
      this->ReadMesh(file);
    else
      printf("Unable to process chunk with id[%d]\n", chunk.id);
  }

  fclose(file);
  file = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Read a submesh
void MeshLoader::ReadSubMesh(FILE *file)
{
  // Ignore the material name
  std::string materialName = ReadString(file);

  bool useSharedVertices = ReadValue<bool>(file);

  int indexCount = ReadValue<int>(file);

  bool idx32Bit = ReadValue<bool>(file);

  if (indexCount > 0)
  {
    std::vector<unsigned int>::iterator maxIter;
    maxIter = std::max_element(this->indices.begin(), this->indices.end());
    unsigned int offset = 0;
    if (maxIter != this->indices.end())
      offset = *maxIter + 1;

    if (idx32Bit)
    {
      std::list<unsigned int> values;
      std::list<unsigned int>::iterator iter;

      values = ReadValues<unsigned int>(file, indexCount);

      for (iter = values.begin(); iter != values.end(); iter++)
        this->indices.push_back( (*iter) + offset);
    }
    else
    {
      std::list<unsigned short> values;
      std::list<unsigned short>::iterator iter;
      values = ReadValues<unsigned short>(file, indexCount);

      for (iter = values.begin(); iter != values.end(); iter++)
        this->indices.push_back( (*iter) + offset);
    }
  }

  // Global vertices have not been specified, so load this submesh's vertex
  // data
  if (!useSharedVertices)
  {
    MeshChunk chunk;
    chunk.Read(file);
    if (chunk.id != M_GEOMETRY)
    {
      printf("Missing geometry data in mesh file");
      exit(0);
    }

    this->ReadGeometry(file);
  }

  // Skip the remaining data
  if (!feof(file))
  {
    MeshChunk chunk;
    chunk.Read(file);

    while (!feof(file) &&
        (chunk.id == M_SUBMESH_BONE_ASSIGNMENT ||
         chunk.id == M_SUBMESH_OPERATION ||
         chunk.id == M_SUBMESH_TEXTURE_ALIAS))
    {
      // Skip this data
      fseek(file, chunk.length - chunk.SizeOf(), SEEK_CUR);

      if (!feof(file))
        chunk.Read(file);
    }

    // Backpedal a little to the start of the chunk
    if (!feof(file))
      fseek(file, -chunk.SizeOf(), SEEK_CUR);
  }

}

////////////////////////////////////////////////////////////////////////////////
// Read the geometry (vertices, normals, and optionally texture coords)
void MeshLoader::ReadGeometry(FILE *file)
{ 
  unsigned short index;
  MeshChunk chunk;
  std::vector<unsigned short> types;
  std::vector<unsigned short> semantics;
  std::vector<unsigned short> offsets;
  std::vector<unsigned short> sources;

  unsigned int vertexCount = this->ReadValue<unsigned int>(file);

  //Read optional geom parameters
  if (!feof(file))
  {
    chunk.Read(file);
    while (!feof(file) &&
        (chunk.id == M_GEOMETRY_VERTEX_DECLRATION ||
         chunk.id == M_GEOMETRY_VERTEX_BUFFER))
    {

      // VERTEX_DECLARTION tells what type of information is provided
      if (chunk.id == M_GEOMETRY_VERTEX_DECLRATION)
      {
        chunk.Read(file);
        while (!feof(file) && chunk.id == M_GEOMETRY_VERTEX_ELEMENT)
        {
          // The source
          unsigned short source = ReadValue<unsigned short>(file);

          // The data type of the data used to store the vertex info
          unsigned short type = ReadValue<unsigned short>(file);

          // The meaning of the data (position, normal, etc)
          unsigned short semantic = ReadValue<unsigned short>(file);

          // Offset in bytes where this element is located in the buffer
          unsigned short offset = ReadValue<unsigned short>(file);

          // We can ignore the index
          index = ReadValue<unsigned short>(file);

          sources.push_back(source);
          types.push_back(type);
          semantics.push_back(semantic);
          offsets.push_back(offset);

          /*printf("Source[%d] Type[%d] Semantic[%d] Offset[%d] Index[%d]\n",
              source, type, semantic,offset, index);
              */

          if (!feof(file))
            chunk.Read(file);
        }

        // backpeddle to start of non-submesh stream
        if (!feof(file))
          fseek(file, -chunk.SizeOf(), SEEK_CUR);
      }
      // VERTEX_BUFFER contains the vertex data
      else if (chunk.id == M_GEOMETRY_VERTEX_BUFFER)
      {
        unsigned short bindIndex, vertexSize;

        bindIndex = ReadValue<unsigned short>(file);
        vertexSize = ReadValue<unsigned short>(file);

        chunk.Read(file);
        if (chunk.id != M_GEOMETRY_VERTEX_BUFFER_DATA)
        {
          printf("Can't find vertex buffer data area\n");
          exit(0);
        }

        // Read the data buffer 
        unsigned char *vbuf = new unsigned char[vertexCount * vertexSize];
        if (fread(vbuf, vertexSize, vertexCount, file) < vertexCount)
          printf("Error reading the vertex buffer\n");

        Vector3 vec;
        unsigned short semantic, type, offset;

        // Extract the information from the buffer
        for (unsigned int v = 0; v < vertexCount; v++)
        {
          for (unsigned int i = 0; i < types.size(); i++)
          {
            if (sources[i] != bindIndex)
              continue;

            semantic = semantics[i];
            type = types[i];
            offset = offsets[i];
            switch (semantic)
            {
              // VES_POSITION
              case 1:
                {
                  // TODO: I'm assuming that all vertex data will be
                  // a three tuple of floats. Should use the "type"
                  // variable
                  float *ptr = (float*)(vbuf + offset);
                  vec.x = *(ptr++); vec.y = *(ptr++); vec.z = *(ptr++);
                  this->vertices.push_back(vec);
                  //vbuf += sizeof(float)*3;
                  break;
                }
                // VES_NORMAL
              case 4:
                {
                  // TODO: I'm assuming that all normal data will be
                  // a three tuple of floats. Should use the "type"
                  // variable
                  float *ptr = (float*)(vbuf + offset);
                  vec.x = *(ptr++); vec.y = *(ptr++); vec.z = *(ptr++);
                  this->normals.push_back(vec);
                  //vbuf += sizeof(float)*3;
                  break;
                }
                // Ignore all other values
              default:
                break;
            }
          }
          vbuf += vertexSize;
        }
      }

      if (!feof(file))
        chunk.Read(file);
    }

    if (!feof(file))
      fseek(file, -chunk.SizeOf(), SEEK_CUR);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Read the mesh. A mesh can be composed of multiple submeshes. Each submesh
// contains a list of indices, and optionally a set of vertices. A global
// set of vertices can be specified, and then each submesh would just have
// a list of indices that reference the global vertices.
bool MeshLoader::ReadMesh(FILE *file)
{
  MeshChunk chunk;

  if (ReadValue<bool>(file))
    printf("Warning: this doesn't fully support animated meshes\n");

  if (!feof(file))
  {
    chunk.Read(file);

    while (!feof(file) &&
          (chunk.id == M_GEOMETRY ||
           chunk.id == M_SUBMESH ||
           chunk.id == M_MESH_SKELETON_LINK ||
           chunk.id == M_MESH_BONE_ASSIGNMENT ||
           chunk.id == M_MESH_LOD ||
           chunk.id == M_MESH_BOUNDS ||
           chunk.id == M_SUBMESH_NAME_TABLE ||
           chunk.id == M_EDGE_LISTS ||
           chunk.id == M_POSES ||
           chunk.id == M_ANIMATIONS ||
           chunk.id == M_TABLE_EXTREMES))
    {
      switch (chunk.id)
      {
        case M_GEOMETRY:
          {
            // Global vertex data
            this->ReadGeometry(file);
            break;
          }
        case M_SUBMESH:
          {
            // Load a submesh
            this->ReadSubMesh(file);
            break;
          }
        // Skip all this data
        case M_MESH_SKELETON_LINK:
        case M_MESH_BONE_ASSIGNMENT:
        case M_MESH_BOUNDS:
        case M_POSES:
        case M_ANIMATIONS:
        case M_TABLE_EXTREMES:
        case M_SUBMESH_NAME_TABLE:
        case M_EDGE_LISTS:
        case M_MESH_LOD:
          fseek(file, chunk.length - chunk.SizeOf(), SEEK_CUR);
          break;
      };

      if (!feof(file))
        chunk.Read(file);
    }

    if (!feof(file))
      fseek(file, -chunk.SizeOf(), SEEK_CUR);
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Render the mesh
/*void MeshLoader::Render()
{
  glColor3f(0.0,0.0,0.0);

  glBegin(GL_TRIANGLES);
  for (int i=0; i < this->indices.size();i++)
  {
    glVertex3f(this->vertices[ this->indices[i] ].x, 
               this->vertices[ this->indices[i] ].y, 
               this->vertices[ this->indices[i] ].z);
  }
  glEnd();

  glColor3f(1.0,0.0,1.0);
  glPointSize(2.0);
  glBegin(GL_POINTS);
  for (int i=0; i < this->vertices.size(); i++)
    glVertex3f(this->vertices[i].x, this->vertices[i].y, this->vertices[i].z);
  glEnd();

}*/
