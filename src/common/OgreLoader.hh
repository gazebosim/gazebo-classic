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
/* Desc: A class to load OGRE meshes
 * Author: Nate Koenig
 * Date: 05 Oct 2009
 * SVN: $Id$
 */

#ifndef OGRELOADER_HH
#define OGRELOADER_HH

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <list>
#include <vector>
#include <stdint.h>

#include "MeshLoader.hh"
#include "GazeboError.hh"
#include "common/Vector3.hh"

#define M_MESH 0x3000
#define M_SUBMESH 0x4000
#define M_SUBMESH_OPERATION  0x4010
#define M_SUBMESH_BONE_ASSIGNMENT 0x4100
#define M_SUBMESH_TEXTURE_ALIAS  0x4200
#define M_GEOMETRY 0x5000
#define M_MESH_SKELETON_LINK 0x6000
#define M_MESH_BONE_ASSIGNMENT 0x7000
#define M_MESH_LOD 0x8000
#define M_MESH_BOUNDS 0x9000
#define M_SUBMESH_NAME_TABLE 0xA000
#define M_EDGE_LISTS 0xB000
#define M_POSES 0xC000
#define M_ANIMATIONS 0xD000
#define M_TABLE_EXTREMES 0xE000
#define M_GEOMETRY_VERTEX_DECLRATION 0x5100
#define M_GEOMETRY_VERTEX_BUFFER 0x5200
#define M_GEOMETRY_VERTEX_ELEMENT  0x5110
#define M_GEOMETRY_VERTEX_BUFFER_DATA  0x5210

namespace gazebo
{
	namespace common
{
  class Mesh;

  /// \brief This class will load vertex information for a mesh
  class OgreLoader : public MeshLoader
  {
    /// \brief Constructor
    public: OgreLoader();

    /// \brief Destructor
    public: virtual ~OgreLoader();

    /// \brief Load a mesh from file
    public: virtual Mesh *Load(const std::string &filename);

    /// \brief Load an ogre mesh file
    private: void LoadOgre(const std::string &filename);
           
    /// \brief Read the file header
    private: bool ReadFileHeader(FILE *file)
             {
               uint16_t id;
  
               if (fread(&id, sizeof(id), 1, file) < 1)
                 printf("Error in ReadFileHeader\n");
  
               if (id == 0x1000)
               {
                 std::string version = ReadString(file);
                 //printf("Version %s\n",version.c_str());
                 return true;
               }
  
               return false;
             }
  
    /// \brief Read a single value
    private: template<typename T>
             T ReadValue(FILE *file)
             {
               T result;
               if (fread(&result, sizeof(T), 1, file) < 1)
                 gzthrow( "Error in ReadValue");
               return result;
             }
  
    /// \brief Read a list of values
    private: template<typename T>
             std::list<T> ReadValues(FILE *file, uint32_t count)
             {
               std::list<T> result;
               for (uint32_t i=0; i < count; i++)
                 result.push_back(ReadValue<T>(file));
               return result;
             }
  
    /// \brief Read a string value
    private: std::string ReadString(FILE *file)
               {
                 std::string result;
                 char *line = NULL;
                 size_t junk, lineSize = 0;
  
                 lineSize = getline(&line, &junk, file);
                 line[lineSize-1] = '\0';
  
                 result = line;
                 free(line);
  
                 return result;
               }
  
    /// \brief Read the mesh
    private: bool ReadMesh(FILE *file, Mesh *mesh);
  
    /// \brief Read a submesh
    private: void ReadSubMesh(FILE *file, Mesh *mesh);
  
    /// \brief Read a geometry
    private: void ReadGeometry(FILE *file, Mesh *mesh, SubMesh *submesh);
    private: SubMesh *gSubMesh;
  };

  
  class MeshChunk
  {
    public: uint16_t id;
    public: uint32_t length;
    public: size_t SizeOf() const
            {
              return sizeof(this->id) + sizeof(this->length);
            }
  
    public: bool Read(FILE *file)
            {
              if (!feof(file) && 
                  fread(&this->id, sizeof(this->id), 1, file) < 1)
                return false;
                
              if (!feof(file) && 
                  fread(&this->length, sizeof(this->length), 1, file) < 1)
                return false;

              return true;
            }
  };
}

}
#endif
