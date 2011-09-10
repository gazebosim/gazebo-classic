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

#ifndef COLLADALOADER_HH
#define COLLADALOADER_HH

#include "common/MeshLoader.hh"
#include "math/MathTypes.hh"

class TiXmlElement;

namespace gazebo
{
	namespace common
  {
    /// \addtogroup gazebo_common Common 
    /// \{

    /// \brief Class used to load Collada mesh files
    class ColladaLoader : public MeshLoader
    {
      /// \brief Constructor
      public: ColladaLoader();
  
      /// \brief Destructor
      public: virtual ~ColladaLoader();
  
      /// \brief Load a mesh
      public: virtual Mesh *Load( const std::string &filename );
 
      private: void LoadGeometries(TiXmlElement *xml, Mesh *mesh);
      private: TiXmlElement *GetElementId(TiXmlElement *_parent, 
                                        const std::string &_id);

      private: void GetVertices(const std::string &_id, 
                                std::vector<math::Vector3> &_values);
      private: void GetNormals(const std::string &_id, 
                               std::vector<math::Vector3> &_values);
      private: void GetTexCoords(const std::string &_id, 
                                 std::vector<math::Vector2d> &_values);

      private: TiXmlElement *colladaXml;
    };
  }
}

#endif
