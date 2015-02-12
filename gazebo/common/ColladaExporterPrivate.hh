/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_COLLADAEXPORTER_PRIVATE_HH_
#define _GAZEBO_COLLADAEXPORTER_PRIVATE_HH_

#include <map>
#include <string>
#include <vector>

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    class Material;

    /// \brief Private data for the ColladaExporter class
    class  ColladaExporterPrivate
    {
      /// \brief Gazebo mesh
      public: const Mesh *mesh;

      /// \brief Material count
      public: unsigned int materialCount;

      /// \brief SubMesh count
      public: unsigned int subMeshCount;

      /// \brief File path
      public: std::string path;

      /// \brief File name
      public: std::string filename;

      /// \brief True to export texture images to '../materials/textures'
      /// folder
      public: bool exportTextures;
    };
  }
}
#endif

