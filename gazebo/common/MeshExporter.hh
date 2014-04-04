/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef MESHEXPORTER_HH
#define MESHEXPORTER_HH

#include <string>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class Mesh;
    class Skeleton;
    class SkeletonNode;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class MeshExporter MeshExporter.hh common/common.hh
    /// \brief Base class for exporting meshes
    class GAZEBO_VISIBLE MeshExporter
    {
      /// \brief Constructor
      public: MeshExporter();

      /// \brief Destructor
      public: virtual ~MeshExporter();

      /// \brief Load a 3D mesh
      /// \param[in] _filename the path to the mesh
      /// \return a pointer to the created mesh
      public: virtual Mesh *Load(const std::string &_filename) = 0;
    };
    /// \}
  }
}
#endif

