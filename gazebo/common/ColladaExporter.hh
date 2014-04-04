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

#ifndef _COLLADAEXPORTER_HH_
#define _COLLADAEXPORTER_HH_

#include <map>
#include <string>
#include <vector>

#include "gazebo/common/MeshExporter.hh"
#include "gazebo/math/MathTypes.hh"
#include "gazebo/util/system.hh"

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    class Material;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ColladaExporter ColladaExporter.hh common/common.hh
    /// \brief Class used to export Collada mesh files
    class GAZEBO_VISIBLE ColladaExporter : public MeshExporter
    {
      /// \brief Constructor
      public: ColladaExporter();

      /// \brief Destructor
      public: virtual ~ColladaExporter();

      /// \brief Export a mesh
      /// \param[in] _mesh Gazebo mesh to export
      public: virtual void Export(const Mesh *_mesh);

      /// \brief Export asset element
      /// \return A pointer to the asset xml element
      private: TiXmlElement *ExportAsset();

      /// \brief Export geometry source
      /// TODO
      private: void FillGeometrySources(
          const gazebo::common::SubMesh *_subMesh,
          TiXmlElement *Mesh, int type, const char *meshID);

      /// \brief Export geometry source
      /// TODO
      private: void FillTextureSource(
          const gazebo::common::SubMesh *_subMesh,
          TiXmlElement *Mesh,
          const char *meshID);

      /// \brief Export library geometries element
      /// \return A pointer to the library_geometries xml element
      private: TiXmlElement *ExportGeometries();

      /// \brief Export library images element
      /// \return A pointer to the library_images xml element
      private: TiXmlElement *ExportImages();

      /// \brief Export library materials element
      /// \return A pointer to the library_materials xml element
      private: TiXmlElement *ExportMaterials();

      /// \brief Export library effects element
      /// \return A pointer to the library_effects xml element
      private: TiXmlElement *ExportEffects();

      /// \brief Export library visual scenes element
      /// \return A pointer to the library_visual_scenes xml element
      private: TiXmlElement *ExportVisualScenes();

      /// \brief Export scene element
      /// \return A pointer to the scene xml element
      private: TiXmlElement *ExportScene();

      /// \brief scaling factor
      private: double meter;

      /// \brief scaling factor
      private: const Mesh *mesh;
    };
    /// \}
  }
}
#endif
