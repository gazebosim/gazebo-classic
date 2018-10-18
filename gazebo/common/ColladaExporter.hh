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

#ifndef _GAZEBO_COLLADAEXPORTER_HH_
#define _GAZEBO_COLLADAEXPORTER_HH_

#include <map>
#include <string>
#include <vector>

#include "gazebo/common/MeshExporter.hh"
#include "gazebo/util/system.hh"

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    class Material;
    class ColladaExporterPrivate;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ColladaExporter ColladaExporter.hh common/common.hh
    /// \brief Class used to export Collada mesh files
    class GZ_COMMON_VISIBLE ColladaExporter : public MeshExporter
    {
      /// \brief Geometry types
      public: enum GeometryType {POSITION, NORMAL, UVMAP};

      /// \brief Constructor
      public: ColladaExporter();

      /// \brief Destructor
      public: virtual ~ColladaExporter();

      /// \brief Export a mesh to a file
      /// \param[in] _mesh Pointer to the mesh to be exported
      /// \param[in] _filename Exported file's path and name
      /// \param[in] _exportTextures True to export texture images to
      /// '../materials/textures' folder
      public: virtual void Export(const Mesh *_mesh,
          const std::string &_filename, bool _exportTextures);

      /// \brief Export asset element
      /// \param[in] _assetXml Pointer to the asset XML instance
      private: void ExportAsset(TiXmlElement *_assetXml);

      /// \brief Export geometry source
      /// \param[in] _subMesh Pointer to a submesh
      /// \param[in] _meshXml Pointer to the mesh XML instance
      /// \param[in] _type POSITION, NORMAL or UVMAP
      /// \param[in] _meshID Mesh ID (mesh_<number>)
      private: void ExportGeometrySource(
          const gazebo::common::SubMesh *_subMesh,
          TiXmlElement *_meshXml, GeometryType _type, const char *_meshID);

      /// \brief Export library geometries element
      /// \param[in] libraryGeometriesXml Pointer to the library geometries
      /// XML instance
      private: void ExportGeometries(TiXmlElement *_libraryGeometriesXml);

      /// \brief Export library images element
      /// \param[in] _libraryImagesXml Pointer to the library images XML
      /// instance
      /// \return integer, number of images
      private: int ExportImages(TiXmlElement *_libraryImagesXml);

      /// \brief Export library materials element
      /// \param[in] _libraryMaterialsXml Pointer to the library materials XML
      /// instance
      private: void ExportMaterials(TiXmlElement *_libraryMaterialsXml);

      /// \brief Export library effects element
      /// \param[in] _libraryEffectsXml Pointer to the library effects XML
      /// instance
      private: void ExportEffects(TiXmlElement *_libraryEffectsXml);

      /// \brief Export library visual scenes element
      /// \param[in] _libraryVisualScenesXml Pointer to the library visual
      /// scenes XML instance
      private: void ExportVisualScenes(TiXmlElement *_libraryVisualScenesXml);

      /// \brief Export scene element
      /// \param[in] _sceneXml Pointer to the scene XML instance
      private: void ExportScene(TiXmlElement *_sceneXml);

      /// \internal
      /// \brief Pointer to private data.
      private: ColladaExporterPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
