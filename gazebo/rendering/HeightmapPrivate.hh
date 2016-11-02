/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RENDERING_HEIGHTMAPPRIVATE_HH_
#define _GAZEBO_RENDERING_HEIGHTMAPPRIVATE_HH_

#include <string>
#include <vector>
#include <boost/filesystem.hpp>

#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/RenderTypes.hh"

namespace Ogre
{
  class PageManager;
  class PagedWorld;
  class TerrainGlobalOptions;
  class TerrainGroup;
  class TerrainPaging;
}

namespace gazebo
{
  namespace common
  {
    class HeightmapData;
  }

  namespace rendering
  {
    /// \internal
    /// \brief Custom terrain material generator for GLSL terrains.
    /// A custom material generator that lets Gazebo use GLSL shaders
    /// (as opposed to the default Cg shaders provided by Ogre) for rendering
    /// terrain.
    class GzTerrainMatGen : public Ogre::TerrainMaterialGeneratorA
    {
      /// \brief Constructor
      public: GzTerrainMatGen();

      /// \brief Destructor
      public: virtual ~GzTerrainMatGen();

      /// \brief Shader model 2 profile target.
      public: class SM2Profile :
              public Ogre::TerrainMaterialGeneratorA::SM2Profile
      {
        /// \brief Constructor
        public: SM2Profile(Ogre::TerrainMaterialGenerator *_parent,
                    const Ogre::String &_name, const Ogre::String &_desc);

        /// \brief Destructor
        public: virtual ~SM2Profile();

        public: Ogre::MaterialPtr generate(const Ogre::Terrain *_terrain);

        public: Ogre::MaterialPtr generateForCompositeMap(
                    const Ogre::Terrain *_terrain);

        public: void UpdateParams(const Ogre::MaterialPtr &_mat,
                                  const Ogre::Terrain *_terrain);

        public: void UpdateParamsForCompositeMap(const Ogre::MaterialPtr &_mat,
                                                 const Ogre::Terrain *_terrain);

        protected: virtual void addTechnique(const Ogre::MaterialPtr &_mat,
                       const Ogre::Terrain *_terrain, TechniqueType _tt);

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#endif  // ifdef __clang__
        /// \brief Utility class to help with generating shaders for GLSL.
        /// The class contains a collection of functions that are used to
        /// dynamically generate a complete vertex or fragment shader program
        /// in a string format.
        protected: class ShaderHelperGLSL :
            public Ogre::TerrainMaterialGeneratorA::SM2Profile::ShaderHelperGLSL
        {
          public: virtual Ogre::HighLevelGpuProgramPtr generateVertexProgram(
                      const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                      TechniqueType _tt);

          public: virtual Ogre::HighLevelGpuProgramPtr generateFragmentProgram(
                      const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                      TechniqueType _tt);

          public: virtual void updateParams(const SM2Profile *_prof,
                      const Ogre::MaterialPtr &_mat,
                      const Ogre::Terrain *_terrain, bool _compositeMap);

          protected: virtual void generateVpHeader(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateVpFooter(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateVertexProgramSource(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void defaultVpParams(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         const Ogre::HighLevelGpuProgramPtr &_prog);

          protected: virtual unsigned int generateVpDynamicShadowsParams(
                         unsigned int _texCoordStart, const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateVpDynamicShadows(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateFpHeader(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateFpLayer(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType tt,
                         Ogre::uint _layer,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateFpFooter(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateFpDynamicShadowsParams(
                         Ogre::uint *_texCoord, Ogre::uint *_sampler,
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateFpDynamicShadowsHelpers(
                         const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType tt,
                         Ogre::StringStream &_outStream);

          protected: void generateFpDynamicShadows(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateFragmentProgramSource(
                         const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void updateVpParams(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         const Ogre::GpuProgramParametersSharedPtr &_params);

          private: Ogre::String GetChannel(Ogre::uint _idx);
        };

        // Needed to allow access from ShaderHelperGLSL to protected members
        // of SM2Profile.
        friend ShaderHelperGLSL;

        /// Keeping the CG shader for reference.
        /// \brief Utility class to help with generating shaders for Cg / HLSL.
        /// Original implementation from Ogre that generates Cg shaders
        protected: class ShaderHelperCg :
            public Ogre::TerrainMaterialGeneratorA::SM2Profile::ShaderHelperCg
        {
          public: virtual Ogre::HighLevelGpuProgramPtr generateFragmentProgram(
                      const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                      TechniqueType _tt);

          public: virtual Ogre::HighLevelGpuProgramPtr generateVertexProgram(
                      const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                      TechniqueType _tt);

          protected: virtual void generateVpHeader(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateVpFooter(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateVertexProgramSource(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void defaultVpParams(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         const Ogre::HighLevelGpuProgramPtr &_prog);

          protected: virtual unsigned int generateVpDynamicShadowsParams(
                         unsigned int _texCoordStart, const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringStream &_outStream);

          protected: virtual void generateVpDynamicShadows(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringStream &_outStream);
        };

        // Needed to allow access from ShaderHelperCg to protected members
        // of SM2Profile.
        friend ShaderHelperCg;

#ifdef __clang__
#pragma clang diagnostic pop

// ifdef __clang__
#endif
      };
    };

    /// \internal
    /// \brief Custom terrain material generator.
    /// A custom material generator that lets user specify their own material
    /// script for rendering the heightmap.
    class TerrainMaterial : public Ogre::TerrainMaterialGenerator
    {
      /// \brief Constructor
      /// \param[in] _materialName Name of material
      public: TerrainMaterial(const std::string &_materialName);

      /// \brief Set terrain material
      /// \param[in] _materialName Name of material
      public: void setMaterialByName(const std::string &_materialname);

      /// \brief Subclassed to provide profile-specific material generation
      class Profile : public Ogre::TerrainMaterialGenerator::Profile
      {
        /// \brief Constructor
        /// \param[in] _parent Ogre terrain material generator object
        /// \param[in] _name Name of the profile
        /// \param[on] _desc Profile description
        public: Profile(Ogre::TerrainMaterialGenerator *_parent,
            const Ogre::String &_name, const Ogre::String &_desc);

        /// \brief Destructor.
        public: ~Profile();

        // Documentation Inherited
        public: bool isVertexCompressionSupported() const;

        // Documentation Inherited
        public: Ogre::MaterialPtr generate(const Ogre::Terrain *_terrain);

        // Documentation Inherited
        public: Ogre::MaterialPtr generateForCompositeMap(
            const Ogre::Terrain *_terrain);

        // Documentation Inherited
        public: void setLightmapEnabled(bool _enabled);

        // Documentation Inherited
        public: Ogre::uint8 getMaxLayers(const Ogre::Terrain *_terrain) const;

        // Documentation Inherited
        public: void updateParams(const Ogre::MaterialPtr& mat,
            const Ogre::Terrain *_terrain);

        // Documentation Inherited
        public: void updateParamsForCompositeMap(const Ogre::MaterialPtr& mat,
            const Ogre::Terrain *_terrain);

        // Documentation Inherited
        public: void requestOptions(Ogre::Terrain *_terrain);
      };

      /// \brief Name of material
      protected: std::string materialName;
    };


    /// \internal
    /// \brief Pretends to provide procedural page content to avoid page loading
    class DummyPageProvider : public Ogre::PageProvider
    {
      /// \brief Give a provider the opportunity to prepare page content
      /// procedurally. The parameters are not used.
      public: bool prepareProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }

      /// \brief Give a provider the opportunity to load page content
      /// procedurally. The parameters are not used.
      public: bool loadProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }

      /// \brief Give a provider the opportunity to unload page content
      /// procedurally. The parameters are not used.
      public: bool unloadProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }

      /// \brief Give a provider the opportunity to unprepare page content
      /// procedurally. The parameters are not used.
      public:
          bool unprepareProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }
    };

    /// \internal
    /// \brief Private data for the Heightmap class
    class HeightmapPrivate
    {
      /// \brief Number of pieces in which a terrain is subdivided for paging.
      public: static const unsigned int numTerrainSubdivisions;

      /// \brief The terrain pages are loaded if the distance from the camera is
      /// within the loadRadius. See Ogre::TerrainPaging::createWorldSection().
      /// LoadRadiusFactor is a multiplier applied to the terrain size to create
      /// a load radius that depends on the terrain size.
      public: static const double loadRadiusFactor;

      /// \brief The terrain pages are held in memory but not loaded if they
      /// are not ready when the camera is within holdRadius distance. See
      /// Ogre::TerrainPaging::createWorldSection(). HoldRadiusFactor is a
      /// multiplier applied to the terrain size to create a hold radius that
      /// depends on the terrain size.
      public: static const double holdRadiusFactor;

      /// \brief Hash file name that should be present for every terrain file
      /// loaded using paging.
      public: static const boost::filesystem::path hashFilename;

      /// \brief Name of the top level directory where all the paging info is
      /// stored
      public: static const boost::filesystem::path pagingDirname;

      /// \brief When the terrain paging is enabled, the terrain information
      /// for every piece of terrain is stored in disk. This is the path of
      /// the top level directory where these files are located.
      public: boost::filesystem::path gzPagingDir;

      /// \brief The scene.
      public: ScenePtr scene;

      /// \brief Size of the terrain.
      public: ignition::math::Vector3d terrainSize;

      /// \brief Size of the heightmap data.
      public: unsigned int dataSize;

      /// \brief Origin of the terrain.
      public: ignition::math::Vector3d terrainOrigin;

      /// \brief Global options.
      public: Ogre::TerrainGlobalOptions *terrainGlobals = nullptr;

      /// \brief Group of terrains.
      public: Ogre::TerrainGroup *terrainGroup;

      /// \brief True if the terrain was imported.
      public: bool terrainsImported;

      /// \brief The diffuse textures.
      public: std::vector<std::string> diffuseTextures;

      /// \brief The normal textures.
      public: std::vector<std::string> normalTextures;

      /// \brief The size of the world sections.
      public: std::vector<double> worldSizes;

      /// \brief The material blending heights.
      public: std::vector<double> blendHeight;

      /// \brief Material blend fade distances.
      public: std::vector<double> blendFade;

      /// \brief The raw height values received from physics.
      public: std::vector<float> heights;

      /// \brief Pointer to the terrain material generator.
      public: GzTerrainMatGen *gzMatGen = nullptr;

      /// \brief A page provider is needed to use the paging system.
      public: DummyPageProvider dummyPageProvider;

      /// \brief Central registration point for extension classes,
      /// such as the PageStrategy, PageContentFactory.
      public: Ogre::PageManager *pageManager = nullptr;

      /// \brief Type of paging applied
      public: Ogre::TerrainPaging *terrainPaging = nullptr;

      /// \brief Collection of world content
      public: Ogre::PagedWorld *world;

      /// \brief Collection of terrains. Every terrain might be paged.
      public: std::vector<std::vector<float> > subTerrains;

      /// \brief Used to iterate over all the terrains
      public: int terrainIdx;

      /// \brief Flag that enables/disables the terrain paging
      public: bool useTerrainPaging;

      /// \brief True if the terrain's hash does not match the image's hash
      public: bool terrainHashChanged;

      /// \brief Name of custom material to use for the terrain. If empty,
      /// default material with glsl shader will be used.
      public: std::string materialName;

      /// \brief Filename of the terrain data
      public: std::string filename;

      /// \brief Pointer to heightmap data
      public: common::HeightmapData *heightmapData = nullptr;
    };
  }
}
#endif
