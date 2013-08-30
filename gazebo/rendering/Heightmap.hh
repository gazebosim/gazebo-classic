/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: Heightmap geometry
 * Author: Nate Koenig
 * Date: 12 May 2009
 */

#ifndef _HEIGHTMAP_HH_
#define _HEIGHTMAP_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Image.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/rendering/Scene.hh"

namespace Ogre
{
  class TerrainGlobalOptions;
  class TerrainGroup;
  class Terrain;
}

namespace gazebo
{
  namespace rendering
  {
    class GzTerrainMatGen;

    class DummyPageProvider : public Ogre::PageProvider
    {
      public:
      bool prepareProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }
      bool loadProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }
      bool unloadProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }
      bool unprepareProceduralPage(Ogre::Page*, Ogre::PagedWorldSection*)
      {
        return true;
      }
    };

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Heightmap Heightmap.hh rendering/rendering.hh
    /// \brief Rendering a terrain using heightmap information
    class Heightmap
    {
      /// \brief Number of pieces in which a terrain is subdivided for paging.
      public: static const unsigned int NumTerrainSubdivisions;

      /// \brief Constructor
      /// \param[in] _scene Pointer to the scene that will contain the heightmap
      public: Heightmap(ScenePtr _scene);

      /// \brief Destructor
      public: virtual ~Heightmap();

      /// \brief Load the heightmap
      public: void Load();

      /// \brief Load the heightmap from a visual message
      /// \param[in] _msg The visual message containing heightmap info
      public: void LoadFromMsg(ConstVisualPtr &_msg);

      /// \brief Get the height at a location
      /// \param[in] _x X location
      /// \param[in] _y Y location
      /// \param[in] _z Z location
      /// \return The height at the specified location
      public: double GetHeight(double _x, double _y, double _z = 1000);

      /// \brief Flatten the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Flatten(CameraPtr _camera, math::Vector2i _mousePos,
                         double _outsideRadius, double _insideRadius,
                         double _weight = 0.1);

      /// \brief Smooth the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Smooth(CameraPtr _camera, math::Vector2i _mousePos,
                         double _outsideRadius, double _insideRadius,
                         double _weight = 0.1);

      /// \brief Raise the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Raise(CameraPtr _camera, math::Vector2i _mousePos,
                         double _outsideRadius, double _insideRadius,
                         double _weight = 0.1);

      /// \brief Lower the terrain based on a mouse press.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \return True if the terrain was modified
      public: bool Lower(CameraPtr _camera, math::Vector2i _mousePos,
                         double _outsideRadius, double _insideRadius,
                         double _weight = 0.1);

      /// \brief Get the average height around a point.
      /// \param[in] _pos Position in world coordinates.
      /// \param[in] _brushSize Controls the radius of effect.
      public: double GetAvgHeight(Ogre::Vector3 _pos, double _brushSize);

      /// \brief Set the heightmap to render in wireframe mode.
      /// \param[in] _show True to render wireframe, false to render solid.
      public: void SetWireframe(bool _show);

      /// \brief Get a pointer to the OGRE terrain group object.
      /// \return Pointer to the OGRE terrain.
      public: Ogre::TerrainGroup *GetOgreTerrain() const;

      /// \brief Get the heightmap as an image
      /// \return An image that contains the terrain data.
      public: common::Image GetImage() const;

      /// \brief Calculate a mouse ray hit on the terrain.
      /// \param[in] _camera Camera associated with the mouse press.
      /// \param[in] _mousePos Position of the mouse in viewport
      /// coordinates.
      /// \return The result of the mouse ray hit.
      public: Ogre::TerrainGroup::RayResult GetMouseHit(CameraPtr _camera,
                  math::Vector2i _mousePos);

      /// \brief Split a terrain into subterrains
      /// \param[in] _heightmap Source vector of floats with the heights.
      /// \param[in] _n Number of subterrains.
      /// \param[out] _v Destination vector with the subterrains.
      /// \return The result of the mouse ray hit.
      public: void SplitHeights(std::vector<float> &_heightmap, int _n,
                  std::vector<std::vector<float> > &_v);

      /// \brief Modify the height at a specific point.
      /// \param[in] _pos Position in world coordinates.
      /// \param[in] _outsideRadius Controls the radius of effect.
      /// \param[in] _insideRadius Controls the size of the radius with the
      /// maximum effect (value between 0 and 1).
      /// \param[in] _weight Controls modification magnitude.
      /// \param[in] _op Type of operation to perform.
      private: void ModifyTerrain(Ogre::Vector3 _pos, double _outsideRadius,
                   double _insideRadius, double _weight,
                   const std::string &_op);

      /// \brief Initialize all the blend material maps.
      /// \param[in] _terrain The terrain to initialize the blend maps.
      private: bool InitBlendMaps(Ogre::Terrain *_terrain);

      /// \brief Configure the terrain default values.
      private: void ConfigureTerrainDefaults();

      /// \brief Define a section of the terrain.
      /// \param[in] _x X coordinate of the terrain.
      /// \param[in] _y Y coordinate of the terrain.
      private: void DefineTerrain(int _x, int _y);

      /// \brief Internal function used to setup shadows for the terrain.
      /// \param[in] _enabled True to enable shadows.
      private: void SetupShadows(bool _enabled);

      /// \brief The scene.
      private: ScenePtr scene;

      /// \brief Image used to generate the heightmap.
      private: common::Image heightImage;

      /// \brief Size of the terrain.
      private: math::Vector3 terrainSize;

      /// \brief Size of the heightmap data.
      private: unsigned int dataSize;

      /// \brief Origin of the terrain.
      private: math::Vector3 terrainOrigin;

      /// \brief Global options.
      private: Ogre::TerrainGlobalOptions *terrainGlobals;

      /// \brief Group of terrains.
      private: Ogre::TerrainGroup *terrainGroup;

      /// \brief True if the terrain was imported.
      private: bool terrainsImported;

      /// \brief The diffuse textures.
      private: std::vector<std::string> diffuseTextures;

      /// \brief The normal textures.
      private: std::vector<std::string> normalTextures;

      /// \brief The size of the world sections.
      private: std::vector<double> worldSizes;

      /// \brief The material blending heights.
      private: std::vector<double> blendHeight;

      /// \brief Material blend fade distances.
      private: std::vector<double> blendFade;

      /// \brief The raw height values received from physics.
      private: std::vector<float> heights;

      /// \brief Pointer to the terrain material generator.
      private: GzTerrainMatGen *gzMatGen;

      /// \brief A page provided is needed to use the paging system.
      private: DummyPageProvider mDummyPageProvider;

      /// \brief Central registration point for extension classes,
      /// such as the PageStrategy, PageContentFactory.
      private: Ogre::PageManager *mPageManager;

      /// \brief Type of paging applied
      private: Ogre::TerrainPaging *mTerrainPaging;

      /// \brief Collection of world content
      private: Ogre::PagedWorld* world;

      /// \brief Collection of terrains. Every terrain might be paged.
      private: std::vector<std::vector<float> > subTerrains;

      /// \brief Used to iterate over all the terrains
      private: int terrainIdx;

      /// \brief Flag that enables/disables the terrain paging
      private: bool useTerrainPaging;
    };
    /// \}

    /// \internal
    /// \brief Custom terrain material generator for GLSL terrains.
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

        /// \brief Utility class to help with generating shaders for GLSL.
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
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateVpFooter(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateVertexProgramSource(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void defaultVpParams(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         const Ogre::HighLevelGpuProgramPtr &_prog);

          protected: virtual unsigned int generateVpDynamicShadowsParams(
                         unsigned int _texCoordStart, const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateVpDynamicShadows(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateFpHeader(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateFpLayer(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType tt,
                         Ogre::uint _layer,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateFpFooter(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateFpDynamicShadowsParams(
                         Ogre::uint *_texCoord, Ogre::uint *_sampler,
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateFpDynamicShadowsHelpers(
                         const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: void generateFpDynamicShadows(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateFragmentProgramSource(
                         const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void updateVpParams(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         const Ogre::GpuProgramParametersSharedPtr &_params);

          private: Ogre::String GetChannel(Ogre::uint _idx);
        };

        /// Keeping the CG shader for reference.
        /// Utility class to help with generating shaders for Cg / HLSL.
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
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateVpFooter(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateVertexProgramSource(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void defaultVpParams(const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         const Ogre::HighLevelGpuProgramPtr &_prog);

          protected: virtual unsigned int generateVpDynamicShadowsParams(
                         unsigned int _texCoordStart, const SM2Profile *_prof,
                         const Ogre::Terrain *_terrain, TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);

          protected: virtual void generateVpDynamicShadows(
                         const SM2Profile *_prof, const Ogre::Terrain *_terrain,
                         TechniqueType _tt,
                         Ogre::StringUtil::StrStreamType &_outStream);
        };
      };
    };
  }
}
#endif
