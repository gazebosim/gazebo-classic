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
#ifndef _LIGHTMATERIALGENERATOR_HH_
#define _LIGHTMATERIALGENERATOR_HH_

#include <OgreStringConverter.h>
#include <OgreException.h>
#include <OgreMaterialManager.h>

#include <OgrePass.h>
#include <OgreTechnique.h>

#include <OgreHighLevelGpuProgram.h>
#include <OgreHighLevelGpuProgramManager.h>

#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/rendering/deferred_shading/TechniqueDefinitions.hh"
#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    template<typename techniquePolicy>
    class LightMaterialGeneratorGLSL;

    template<typename techniquePolicy>
    class LightMaterialGenerator :
      public MaterialGenerator, techniquePolicy
    {
      /// Permutation of light materials
      public: enum MaterialID
              {
                MI_POINT = 0x01,
                MI_SPOTLIGHT = 0x02,
                MI_DIRECTIONAL = 0x04,
                MI_ATTENUATED = 0x08,
                MI_SPECULAR = 0x10,
                MI_SHADOW_CASTER = 0x20
              };

      public: LightMaterialGenerator()
      {
        vsMask = 0x00000004;
        fsMask = 0x0000003F;
        this->matMask = LightMaterialGenerator::MI_DIRECTIONAL |
                        LightMaterialGenerator::MI_SHADOW_CASTER;

        this->schemeName.clear();
        materialBaseName = this->GetMaterialPrefix() + "/LightMaterial/";
        this->impl = new LightMaterialGeneratorGLSL<techniquePolicy>(
            materialBaseName);
      }

      public: virtual ~LightMaterialGenerator()
              {}
    };

    template<typename techniquePolicy>
    class LightMaterialGeneratorGLSL :
      public MaterialGenerator::Impl, public techniquePolicy
    {
      public: typedef MaterialGenerator::Perm Perm;

      ///////////////////////////////////////////////
      public: LightMaterialGeneratorGLSL(const std::string &_baseName)
              : baseName(_baseName) {}

      ///////////////////////////////////////////////
      public: virtual ~LightMaterialGeneratorGLSL() {}

      ///////////////////////////////////////////////
      public: virtual Ogre::GpuProgramPtr GenerateVertexShader(
                  Perm _permutation)
      {
        std::string programName = "DeferredShading/";

        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_DIRECTIONAL)
          programName += "vs";
        else
          programName += "LightMaterial_vs";

        Ogre::GpuProgramPtr ptr =
          Ogre::HighLevelGpuProgramManager::getSingleton().getByName(
              programName);

        if (ptr.isNull())
          gzthrow("Null pointer");

        return ptr;
      }
      ///////////////////////////////////////////////
      /* public: virtual Ogre::GpuProgramPtr GenerateFragmentShader(
                  Perm _permutation)
      {
        /// Create shader
        if (this->masterSource.empty())
        {
          Ogre::DataStreamPtr ptrMasterSource =
            Ogre::ResourceGroupManager::getSingleton().openResource(
                //NEW "deferred_rendering/deferred_shading/LightMaterial_ps.cg",
                "DeferredShading/post/LightMaterial_ps.cg",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

          if (ptrMasterSource.isNull())
            gzthrow("Null Pointer\n");

          this->masterSource = ptrMasterSource->getAsString();
        }

        if (this->masterSource.empty())
          gzthrow("Empty string");

        // Create name
        std::string name = this->baseName +
          Ogre::StringConverter::toString(_permutation) + "_ps";

        // Create shader object
        Ogre::HighLevelGpuProgramPtr ptrProgram =
          Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
              name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
              "cg", Ogre::GPT_FRAGMENT_PROGRAM);

        ptrProgram->setSource(this->masterSource);
        ptrProgram->setParameter("entry_point","main");
        ptrProgram->setParameter("profiles","ps_3_0 arbfp1");

        // set up the preprocessor defines
        // Important to do this before any call to get parameters,
        // i.e. before the program gets loaded
        ptrProgram->setParameter("compile_arguments",
            this->GetPPDefines(_permutation));
        this->SetUpBaseParameters(ptrProgram->getDefaultParameters());

        return Ogre::GpuProgramPtr(ptrProgram);
      }*/
      ///////////////////////////////////////////////
      public: virtual Ogre::GpuProgramPtr GenerateFragmentShader(
                  Perm _permutation)
      {
        /// Create shader
        if (this->masterSource.empty())
        {
          Ogre::DataStreamPtr ptrMasterSource =
            Ogre::ResourceGroupManager::getSingleton().openResource(
                "deferred_rendering/deferred_shading/light_material_ps.glsl",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

          if (ptrMasterSource.isNull())
            gzthrow("Null Pointer\n");

          this->masterSource = ptrMasterSource->getAsString();
        }

        if (this->masterSource.empty())
          gzthrow("Empty string");

        // Create name
        std::string name = this->baseName +
          Ogre::StringConverter::toString(_permutation) + "_ps";

        // Create shader object
        Ogre::HighLevelGpuProgramPtr ptrProgram =
          Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
              name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
              "glsl", Ogre::GPT_FRAGMENT_PROGRAM);

        ptrProgram->setSource(this->masterSource);

        // set up the preprocessor defines
        // Important to do this before any call to get parameters,
        // i.e. before the program gets loaded
        // ptrProgram->setParameter("compile_arguments",
        //    this->GetPPDefines(_permutation));
        ptrProgram->setParameter("preprocessor_defines",
            this->GetPPDefines(_permutation));

        this->SetUpBaseParameters(ptrProgram->getDefaultParameters());

        ptrProgram->getDefaultParameters()->setNamedConstant(
            "tex0", static_cast<int>(0));

        // TODO: This requires a test for Deferred Shading. Deferred
        // Lighting does not use this
        if (ptrProgram->getDefaultParameters()->_findNamedConstantDefinition(
            "tex1"))
        {
          ptrProgram->getDefaultParameters()->setNamedConstant(
              "tex1", static_cast<int>(1));
        }

        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_SHADOW_CASTER)
        {
          ptrProgram->getDefaultParameters()->setNamedConstant(
              "shadowTex", static_cast<int>(1));
        }

        return Ogre::GpuProgramPtr(ptrProgram);
      }

      ///////////////////////////////////////////////
      public: virtual Ogre::MaterialPtr GenerateTemplateMaterial(
                  Perm _permutation)
      {
        std::string materialName = this->baseName;

        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_DIRECTIONAL)
          materialName += "Quad";
        else
          materialName += "Geometry";

        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_SHADOW_CASTER)
          materialName += "Shadow";

        return Ogre::MaterialManager::getSingleton().getByName(
            materialName);
      }

    /*Ogre::String GetPPDefines(Perm permutation)
    {
      Ogre::String strPPD;

      //Get the type of light
      Ogre::String lightType;
      if (permutation & LightMaterialGenerator<techniquePolicy>::MI_POINT)
      {
        lightType = "POINT";
      }
      else if (permutation & LightMaterialGenerator<techniquePolicy>::MI_SPOTLIGHT)
      {
        lightType = "SPOT";
      }
      else if (permutation & LightMaterialGenerator<techniquePolicy>::MI_DIRECTIONAL)
      {
        lightType = "DIRECTIONAL";
      }
      else
      {
        assert(false && "Permutation must have a light type");
      }
      strPPD += "-DLIGHT_TYPE=LIGHT_" + lightType + " ";

      //Optional parameters
            if (permutation & LightMaterialGenerator<techniquePolicy>::MI_SPECULAR)
      {
        strPPD += "-DIS_SPECULAR ";
      }
      if (permutation & LightMaterialGenerator<techniquePolicy>::MI_ATTENUATED)
      {
        strPPD += "-DIS_ATTENUATED ";
      }
      if (permutation & LightMaterialGenerator<techniquePolicy>::MI_SHADOW_CASTER)
      {
        strPPD += "-DIS_SHADOW_CASTER ";
      }
      if(this->UseMaterialProperties()){
        strPPD += "-DUSE_MAT_PROPERTIES ";
      }

      return strPPD;
    }*/
      ///////////////////////////////////////////////
      // Utility method
      protected: std::string GetPPDefines(Perm _permutation)
      {
        std::string strPPD;

        // Get the type of light
        std::string lightType;
        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_POINT)
        {
          lightType = "POINT";
        }
        else if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_SPOTLIGHT)
        {
          lightType = "SPOT";
        }
        else if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_DIRECTIONAL)
        {
          lightType = "DIRECTIONAL";
        }
        else
          gzthrow("Permutation must have a light type");

        strPPD += "LIGHT_TYPE=LIGHT_" + lightType + ";";

        // Optional parameters
        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_SPECULAR)
        {
          strPPD += "IS_SPECULAR=1;";
        }

        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_ATTENUATED)
        {
          strPPD += "IS_ATTENUATED=1;";
        }

        if (_permutation &
            LightMaterialGenerator<techniquePolicy>::MI_SHADOW_CASTER)
        {
          strPPD += "IS_SHADOW_CASTER=1;";
        }

        if (this->UseMaterialProperties())
          strPPD += "USE_MAT_PROPERTIES=1;";

        return strPPD;
      }

      ///////////////////////////////////////////////
      protected: void SetUpBaseParameters(
                     const Ogre::GpuProgramParametersSharedPtr &_params)
      {
        if (_params.isNull())
          gzthrow("Params is null");

        struct AutoParamPair
        {
          std::string name;
          Ogre::GpuProgramParameters::AutoConstantType type;
        };

        // A list of auto params that might be present in the
        // shaders generated
        static const AutoParamPair AUTO_PARAMS[] =
        {
          {"vpWidth",
            Ogre::GpuProgramParameters::ACT_VIEWPORT_WIDTH},
          {"vpHeight",
            Ogre::GpuProgramParameters::ACT_VIEWPORT_HEIGHT},
          {"worldView",
            Ogre::GpuProgramParameters::ACT_WORLDVIEW_MATRIX},
          {"invProj",
            Ogre::GpuProgramParameters::ACT_INVERSE_PROJECTION_MATRIX},
          {"invView",
            Ogre::GpuProgramParameters::ACT_INVERSE_VIEW_MATRIX},
          {"flip",
            Ogre::GpuProgramParameters::ACT_RENDER_TARGET_FLIPPING},
          {"lightDiffuseColor",
            Ogre::GpuProgramParameters::ACT_LIGHT_DIFFUSE_COLOUR},
          {"lightSpecularColor",
            Ogre::GpuProgramParameters::ACT_LIGHT_SPECULAR_COLOUR},
          {"lightFalloff",
            Ogre::GpuProgramParameters::ACT_LIGHT_ATTENUATION},
          {"lightPos",
            Ogre::GpuProgramParameters::ACT_LIGHT_POSITION_VIEW_SPACE},
          {"lightDir",
            Ogre::GpuProgramParameters::ACT_LIGHT_DIRECTION_VIEW_SPACE},
          {"spotParams",
            Ogre::GpuProgramParameters::ACT_SPOTLIGHT_PARAMS},
          {"farClipDistance",
            Ogre::GpuProgramParameters::ACT_FAR_CLIP_DISTANCE},
          {"shadowViewProjMat",
            Ogre::GpuProgramParameters::ACT_TEXTURE_VIEWPROJ_MATRIX}
        };

        int numParams = sizeof(AUTO_PARAMS) / sizeof(AutoParamPair);

        for (int i = 0; i < numParams; ++i)
        {
          if (_params->_findNamedConstantDefinition(AUTO_PARAMS[i].name))
          {
            _params->setNamedAutoConstant(
                AUTO_PARAMS[i].name, AUTO_PARAMS[i].type);
          }
        }
      }

      protected: std::string baseName;
      protected: std::string masterSource;
    };
  }
}
#endif
