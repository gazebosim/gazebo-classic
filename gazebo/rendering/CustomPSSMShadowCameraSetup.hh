/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_RENDERING_CUSTOMPSSMSHADOWCAMERASETUP_HH_
#define GAZEBO_RENDERING_CUSTOMPSSMSHADOWCAMERASETUP_HH_

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Custom PSSM shadow receiver that overrides one deficient method
    /// in IntegratedPSSM3.
    class GAZEBO_VISIBLE CustomPSSM3 : public Ogre::RTShader::IntegratedPSSM3
    {
      /// \brief Constructor
      public: CustomPSSM3() {}

      // Documentation inherited
      public: virtual const Ogre::String &getType() const override;

      /// \brief This is a duplicate of the method from the parent class with
      /// one line changed to use sampler2DShadow, enabling hardware PCF in
      /// GLSL. Couldn't find a way with Ogre's API to simply call the parent
      /// class method and then modify the one uniform type.
      /// \sa SubRenderState::resolveParameters.
      protected: virtual bool resolveParameters(
                  Ogre::RTShader::ProgramSet *_programSet) override;

      /// \brief Type of sub render state.
      public: static Ogre::String Type;
    };

    /// \brief A factory that enables creation of CustomPSSM3 instances.
    /// Sub class of SubRenderStateFactory
    class GAZEBO_VISIBLE CustomPSSM3Factory :
        public Ogre::RTShader::SubRenderStateFactory
    {
      // Documentation inherited.
      public: virtual const Ogre::String &getType() const override;

      // Documentation inherited.
      public: virtual Ogre::RTShader::SubRenderState *createInstance(
                  Ogre::ScriptCompiler *_compiler,
                  Ogre::PropertyAbstractNode *_prop,
                  Ogre::Pass *_pass,
                  Ogre::RTShader::SGScriptTranslator *_translator) override;

      // Documentation inherited
      protected: virtual Ogre::RTShader::SubRenderState *createInstanceImpl()
                  override;
    };

    /// \brief Parallel Split Shadow Map (PSSM) shadow camera setup.
    /// Ogre's LiSPSM algorithm makes buggy shadow frusta that often put high
    /// resolution areas in the wrong places. To fix this we subclass a new
    /// ShadowCameraSetup class that does not use LiSPSM.
    /// The solution to this problem in Ogre 2.1 was to derive from
    /// FocusedShadowCameraSetup instead of LiSPSMShadowCameraSetup.
    /// Instead, we are using a modified version of Focused shadow maps.
    /// Modified because we want the light's shadow frusta to always be z-up
    /// just like gazebo. The original Focused shadows in Ogre are y-up.
    /// Aligning shadow map frusta with the up direction tends to minimize
    /// shadow map stretching artifacts by letting stretched shadow texels form
    /// long rectangles instead of sawtooth patterns on most surfaces such as
    /// terrain and man-made objects.
    ///
    /// Because we are deriving from LiSPSM but not using all its
    /// functionality, th efollowing member functions will have no effect:
    /// setOptimalAdjustFactor(), setUseSimpleOptimalAdjust(),
    /// setCameraLightDirectionThreshold().
    class GAZEBO_VISIBLE CustomPSSMShadowCameraSetup
          : public Ogre::PSSMShadowCameraSetup
    {
      /// \brief Constructor, defaults to 3 splits
      public: CustomPSSMShadowCameraSetup();

      /// \brief Destructor
      public: ~CustomPSSMShadowCameraSetup();

      /// \brief lightly modified
      /// FocusedShadowCameraSetup::calculateShadowMappingMatrix().
      void calculateShadowMappingMatrix(const Ogre::SceneManager &_sm,
          const Ogre::Camera &_cam, const Ogre::Light &_light,
          Ogre::Matrix4 *_out_view, Ogre::Matrix4 *_outProj,
          Ogre::Camera *_outCam) const;

      /// \brief The same as FocusedShadowCameraSetup::buildViewMatrix() except
      /// resulting matrices are z-up instead of y-up.
      /// \sa FocusedShadowCameraSetup::buildViewMatrix()
      public: Ogre::Matrix4 buildViewMatrix(const Ogre::Vector3 &_pos,
          const Ogre::Vector3 &_dir, const Ogre::Vector3 & _up) const;

      /// \brief The same as FocusedShadowCameraSetup::ShadowCameraSetup()
      /// except resulting light frusta are z-up instead of y-up.
      /// \sa FocusedShadowCameraSetup::ShadowCameraSetup()
      public: virtual void getZUpFocusedShadowCamera(
          const Ogre::SceneManager *_sm, const Ogre::Camera *_cam,
          const Ogre::Viewport *_vp, const Ogre::Light *_light,
          Ogre::Camera *_texCam, size_t _iteration) const;

      /// \brief Returns a shadow camera with PSSM splits based on iteration.
      /// \sa FocusedShadowCameraSetup::getShadowCamera()
      public: virtual void getShadowCamera(const Ogre::SceneManager *_sm,
          const Ogre::Camera *_cam, const Ogre::Viewport *_vp,
          const Ogre::Light *_light, Ogre::Camera *_texCam, size_t _iteration)
          const override;
    };

    /// \brief This overrides ogre's default GLSLProgramWriter to fix
    /// a bug in ogre versions <= 1.8 where 'sampler2DShadow' sampler type
    /// is missing
    class GAZEBO_VISIBLE CustomGLSLProgramWriter :
        public Ogre::RTShader::GLSLProgramWriter
    {
      /// \brief Constructor
      public: CustomGLSLProgramWriter();

      /// \brief Destructor
      public: ~CustomGLSLProgramWriter() = default;
    };

    /// \brief A factory to create our own CustomGLSLProgramWriter.
    class GAZEBO_VISIBLE CustomGLSLProgramWriterFactory :
        public Ogre::RTShader::ProgramWriterFactory
    {
      /// \brief Constructor
      public: CustomGLSLProgramWriterFactory();

      /// \brief Destructor
      public: ~CustomGLSLProgramWriterFactory() = default;

      /// \brief Get shader language supported by this factory
      /// \return Language supported - "glsl"
      public: const Ogre::String &getTargetLanguage() const override;

      /// \brief Creates the GLSLProgramWriter
      /// \return Ogre's program writer
      public: virtual Ogre::RTShader::ProgramWriter* create() override;
    };
  }
}

#endif
