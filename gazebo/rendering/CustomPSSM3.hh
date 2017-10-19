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

#ifndef GAZEBO_RENDERING_CUSTOMPSSM3_HH_
#define GAZEBO_RENDERING_CUSTOMPSSM3_HH_

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
    	public: virtual const Ogre::String &getType() const;

      /// \brief This is a duplicate of the method from the parent class with
      /// one line changed to use sampler2DShadow, enabling hardware PCF in
      /// GLSL. Couldn't find a way with Ogre's API to simply call the parent
      /// class method and then modify the one uniform type.
    	/// \sa SubRenderState::resolveParameters.
    	protected: virtual bool resolveParameters(
                  Ogre::RTShader::ProgramSet *_programSet);
    };

    /// \brief A factory that enables creation of CustomPSSM3 instances.
    /// Sub class of SubRenderStateFactory
    class GAZEBO_VISIBLE CustomPSSM3Factory :
        public Ogre::RTShader::SubRenderStateFactory
    {
      // Documentation inherited.
      public: virtual const Ogre::String &getType() const;

      // Documentation inherited.
      public: virtual Ogre::RTShader::SubRenderState *createInstance(
                  Ogre::ScriptCompiler *_compiler,
                  Ogre::PropertyAbstractNode *_prop,
                  Ogre::Pass *_pass,
                  Ogre::RTShader::SGScriptTranslator *_translator);

      // Documentation inherited
      protected: virtual Ogre::RTShader::SubRenderState *createInstanceImpl();
    };
  }
}
#endif

