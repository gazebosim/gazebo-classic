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
#ifndef _TECHNIQUEDEFINITIONS_HH_
#define _TECHNIQUEDEFINITIONS_HH_

namespace gazebo
{
  namespace rendering
  {
    // Technique related policies. The purpose here is to allow new deferred
    // techniques to be added with as little effort as possible. Deferred
    // shading can also be modified to allow more data to be stored in the
    // GBuffer(specular power, screen space speed, etc).
    class NullTechnique
    {
    	Ogre::String GetMaterialPrefix() {return "NullTechnique";}
    	int GetNumInputs() {return 0;}
    	bool UseMaterialProperties() {return true;}
    };

    class DeferredShading
    {
      protected: Ogre::String GetMaterialPrefix(){return "DeferredShading";}
      protected: int GetGBufferSize(){return 2;}
      protected: bool UseMaterialProperties(){return true;}
    };

    class DeferredLighting
    {
      protected: Ogre::String GetMaterialPrefix(){return "DeferredLighting";}
      protected: int GetGBufferSize(){return 1;}
      protected: bool UseMaterialProperties(){return false;}
    };

    class InferredLighting
    {
      protected: Ogre::String GetMaterialPrefix(){return "InferredLighting";}
      protected: int GetGBufferSize(){return 1;}
      protected: bool UseMaterialProperties(){return false;}
    };
  }
}
#endif
