/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GAZEBO_RENDERING_CUSTOMSCENEMANAGER_HH_
#define GAZEBO_RENDERING_CUSTOMSCENEMANAGER_HH_

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/CustomAutoParamDataSource.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    const Ogre::String CustomSMTypeName = "CustomSceneManager";

    /// \brief Custom scene manager that overrides one deficient method
    /// in a member of Ogre::SceneManager.
    class GAZEBO_VISIBLE CustomSceneManager : public Ogre::SceneManager
    {
    public:
      /// \brief Constructor
      CustomSceneManager(const Ogre::String& name) : Ogre::SceneManager(name)
      {
        // Replace mAutoParamDataSource using this derived class's method
        OGRE_DELETE mAutoParamDataSource;
        mAutoParamDataSource = createAutoParamDataSource();
      }

      /// \brief Destructor
      ~CustomSceneManager() {}

      virtual const Ogre::String& getTypeName(void) const override { return CustomSMTypeName; }

    protected:
      virtual Ogre::AutoParamDataSource* createAutoParamDataSource(void) const override
      {
        return OGRE_NEW CustomAutoParamDataSource();
      }
    };

    /// \brief Custom factory for creating CustomSceneManager.
    class _OgreExport CustomSceneManagerFactory : public Ogre::SceneManagerFactory
    {
    protected:
      void initMetaData(void) const
      {
        mMetaData.typeName = CustomSMTypeName;
        mMetaData.description = "Custom scene manager for Gazebo";
        mMetaData.sceneTypeMask = Ogre::ST_GENERIC;
        mMetaData.worldGeometrySupported = false;
      }
    public:
      CustomSceneManagerFactory() {}
      ~CustomSceneManagerFactory() {}
      /// Factory type name
      static const Ogre::String FACTORY_TYPE_NAME;
      Ogre::SceneManager* createInstance(const Ogre::String& instanceName) {
        return OGRE_NEW CustomSceneManager(instanceName);
      }
      void destroyInstance(Ogre::SceneManager* instance){
        OGRE_DELETE instance;
      }
    };
  }
}

#endif

