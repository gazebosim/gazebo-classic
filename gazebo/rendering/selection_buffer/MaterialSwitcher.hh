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

#ifndef _MATERIALSWITCHER_HH_
#define _MATERIALSWITCHER_HH_

#include <map>
#include <string>
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/common/Color.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /*struct cmp_color
    {
      bool operator()(const common::Color &_a, const common::Color &_b) const
      {
        return _a.GetAsBGRA() < _b.GetAsBGRA();
      }
    };*/

    class SelectionBuffer;
    class GAZEBO_VISIBLE MaterialSwitcher :
      public Ogre::MaterialManager::Listener
    {
      /// \brief Constructor
      public: MaterialSwitcher();

      /// \brief Destructor
      public: ~MaterialSwitcher();

      /// \brief Get the entity with a specific color
      public: const std::string &GetEntityName(
                  const common::Color &_color) const;

      /// \brief Reset the color value incrementor
      public: void Reset();

      /// \brief Ogre callback that assigns colors to new renderables
      public: virtual Ogre::Technique *handleSchemeNotFound(
                  uint16_t _schemeIndex, const Ogre::String &_schemeName,
                  Ogre::Material *_originalMaterial, uint16_t _lodIndex,
                  const Ogre::Renderable *_rend);

      // private: typedef std::map<unsigned int, std::string, cmp_color>
      // ColorMap;
      private: typedef std::map<unsigned int, std::string> ColorMap;
      private: typedef ColorMap::const_iterator ColorMapConstIter;
      private: std::string emptyString;
      private: common::Color currentColor;
      private: std::string lastEntity;
      private: Ogre::Technique *lastTechnique;
      private: MaterialSwitcher::ColorMap colorDict;

      private: void GetNextColor();

      public: friend class SelectionBuffer;
    };
  }
}
#endif
