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

#ifndef _BUILDING_ITEM_HH_
#define _BUILDING_ITEM_HH_

namespace gazebo
{
  namespace gui
  {
    class BuildingItem
    {
        public: BuildingItem();

        public: ~BuildingItem();

        public: int GetLevel() const;

        public: void SetLevel(int _level);

        public: double GetLevelBaseHeight() const;

        public: void SetLevelBaseHeight(double _height);

        protected: int level;

        protected: double levelBaseHeight;
    };
  }
}

#endif
