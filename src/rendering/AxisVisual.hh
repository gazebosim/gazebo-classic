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
/* Desc: 3D Axis Visualization Class
 * Author: Nate Koenig
 */

#ifndef AXISVISUAL_HH
#define AXISVISUAL_HH

#include <string>

#include "rendering/Visual.hh"

namespace gazebo
{
  namespace rendering
  {
    class AxisVisual : public Visual
    {
      public: AxisVisual(const std::string &_name, VisualPtr _vis);
      public: virtual ~AxisVisual();

      public: virtual void Load();

      private: ArrowVisualPtr xAxis;
      private: ArrowVisualPtr yAxis;
      private: ArrowVisualPtr zAxis;
    };
  }
}
#endif
