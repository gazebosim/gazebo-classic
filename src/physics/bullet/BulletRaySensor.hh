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
/* Desc: Bullet ray sensor
 * Author: Nate Koenig
 * Date: 21 May 2009
 * SVN: $Id:$
 */

#ifndef BULLETRAYSENSOR_HH
#define BULLETRAYSENSOR_HH

#include <vector>

#include "PhysicsRaySensor.hh"

namespace gazebo
{
	namespace physics
{
  class BulletRayGeom;
  class BulletBody;

  /// \brief An Bullet Ray sensor
  class BulletRaySensor : public PhysicsRaySensor
  {
    /// \brief Constructor
    public: BulletRaySensor(Body *body);

    /// \brief Destructor
    public: virtual ~BulletRaySensor();

    /// \brief Add a ray to the sensor
    public: void AddRay(common::Vector3 start, common::Vector3 end, double minRange, 
                        double maxRange, bool display);

    /// \brief Get the number of rays
    public: int GetCount() const;

    /// \brief Get the relative starting and ending points of a ray
    public: void GetRelativePoints(int index, common::Vector3 &a, common::Vector3 &b);

    /// \brief Get the range of a ray
    public: double GetRange(int index) const;

    /// \brief Get the retro reflectance value of a ray
    public: double GetRetro(int index) const;

    /// \brief Get the fiducial value of a ray
    public: double GetFiducial(int index) const;

    /// \brief Update the ray sensor
    public: virtual void Update();

    /// All the rays
    private: std::vector<BulletRayGeom*> rays;

    private: BulletBody *body;
  };
}
}
}
#endif
