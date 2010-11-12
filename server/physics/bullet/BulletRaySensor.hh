/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
    public: void AddRay(Vector3 start, Vector3 end, double minRange, 
                        double maxRange, bool display);

    /// \brief Get the number of rays
    public: int GetCount() const;

    /// \brief Get the relative starting and ending points of a ray
    public: void GetRelativePoints(int index, Vector3 &a, Vector3 &b);

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
#endif
