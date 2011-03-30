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

#ifndef IMUSENSOR_HH
#define IMUSENSOR_HH

#include <vector>

#include "Sensor.hh"
#include "Body.hh"

namespace gazebo
{

  class XMLConfigNode;
  class ImuSensor: public Sensor
  {
    /// \brief Constructor
    /// \param body The IMU sensor must be attached to a body.
    public: ImuSensor(Body *body);
  
    /// \brief Destructor
    public: virtual ~ImuSensor();
  
    /// \param node The XMLConfig node
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// \brief Save the sensor info in XML format
    protected: virtual void SaveChild(std::string &prefix, std::ostream &stream);
  
    /// Initialize the ray
    protected: virtual void InitChild();
  
    ///  Update sensed values
    protected: virtual void UpdateChild();
    
    /// Finalize the ray
    protected: virtual void FiniChild();
  
    public: Pose3d GetVelocity();
    public: Vector3 GetEulerAngles();
  
    private: Pose3d prevPose;
    private: Pose3d imuVel;
    private: Vector3 eulerAngles;
  
  };
}

#endif
