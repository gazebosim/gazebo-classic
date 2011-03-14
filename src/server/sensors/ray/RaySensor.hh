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
/* Desc: RaySensor proximity sensor
 * Author: Nate Koenig
 * Date: 23 february 2004
 * SVN: $Id$
*/

#ifndef RAYSENSOR_HH
#define RAYSENSOR_HH

#include <vector>

#include "Angle.hh"
#include "Sensor.hh"
#include "Body.hh"

namespace gazebo
{
  class OgreDynamicLines;
  class XMLConfigNode;
  class Geom;
  class MultiRayShape;

  /// \addtogroup gazebo_sensor
  /// \brief Sensor with one or more rays.
  /// \{
  /// \defgroup gazebo_ray Ray
  /// \brief Sensor with one or more rays.
  // \{
  
  /// \brief Sensor with one or more rays.
  ///
  /// This sensor cast rays into the world, tests for intersections, and
  /// reports the range to the nearest object.  It is used by ranging
  /// sensor models (e.g., sonars and scanning laser range finders).
  class RaySensor: public Sensor
  {
    /// \brief Constructor
    /// \param body The underlying collision test uses an ODE geom, so
    ///             ray sensors must be attached to a body.
    public: RaySensor(Body *body);
  
    /// \brief Destructor
    public: virtual ~RaySensor();
  
    /// Load the ray using parameter from an XMLConfig node
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
  
    /// \brief Get the minimum angle
    /// \return The minimum angle
    public: Angle GetMinAngle() const;
  
    /// \brief Get the maximum angle
    /// \return the maximum angle
    public: Angle GetMaxAngle() const;
  
    /// \brief Get the minimum range
    /// \return The minimum range
    public: double GetMinRange() const;
  
    /// \brief Get the maximum range
    /// \return The maximum range
    public: double GetMaxRange() const;
  
    /// \brief Get the range resolution
    public: double GetResRange() const;
  
    /// \brief Get the ray count
    /// \return The number of rays
    public: int GetRayCount() const;
  
    /// \brief Get the range count
    /// \return The number of ranges
    public: int GetRangeCount() const;
  
    /// \brief Get the vertical scan line count
    /// \return The number of scan lines vertically
    public: int GetVerticalRayCount() const;
  
    /// \brief Get the vertical scan line count
    /// \return The number of scan lines vertically
    public: int GetVerticalRangeCount() const;
  
    /// \brief Get the vertical scan bottom angle
    /// \return The minimum angle of the scan block
    public: Angle GetVerticalMinAngle() const;
  
    /// \brief Get the vertical scan line top angle
    /// \return The Maximum angle of the scan block
    public: Angle GetVerticalMaxAngle() const;

    /// \brief Get detected range for a ray.
    /// \returns Returns DBL_MAX for no detection.
    public: double GetRange(int index);   
  
    /// \brief Get detected retro (intensity) value for a ray.
    public: double GetRetro(int index);   
  
    /// \brief Get detected fiducial value for a ray.
    public: int GetFiducial(int index);   
  
    private: Pose3d prevPose;
    private: Geom *laserGeom;
    private: MultiRayShape *laserShape;
  };
  /// \}
  /// \}
}

#endif
