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
/* Desc: IRSensor proximity sensor
 * Author: Carle Cote
 * Date: 23 february 2004
 * SVN: $Id: IRSensor.hh 4402 2008-03-09 14:39:27Z robotos $
*/

#ifndef IRSENSOR_HH
#define IRSENSOR_HH

#include <vector>

#include "Sensor.hh"
#include "Body.hh"
#include <gazebo.h>

namespace gazebo
{

  class XMLConfigNode;
  class RayGeom;
  class RaySensor;

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
class IRSensor: public Sensor
{
  /// \brief Constructor
  /// \param body The underlying collision test uses an ODE geom, so
  ///             ray sensors must be attached to a body.
  public: IRSensor(Body *body);

  /// \brief Destructor
  public: virtual ~IRSensor();

  /// Load the ray using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Initialize the ray
  protected: virtual void InitChild();

  ///  Update sensed values
  protected: virtual void UpdateChild();
  
  /// Finalize the ray
  protected: virtual void FiniChild();

  /// \brief Get the ray count
  /// \return The number of rays
  public: unsigned int GetIRCount() const;

  /// \brief Get detected range for a ray.
  /// \returns Returns DBL_MAX for no detection.
  public: double GetRange(unsigned int index) const;

  public: Pose3d GetPose(unsigned int index) const;

  private: std::vector<RaySensor*> irBeams;

};
/// \}
/// \}
}

#endif
