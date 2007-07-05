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
/* Desc: RaySensor proximity sensor
 * Author: Carle Cote
 * Date: 23 february 2004
 * SVN: $Id:$
*/

#ifndef RAYSENSOR_HH
#define RAYSENSOR_HH

#include <vector>

#include "Sensor.hh"
#include "Body.hh"

namespace gazebo
{

  class XMLConfigNode;
  class RayGeom;

/// @brief  sensor with one or more rays.
///
/// This sensor cast rays into the world, tests for intersections, and
/// reports the range to the nearest object.  It is used by ranging
/// sensor models (e.g., sonars and scanning laser range finders).
class RaySensor: public Sensor
{
  /// @brief Constructor
  /// @param world The world in which to place the sensor
  /// @param body The underlying collision test uses an ODE geom, so
  /// ray sensors must be attached to a body.
  /// @param rayCount Maximum number of rays in sensor.
  public: RaySensor(Body *body);

  /// @brief Destructor
  public: virtual ~RaySensor();

  /// Load the ray using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Initialize the ray
  protected: virtual void InitChild();

  ///  Update sensed values
  protected: virtual void UpdateChild(UpdateParams &params);
  
  /// Finalize the ray
  protected: virtual void FiniChild();

  /// \brief Get the minimum angle
  /// \return The minimum angle
  public: double GetMinAngle() const;

  /// \brief Get the maximum angle
  /// \return the maximum angle
  public: double GetMaxAngle() const;

  /// \brief Get the minimum range
  /// \return The minimum range
  public: double GetMinRange() const;

  /// \brief Get the maximum range
  /// \return The maximum range
  public: double GetMaxRange() const;

  /// \brief Get the ray count
  /// \return The number of rays
  public: int GetRayCount() const;

  /// \brief Get the range count
  /// \return The number of ranges
  public: int GetRangeCount() const;

  /// \brief Set ray parameters
  /// \param index Rayindex (from 0 to rayCount - 1).
  /// \param a, b Ray endpoints (initial and final points).  These are
  /// in body-relative coordiantes.
  public: void SetRay(int index, const Vector3 &a, const Vector3 &b);

  /// Get ray parameters
  /// \param index Ray index (from 0 to rayCount -1).
  /// \param pos, dir Ray position and direction.
  public: void GetRay(int index, Vector3 &pos, Vector3 &dir);

  /// \brief Get detected range for a ray.
  /// \returns Returns DBL_MAX for no detection.
  public: double GetRange(int index);   

  /// \brief Get detected retro (intensity) value for a ray.
  public: double GetRetro(int index);   

  /// \brief Get detected fiducial value for a ray.
  public: int GetFiducial(int index);   

  /// \brief Ray-intersection callback
  private: static void UpdateCallback( void *data, dGeomID o1, dGeomID o2 );
  
  /// Ray space for collision detector
  private: dSpaceID superSpaceId;
  private: dSpaceID raySpaceId; 

  /// Ray data
  private: std::vector<RayGeom*> rays;

  private: double minAngle, maxAngle;
  private: double minRange, maxRange;
  private: Vector3 origin;

  private: Pose3d prevPose;
  private: int rayCount;
  private: int rangeCount;

};

}

#endif
