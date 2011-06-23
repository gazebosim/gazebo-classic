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
#ifndef MULTIRAYGEOM_HH
#define MULTIRAYGEOM_HH

#include <vector>

#include "common/Exception.hh"
#include "common/Console.hh"
#include "math/Vector3.hh"
#include "math/Angle.hh"

#include "physics/Geom.hh"
#include "physics/Shape.hh"
#include "physics/RayShape.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

	namespace physics
  {
    /// \brief Laser geom contains a set of ray-geoms, structured to simulate
    ///        a laser range scanner
    class MultiRayShape : public Shape
    {
      /// \brief Constructor
      public: MultiRayShape(GeomPtr parent);
    
      /// \brief Destructor
      public: virtual ~MultiRayShape();
  
      /// \brief Load a multi-ray shape from xml file
      public: virtual void Load(common::XMLConfigNode *node);
 
      /// \brief Init the shape 
      public: virtual void Init();
              
      /// \brief Save child parameters
      public: virtual void Save(std::string &prefix, std::ostream &stream);
  
      /*public: void Load(unsigned int vertRayCount, unsigned int rayCount,
                        math::Vector3 origin, double minRange, double maxRange,
                        math::Angle minVertmath::Angle, math::Angle maxVertmath::Angle,
                        math::Angle minmath::Angle, math::Angle maxmath::Angle );
                        */
              
  
      public: void SetDisplayType(const std::string &type);
    
      /// \brief Get detected range for a ray.
      /// \returns Returns DBL_MAX for no detection.
      public: double GetRange(int index);
    
      /// \brief Get detected retro (intensity) value for a ray.
      public: double GetRetro(int index);
    
      /// \brief Get detected fiducial value for a ray.
      public: int GetFiducial(int index);
  
      /// \brief Get the minimum angle
      public: math::Angle GetMinAngle() const;
              
      /// \brief Get the maximum angle
      public: math::Angle GetMaxAngle() const;
  
      /// \brief Get the minimum range
      public: double GetMinRange() const;
  
      /// \brief Get the maximum range
      public: double GetMaxRange() const;
  
      /// \brief Get the range resolution
      public: double GetResRange() const;
  
      /// \brief Get the ray count
      public: int GetRayCount() const;
  
      /// \brief Get the range count
      public: int GetRangeCount() const;
  
      /// \brief Get the vertical scan line count
      public: int GetVerticalRayCount() const;
  
      /// \brief Get the vertical scan line count
      public: int GetVerticalRangeCount() const;
  
      /// \brief Get the vertical min angle
      public: math::Angle GetVerticalMinAngle() const;
  
      /// \brief Get the vertical max angle
      public: math::Angle GetVerticalMaxAngle() const;
  
      /// \brief Update the geom
      public: void Update();
    
       /// \brief Physics engine specific method for updating the rays
      protected: virtual void UpdateRays() = 0;
    
      /// \brief Add a ray to the geom
      protected: virtual void AddRay(const math::Vector3 &start, const math::Vector3 &end );
    
      protected: msgs::Visual *rayFanMsg;
      protected: msgs::Visual *rayFanOutlineMsg;
    
      /// Ray data
      protected: std::vector< RayShapePtr > rays;
  
      protected: double maxRange;
  
      protected: common::ParamT<math::Angle> *minAngleP, *maxAngleP;
      protected: common::ParamT<double> *minRangeP, *maxRangeP, *resRangeP;
      protected: common::ParamT<math::Vector3> *originP;
      protected: common::ParamT<int> *rayCountP;
      protected: common::ParamT<int> *rangeCountP;
  
       /// Display rays when rendering images
      protected: common::ParamT<std::string> *displayTypeP;
    
      // For ray blocks such as Velodyne
      protected: common::ParamT<int> *verticalRayCountP;
      protected: common::ParamT<int> *verticalRangeCountP;
      protected: common::ParamT<math::Angle> *verticalMinAngleP;
      protected: common::ParamT<math::Angle> *verticalMaxAngleP;
    };
  
  }

}
#endif
