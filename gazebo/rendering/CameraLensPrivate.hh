/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RENDERING_CAMERA_LENS_PRIVATE_HH_
#define _GAZEBO_RENDERING_CAMERA_LENS_PRIVATE_HH_

#include <map>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

#include <sdf/sdf.hh>


namespace gazebo
{
  namespace rendering
  {
    // forward declaration
    class WideAngleCamera;

    /// \brief Private fields of camera lens
    class CameraLensPrivate
    {
      /// \brief Linear scale factor
      public: double c1 = 1.0;

      /// \brief Angle scale factor
      public: double c2 = 1.0;

      /// \brief Angle offset factor
      public: double c3 = 0.0;

      /// \brief Linear scale factor, may be adjusted in runtime
      public: double f = 1.0;

      /// \brief Visible field of view
      public: double cutOffAngle = IGN_PI*0.5;

      /// \brief Enumeration of functions that can be casted to some other types
      public: class MapFunctionEnum
              {
                /// \brief Constructor
                /// \param[in] str Function name 'sin', 'tan' or 'id'
                public: explicit MapFunctionEnum(const std::string &_str)
                {
                  for (auto item : variants)
                  {
                    if (std::get<0>(item) == _str)
                    {
                      value = item;
                      return;
                    }
                  }

                  // function provided is not in array
                  throw std::invalid_argument("Unknown function");
                }

                /// \brief Cast to ignition::math::Vector3d,
                ///   this vector is passed to shader to avoid branching
                /// \return Vector3 Vector whose one component is 1
                ///   and the rest are nulls
                public: ignition::math::Vector3d AsVector3d() const
                {
                  return std::get<1>(value);
                }

                /// \brief Cast to std::string
                /// \return The same string which was passed to constructor
                public: std::string AsString() const
                {
                  return std::get<0>(value);
                }

                /// \brief Apply function to float value
                /// \result The result of application
                public: float Apply(float _t)
                {
                  return std::get<2>(value)(_t);
                }

                /// \brief Assignment operator
                /// \param[in] _fun Rvalue
                /// \result Reference to (*this)
                public: MapFunctionEnum &operator=(const MapFunctionEnum &_fun)
                {
                  this->value = _fun.value;
                  return *this;
                }

                /// List of all available functions and it's representations
                private: const std::vector<
                    std::tuple<std::string, ignition::math::Vector3d,
                        std::function<float (float)> > > variants = {
                          std::make_tuple("sin",
                              ignition::math::Vector3d::UnitX,
                              std::function<float (float)>(
                                  static_cast<float (*)(float)>(&std::sin))),
                          std::make_tuple("tan",
                              ignition::math::Vector3d::UnitY,
                              std::function<float (float)>(
                                  static_cast<float (*)(float)>(&std::tan))),
                          std::make_tuple("id",
                              ignition::math::Vector3d::UnitZ,
                              std::function<float (float)>(
                                  [](float t) -> float
                                  {
                                    return t;
                                  }))};

                /// \brief Current value of enumeration
                private: decltype(variants)::value_type value;
              };

      /// \brief `fun` component of the mapping function,
      ///   see CameraLens description
      public: MapFunctionEnum fun = MapFunctionEnum("id");

      /// \brief SDF element of the lens
      public: sdf::ElementPtr sdf;

      /// \brief Mutex to lock when getting or setting lens data
      public: std::recursive_mutex dataMutex;
    };
  }
}

#endif
