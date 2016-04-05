/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_PLOT_INTROSPECTIONCURVEHANDLER_HH_
#define GAZEBO_GUI_PLOT_INTROSPECTIONCURVEHANDLER_HH_

#include <memory>
#include <string>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class IntrospectionCurveHandlerPrivate;

    /// \brief Manages and updates curves based on introspection data.
    class GZ_GUI_VISIBLE IntrospectionCurveHandler
    {
      /// \brief Constructor.
      public: IntrospectionCurveHandler();

      /// \brief Destructor.
      public: ~IntrospectionCurveHandler();

      /// \brief Add a curve to be updated
      /// \param[in] _query URI query string containing the param the curve is
      /// associated with.
      /// \param[in] _curve Pointer to the plot curve to add.
      public: void AddCurve(const std::string &_query, PlotCurveWeakPtr _curve);

      /// \brief Remove a curve from the topic data hander
      /// \param[in] _curve Pointer to the plot curve to remove.
      public: void RemoveCurve(PlotCurveWeakPtr _curve);

      /// \brief Get the number of curves managed by this handler
      /// \return Number of curves
      public: unsigned int CurveCount() const;

      /// \brief Set whether or not to pause updating the plot curves.
      /// \param[in] _paused True to pause update.
      public: void SetPaused(const bool _paused);

      /// \brief Get whether or not the introspection curve handler has been
      /// initialized.
      /// \return True if initialized.
      public: bool Initialized() const;

      /// \brief Set up introspection client
      private: void SetupIntrospection();

      /// \brief Function called each time a topic update is received.
      /// \param[in] _msg Introspection message filled with param data
      private: void OnIntrospection(const gazebo::msgs::Param_V &_msg);

      /// \brief Helper function to get the value of an vector3 attribute
      /// based on a query string
      /// \param[in] _query Query string
      /// \param[in] _vec Vector3d data
      /// \param[out] _value Value of the attribute
      /// \return True if value is found
      private: bool Vector3dFromQuery(const std::string &_query,
          const ignition::math::Vector3d &_vec, double &_value) const;

      /// \brief Helper function to get the value of an euler angle
      /// based on a query string
      /// \param[in] _query Query string
      /// \param[in] _quat Quaternion data
      /// \param[out] _value Euler angle
      /// \return True if value is found
      private: bool QuaterniondFromQuery(const std::string &_query,
          const ignition::math::Quaterniond &_quat, double &_value) const;

      /// \brief Add an item to the introspection filter
      /// \param[in] _name Name of item
      /// \param[in] _cb Async callback to indicate the result of the filter
      /// update
      private: void AddItemToFilter(const std::string &_name,
          const std::function<void(const bool _result)> &_cb = nullptr);

      /// \brief Remove an item from the introspection filter
      /// \param[in] _name Name of item
      /// \param[in] _cb Async callback to indicate the result of the filter
      /// update
      private: void RemoveItemFromFilter(const std::string &_name,
          const std::function<void(const bool _result)> &_cb = nullptr);

      /// \brief Private data pointer
      private: std::unique_ptr<IntrospectionCurveHandlerPrivate> dataPtr;
    };
  }
}
#endif
