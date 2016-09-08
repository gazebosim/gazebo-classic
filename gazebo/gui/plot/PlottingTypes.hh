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
#ifndef GAZEBO_GUI_PLOT_PLOTTINGTYPES_HH_
#define GAZEBO_GUI_PLOT_PLOTTINGTYPES_HH_

#include <memory>
#include <set>

#include "gazebo/util/system.hh"

/// \file
/// \ingroup gazebo_gui
/// \brief default namespace for gazebo
namespace gazebo
{
  /// \brief Plotting tool forward declarations and type defines
  namespace gui
  {
    class PlotCurve;

    /// \def PlotCurvePtr
    /// \brief std shared pointer to a PlotCurve object
    typedef std::shared_ptr<PlotCurve> PlotCurvePtr;

    /// \def PlotCurveWeakPtr
    /// \brief std weak pointer to a PlotCurve object
    typedef std::weak_ptr<PlotCurve> PlotCurveWeakPtr;

    /// \def CurveVariableSet
    /// \brief A set of unique plot curve pointers
    using CurveVariableSet = std::set<PlotCurveWeakPtr,
        std::owner_less<PlotCurveWeakPtr> >;
  }
}

#endif
