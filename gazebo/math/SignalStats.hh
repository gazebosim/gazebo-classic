/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef _SIGNAL_STATS_HH_
#define _SIGNAL_STATS_HH_

#include <string>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \class SignalStats SignalStats.hh math/gzmath.hh
    /// \brief Statistical properties of a discrete time scalar signal.
    class GAZEBO_VISIBLE SignalStats
    {
      /// \brief Constructor
      public: SignalStats();

      /// \brief Destructor
      public: virtual ~SignalStats();

      /// \brief Get the current value of the statistical measure.
      /// \return Current value of the statistical measure.
      public: virtual double Get() const = 0;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return Short name of the statistical measure.
      public: virtual std::string GetShortName() const = 0;

      /// \brief Get number of data points in measurement.
      public: virtual unsigned int GetCount() const;

      /// \brief Add a new sample to the statistical measure.
      /// \param[in] _data New signal data point.
      public: virtual void Insert(double _data) = 0;

      /// \brief Forget all previous data.
      public: virtual void Reset();

      /// \brief Scalar representation of signal data.
      protected: double data;

      /// \brief Count of data values in mean.
      protected: unsigned int count;
   };
    /// \}

    /// \class SignalMean SignalStats.hh math/gzmath.hh
    /// \brief Computing the mean value of a discretely sampled signal.
    class GAZEBO_VISIBLE SignalMean : public SignalStats
    {
      // Documentation inherited.
      public: virtual double Get() const;

      // Documentation inherited.
      public: virtual std::string GetShortName() const;

      // Documentation inherited.
      public: virtual void Insert(double _data);
    };
    /// \}

    /// \class SignalRootMeanSquare SignalStats.hh math/gzmath.hh
    /// \brief Computing the square root of the mean squared value
    /// of a discretely sampled signal.
    class GAZEBO_VISIBLE SignalRootMeanSquare : public SignalStats
    {
      // Documentation inherited.
      public: virtual double Get() const;

      // Documentation inherited.
      public: virtual std::string GetShortName() const;

      // Documentation inherited.
      public: virtual void Insert(double _data);
    };
    /// \}

    /// \class SignalMaxAbsoluteValue SignalStats.hh math/gzmath.hh
    /// \brief Computing the maximum of the absolute value
    /// of a discretely sampled signal.
    /// Also known as the maximum norm, infinity norm, or supremum norm.
    class GAZEBO_VISIBLE SignalMaxAbsoluteValue : public SignalStats
    {
      // Documentation inherited.
      public: virtual double Get() const;

      // Documentation inherited.
      public: virtual std::string GetShortName() const;

      // Documentation inherited.
      public: virtual void Insert(double _data);
    };
    /// \}
  }
}
#endif

