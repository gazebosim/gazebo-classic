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
      public: SignalStats() {}

      /// \brief Destructor
      public: virtual ~SignalStats() {}

      /// \brief Get the current value of the statistical measure.
      /// \return Current value of the statistical measure.
      public: virtual double Get() const = 0;

      /// \brief Add a new sample to the statistical measure.
      /// \param[in] _data New signal data point.
      public: virtual void Insert(double _data) = 0;

      /// \brief Forget all previous data.
      public: virtual void Reset() = 0;
    };
    /// \}

    /// \class SignalMean SignalStats.hh math/gzmath.hh
    /// \brief Computing the mean value of a discretely sampled signal.
    class GAZEBO_VISIBLE SignalMean : SignalStats
    {
      // Documentation inherited.
      public: SignalMean();

      // Documentation inherited.
      public: virtual ~SignalMean();

      // Documentation inherited.
      public: virtual double Get() const;

      /// \brief Get number of data points in measurement.
      public: virtual unsigned int GetCount() const;

      // Documentation inherited.
      public: virtual void Insert(double _data);

      // Documentation inherited.
      public: virtual void Reset();

      /// \brief Sum of all data values.
      protected: double sum;

      /// \brief Count of data values in mean.
      protected: unsigned int count;
    };
    /// \}

    /// \class SignalRootMeanSquare SignalStats.hh math/gzmath.hh
    /// \brief Computing the square root of the mean squared value
    /// of a discretely sampled signal.
    class GAZEBO_VISIBLE SignalRootMeanSquare : public SignalMean
    {
      // Documentation inherited.
      public: SignalRootMeanSquare();

      // Documentation inherited.
      public: virtual ~SignalRootMeanSquare();

      // Documentation inherited.
      public: virtual double Get() const;

      // Documentation inherited.
      public: virtual void Insert(double _data);
    };
    /// \}
  }
}
#endif

