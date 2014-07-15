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

#include <map>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace math
  {
    /// \addtogroup gazebo_math
    /// \{

    /// \class SignalStatistic SignalStats.hh math/gzmath.hh
    /// \brief Statistical properties of a discrete time scalar signal.
    class GAZEBO_VISIBLE SignalStatistic
    {
      /// \brief Constructor
      public: SignalStatistic();

      /// \brief Destructor
      public: virtual ~SignalStatistic();

      /// \brief Get the current value of the statistical measure.
      /// \return Current value of the statistical measure.
      public: virtual double Get() const = 0;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return Short name of the statistical measure.
      public: virtual std::string GetShortName() const = 0;

      /// \brief Get number of data points in measurement.
      /// \return Number of data points in measurement.
      public: virtual unsigned int GetCount() const;

      /// \brief Add a new sample to the statistical measure.
      /// \param[in] _data New signal data point.
      public: virtual void InsertData(double _data) = 0;

      /// \brief Forget all previous data.
      public: virtual void Reset();

      /// \brief Scalar representation of signal data.
      protected: double data;

      /// \brief Count of data values in mean.
      protected: unsigned int count;
    };
    /// \}

    /// \def SignalStatisticPtr
    /// \brief Boost shared pointer to SignalStatistic object
    typedef boost::shared_ptr<SignalStatistic> SignalStatisticPtr;

    /// \def SignalStatistic_V
    /// \brief Vector of SignalStatisticPtr
    typedef std::vector<SignalStatisticPtr> SignalStatistic_V;

    /// \class SignalMean SignalStats.hh math/gzmath.hh
    /// \brief Computing the mean value of a discretely sampled signal.
    class GAZEBO_VISIBLE SignalMean : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Get() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "Mean"
      public: virtual std::string GetShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(double _data);
    };
    /// \}

    /// \class SignalRootMeanSquare SignalStats.hh math/gzmath.hh
    /// \brief Computing the square root of the mean squared value
    /// of a discretely sampled signal.
    class GAZEBO_VISIBLE SignalRootMeanSquare : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Get() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "Rms"
      public: virtual std::string GetShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(double _data);
    };
    /// \}

    /// \class SignalMaxAbsoluteValue SignalStats.hh math/gzmath.hh
    /// \brief Computing the maximum of the absolute value
    /// of a discretely sampled signal.
    /// Also known as the maximum norm, infinity norm, or supremum norm.
    class GAZEBO_VISIBLE SignalMaxAbsoluteValue : public SignalStatistic
    {
      // Documentation inherited.
      public: virtual double Get() const;

      /// \brief Get a short version of the name of this statistical measure.
      /// \return "MaxAbs"
      public: virtual std::string GetShortName() const;

      // Documentation inherited.
      public: virtual void InsertData(double _data);
    };
    /// \}

    /// \class SignalStats SignalStats.hh math/gzmath.hh
    /// \brief Collection of statistics for a scalar signal.
    class GAZEBO_VISIBLE SignalStats
    {
      /// \brief Constructor
      public: SignalStats();

      /// \brief Destructor
      public: ~SignalStats();

      /// \brief Get number of data points in first statistic.
      /// \return Number of data points in first statistic.
      public: unsigned int GetCount() const;

      /// \brief Get the current values of each statistical measure,
      /// stored in a map using the short name as the key.
      /// \return Map with short name of each statistic as key
      /// and value of statistic as the value.
      public: std::map<std::string, double> GetMap() const;

      /// \brief Add a new sample to the statistical measures.
      /// \param[in] _data New signal data point.
      public: void InsertData(double _data);

      /// \brief Add a new type of statistic.
      /// \param[in] _name Short name of new statistic.
      /// Valid values include:
      ///  "MaxAbs"
      ///  "Mean"
      ///  "Rms"
      /// \return True if statistic was successfully added,
      /// false if name was not recognized or had already
      /// been inserted.
      public: bool InsertStatistic(const std::string &_name);

      /// \brief Add a new type of statistic.
      /// \param[in] _names Comma-separated list of new statistics.
      /// For example, all statistics could be added with:
      ///  "MaxAbs,Mean,Rms"
      /// \return True if all statistics were successfully added,
      /// false if any names were not recognized or had already
      /// been inserted.
      public: bool InsertStatistics(const std::string &_names);

      /// \brief Forget all previous data.
      public: void Reset();

      /// \brief Vector of `SignalStatistic`s.
      private: SignalStatistic_V stats;
    };
    /// \}

    /// \class Vector3Stats SignalStats.hh math/gzmath.hh
    /// \brief Collection of statistics for a Vector3 signal.
    class GAZEBO_VISIBLE Vector3Stats
    {
      /// \brief Constructor
      public: Vector3Stats();

      /// \brief Destructor
      public: ~Vector3Stats();

      /// \brief Add a new sample to the statistical measures.
      /// \param[in] _data New signal data point.
      public: void InsertData(const Vector3 &_data);

      /// \brief Add a new type of statistic.
      /// \param[in] _name Short name of new statistic.
      /// Valid values include:
      ///  "MaxAbs"
      ///  "Mean"
      ///  "Rms"
      /// \return True if statistic was successfully added,
      /// false if name was not recognized or had already
      /// been inserted.
      public: bool InsertStatistic(const std::string &_name);

      /// \brief Add a new type of statistic.
      /// \param[in] _names Comma-separated list of new statistics.
      /// For example, all statistics could be added with:
      ///  "MaxAbs,Mean,Rms"
      /// \return True if all statistics were successfully added,
      /// false if any names were not recognized or had already
      /// been inserted.
      public: bool InsertStatistics(const std::string &_names);

      /// \brief Forget all previous data.
      public: void Reset();

      /// \brief Statistics for x component of signal.
      public: SignalStats x;

      /// \brief Statistics for y component of signal.
      public: SignalStats y;

      /// \brief Statistics for z component of signal.
      public: SignalStats z;

      /// \brief Statistics for magnitude of signal.
      public: SignalStats mag;
    };
    /// \}
  }
}
#endif

