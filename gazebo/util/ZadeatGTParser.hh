/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#ifndef _ZadeatGTParser_HH_
#define _ZadeatGTParser_HH_

#include <fstream>
#include <string>

namespace gazebo
{
  namespace util
  {
    /// addtogroup gazebo_util
    /// \{

    /// \class ZadeatGTParser ZadeatGTParser.hh util/util.hh
    /// \brief Handles loading and parsing a Zadeat dataset with ground truth
    /// positions.
    ///
    class ZadeatGTParser
    {
      /// \brief Constructor.
      /// \param[in] _filename the path to the ground truth data.
      public: ZadeatGTParser(const std::string &_filename);

      /// \brief Destructor.
      public: ~ZadeatGTParser();

      /// \brief Get the next ground truth set of values.
      /// \param[out] _timestamp Timestamp.
      /// \param[out] _headRoll Head roll angle in radians.
      /// \param[out] _headPitch Head pitch angle in radians.
      /// \param[out] _headYaw Head yaw angle in radians.
      /// \param[out] _headX Head X position in meters.
      /// \param[out] _headY Head Y position in meters.
      /// \param[out] _headZ Head Z position in meters.
      /// \param[out] _torsoRoll Torso roll angle in radians.
      /// \param[out] _torsoPitch Torso pitch angle in radians.
      /// \param[out] _torsoYaw Torso yaw angle in radians.
      /// \param[out] _torsoX Torso X position in meters.
      /// \param[out] _torsoY Torso Y position in meters.
      /// \param[out] _torsoZ Torso Z position in meters.
      public: std::istream & GetNextGT(double &_timestamp, double &_headRoll,
        double &_headPitch, double &_headYaw, double &_headX, double &_headY,
        double &_headZ, double &_torsoRoll, double &_torsoPitch,
        double &_torsoYaw, double &_torsoX, double &_torsoY, double &_torsoZ);

      /// \brief File containing the ground truth dataset.
      protected: std::ifstream logFile;
    };
    /// \}
  }
}
#endif
