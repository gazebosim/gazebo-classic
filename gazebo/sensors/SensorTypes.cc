/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/sensors/SensorTypes.hh"

using namespace gazebo;
using namespace sensors;

// Initialize enum iterator, and string converter
GZ_ENUM(SensorNoiseType,
    sensors::SENSOR_NOISE_TYPE_BEGIN,
    sensors::SENSOR_NOISE_TYPE_END,
    "NO_NOISE",
    "CAMERA_NOISE",
    "GPU_RAY_NOISE",
    "GPS_POSITION_LATITUDE_NOISE_METERS",
    "GPS_POSITION_LONGITUDE_NOISE_METERS",
    "GPS_POSITION_ALTITUDE_NOISE_METERS",
    "GPS_VELOCITY_LATITUDE_NOISE_METERS",
    "GPS_VELOCITY_LONGITUDE_NOISE_METERS",
    "GPS_VELOCITY_ALTITUDE_NOISE_METERS",
    "RAY_NOISE",
    "MAGNETOMETER_X_NOISE_TESLA",
    "MAGNETOMETER_Y_NOISE_TESLA",
    "MAGNETOMETER_Z_NOISE_TESLA",
    "ALTIMETER_POSITION_NOISE_METERS",
    "ALTIMETER_VELOCITY_NOISE_METERS_PER_S",
    "IMU_ANGVEL_X_NOISE_RADIANS_PER_S",
    "IMU_ANGVEL_Y_NOISE_RADIANS_PER_S",
    "IMU_ANGVEL_Z_NOISE_RADIANS_PER_S",
    "IMU_LINACC_X_NOISE_METERS_PER_S_SQR",
    "IMU_LINACC_Y_NOISE_METERS_PER_S_SQR",
    "IMU_LINACC_Z_NOISE_METERS_PER_S_SQR",
    "SENSOR_NOISE_TYPE_END")
