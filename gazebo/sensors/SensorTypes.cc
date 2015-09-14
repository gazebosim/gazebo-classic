#include "gazebo/sensors/SensorTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    template<> SensorNoiseType common::EnumId<SensorNoiseType>::range[] =
    {
      sensors::SENSOR_NOISE_TYPE_BEGIN,
      sensors::SENSOR_NOISE_TYPE_END
    };

    template<> std::vector<std::string> common::EnumId<SensorNoiseType>::names = {
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
      "SENSOR_NOISE_TYPE_END"
    };
  }
}
