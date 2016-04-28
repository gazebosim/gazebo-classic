/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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
#include <ignition/math/Helpers.hh>
#include "gazebo/common/Temperature.hh"

using namespace gazebo;
using namespace common;

/// The offset between Kelivin and Celsius.
///
/// Conversion equation:
/// Temperature_celsius = Temperature_kelvin - 273.15
///
/// This is only in the source file to prevent symbol collisions, and
/// the raw number does not seem useful to share publicly.
#define KELVIN_CELSIUS_OFFSET 273.15

namespace gazebo
{
  namespace common
  {
    /// \brief Private data for the Temperatue class.
    class TemperaturePrivate
    {
      /// \brief Default constructor
      public: TemperaturePrivate() : kelvin(0.0) {}

      /// \brief Constructor
      /// \param[in] _temp Temperature in Kelvin
      public: TemperaturePrivate(const double _temp) : kelvin(_temp) {}

      /// \brief Temeperate value in Kelvin.
      public: double kelvin;
    };
  }
}

/////////////////////////////////////////////////
Temperature::Temperature()
: dataPtr(new TemperaturePrivate)
{
}

/////////////////////////////////////////////////
Temperature::Temperature(const double _temp)
: dataPtr(new TemperaturePrivate(_temp))
{
}

/////////////////////////////////////////////////
Temperature::Temperature(const Temperature &_temp)
: dataPtr(new TemperaturePrivate(_temp.Kelvin()))
{
}

/////////////////////////////////////////////////
Temperature::~Temperature()
{
}

/////////////////////////////////////////////////
double Temperature::KelvinToCelsius(const double _temp)
{
  return _temp - KELVIN_CELSIUS_OFFSET;
}

/////////////////////////////////////////////////
double Temperature::KelvinToFarenheit(const double _temp)
{
  return _temp * 1.8 - 459.67;
}

/////////////////////////////////////////////////
double Temperature::CelsiusToFarenheit(const double _temp)
{
  return _temp * 1.8 + 32.0;
}

/////////////////////////////////////////////////
double Temperature::CelsiusToKelvin(const double _temp)
{
  return std::max(_temp + KELVIN_CELSIUS_OFFSET, 0.0);
}

/////////////////////////////////////////////////
double Temperature::FarenheitToCelsius(const double _temp)
{
  return (_temp - 32.0) / 1.8;
}

/////////////////////////////////////////////////
double Temperature::FarenheitToKelvin(const double _temp)
{
  return std::max((_temp + 459.67) / 1.8, 0.0);
}

/////////////////////////////////////////////////
void Temperature::SetKelvin(const double _temp)
{
  this->dataPtr->kelvin = std::max(_temp, 0.0);
}

/////////////////////////////////////////////////
void Temperature::SetCelsius(const double _temp)
{
  this->SetKelvin(CelsiusToKelvin(_temp));
}

/////////////////////////////////////////////////
void Temperature::SetFarenheit(const double _temp)
{
  this->SetKelvin(FarenheitToKelvin(_temp));
}

/////////////////////////////////////////////////
double Temperature::Kelvin() const
{
  return this->dataPtr->kelvin;
}

/////////////////////////////////////////////////
double Temperature::Celsius() const
{
  return KelvinToCelsius(this->dataPtr->kelvin);
}

/////////////////////////////////////////////////
double Temperature::Farenheit() const
{
  return KelvinToFarenheit(this->dataPtr->kelvin);
}

/////////////////////////////////////////////////
double Temperature::operator()() const
{
  return this->dataPtr->kelvin;
}

/////////////////////////////////////////////////
Temperature &Temperature::operator=(const double _temp)
{
  this->SetKelvin(_temp);
  return *this;
}

/////////////////////////////////////////////////
Temperature &Temperature::operator=(const Temperature &_temp)
{
  this->SetKelvin(_temp.Kelvin());
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator+(const double _temp)
{
  return Temperature(this->dataPtr->kelvin + _temp);
}

/////////////////////////////////////////////////
Temperature Temperature::operator+(const Temperature &_temp)
{
  return Temperature(this->dataPtr->kelvin + _temp.Kelvin());
}

/////////////////////////////////////////////////
Temperature &Temperature::operator+=(const double _temp)
{
  this->dataPtr->kelvin += _temp;
  return *this;
}

/////////////////////////////////////////////////
Temperature &Temperature::operator+=(const Temperature &_temp)
{
  this->dataPtr->kelvin += _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator-(const double _temp)
{
  return Temperature(this->dataPtr->kelvin - _temp);
}

/////////////////////////////////////////////////
Temperature Temperature::operator-(const Temperature &_temp)
{
  return Temperature(this->dataPtr->kelvin - _temp.Kelvin());
}

/////////////////////////////////////////////////
Temperature &Temperature::operator-=(const double _temp)
{
  this->dataPtr->kelvin -= _temp;
  return *this;
}

/////////////////////////////////////////////////
Temperature &Temperature::operator-=(const Temperature &_temp)
{
  this->dataPtr->kelvin -= _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator*(const double _temp)
{
  return Temperature(this->dataPtr->kelvin * _temp);
}

/////////////////////////////////////////////////
Temperature Temperature::operator*(const Temperature &_temp)
{
  return Temperature(this->dataPtr->kelvin * _temp.Kelvin());
}

/////////////////////////////////////////////////
Temperature &Temperature::operator*=(const double _temp)
{
  this->dataPtr->kelvin *= _temp;
  return *this;
}

/////////////////////////////////////////////////
Temperature &Temperature::operator*=(const Temperature &_temp)
{
  this->dataPtr->kelvin *= _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
Temperature Temperature::operator/(const double _temp)
{
  return Temperature(this->dataPtr->kelvin /= _temp);
}

/////////////////////////////////////////////////
Temperature Temperature::operator/(const Temperature &_temp)
{
  return Temperature(this->dataPtr->kelvin /= _temp.Kelvin());
}

/////////////////////////////////////////////////
Temperature &Temperature::operator/=(const double _temp)
{
  this->dataPtr->kelvin /= _temp;
  return *this;
}

/////////////////////////////////////////////////
Temperature &Temperature::operator/=(const Temperature &_temp)
{
  this->dataPtr->kelvin /= _temp.Kelvin();
  return *this;
}

/////////////////////////////////////////////////
bool Temperature::operator==(const Temperature &_temp) const
{
  return ignition::math::equal(this->dataPtr->kelvin, _temp.Kelvin());
}

/////////////////////////////////////////////////
bool Temperature::operator==(const double _temp) const
{
  return ignition::math::equal(this->dataPtr->kelvin, _temp);
}

/////////////////////////////////////////////////
bool Temperature::operator!=(const Temperature &_temp) const
{
  return !(*this == _temp);
}

/////////////////////////////////////////////////
bool Temperature::operator!=(const double _temp) const
{
  return !(*this == _temp);
}

/////////////////////////////////////////////////
bool Temperature::operator<(const Temperature &_temp) const
{
  return this->dataPtr->kelvin < _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator<(const double _temp) const
{
  return this->dataPtr->kelvin < _temp;
}

/////////////////////////////////////////////////
bool Temperature::operator<=(const Temperature &_temp) const
{
  return this->dataPtr->kelvin <= _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator<=(const double _temp) const
{
  return this->dataPtr->kelvin <= _temp;
}

/////////////////////////////////////////////////
bool Temperature::operator>(const Temperature &_temp) const
{
  return this->dataPtr->kelvin > _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator>(const double _temp) const
{
  return this->dataPtr->kelvin > _temp;
}

/////////////////////////////////////////////////
bool Temperature::operator>=(const Temperature &_temp) const
{
  return this->dataPtr->kelvin >= _temp.Kelvin();
}

/////////////////////////////////////////////////
bool Temperature::operator>=(const double _temp) const
{
  return this->dataPtr->kelvin >= _temp;
}
