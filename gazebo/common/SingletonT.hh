/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
/* Desc: Singleton base class
 * Author: Nate Koenig
 * Date: 2 Sept 2007
 */

#ifndef GAZEBO_COMMON_SINGLETONT_HH_
#define GAZEBO_COMMON_SINGLETONT_HH_

#include "gazebo/util/system.hh"

/// \addtogroup gazebo_common Common
/// \{

/// \class SingletonT SingletonT.hh common/common.hh
/// \brief Singleton template class
template <class T>
class SingletonT
{
  /// \brief Get an instance of the singleton
  public: static T *Instance()
          {
            return &GetInstance();
          }

  /// \brief Constructor
  protected: SingletonT() {}

  /// \brief Destructor
  protected: virtual ~SingletonT() {}

  /// \brief Creates and returns a reference to the unique (static) instance
  private: static T &GetInstance()
           {
             static T t;
             return static_cast<T &>(t);
           }

  /// \brief A reference to the unique instance
  private: static T &myself;
};

/// \brief Initialization of the singleton instance.
template <class T>
T &SingletonT<T>::myself = SingletonT<T>::GetInstance();

/// \brief Explicit instantiation for all derived classes
namespace gazebo
{
  namespace common
  {
    class FuelModelDatabase;
    class MeshManager;
    class ModelDatabase;
    class SystemPaths;
  }
  namespace gui
  {
    class KeyEventHandler;
    class ModelAlign;
    class ModelManipulator;
    class ModelSnap;
    class MouseEventHandler;
    class PlotManager;
  }
  namespace rendering
  {
    class RenderEngine;
    class RTShaderSystem;
  }
  namespace sensors
  {
    class SensorManager;
  }
  namespace transport
  {
    class ConnectionManager;
    class TopicManager;
  }
  namespace util
  {
    class DiagnosticManager;
    class IntrospectionManager;
    class LogPlay;
    class LogRecord;
    class OpenAL;
  }
}

template class GZ_COMMON_VISIBLE ::SingletonT<gazebo::common::FuelModelDatabase>;
template class GZ_COMMON_VISIBLE ::SingletonT<gazebo::common::MeshManager>;
template class GZ_COMMON_VISIBLE ::SingletonT<gazebo::common::ModelDatabase>;
template class GZ_COMMON_VISIBLE ::SingletonT<gazebo::common::SystemPaths>;

template class GZ_GUI_VISIBLE ::SingletonT<gazebo::gui::KeyEventHandler>;
template class GZ_GUI_VISIBLE ::SingletonT<gazebo::gui::ModelAlign>;
template class GZ_GUI_VISIBLE ::SingletonT<gazebo::gui::ModelManipulator>;
template class GZ_GUI_VISIBLE ::SingletonT<gazebo::gui::ModelSnap>;
template class GZ_GUI_VISIBLE ::SingletonT<gazebo::gui::MouseEventHandler>;
template class GZ_GUI_VISIBLE ::SingletonT<gazebo::gui::PlotManager>;

template class GZ_RENDERING_VISIBLE ::SingletonT<gazebo::rendering::RenderEngine>;
template class GZ_RENDERING_VISIBLE ::SingletonT<gazebo::rendering::RTShaderSystem>;

template class GZ_SENSORS_VISIBLE ::SingletonT<gazebo::sensors::SensorManager>;

template class GZ_TRANSPORT_VISIBLE ::SingletonT<gazebo::transport::ConnectionManager>;
template class GZ_TRANSPORT_VISIBLE ::SingletonT<gazebo::transport::TopicManager>;

template class GZ_UTIL_VISIBLE ::SingletonT<gazebo::util::DiagnosticManager>;
template class GZ_UTIL_VISIBLE ::SingletonT<gazebo::util::IntrospectionManager>;
template class GZ_UTIL_VISIBLE ::SingletonT<gazebo::util::LogPlay>;
template class GZ_UTIL_VISIBLE ::SingletonT<gazebo::util::LogRecord>;
template class GZ_UTIL_VISIBLE ::SingletonT<gazebo::util::OpenAL>;

/// \}

#endif
