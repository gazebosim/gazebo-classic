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
///
/// This class can be used to simplify the implementation of singletons,
/// i.e. classes that only have one instance. An important constraint to
/// respect is that for each type `T`, the method SingletonT<T>::Instance
/// should be instantiated only once. For this reason this method should
/// be called only once for class, and only in a non-inline method not 
/// defined in an header file. To ensure that the SingletonT<T>::Instance 
/// method is not accidentally called in a inline method, or multiple times, 
/// the method is marked as deprecated. To call it (after making sure that 
/// you are calling it respecting the constraint given in documentation), 
/// make sure to temporary suppress deprecation warnings before calling it,
/// and suppress them again after the Instance method has been called.
///
/// See https://github.com/gazebosim/gazebo-classic/pull/3269 for further
/// details on the use of this class.


template <class T>
class SingletonT
{
  /// \brief Get an instance of the singleton
  public: static T *Instance() GAZEBO_DEPRECATED(11.0)
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
};

/// \brief Helper to declare typed SingletonT
// clang doesn't compile if it explicitly specializes a type before
// the type is defined. (forward declaration is not enough.)
#ifdef __clang__
#define GZ_SINGLETON_DECLARE(visibility, n1, n2, singletonType)
#else
#define GZ_SINGLETON_DECLARE(visibility, n1, n2, singletonType) \
namespace n1 \
{ \
  namespace n2 \
  { \
    class singletonType; \
  } \
} \
template class visibility ::SingletonT<n1::n2::singletonType>;
#endif

/// \}

#endif
