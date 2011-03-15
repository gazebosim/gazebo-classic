/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef PROPERTIES_HH
#define PROPERTIES_HH

#include <wx/wx.h>

#include "common/Param.hh"
#include "common/StaticPluginRegister.hh"

class wxPropertyGrid;
class wxProperty;

namespace gazebo
{
	namespace gui
  {
    class Property
    {
      /// \brief Constructor
      public: Property(common::Param *p, wxPropertyGrid *grid);
  
      /// \brief Destructor
      public: virtual ~Property();
  
      public: virtual void Changed();
  
      protected: comon::Param *param;
  
      protected: wxPropertyGrid *grid;
      protected: wxPGProperty *property;
    };
  
    class PropertyManager
    {
      public: PropertyManager(wxPropertyGrid *g);
      public: virtual ~PropertyManager();
  
      public: void AddProperty(common::Param *p);
  
  
      private: wxPropertyGrid *grid;
      private: std::list<Property*> properties;
    };
  
  
    typedef Property* (*PropertyFactoryFn) (common::Param *, wxPropertyGrid *grid);
  
    class PropertyFactory
    {
  
      public: static void RegisterAll();
  
      public: static void RegisterProperty(std::string type, 
                                           PropertyFactoryFn factoryfn);
  
      public: static Property *CreateProperty(common::Param *, wxPropertyGrid *grid);
  
      private: static std::map<std::string, PropertyFactoryFn> properties;
    };
  
    /// \brief Static sensor registration macro
    ///
    /// Use this macro to register sensors with the server.
    /// @param name Sensor type name, as it appears in the world file.
    /// @param classname C++ class name for the sensor.
    #define GZ_REGISTER_WX_PROPERTY(type, classname) \
    Property *New##classname(common::Param *p, wxPropertyGrid *g) \
    { \
      return new classname(p,g); \
    } \
    void Register##classname() \
    {\
      PropertyFactory::RegisterProperty(type, New##classname);\
    }
  
  
    class FloatProperty : public Property
    {
      public: FloatProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~FloatProperty();
    };
  
    class DoubleProperty : public Property
    {
      public: DoubleProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~DoubleProperty();
    };
  
    class IntProperty : public Property
    {
      public: IntProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~IntProperty();
    };
  
    class UIntProperty : public Property
    {
      public: UIntProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~UIntProperty();
    };
  
    class BoolProperty : public Property
    {
      public: BoolProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~BoolProperty();
    };
  
    class StringProperty : public Property
    {
      public: StringProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~StringProperty();
    };
  
    class common::Vector3Property : public Property
    {
      public: common::Vector3Property(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~common::Vector3Property();
  
      private: wxPGProperty *x;
      private: wxPGProperty *y;
      private: wxPGProperty *z;
    };
  
    class QuaternProperty : public Property
    {
      public: QuaternProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~QuaternProperty();
  
      private: wxPGProperty *roll;
      private: wxPGProperty *pitch;
      private: wxPGProperty *yaw;
    };
  
    class TimeProperty : public Property
    {
      public: TimeProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~TimeProperty();
  
      private: wxPGProperty *sec;
      private: wxPGProperty *msec;
    };
  
    class ColorProperty : public Property
    {
      public: ColorProperty(common::Param *p, wxPropertyGrid *grid);
      public: virtual ~ColorProperty();
    };
  
  
  }

}
#endif
