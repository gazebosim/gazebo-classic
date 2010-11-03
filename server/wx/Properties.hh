#ifndef PROPERTIES_HH
#define PROPERTIES_HH

#include <wx/wx.h>

#include "Param.hh"
#include "StaticPluginRegister.hh"

class wxPropertyGrid;
class wxProperty;

namespace gazebo
{
  class Property
  {
    /// \brief Constructor
    public: Property(Param *p, wxPropertyGrid *grid);

    /// \brief Destructor
    public: virtual ~Property();

    public: virtual void Changed();

    protected: Param *param;

    protected: wxPropertyGrid *grid;
    protected: wxPGProperty *property;
  };

  class PropertyManager
  {
    public: PropertyManager(wxPropertyGrid *g);
    public: virtual ~PropertyManager();

    public: void AddProperty(Param *p);


    private: wxPropertyGrid *grid;
    private: std::list<Property*> properties;
  };


  typedef Property* (*PropertyFactoryFn) (Param *, wxPropertyGrid *grid);

  class PropertyFactory
  {

    public: static void RegisterAll();

    public: static void RegisterProperty(std::string type, 
                                         PropertyFactoryFn factoryfn);

    public: static Property *CreateProperty(Param *, wxPropertyGrid *grid);

    private: static std::map<std::string, PropertyFactoryFn> properties;
  };

  /// \brief Static sensor registration macro
  ///
  /// Use this macro to register sensors with the server.
  /// @param name Sensor type name, as it appears in the world file.
  /// @param classname C++ class name for the sensor.
  #define GZ_REGISTER_WX_PROPERTY(type, classname) \
  Property *New##classname(Param *p, wxPropertyGrid *g) \
  { \
    return new classname(p,g); \
  } \
  void Register##classname() \
  {\
    PropertyFactory::RegisterProperty(type, New##classname);\
  }


  class FloatProperty : public Property
  {
    public: FloatProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~FloatProperty();
  };

  class DoubleProperty : public Property
  {
    public: DoubleProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~DoubleProperty();
  };

  class IntProperty : public Property
  {
    public: IntProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~IntProperty();
  };

  class UIntProperty : public Property
  {
    public: UIntProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~UIntProperty();
  };

  class BoolProperty : public Property
  {
    public: BoolProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~BoolProperty();
  };

  class StringProperty : public Property
  {
    public: StringProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~StringProperty();
  };

  class Vector3Property : public Property
  {
    public: Vector3Property(Param *p, wxPropertyGrid *grid);
    public: virtual ~Vector3Property();

    private: wxPGProperty *x;
    private: wxPGProperty *y;
    private: wxPGProperty *z;
  };

  class QuaternProperty : public Property
  {
    public: QuaternProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~QuaternProperty();

    private: wxPGProperty *roll;
    private: wxPGProperty *pitch;
    private: wxPGProperty *yaw;
  };

  class TimeProperty : public Property
  {
    public: TimeProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~TimeProperty();

    private: wxPGProperty *sec;
    private: wxPGProperty *msec;
  };

  class ColorProperty : public Property
  {
    public: ColorProperty(Param *p, wxPropertyGrid *grid);
    public: virtual ~ColorProperty();
  };


}

#endif
