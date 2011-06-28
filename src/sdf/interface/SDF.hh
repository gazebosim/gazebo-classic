#ifndef SDF_HH
#define SDF_HH

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "sdf/interface/Param.hh"

namespace sdf
{
  class SDFElement
  {
    public: boost::shared_ptr<SDFElement> Clone() const;

    public: void SetName(const std::string &_name);
    public: const std::string &GetName() const;

    public: void SetRequired(const std::string &_req);
    public: const std::string &GetRequired() const;

    public: void PrintDescription(std::string _prefix);
    public: void PrintValues(std::string _prefix);

    public: void AddAttribute(const std::string &_key, 
                              const std::string &_type, 
                              const std::string &_defaultvalue,
                              bool _required);

    public: boost::shared_ptr<Param> GetAttribute(const std::string &_key);

    public: boost::shared_ptr<SDFElement> AddElement(const std::string &_name);

    private: std::string name;
    private: std::string required;

    public: std::vector< boost::shared_ptr<Param> > attributes;
    public: std::vector< boost::shared_ptr<SDFElement> > elements;
    public: std::vector< boost::shared_ptr<SDFElement> > elementDescriptions;
  };

  class SDF
  {
    public: SDF();
    public: void PrintDescription();
    public: void PrintValues();

    public: boost::shared_ptr<SDFElement> root;
  };
}
#endif
