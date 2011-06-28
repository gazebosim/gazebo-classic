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
    public: boost::shared_ptr<SDFElement> Clone() const
            {
              boost::shared_ptr<SDFElement> clone(new SDFElement);
              clone->name = this->name;
              clone->required = this->required;

              std::vector< boost::shared_ptr<Param> >::const_iterator aiter;
              for (aiter = this->attributes.begin(); 
                   aiter != this->attributes.end(); aiter++)
              {
                clone->attributes.push_back((*aiter)->Clone());
              }

              std::vector< boost::shared_ptr<SDFElement> >::const_iterator eiter;
              for (eiter = this->elementDescriptions.begin(); 
                   eiter != this->elementDescriptions.end(); eiter++)
              {
                clone->elementDescriptions.push_back((*eiter)->Clone());
              }

              return clone;
            }

    public: void PrintDescription(std::string _prefix)
            {
              std::cout << _prefix << "<element name='" << this->name << "' required='" << this->required << "'>\n";

              std::vector< boost::shared_ptr<Param> >::iterator aiter;
              for (aiter = this->attributes.begin(); 
                   aiter != this->attributes.end(); aiter++)
              {
                std::cout << _prefix << "  <attribute name='" << (*aiter)->GetKey() << "' type='" << (*aiter)->GetTypeName() << "' default='" << (*aiter)->GetDefaultAsString() << "' required='" << (*aiter)->GetRequired() << "'/>\n";
              }

              std::vector< boost::shared_ptr<SDFElement> >::iterator eiter;
              for (eiter = this->elementDescriptions.begin(); 
                   eiter != this->elementDescriptions.end(); eiter++)
              {
                (*eiter)->PrintDescription(_prefix + "  ");
              }

              std::cout << _prefix << "</element>\n";
            }

    public: void PrintValues(std::string _prefix)
            {
              std::cout << _prefix << "<" << this->name << " ";

              std::vector< boost::shared_ptr<Param> >::iterator aiter;
              for (aiter = this->attributes.begin(); 
                   aiter != this->attributes.end(); aiter++)
              {
                std::cout << (*aiter)->GetKey() << "='" 
                          << (*aiter)->GetAsString() << "' ";
              }

              if(this->elements.size() > 0)
              {
                std::cout << ">\n";
                std::vector< boost::shared_ptr<SDFElement> >::iterator eiter;
                for (eiter = this->elements.begin(); 
                    eiter != this->elements.end(); eiter++)
                {
                  (*eiter)->PrintValues(_prefix + "  ");
                }
                std::cout << _prefix << "</" << this->name << ">\n";
              }
              else
                std::cout << "/>\n";
            }

    public: boost::shared_ptr<Param> GetAttribute(const std::string &_key)
            {
              std::vector< boost::shared_ptr<Param> >::const_iterator iter;
              for (iter = this->attributes.begin(); 
                   iter != this->attributes.end(); iter++)
              {
                if ( (*iter)->GetKey() == _key)
                  return (*iter);
              }
              gzerr << "Unable to find attribute [" << _key << "]\n";
              return boost::shared_ptr<Param>();
            }

    public: boost::shared_ptr<SDFElement> AddElement(const std::string &_name)
            {
              std::vector< boost::shared_ptr<SDFElement> >::const_iterator iter;
              for (iter = this->elementDescriptions.begin(); 
                   iter != this->elementDescriptions.end(); iter++)
              {
                if ((*iter)->name == _name)
                {
                  this->elements.push_back( (*iter)->Clone() );
                  return this->elements.back();
                }
              }
              gzerr << "Missing element description for [" << _name << "]\n";
              return boost::shared_ptr<SDFElement>();
            }

    public: std::string name;
    public: std::string required;

    public: std::vector< boost::shared_ptr<Param> > attributes;
    public: std::vector< boost::shared_ptr<SDFElement> > elementDescriptions;
    public: std::vector< boost::shared_ptr<SDFElement> > elements;
  };

  class SDF
  {

    public: void PrintDescription()
            {
              std::vector< boost::shared_ptr<SDFElement> >::iterator iter;
              for (iter = this->elementDescriptions.begin(); 
                   iter != this->elementDescriptions.end(); iter++)
              {
                (*iter)->PrintDescription("");
              }
            }

    public: void PrintValues()
            {
              std::vector< boost::shared_ptr<SDFElement> >::iterator iter;
              for (iter = this->elements.begin(); 
                   iter != this->elements.end(); iter++)
              {
                (*iter)->PrintValues("");
              }
            }

    public: boost::shared_ptr<SDFElement> AddElement(const std::string &_name)
            {
              std::vector< boost::shared_ptr<SDFElement> >::const_iterator iter;
              for (iter = this->elementDescriptions.begin(); 
                   iter != this->elementDescriptions.end(); iter++)
              {
                if ((*iter)->name == _name)
                {
                  this->elements.push_back( (*iter)->Clone() );
                  return this->elements.back();
                }
              }
              gzerr << "Missing element description for [" << _name << "]\n";
              return boost::shared_ptr<SDFElement>();
            }

    public: std::vector< boost::shared_ptr<SDFElement> > elementDescriptions;
    public: std::vector< boost::shared_ptr<SDFElement> > elements;
  };
}
#endif
