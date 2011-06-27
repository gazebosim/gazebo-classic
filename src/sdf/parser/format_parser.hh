#ifndef SDF_FORMAT_PARSER_HH
#define SDF_FORMAT_PARSER_HH

#include <string>
#include <boost/shared_ptr.hpp>
#include <tinyxml.h>

#include "sdf/interface/SDF.hh"

namespace sdf
{
  bool initFile(const std::string &_filename, boost::shared_ptr<SDF> _sdf);
  bool initString(const std::string &_xmlString, boost::shared_ptr<SDF> &_sdf);
  bool initDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<SDF> &_sdf);
  bool initXml(TiXmlElement *_xml, boost::shared_ptr<SDFElement> &_sdf);

  bool readFile(const std::string &_filename, boost::shared_ptr<SDF> &_sdf);
  bool readString(const std::string &_xmlString, boost::shared_ptr<SDF> &_sdf);
  bool readDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<SDF> &_sdf);
  bool readXml(TiXmlElement *_xml, boost::shared_ptr<SDF> &_sdf);
  bool readXml(TiXmlElement *_xml, boost::shared_ptr<SDFElement> &_sdf);
}

#endif
