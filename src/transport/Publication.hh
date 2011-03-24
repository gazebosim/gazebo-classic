#ifndef PUBLICATION_HH
#define PUBLICATION_HH

#include <boost/shared_ptr.hpp>
#include <list>

#include "CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    class Publication
    {
      public: Publication( const std::string &topic );
      public: virtual ~Publication();

      /// \brief Get the topic for this publication
      public: std::string GetTopic() const;

      public: void AddSubscription(const CallbackHelperPtr &callback);

      /// \brief Publish data
      public: void Publish(const std::string &data);

      private: std::string topic;
      private: std::list< CallbackHelperPtr > callbacks;
    };
    typedef boost::shared_ptr<Publication> PublicationPtr;
  }
}
#endif
