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
      /// \brief Constructor
      public: Publication( const std::string &topic, 
                           const std::string &msgType );

      /// \brief Destructor
      public: virtual ~Publication();

      /// \brief Get the topic for this publication
      public: std::string GetTopic() const;

      /// \brief Get the type of message
      public: std::string GetMsgType() const;

      public: void AddSubscription(const CallbackHelperPtr &callback);

      /// \brief Publish data
      public: void Publish(const std::string &data);

      private: std::string topic;
      private: std::string msgType;
      private: std::list< CallbackHelperPtr > callbacks;
    };
    typedef boost::shared_ptr<Publication> PublicationPtr;
  }
}
#endif
