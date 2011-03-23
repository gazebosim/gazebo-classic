#ifndef CALLBACKHELPER_HH
#define CALLBACKHELPER_HH

#include <vector>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <google/protobuf/message.h>

#include "common/GazeboError.hh"

namespace gazebo
{
  namespace transport
  {
    class CallbackHelper
    {
      public: CallbackHelper() {}

      /// \brief Get the typename of the message that is handled
      public: virtual std::string GetMsgType() const = 0;

      public: virtual void HandleMessage(const std::string &newdata) = 0;
    };
    typedef boost::shared_ptr<CallbackHelper> CallbackHelperPtr;


    template<class M>
    class CallbackHelperT : public CallbackHelper
    {
      public: CallbackHelperT( const boost::function<void (const boost::shared_ptr<M const> &)> &cb) : callback(cb) 
              {
                // Just some code to make sure we have a google protobuf.
                M test;
                google::protobuf::Message *m;
                if ( (m=dynamic_cast<google::protobuf::Message*>(&test)) ==NULL)
                  gzthrow( "Message type must be a google::protobuf type\n" );
              }

      /// \brief Get the typename of the message that is handled
      public: std::string GetMsgType() const
              {
                M test;
                google::protobuf::Message *m;
                if ((m=dynamic_cast<google::protobuf::Message*>(&test)) ==NULL)
                  gzthrow( "Message type must be a google::protobuf type\n" );
                return m->GetTypeName();
              }

      public: virtual void HandleMessage(const std::string &newdata)
              {
                boost::shared_ptr<M> m( new M );
                m->ParseFromString( newdata );
                this->callback( m );
              }

      private: boost::function<void (const boost::shared_ptr<M const> &)> callback;
    };
    typedef boost::shared_ptr<CallbackHelper> CallbackHelperPtr;
  }
}

#endif
